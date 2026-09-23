"""
ground_station_core_sphere.py

KÜRE FORMASYONU YER İSTASYONU
=============================
Bu dosya, TÜBİTAK 123E294 projesinin İP-3 ve İP-5 iş paketlerinde
tanımlanan "Hedefin Üzerine Yönelmeden Konumlandırma (Küre Formasyonu)"
yönteminin tam uygulamasıdır.

Bu dosyayı DOĞRUDAN ÇALIŞTIRMAYIN. Bunun yerine:
  - Windows'ta:  ground_station_windows_sphere.py
  - Linux'ta:    ground_station_linux_sphere.py
dosyalarını çalıştırın.

==========================================================================
UYGULANAN YÖNTEM (Güzey 2022 + Gelişme Raporu 2, İP-3)
==========================================================================
Proje raporlarında tanımlanan küre formasyonu yöntemi şu adımlardan
oluşur:

GRUP YAPISI:
  - Grup 1: D1 ve D3 → x-y düzleminde (yatay) çember çizerler
    (hedefin yatay yönünü bulur). Dönme açısı: α (alfa)
  - Grup 2: D2 ve D4 → x-z düzleminde (dikey) çember çizerler
    (hedefin dikey/yükseklik yönünü bulur). Dönme açısı: β (beta)

İHA POZİSYONLARI (Rapor Denklem 29/34):
    D1: x = xc - R·cos(α),  y = yc - R·sin(α),  z = zc
    D3: x = xc + R·cos(α),  y = yc + R·sin(α),  z = zc
    D2: x = xc - R·sin(β),  y = yc,               z = zc - R·cos(β)
    D4: x = xc + R·cos(β),  y = yc,               z = zc + R·sin(β)

GÜÇ FARKLARI (Denklem 30):
    e1 = P3 - P1   (Grup 1 → α'yı sürer)
    e2 = P4 - P2   (Grup 2 → β'yı sürer)

PID KONTROL İLE DÖNME AÇISI TÜREVLERİ (Denklem 32):
    α̇ = Kp·e1 + Ki·∫e1·dτ + Kd·ė1
    β̇ = Kp·e2 + Ki·∫e2·dτ + Kd·ė2

AÇI GÜNCELLEMESİ (Denklem 33):
    α(t) = α(t₀) + ∫α̇·dτ   (sayısal: α += α̇ · dt)
    β(t) = β(t₀) + ∫β̇·dτ

GEÇİŞ NOKTASI HESABI (Denklem 36):
    ra   = |R · cos(β)|
    zaug = zc + R · sin(α) · sin(β)
    xaug = xc - ra · sin(α)
    yaug = yc - ra · cos(α)

HEDEF YÖN DOĞRUSU (Denklem 37):
    C(xc,yc,zc) → A(xaug,yaug,zaug) doğrusu hedefin yönünü verir.
    Merkez bu doğru boyunca kaydırılarak hedef lokalize edilir.

KALMAN FİLTRESİ (Rapor 2, Denklem 24-28):
    Her drone'un ham RSSI değeri ayrı bir skaler Kalman filtresinden
    geçirilerek gürültü bastırılır.

BENCH TEST NOTU:
    Küre formasyonunda D2 ve D4 dronları x-z düzleminde (dikey) çember
    çizer → yani gerçek uçuşta İRTİFA DEĞİŞTİRİRLER. Pervanesiz
    masaüstü testinde bu fiziksel olarak mümkün değildir; sadece yatay
    komutlar MAVLink üzerinden gönderilir, irtifa sabit tutulur.
    BENCH_TEST_MODE = True iken β açısı güncellenmez (β̇ = 0) ve tüm
    dronlar sabit irtifada kalır. Sadece α ekseni (yatay) aktif çalışır.
==========================================================================
"""

import time
import random
import sys
import threading
import math
import socket
import json

try:
    import serial
except ImportError:
    print("pyserial kütüphanesi eksik. 'pip install pyserial'")
    sys.exit(1)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


# ==========================================
# AYARLAR (CONFIG)
# ==========================================
FORMATION_RADIUS_M = 5.0       # R: küre yarıçapı (dronlar arası 2R = 10m)
FORMATION_ALT_M    = 15.0      # Dronların uçuş irtifası (merkez z_c)

MAX_CENTER_DRIFT_M = 50.0      # Merkezin origin'den max sapması

RSSI_MAX_AGE_S = 30.0
RSSI_MAX_AGE_S = max(RSSI_MAX_AGE_S, 6.0)

ADJUST_ORIGIN_FOR_DRONE1_POSITION = True

COMMAND_LOOP_HZ = 0.2          # 5 saniyede bir döngü (dt = 5s)

# ==========================================
# KALMAN FİLTRESİ AYARLARI (Rapor 2)
# ==========================================
KALMAN_Q = 1.0
KALMAN_R = 20.0

# ==========================================
# PID KONTROL PARAMETRELERİ (Denklem 32)
# ==========================================
# α̇ = Kp·e1 + Ki·∫e1·dτ + Kd·ė1
# β̇ = Kp·e2 + Ki·∫e2·dτ + Kd·ė2
#
# Bu katsayılar dönme açısının türevini (radyan/saniye) belirler.
# Güç farkı dBm cinsinden geldiği için, 1 dB fark ≈ Kp rad/s
# dönme hızına karşılık gelir.
#
# Çok büyük Kp → hızlı ama salınımlı dönüş
# Çok küçük Kp → yavaş ama kararlı dönüş
# Ki → kalıcı hatayı (steady-state error) sıfırlar
# Kd → ani değişimleri sönümler
PID_KP = 0.02     # rad/s per dBm fark
PID_KI = 0.005    # rad/s per dBm·s (integral)
PID_KD = 0.01     # rad/s per dBm/s (türev)

# İntegral windup koruması (birikim sınırı)
PID_INTEGRAL_MAX = 50.0

# Güç farkı bu eşiğin altına düştüğünde "yakınsadı" sayılır (dBm)
CONVERGENCE_THRESHOLD_DBM = 1.0

# ==========================================
# KALİBRASYON OFSETLERİ
# ==========================================
CALIBRATION_OFFSET_DB = {
    1: -2.1,
    2:  1.9,
    3: -0.1,
    4:  0.2,
}

# ==========================================
# TEST MODU
# ==========================================
BENCH_TEST_MODE = True

if BENCH_TEST_MODE:
    AIRBORNE_RELATIVE_ALT_THRESHOLD_M = -10.0
else:
    AIRBORNE_RELATIVE_ALT_THRESHOLD_M = 2.0

EARTH_RADIUS_M = 6378137.0
LORA_BAUD     = 9600
PIXHAWK_BAUD  = 57600
LORA_RECONNECT_DELAY_S    = 2.0
MAVLINK_HEARTBEAT_TIMEOUT_S = 10.0
MAVLINK_RECONNECT_DELAY_S   = 3.0


def compute_checksum(payload: str) -> int:
    return sum(ord(c) for c in payload) % 256


# ==========================================
# KALMAN FİLTRESİ (Rapor 2, Denklem 24-28)
# ==========================================
class ScalarKalmanFilter:
    """
    Tek durumlu skaler Kalman filtresi.
    F = 1 (sabit sinyal varsayımı), H = 1 (doğrudan RSSI ölçümü).
    """
    def __init__(self, initial_value, q=KALMAN_Q, r=KALMAN_R):
        self.x = float(initial_value)
        self.p = float(r)
        self.q = float(q)
        self.r = float(r)

    def update(self, measurement):
        x_pred = self.x
        p_pred = self.p + self.q
        w = p_pred / (p_pred + self.r)
        self.x = x_pred + w * (measurement - x_pred)
        self.p = (1.0 - w) * p_pred
        return self.x


# ==========================================
# PID KONTROLCÜ (Denklem 32)
# ==========================================
class PIDController:
    """
    Tek eksenli PID kontrolcü.
    Girdi: hata (e, dBm cinsinden güç farkı)
    Çıktı: dönme açısı türevi (α̇ veya β̇, rad/s)
    """
    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = None

    def compute(self, error, dt):
        """error = e (dBm), dt = döngü periyodu (saniye). Döndürür: açı türevi (rad/s)."""
        # Oransal (Proportional)
        p_term = self.kp * error

        # İntegral (birikimli)
        self.integral += error * dt
        self.integral = max(min(self.integral, PID_INTEGRAL_MAX), -PID_INTEGRAL_MAX)
        i_term = self.ki * self.integral

        # Türev (Derivative)
        if self.prev_error is not None:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error

        return p_term + i_term + d_term

    def reset(self):
        self.integral = 0.0
        self.prev_error = None


# ==========================================
# SİMÜLASYON YARDIMCILARI
# ==========================================
class MockSerial:
    def __init__(self):
        self._next_id = 1

    def read(self, size):
        drone_id = self._next_id
        self._next_id = (self._next_id % 4) + 1
        fake_rssi = random.randint(-90, -60)
        body = f"N{drone_id},{fake_rssi}"
        checksum = compute_checksum(body)
        reply = f"{body},{checksum}\n"
        time.sleep(0.25)
        return reply.encode("ascii")


class MockMavlinkConnection:
    def __init__(self, drone_id):
        self.target_id = drone_id
        self.target_system = drone_id
        self.target_component = 1
        self._armed = True
        self._mode = "GUIDED"
        self.relative_alt_m = 15.0
        self.lat = 41.0 + drone_id * 0.0001
        self.lon = 34.0 + drone_id * 0.0001

    def wait_heartbeat(self, timeout=10):
        return True

    def motors_armed(self):
        return self._armed

    @property
    def flightmode(self):
        return self._mode

    def set_position_target_global_int_send(self, *args):
        lat_int = args[5]
        lon_int = args[6]
        alt = args[7]
        print(f"[SİM-MAVLink] Drone {self.target_id} -> "
              f"LAT: {lat_int/1e7:.6f}, LON: {lon_int/1e7:.6f}, ALT: {alt:.1f}m")


class DroneLink:
    """Bir drone'un MAVLink bağlantısı + arka planda güncellenen durumu."""
    def __init__(self, drone_id, connection, simulation):
        self.drone_id = drone_id
        self.connection = connection
        self.simulation = simulation
        self.lock = threading.Lock()

        self.armed = False
        self.flightmode = None
        self.lat = None
        self.lon = None
        self.relative_alt_m = None
        self.last_heartbeat_time = 0.0
        self.last_position_time = 0.0
        self._stop = False

        if simulation:
            self.armed = True
            self.flightmode = "GUIDED"
            self.lat = connection.lat
            self.lon = connection.lon
            self.relative_alt_m = connection.relative_alt_m
            self.last_heartbeat_time = time.time()
            self.last_position_time = time.time()
        else:
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()

    def _reader_loop(self):
        while not self._stop:
            try:
                msg = self.connection.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT"],
                    blocking=True, timeout=1.0
                )
                if msg is None:
                    continue
                if msg.get_type() == "HEARTBEAT":
                    with self.lock:
                        self.armed = bool(self.connection.motors_armed())
                        self.flightmode = self.connection.flightmode
                        self.last_heartbeat_time = time.time()
                elif msg.get_type() == "GLOBAL_POSITION_INT":
                    with self.lock:
                        self.lat = msg.lat / 1e7
                        self.lon = msg.lon / 1e7
                        self.relative_alt_m = msg.relative_alt / 1000.0
                        self.last_position_time = time.time()
            except Exception as e:
                print(f"[UYARI MAVLINK] Drone {self.drone_id}: {e}")
                time.sleep(1.0)

    def is_ready_for_commands(self):
        with self.lock:
            if time.time() - self.last_heartbeat_time > MAVLINK_HEARTBEAT_TIMEOUT_S:
                return False, "heartbeat kaybı"
            if not self.armed:
                return False, "armed değil"
            if self.flightmode != "GUIDED":
                return False, f"mod GUIDED değil ({self.flightmode})"
            if self.relative_alt_m is None or self.relative_alt_m < AIRBORNE_RELATIVE_ALT_THRESHOLD_M:
                return False, "havada değil"
            return True, "hazır"

    def stop(self):
        self._stop = True


def offset_latlon(base_lat, base_lon, north_m, east_m):
    """Küçük mesafeler için equirectangular lat/lon ofseti."""
    dlat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    dlon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(base_lat)))) * (180.0 / math.pi)
    return base_lat + dlat, base_lon + dlon


# ==========================================
# YER İSTASYONU (KÜRE FORMASYONU)
# ==========================================
class GroundStation:
    """
    Küre formasyonu karar motoru.

    DURUM MAKİNESİ:
    ───────────────
    1) ORBIT  : Dronlar küre üzerinde PID kontrollü döner.
                α ve β açıları, güç farkları (e1, e2) sıfıra
                yaklaşana kadar güncellenir.

    2) CONVERGED: Güç farkları eşik altına düştüğünde geçiş
                  noktası A hesaplanır ve C→A doğrusu (hedef
                  yönü) ekrana yazdırılır. Merkez bu doğru
                  boyunca adım adım kaydırılabilir.
    """

    # Durum sabitleri
    STATE_ORBIT     = "ORBIT"
    STATE_CONVERGED = "CONVERGED"

    def __init__(self, lora_port, pixhawk_ports, simulation, platform_label=""):
        self.lora_port = lora_port
        self.pixhawk_ports = pixhawk_ports
        self.simulation = simulation
        self.platform_label = platform_label

        self.lora = None
        self.drone_links = {}

        # RSSI verileri
        self.rssi_lock = threading.Lock()
        self.rssi_data = {}
        self.window_samples = {}
        self.kalman_filters = {}

        # Origin (GPS referans noktası)
        self.origin_lat = None
        self.origin_lon = None

        # Küre merkezi (metre cinsinden, origin'e göre)
        self.center_x = 0.0   # Doğu
        self.center_y = 0.0   # Kuzey
        self.center_z = FORMATION_ALT_M  # İrtifa

        # Dönme açıları (radyan) - başlangıçta π/4 (45°)
        self.alpha = math.pi / 4.0   # x-y düzlemi (D1, D3)
        self.beta  = math.pi / 4.0   # x-z düzlemi (D2, D4)

        # PID kontrolcüler
        self.pid_alpha = PIDController()  # e1 → α̇
        self.pid_beta  = PIDController()  # e2 → β̇

        # Durum makinesi
        self.state = self.STATE_ORBIT
        self.convergence_count = 0  # Kaç ardışık döngüde yakınsadı

        # Hedef doğruları (iterasyon bazında biriktirme)
        self.direction_lines = []   # [(C, A), ...]

        self._lora_stop = False

        # ── Başlangıç Mesajları ──
        print("=" * 60)
        print("  🌐 KÜRE FORMASYONU YER İSTASYONU 🌐")
        print("  TÜBİTAK 123E294 - Güzey vd. (2022)")
        if platform_label:
            print(f"  Platform: {platform_label}")
        print("=" * 60)
        print(f"Mod: {'SİMÜLASYON' if simulation else 'GERÇEK UÇUŞ'}")
        print(f"R={FORMATION_RADIUS_M}m | ALT={FORMATION_ALT_M}m | "
              f"Geofence=±{MAX_CENTER_DRIFT_M}m")
        print(f"PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
        print(f"Kalman: Q={KALMAN_Q}, R={KALMAN_R}")
        print(f"Kalibrasyon: {CALIBRATION_OFFSET_DB}")
        print(f"Yakınsama eşiği: ±{CONVERGENCE_THRESHOLD_DBM} dBm")

        if BENCH_TEST_MODE:
            print("\n" + "!" * 60)
            print("!!  BENCH_TEST_MODE = True                               !!")
            print("!!  β açısı DONDURULDU (dikey dönüş yok).                !!")
            print("!!  Tüm dronlar sabit irtifada kalacak.                  !!")
            print("!!  Sadece α (yatay) ekseni aktif.                       !!")
            print("!!  GERÇEK UÇUŞTAN ÖNCE False YAPIN.                     !!")
            print("!" * 60 + "\n")

        self.connect_hardware()
        self.start_lora_listener()
        self.capture_origin()

    # ─────────────────────────────────────────────
    # BAĞLANTI KURULUMU
    # ─────────────────────────────────────────────
    def connect_hardware(self):
        if self.simulation:
            print("[SİSTEM] Simülasyon modu.")
            for i in range(1, 5):
                self.drone_links[i] = DroneLink(i, MockMavlinkConnection(i), True)
            return

        print(f"[SİSTEM] LoRa bağlanıyor: {self.lora_port}")
        while self.lora is None:
            try:
                self.lora = serial.Serial(self.lora_port, LORA_BAUD, timeout=0.1)
                print(f"[OK] LoRa: {self.lora_port}")
            except Exception as e:
                print(f"[HATA] LoRa: {e}. {LORA_RECONNECT_DELAY_S}s sonra tekrar...")
                time.sleep(LORA_RECONNECT_DELAY_S)

        if mavutil is None:
            print("[HATA] pymavlink gerekli!")
            sys.exit(1)

        for drone_id, port in self.pixhawk_ports.items():
            conn = None
            while conn is None:
                try:
                    conn = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
                    conn.wait_heartbeat(timeout=MAVLINK_HEARTBEAT_TIMEOUT_S)
                    print(f"[OK] Drone {drone_id}: {port}")
                except Exception as e:
                    print(f"[HATA] Drone {drone_id}: {e}")
                    conn = None
                    time.sleep(MAVLINK_RECONNECT_DELAY_S)
            self.drone_links[drone_id] = DroneLink(drone_id, conn, False)

    # ─────────────────────────────────────────────
    # ORIGIN YAKALAMA
    # ─────────────────────────────────────────────
    def capture_origin(self):
        print("[SİSTEM] Origin için Drone 1 GPS bekleniyor...")
        link = self.drone_links.get(1)
        deadline = time.time() + 300.0

        while time.time() < deadline:
            with link.lock:
                lat, lon = link.lat, link.lon
            if lat is not None and lon is not None:
                if ADJUST_ORIGIN_FOR_DRONE1_POSITION:
                    self.origin_lat, self.origin_lon = offset_latlon(
                        lat, lon, -FORMATION_RADIUS_M, 0.0
                    )
                else:
                    self.origin_lat, self.origin_lon = lat, lon
                print(f"[OK] Origin: LAT {self.origin_lat:.6f}, LON {self.origin_lon:.6f}")
                return
            time.sleep(0.5)

        print("[UYARI] Origin ayarlanamadı (5dk timeout).")

    # ─────────────────────────────────────────────
    # LORA DİNLEYİCİ
    # ─────────────────────────────────────────────
    def start_lora_listener(self):
        threading.Thread(target=self._lora_loop, daemon=True).start()

    def _lora_loop(self):
        print("[SİSTEM] LoRa dinleyici başlatıldı.")
        rx_buf = ""
        while not self._lora_stop:
            try:
                if self.simulation:
                    if not hasattr(self, "_mock"):
                        self._mock = MockSerial()
                    data = self._mock.read(256)
                else:
                    if self.lora is None or not self.lora.is_open:
                        self._reconnect_lora()
                        continue
                    data = self.lora.read(256)

                if data:
                    rx_buf += data.decode(errors="ignore")
                    while "\n" in rx_buf:
                        line, rx_buf = rx_buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._handle_line(line)
                time.sleep(0.01)
            except Exception as e:
                print(f"[HATA RADYO] {e}")
                if not self.simulation:
                    self._reconnect_lora()
                time.sleep(0.5)

    def _reconnect_lora(self):
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass
        self.lora = None
        while self.lora is None and not self._lora_stop:
            try:
                self.lora = serial.Serial(self.lora_port, LORA_BAUD, timeout=0.1)
                print(f"[OK] LoRa yeniden bağlandı.")
            except Exception:
                time.sleep(LORA_RECONNECT_DELAY_S)

    def _handle_line(self, line):
        print(f"[RADYO] {line}")
        try:
            if not line.startswith("N"):
                return
            parts = line[1:].split(",")
            drone_id = int(parts[0])
            if len(parts) != 3:
                return
            rssi = int(parts[1])
            chk = int(parts[2])
            body = f"N{drone_id},{rssi}"
            if chk != compute_checksum(body):
                print(f"[UYARI] Checksum hatalı: '{line}'")
                return
            if drone_id not in self.pixhawk_ports and not self.simulation:
                return

            with self.rssi_lock:
                cal = rssi + CALIBRATION_OFFSET_DB.get(drone_id, 0.0)
                if drone_id not in self.kalman_filters:
                    self.kalman_filters[drone_id] = ScalarKalmanFilter(cal)
                filt = self.kalman_filters[drone_id].update(cal)
                self.rssi_data[drone_id] = (filt, time.time())
                self.window_samples.setdefault(drone_id, []).append(filt)

                # UDP yayını (Radar GUI)
                try:
                    if not hasattr(self, 'udp_sock'):
                        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    msg = json.dumps({d: int(v[0]) for d, v in self.rssi_data.items()})
                    self.udp_sock.sendto(msg.encode('utf-8'), ("127.0.0.1", 5555))
                except Exception:
                    pass

        except (ValueError, IndexError):
            print(f"[UYARI] Format hatalı: '{line}'")

    def get_windowed_rssi(self):
        """5sn penceresi üzerinden ortalanmış RSSI snapshot'ı."""
        now = time.time()
        snap = {}
        counts = {}
        with self.rssi_lock:
            for did in (1, 2, 3, 4):
                samples = self.window_samples.get(did, [])
                if samples:
                    snap[did] = sum(samples) / len(samples)
                    counts[did] = len(samples)
                elif did in self.rssi_data:
                    r, ts = self.rssi_data[did]
                    if now - ts <= RSSI_MAX_AGE_S:
                        snap[did] = r
                        counts[did] = 0
            self.window_samples = {}
        if counts:
            s = ", ".join(f"D{d}:{c}" for d, c in sorted(counts.items()))
            print(f"[PENCERE] Örnekler: {s}")
        return snap

    # ─────────────────────────────────────────────
    # KÜRE FORMASYONU KARAR MOTORU
    # ─────────────────────────────────────────────
    def compute_drone_positions(self):
        """
        Mevcut α, β ve merkez konumuna göre 4 drone'un
        hedef pozisyonlarını hesaplar (metre, origin'e göre).

        Rapor Denklem 29/34:
          D1: (xc - R·cos(α), yc - R·sin(α), zc)
          D3: (xc + R·cos(α), yc + R·sin(α), zc)
          D2: (xc - R·sin(β), yc,             zc - R·cos(β))
          D4: (xc + R·cos(β), yc,             zc + R·sin(β))

        Koordinat eşleştirmesi:
          Rapordaki x → bizim east_m (Doğu)
          Rapordaki y → bizim north_m (Kuzey)
          Rapordaki z → bizim altitude (İrtifa)
        """
        R = FORMATION_RADIUS_M
        cx, cy, cz = self.center_x, self.center_y, self.center_z

        cos_a = math.cos(self.alpha)
        sin_a = math.sin(self.alpha)
        cos_b = math.cos(self.beta)
        sin_b = math.sin(self.beta)

        # Grup 1: D1, D3 → x-y düzlemi (yatay çember)
        d1_east  = cx - R * cos_a
        d1_north = cy - R * sin_a
        d1_alt   = cz

        d3_east  = cx + R * cos_a
        d3_north = cy + R * sin_a
        d3_alt   = cz

        if BENCH_TEST_MODE:
            # Bench test: D2, D4 de yatayda kalır, irtifa sabit
            # β'yı sadece yatay düzleme yansıtıyoruz
            d2_east  = cx - R * sin_b
            d2_north = cy
            d2_alt   = cz

            d4_east  = cx + R * cos_b
            d4_north = cy
            d4_alt   = cz
        else:
            # Gerçek uçuş: D2, D4 → x-z düzlemi (dikey çember, irtifa değişir)
            d2_east  = cx - R * sin_b
            d2_north = cy
            d2_alt   = cz - R * cos_b

            d4_east  = cx + R * cos_b
            d4_north = cy
            d4_alt   = cz + R * sin_b

        return {
            1: (d1_north, d1_east, d1_alt),
            3: (d3_north, d3_east, d3_alt),
            2: (d2_north, d2_east, d2_alt),
            4: (d4_north, d4_east, d4_alt),
        }

    def compute_augmented_point(self):
        """
        Geçiş noktası A hesabı (Rapor Denklem 36):
          ra   = |R · cos(β)|
          zaug = zc + R · sin(α) · sin(β)
          xaug = xc - ra · sin(α)
          yaug = yc - ra · cos(α)
        """
        R = FORMATION_RADIUS_M
        ra = abs(R * math.cos(self.beta))
        x_aug = self.center_x - ra * math.sin(self.alpha)
        y_aug = self.center_y - ra * math.cos(self.alpha)
        z_aug = self.center_z + R * math.sin(self.alpha) * math.sin(self.beta)

        return x_aug, y_aug, z_aug

    def compute_direction_vector(self, x_aug, y_aug, z_aug):
        """
        Merkez C'den geçiş noktası A'ya yön vektörü.
        Bu vektör, hedefin konumunu gösterir (Denklem 37).
        """
        dx = self.center_x - x_aug
        dy = self.center_y - y_aug
        dz = self.center_z - z_aug
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 0.001:
            return 0.0, 0.0, 0.0
        return dx / mag, dy / mag, dz / mag

    def update_rotation_angles(self, rssi_snap, dt):
        """
        PID kontrolcü ile α ve β açılarını günceller.

        Güç farkları (Denklem 30):
          e1 = P3 - P1  → α'yı sürer (Grup 1, yatay)
          e2 = P4 - P2  → β'yı sürer (Grup 2, dikey)
        """
        have_g1 = 1 in rssi_snap and 3 in rssi_snap
        have_g2 = 2 in rssi_snap and 4 in rssi_snap

        e1 = None
        e2 = None
        alpha_dot = 0.0
        beta_dot  = 0.0

        # ── Grup 1: D1, D3 → α ──
        if have_g1:
            e1 = rssi_snap[3] - rssi_snap[1]
            alpha_dot = self.pid_alpha.compute(e1, dt)
            self.alpha += alpha_dot * dt
            # α'yı 0..2π arasında tut
            self.alpha = self.alpha % (2 * math.pi)
        else:
            missing = [d for d in [1, 3] if d not in rssi_snap]
            print(f"[UYARI] Grup 1 (D1/D3) taze değil. Eksik: {missing}. α sabit.")

        # ── Grup 2: D2, D4 → β ──
        if have_g2 and not BENCH_TEST_MODE:
            e2 = rssi_snap[4] - rssi_snap[2]
            beta_dot = self.pid_beta.compute(e2, dt)
            self.beta += beta_dot * dt
            self.beta = self.beta % (2 * math.pi)
        elif have_g2 and BENCH_TEST_MODE:
            e2 = rssi_snap[4] - rssi_snap[2]
            # β donduruldu ama e2'yi logluyoruz
            print(f"[BENCH] β donduruldu. e2={e2:.1f}dBm (sadece log)")
        else:
            missing = [d for d in [2, 4] if d not in rssi_snap]
            print(f"[UYARI] Grup 2 (D2/D4) taze değil. Eksik: {missing}. β sabit.")

        # ── Yakınsama kontrolü ──
        converged_g1 = (e1 is not None and abs(e1) < CONVERGENCE_THRESHOLD_DBM)
        converged_g2 = (e2 is not None and abs(e2) < CONVERGENCE_THRESHOLD_DBM) or BENCH_TEST_MODE

        if converged_g1 and converged_g2:
            self.convergence_count += 1
        else:
            self.convergence_count = 0

        # ── Log ──
        e1_str = f"{e1:.1f}dBm" if e1 is not None else "N/A"
        e2_str = f"{e2:.1f}dBm" if e2 is not None else "N/A"
        print(f"[KÜRE] α={math.degrees(self.alpha):.1f}° (α̇={alpha_dot:.4f} rad/s) | "
              f"β={math.degrees(self.beta):.1f}° (β̇={beta_dot:.4f} rad/s)")
        print(f"[KÜRE] e1(P3-P1)={e1_str} | e2(P4-P2)={e2_str} | "
              f"Yakınsama: {self.convergence_count}/3")

        return e1, e2

    # ─────────────────────────────────────────────
    # MAVLINK KOMUT GÖNDERİMİ
    # ─────────────────────────────────────────────
    def send_mavlink_commands(self):
        if self.origin_lat is None or self.origin_lon is None:
            print("[UYARI] Origin yok, komut gönderilmiyor.")
            return

        positions = self.compute_drone_positions()

        for drone_id, (north_m, east_m, alt_m) in positions.items():
            link = self.drone_links.get(drone_id)
            if link is None:
                continue

            ready, reason = link.is_ready_for_commands()
            if not ready and not self.simulation:
                print(f"[GÜVENLİK] Drone {drone_id}: {reason}, ATLANIYOR.")
                continue

            target_lat, target_lon = offset_latlon(
                self.origin_lat, self.origin_lon, north_m, east_m
            )

            conn = link.connection
            if hasattr(conn, "mav"):
                conn.mav.set_position_target_global_int_send(
                    0,
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,
                    int(target_lat * 1e7),
                    int(target_lon * 1e7),
                    alt_m,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print(f"[MAVLINK] Drone {drone_id} -> "
                      f"LAT:{target_lat:.6f} LON:{target_lon:.6f} ALT:{alt_m:.1f}m "
                      f"(N:{north_m:.1f}m E:{east_m:.1f}m)")
            else:
                conn.set_position_target_global_int_send(
                    0, 0, 0, 6, 0,
                    int(target_lat * 1e7), int(target_lon * 1e7), alt_m,
                    0, 0, 0, 0, 0, 0, 0, 0
                )

    # ─────────────────────────────────────────────
    # MERKEZ KAYDIRMA (hedef doğrusu boyunca)
    # ─────────────────────────────────────────────
    def shift_center_toward_target(self):
        """
        Yakınsama sağlandığında geçiş noktası A hesaplanır,
        C→A doğrusu boyunca merkez 3 metre kaydırılır.
        Ardından PID sıfırlanır ve yeni iterasyona geçilir.
        """
        x_aug, y_aug, z_aug = self.compute_augmented_point()
        dir_x, dir_y, dir_z = self.compute_direction_vector(x_aug, y_aug, z_aug)

        step_m = 3.0  # Her iterasyonda merkez 3m kaydırılır

        self.center_x += dir_x * step_m
        self.center_y += dir_y * step_m
        if not BENCH_TEST_MODE:
            self.center_z += dir_z * step_m

        # Geofence
        self.center_x = max(min(self.center_x, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
        self.center_y = max(min(self.center_y, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)

        # Doğruyu kaydet
        self.direction_lines.append({
            'center': (self.center_x, self.center_y, self.center_z),
            'augmented': (x_aug, y_aug, z_aug),
            'direction': (dir_x, dir_y, dir_z),
            'alpha_deg': math.degrees(self.alpha),
            'beta_deg': math.degrees(self.beta),
        })

        print(f"\n{'*' * 55}")
        print(f"[YAKINSAMA] Geçiş noktası A = "
              f"({x_aug:.1f}, {y_aug:.1f}, {z_aug:.1f})m")
        print(f"[YAKINSAMA] Hedef yön vektörü = "
              f"({dir_x:.3f}, {dir_y:.3f}, {dir_z:.3f})")
        print(f"[YAKINSAMA] Merkez {step_m}m kaydırıldı → "
              f"({self.center_x:.1f}, {self.center_y:.1f}, {self.center_z:.1f})m")
        print(f"[YAKINSAMA] Toplam {len(self.direction_lines)} doğru birikiyor.")
        print(f"{'*' * 55}\n")

        # PID sıfırla, yeni iterasyona başla
        self.pid_alpha.reset()
        self.pid_beta.reset()
        self.convergence_count = 0
        self.state = self.STATE_ORBIT

    # ─────────────────────────────────────────────
    # ANA DÖNGÜ
    # ─────────────────────────────────────────────
    def run(self):
        print("\n[BİLGİ] Küre Formasyonu Döngüsü Başlıyor. Ctrl+C ile durdurun.")
        dt = 1.0 / COMMAND_LOOP_HZ  # 5 saniye

        while True:
            loop_start = time.monotonic()

            rssi = self.get_windowed_rssi()

            print("\n" + "=" * 55)
            print(f"[DURUM] {self.state} | Merkez: "
                  f"E:{self.center_x:.1f}m N:{self.center_y:.1f}m "
                  f"ALT:{self.center_z:.1f}m")

            # 1) Açı güncelleme (PID)
            e1, e2 = self.update_rotation_angles(rssi, dt)

            # 2) Drone pozisyonlarını hesapla ve gönder
            self.send_mavlink_commands()

            # 3) Yakınsama kontrolü
            if self.convergence_count >= 3:
                print("[DURUM] → CONVERGED! Merkez kaydırılıyor...")
                self.state = self.STATE_CONVERGED
                self.shift_center_toward_target()

            print("=" * 55 + "\n")

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    def shutdown(self):
        self._lora_stop = True
        for link in self.drone_links.values():
            link.stop()
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass

        # Son rapor
        if self.direction_lines:
            print("\n" + "=" * 55)
            print("[RAPOR] Hesaplanan hedef doğruları:")
            for i, dl in enumerate(self.direction_lines, 1):
                c = dl['center']
                a = dl['augmented']
                d = dl['direction']
                print(f"  #{i}: C=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) "
                      f"A=({a[0]:.1f},{a[1]:.1f},{a[2]:.1f}) "
                      f"Yön=({d[0]:.3f},{d[1]:.3f},{d[2]:.3f}) "
                      f"α={dl['alpha_deg']:.1f}° β={dl['beta_deg']:.1f}°")
            print("=" * 55)
