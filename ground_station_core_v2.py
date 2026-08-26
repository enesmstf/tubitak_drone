"""
ground_station_core.py

Bu dosya YER İSTASYONU mantığının TAMAMINI içerir (LoRa dinleme, checksum
doğrulama, RSSI/geofence/formasyon matematiği, MAVLink komut gönderimi).

Bu dosyayı DOĞRUDAN ÇALIŞTIRMAYIN. Bunun yerine işletim sisteminize göre:
  - Windows'ta:  ground_station_windows.py
  - Linux'ta:    ground_station_linux.py
dosyalarını çalıştırın. İkisi de bu dosyayı import eder, sadece port
isimlerini (COM5 vs /dev/ttyUSB0) farklı verir. Mantık tek yerde olduğu
için hangi bilgisayarda stabil çalıştığını test edip onu kullanabilirsiniz;
iki platform arasında kod davranışı asla farklılaşmaz.
"""

import time
import random
import sys
import threading
import math

try:
    import serial
except ImportError:
    print("pyserial kütüphanesi eksik. Lütfen 'pip install pyserial' komutunu çalıştırın.")
    sys.exit(1)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


# ==========================================
# AYARLAR (CONFIG) - Platformdan bağımsız sabitler
# ==========================================
FORMATION_DISTANCE_M = 5.0
FORMATION_ALT_M = 15.0

# Geofence: sanal merkezin origin'den sapabileceği maksimum mesafe (metre)
MAX_CENTER_DRIFT_M = 20.0

# --- DÜZELTME 1: RSSI verisinin "taze" sayılacağı maksimum yaş ---
# Check1.py'de CYCLE_DURATION = 4.0s (her drone 4 saniyede bir sıra alıyor).
# RSSI_MAX_AGE_S'i cycle süresine yapışık (4.0) tutmak, RF ortamındaki normal
# %20-25 paket kaybında (gecikme, saat kayması, LoRa retry) anlık olarak
# "merkez sabit tutuluyor" durumuna düşürür. En az 2 tam cycle'lık tolerans
# bırakmak (8.0s) bunu önler; drone havada ani duraksama yaşamaz.
RSSI_MAX_AGE_S = 8.0

# --- DÜZELTME 2: Origin'in Drone 1'in KENDİ konumu mu, yoksa sürü merkezi mi olduğu ---
# Drone 1 formasyonda KUZEY nöbetçisi konumunda duruyor. Eğer sahada droneları
# yerde bir "+" şeklinde dizip birlikte kaldırıyorsanız, Drone 1'in ham GPS'i
# gerçek sürü merkezi DEĞİLDİR - vericinin/hedefin olduğu nokta değil, Drone
# 1'in kendi durduğu noktadır. Bu flag True olduğunda, origin otomatik olarak
# Drone 1'in konumundan FORMATION_DISTANCE_M kadar GÜNEYE kaydırılıp gerçek
# sürü merkezi olarak kaydedilir.
#
#   True  -> Drone 1 "+" formasyonunun Kuzey ucunda yerde duruyor, origin
#            ondan 5m güneye kaydırılarak hesaplanır (ÖNERİLEN, tipik saha
#            kurulumu için doğru olan budur).
#   False -> Drone 1'in GPS'i doğrudan sürü merkezi olarak kabul edilir
#            (droneları zaten merkez etrafında ayrı ayrı konumlandırıp
#            kaldırıyorsanız kullanın).
ADJUST_ORIGIN_FOR_DRONE1_POSITION = True

RSSI_MAX_AGE_S = max(RSSI_MAX_AGE_S, 6.0)  # güvenlik alt sınırı

COMMAND_LOOP_HZ = 1.0

# "Havada" sezgisel eşiği (metre). NOT: gerçek sahada ArduPilot'un
# EXTENDED_SYS_STATE / LANDED_STATE mesajıyla değiştirilmesi önerilir.
AIRBORNE_RELATIVE_ALT_THRESHOLD_M = 2.0

EARTH_RADIUS_M = 6378137.0

LORA_BAUD = 9600
PIXHAWK_BAUD = 57600

LORA_RECONNECT_DELAY_S = 2.0
MAVLINK_HEARTBEAT_TIMEOUT_S = 10.0
MAVLINK_RECONNECT_DELAY_S = 3.0

# --- DÜZELTME 3 (hatırlatma, kod değil) ---
# Bilgisayar çöker veya telemetri anteni sökülürse bu script komut
# göndermeyi durdurur. Bu DURUMUN kendisini koddan çözemeyiz - ArduPilot
# tarafında Mission Planner üzerinden GCS Failsafe AYARLANMALIDIR:
#   FS_GCS_ENABLE  = 1 (veya kullandığınız firmware'in karşılığı)
#   FS_GCS_TIMEOUT = 5 (saniye - 5s komut/heartbeat gelmezse tetiklenir)
#   Aksiyon        = RTL veya LAND (droneların havada asılı kalmaması için)
# Bu script bu parametreleri OTOMATİK YAZMAZ; bilinçli olarak sizin
# Mission Planner'dan elle doğrulamanız gerekir.


def compute_checksum(payload: str) -> int:
    """Node (Check1.py) tarafındaki ile birebir aynı algoritma."""
    return sum(ord(c) for c in payload) % 256


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
        # İmza: (time_boot_ms, target_system, target_component, frame, type_mask,
        #        lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate)
        # -> lat_int index 5, lon_int index 6, alt index 7
        lat_int = args[5]
        lon_int = args[6]
        alt = args[7]
        print(f"[SİMÜLASYON-MAVLink] Drone {self.target_id} -> "
              f"LAT: {lat_int/1e7:.6f}, LON: {lon_int/1e7:.6f}, ALT: {alt:.1f}m")


class DroneLink:
    """Bir drone'un MAVLink bağlantısı + arka planda güncellenen son bilinen durumu."""
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
                    blocking=True,
                    timeout=1.0
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
                print(f"[UYARI MAVLINK] Drone {self.drone_id} okuma hatası: {e}")
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
                return False, "havada değil (irtifa eşiği altında)"
            return True, "hazır"

    def stop(self):
        self._stop = True


def offset_latlon(base_lat, base_lon, north_m, east_m):
    """Küçük mesafeler için düzlem (equirectangular) yaklaşımıyla lat/lon ofseti."""
    dlat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    dlon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(base_lat)))) * (180.0 / math.pi)
    return base_lat + dlat, base_lon + dlon


# ==========================================
# YER İSTASYONU ANA SINIFI (BEYİN)
# ==========================================
class GroundStation:
    def __init__(self, lora_port, pixhawk_ports, simulation, platform_label=""):
        self.lora_port = lora_port
        self.pixhawk_ports = pixhawk_ports
        self.simulation = simulation
        self.platform_label = platform_label

        self.lora = None
        self.drone_links = {}

        self.rssi_lock = threading.Lock()
        self.rssi_data = {}

        self.origin_lat = None
        self.origin_lon = None

        self.virtual_center_x = 0.0  # Doğu (East)
        self.virtual_center_y = 0.0  # Kuzey (North)

        self._lora_stop = False

        print("=" * 40)
        print("  🚁 OTONOM SÜRÜ YER İSTASYONU 🚁")
        if platform_label:
            print(f"  Platform: {platform_label}")
        print("=" * 40)
        print(f"Mod: {'SİMÜLASYON (TEST)' if simulation else 'GERÇEK DONANIM UÇUŞU'}")
        print(f"RSSI_MAX_AGE_S={RSSI_MAX_AGE_S}s | "
              f"ADJUST_ORIGIN_FOR_DRONE1_POSITION={ADJUST_ORIGIN_FOR_DRONE1_POSITION} | "
              f"Geofence=±{MAX_CENTER_DRIFT_M}m")

        self.connect_hardware()
        self.start_lora_listener()
        self.capture_origin()

    # -------------------------------------------------
    # BAĞLANTI KURULUMU
    # -------------------------------------------------
    def connect_hardware(self):
        if self.simulation:
            print("[SİSTEM] Simülasyon modunda USB/COM portları aranmıyor.")
            for i in range(1, 5):
                mock_conn = MockMavlinkConnection(i)
                self.drone_links[i] = DroneLink(i, mock_conn, simulation=True)
            return

        print(f"[SİSTEM] Gerçek LoRa portuna bağlanılıyor: {self.lora_port}")
        while self.lora is None:
            try:
                self.lora = serial.Serial(self.lora_port, LORA_BAUD, timeout=0.1)
                print(f"[BAŞARILI] LoRa Bağlandı: {self.lora_port}")
            except Exception as e:
                print(f"[HATA KABLO] LoRa portu açılamadı: {e}. "
                      f"{LORA_RECONNECT_DELAY_S}s sonra tekrar denenecek...")
                time.sleep(LORA_RECONNECT_DELAY_S)

        print("[SİSTEM] Pixhawk Telemetri portlarına bağlanılıyor...")
        if mavutil is None:
            print("[HATA YAZILIM] Gerçek mod için 'pymavlink' kütüphanesi şart!")
            sys.exit(1)

        for drone_id, port in self.pixhawk_ports.items():
            connection = None
            while connection is None:
                try:
                    connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
                    print(f"[BAĞLANTI] Drone {drone_id} portu açıldı: {port}. Heartbeat bekleniyor...")
                    connection.wait_heartbeat(timeout=MAVLINK_HEARTBEAT_TIMEOUT_S)
                    print(f"[BAŞARILI] Drone {drone_id} Heartbeat alındı "
                          f"(sysid={connection.target_system}, compid={connection.target_component})")
                except Exception as e:
                    print(f"[UYARI KABLO] Drone {drone_id} telemetrisi açılamadı: {e}. "
                          f"{MAVLINK_RECONNECT_DELAY_S}s sonra tekrar denenecek...")
                    connection = None
                    time.sleep(MAVLINK_RECONNECT_DELAY_S)

            self.drone_links[drone_id] = DroneLink(drone_id, connection, simulation=False)

    # -------------------------------------------------
    # ORIGIN (ANA MERKEZ) YAKALAMA
    # -------------------------------------------------
    def capture_origin(self):
        print("[SİSTEM] Origin için Drone 1'in GPS konumu bekleniyor...")
        origin_link = self.drone_links.get(1)

        deadline = time.time() + 30.0
        while time.time() < deadline:
            with origin_link.lock:
                raw_lat, raw_lon = origin_link.lat, origin_link.lon
            if raw_lat is not None and raw_lon is not None:
                if ADJUST_ORIGIN_FOR_DRONE1_POSITION:
                    # Drone 1, "+" formasyonunun Kuzey ucunda duruyor; gerçek
                    # sürü merkezi ondan FORMATION_DISTANCE_M kadar güneyde.
                    self.origin_lat, self.origin_lon = offset_latlon(
                        raw_lat, raw_lon, -FORMATION_DISTANCE_M, 0.0
                    )
                    print(f"[BAŞARILI] Drone 1 ham GPS: LAT {raw_lat:.6f}, LON {raw_lon:.6f}")
                    print(f"[BAŞARILI] Origin (Drone1'den {FORMATION_DISTANCE_M}m güneye kaydırıldı) -> "
                          f"LAT: {self.origin_lat:.6f}, LON: {self.origin_lon:.6f}")
                else:
                    self.origin_lat, self.origin_lon = raw_lat, raw_lon
                    print(f"[BAŞARILI] Origin (Drone1'in konumu doğrudan kullanıldı) -> "
                          f"LAT: {raw_lat:.6f}, LON: {raw_lon:.6f}")
                return
            time.sleep(0.5)

        print("[UYARI] 30 saniye içinde Drone 1'den GPS konumu alınamadı. "
              "Origin ayarlanana kadar komut gönderimi yapılmayacak.")

    # -------------------------------------------------
    # LORA DİNLEYİCİ (PASİF)
    # -------------------------------------------------
    def start_lora_listener(self):
        thread = threading.Thread(target=self._lora_listener_loop, daemon=True)
        thread.start()

    def _lora_listener_loop(self):
        print("[SİSTEM] LoRa pasif dinleyici başlatıldı (TDMA broadcast bekleniyor).")
        rx_buffer = ""

        while not self._lora_stop:
            try:
                if self.simulation:
                    if not hasattr(self, "_mock_serial"):
                        self._mock_serial = MockSerial()
                    data = self._mock_serial.read(256)
                else:
                    if self.lora is None or not self.lora.is_open:
                        self._reconnect_lora()
                        continue
                    data = self.lora.read(256)

                if data:
                    rx_buffer += data.decode(errors="ignore")
                    while "\n" in rx_buffer:
                        line, rx_buffer = rx_buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._handle_line(line)

                time.sleep(0.01)

            except Exception as e:
                print(f"[HATA RADYO] LoRa dinleme hatası: {e}")
                if not self.simulation:
                    self._reconnect_lora()
                time.sleep(0.5)

    def _reconnect_lora(self):
        print("[SİSTEM] LoRa yeniden bağlanıyor...")
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass
        self.lora = None

        while self.lora is None and not self._lora_stop:
            try:
                self.lora = serial.Serial(self.lora_port, LORA_BAUD, timeout=0.1)
                print(f"[BAŞARILI] LoRa yeniden bağlandı: {self.lora_port}")
            except Exception as e:
                print(f"[HATA KABLO] LoRa yeniden bağlanamadı: {e}. Tekrar denenecek...")
                time.sleep(LORA_RECONNECT_DELAY_S)

    def _handle_line(self, line):
        """
        Beklenen format: N{id},{rssi},{checksum}

        RealTerm benzeri canlı izleme: RealTerm'i ayrıca açmanıza gerek
        kalmasın diye HER ham satır (geçerli ya da geçersiz) burada
        konsola basılıyor - COM/tty portunu tek bir program tuttuğu için
        RealTerm aynı anda açılamıyordu, bu satır onun yerini alıyor.
        """
        print(f"[RADYO-MONITOR] {line}")

        try:
            if not line.startswith("N"):
                return
            parts = line[1:].split(",")
            
            drone_id = int(parts[0])
            
            # --- ÖZEL İSTİSNA (BYPASS): DRONE 3 ---
            # Drone 3'e HDMI arızası yüzünden yeni kod atılamadığı için, 
            # onun şifresiz (2 parçalı) verisini kabul ediyoruz.
            if drone_id == 3 and len(parts) == 2:
                rssi = int(parts[1])
                with self.lock:
                    self.latest_rssi[drone_id] = (time.time(), rssi)
                return
            # --------------------------------------
            
            if len(parts) != 3:
                return

            drone_id = int(parts[0])
            rssi = int(parts[1])
            received_checksum = int(parts[2])

            body = f"N{drone_id},{rssi}"
            expected_checksum = compute_checksum(body)

            if received_checksum != expected_checksum:
                print(f"[UYARI GÜVENLİK] Checksum uyuşmadı, paket reddedildi: '{line}'")
                return

            if drone_id not in self.pixhawk_ports and not self.simulation:
                print(f"[UYARI] Tanınmayan drone ID'sinden paket, reddedildi: '{line}'")
                return

            with self.rssi_lock:
                if drone_id in self.rssi_data:
                    old_rssi, _ = self.rssi_data[drone_id]
                    # EMA Filtresi: %40 Yeni Veri, %60 Eski Veri (Pürüzsüzleştirme)
                    smoothed_rssi = (0.4 * rssi) + (0.6 * old_rssi)
                else:
                    smoothed_rssi = rssi
                self.rssi_data[drone_id] = (smoothed_rssi, time.time())

        except (ValueError, IndexError):
            print(f"[UYARI RADYO] Format hatalı paket, reddedildi: '{line}'")

    def get_fresh_rssi_snapshot(self):
        now = time.time()
        snapshot = {}
        with self.rssi_lock:
            for drone_id, (rssi, ts) in self.rssi_data.items():
                if now - ts <= RSSI_MAX_AGE_S:
                    snapshot[drone_id] = rssi
        return snapshot

    # -------------------------------------------------
    # KARAR MOTORU
    # -------------------------------------------------
    def calculate_virtual_center_shift(self, rssi_data):
        if len(rssi_data) < 4:
            print(f"[UYARI] Tüm dronelardan güncel veri alınamadı ({len(rssi_data)}/4, "
                  f"tazelik sınırı: {RSSI_MAX_AGE_S}s). Merkez son bilinen konumda sabit tutuluyor.")
            return 0.0, 0.0

        delta_y_dbm = rssi_data[1] - rssi_data[2]   # Kuzey - Güney
        delta_x_dbm = rssi_data[3] - rssi_data[4]   # Doğu - Batı

        K_GAIN = 0.5
        shift_y = max(min(delta_y_dbm * K_GAIN, 5.0), -5.0)
        shift_x = max(min(delta_x_dbm * K_GAIN, 5.0), -5.0)

        print(f"[KARAR MOTORU] Sinyal Farkları -> K/G: {delta_y_dbm}dB, D/B: {delta_x_dbm}dB")
        print(f"[KARAR MOTORU] Adım Kayması -> KUZEYE: {shift_y:.1f}m, DOĞUYA: {shift_x:.1f}m")

        return shift_x, shift_y

    # -------------------------------------------------
    # MAVLINK KOMUT GÖNDERİMİ
    # -------------------------------------------------
    def send_mavlink_commands(self, shift_x, shift_y):
        if self.origin_lat is None or self.origin_lon is None:
            print("[UYARI] Origin henüz ayarlanmadı, komut gönderilmiyor.")
            return

        self.virtual_center_x += shift_x
        self.virtual_center_y += shift_y

        self.virtual_center_x = max(min(self.virtual_center_x, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
        self.virtual_center_y = max(min(self.virtual_center_y, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)

        print(f"[KARAR MOTORU] Sanal Merkez (Origin'e göre) -> "
              f"KUZEY: {self.virtual_center_y:.1f}m, DOĞU: {self.virtual_center_x:.1f}m "
              f"(Geofence: ±{MAX_CENTER_DRIFT_M}m)")

        offsets = {
            1: (self.virtual_center_y + FORMATION_DISTANCE_M, self.virtual_center_x),
            2: (self.virtual_center_y - FORMATION_DISTANCE_M, self.virtual_center_x),
            3: (self.virtual_center_y, self.virtual_center_x + FORMATION_DISTANCE_M),
            4: (self.virtual_center_y, self.virtual_center_x - FORMATION_DISTANCE_M),
        }

        for drone_id, (north_m, east_m) in offsets.items():
            link = self.drone_links.get(drone_id)
            if link is None:
                continue

            ready, reason = link.is_ready_for_commands()
            if not ready and not self.simulation:
                print(f"[GÜVENLİK] Drone {drone_id} komuta hazır değil ({reason}), komut ATLANIYOR.")
                continue

            target_lat, target_lon = offset_latlon(self.origin_lat, self.origin_lon, north_m, east_m)

            connection = link.connection
            if hasattr(connection, "mav"):
                connection.mav.set_position_target_global_int_send(
                    0,
                    connection.target_system, connection.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,
                    int(target_lat * 1e7),
                    int(target_lon * 1e7),
                    FORMATION_ALT_M,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )
                print(f"[MAVLINK] Drone {drone_id} -> LAT: {target_lat:.6f}, LON: {target_lon:.6f}, "
                      f"ALT: {FORMATION_ALT_M}m")
            else:
                connection.set_position_target_global_int_send(
                    0, 0, 0, 6, 0,
                    int(target_lat * 1e7), int(target_lon * 1e7), FORMATION_ALT_M,
                    0, 0, 0, 0, 0, 0, 0, 0
                )

    # -------------------------------------------------
    # ANA DÖNGÜ
    # -------------------------------------------------
    def run(self):
        print("\n[BİLGİ] Görev Döngüsü Başlıyor (Pasif Dinleme + 1Hz Komut). Durdurmak için Ctrl+C.")
        loop_period = 1.0 / COMMAND_LOOP_HZ

        while True:
            loop_start = time.monotonic()

            rssi_snapshot = self.get_fresh_rssi_snapshot()

            print("\n" + "=" * 55)
            shift_x, shift_y = self.calculate_virtual_center_shift(rssi_snapshot)
            self.send_mavlink_commands(shift_x, shift_y)
            print("=" * 55 + "\n")

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, loop_period - elapsed))

    def shutdown(self):
        self._lora_stop = True
        for link in self.drone_links.values():
            link.stop()
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass
