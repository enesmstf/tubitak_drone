"""
ground_station_core.py

Bu dosya YER İSTASYONU mantığının TAMAMINI içerir (LoRa dinleme, Kalman
filtresi, RSSI/geofence/formasyon matematiği, MAVLink komut gönderimi).

Bu dosyayı DOĞRUDAN ÇALIŞTIRMAYIN. Bunun yerine işletim sisteminize göre:
  - Windows'ta:  ground_station_windows.py
  - Linux'ta:    ground_station_linux.py
dosyalarını çalıştırın. İkisi de bu dosyayı import eder, sadece port
isimlerini (COM5 vs /dev/ttyUSB0) farklı verir.

==========================================================================
BU SÜRÜMDE UYGULANAN YÖNTEM (TÜBİTAK 123E294 gelişme raporlarına göre)
==========================================================================
Proje raporlarında (İP-2, "Hedef takip için teorik filtreleme") tanımlanan
yöntem iki parçadan oluşuyor - bu dosya artık ikisini de birebir uyguluyor:

1) KALMAN FİLTRESİ (rapordaki adımlarla birebir aynı): her drone'un ham
   RSSI'sı, tek durumlu (skaler) bir Kalman filtresinden geçirilir:
     - Tahmin:        X(k+1|k) = X(k|k)              [F = I]
                       P(k+1|k) = P(k|k) + Q
     - Kalman Kazancı: W(k+1) = P(k+1|k) / (P(k+1|k) + R)
     - Güncelleme:     X(k+1|k+1) = X(k+1|k) + W(k+1)*(Z - X(k+1|k))
                       P(k+1|k+1) = (1 - W(k+1)) * P(k+1|k)
   Bu, önceki sürümdeki keyfi "%40 yeni / %60 eski" sabit ağırlıklı
   yumuşatmanın yerini alır - rapor bunun yerine istatistiksel olarak
   OPTIMAL olan Kalman filtresini öneriyor (bkz. Rapor 2, "KALMAN
   FİLTRESİ KULLANILARAK KONUM TESPİTİ").

2) BİÇİMLENDİRME (FORMASYON) KONTROLÜ - Güzey vd. (2022)'nin geometrik
   RSSI yöntemi: '+' formasyonundaki 4 İHA, doğu-batı ve kuzey-güney
   gruplarının Kalman-filtrelenmiş güç farkları SIFIR olana kadar
   SÜRÜLÜR (yani hata birikimli/integral bir kontrol sinyaliyle
   sürülür - anlık bir hedefe "yakınsamak" değil). Bu, önceki
   düzeltmemde (EMA ile anlık hedefe yakınsama) yanlışlıkla değiştirdiğim
   ve şimdi GERİ ALDIĞIM kısımdır: raporun tarif ettiği kontrol yasası
   gerçekten de bir biriktiricidir (integrator) - İHA'lar fiziksel
   olarak hedefe yaklaştıkça güç farkı gerçekten sıfıra iner ve sistem
   durur. PERVANESİZ MASAÜSTÜ TESTİNDE İHA'lar fiziksel olarak
   hareket ETMEDİĞİ için hata hiçbir zaman sıfırlanmaz - bu yüzden sanal
   merkezin ±20m geofence sınırına dayanıp orada durması/titremesi bu
   test koşulunda BEKLENEN bir davranıştır, yazılım hatası değildir.
   Gerçek uçuşta İHA'lar fiziksel olarak yaklaştıkça sistem doğal olarak
   yakınsayıp duracaktır.
==========================================================================

v2 DEĞİŞİKLİKLERİ (sphere_sim/scenario1_sim.py ve E8 deneyi ile doğrulandı):
  1. CALIBRATION_OFFSET_DB, kabul edilen saha kalibrasyonuna güncellendi
     (kalibrasyon_sonucu_saha.json: D1 0.00, D2 -1.32, D3 +2.38, D4 +1.10 dB).
     v1'deki {-2.1, +1.9, -0.1, +0.2} değerleri eski, elle girilmiş değerlerdi.
  2. RSSI filtresi ve eksen kontrolcüsü AYRI anahtarlarla seçilir:
     SIGNAL_FILTER ("KALMAN" | "SISMF" | "KALMAN+SISMF" | "HAM") ve
     AXIS_CONTROLLER ("P" | "SISMF"). v1'de USE_SISMF hem filtreyi hem
     kontrolcüyü birlikte değiştiriyordu, etkileri ayrılamıyordu.
  3. Varsayılan: KALMAN + P (K=0.5). Simülasyonda (40 Monte Carlo, saha gürültüsü)
     medyan son hata P ile 6.4 m / 10.6 m, eksen-SISMF ile 7.0 m / 15.9 m; uçulan
     yol / düz mesafe P ile 2.4 / 4.9, eksen-SISMF ile 6.4 / 9.9 (zikzak, batarya).
     Eksen-SISMF hedefe daha hızlı ulaşır (medyan 86 s'ye karşı 180 s) ama
     hedef üstünde 4-8 m salınır. CSV kaydı yine 4 filtreyi birden yazar.
  4. "[KARAR MOTORU]" log satırındaki koşul/öncelik hatası düzeltildi.
"""

import time
import random
import sys
import threading
import math
import socket
import json
import csv
from datetime import datetime

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
MAX_CENTER_DRIFT_M = 50.0

# RSSI verisinin "taze" sayılacağı maksimum yaş
RSSI_MAX_AGE_S = 30.0
RSSI_MAX_AGE_S = max(RSSI_MAX_AGE_S, 6.0)  # güvenlik alt sınırı

# Origin'in Drone 1'in KENDİ konumu mu, yoksa sürü merkezi mi olduğu
ADJUST_ORIGIN_FOR_DRONE1_POSITION = True

COMMAND_LOOP_HZ = 0.2  # 1/0.2 = 5 saniyede bir döngü (okunması kolay olsun diye yavaşlatıldı)

# ==========================================
# FİLTRE VE KONTROL MODU SEÇİMİ
# ==========================================
# NOT: Deney verisi (CSV) toplanırken bu ayarlara dokunmanıza GEREK YOKTUR!
#      Kod her pakette 4 senaryoyu da (Ham, Kalman, SISMF, Kalman+SISMF)
#      aynı anda hesaplayıp kaydeder.
#
# Bu ayarlar sadece İHA'ların fiziki formasyon sürüşünde hangisini baz alacağını belirler:
# 1) USE_KALMAN = True  ve USE_SISMF = True  -> ZİNCİR COMBO (Kalman + SISMF)
# 2) USE_KALMAN = False ve USE_SISMF = True  -> Sadece SISMF (SCI_4 2026) [ÖNERİLEN]
# 3) USE_KALMAN = True  ve USE_SISMF = False -> Sadece Kalman (Rapor 2)
# 4) Her ikisi False                         -> HAM (Filtresiz ham RSSI)
# v2: filtre ve kontrolcü ayrı seçilir (aşağıdaki eski USE_* bayrakları bunlardan türetilir)
SIGNAL_FILTER   = "KALMAN"   # "KALMAN" | "SISMF" | "KALMAN+SISMF" | "HAM"
AXIS_CONTROLLER = "P"        # "P" (Rapor 2, K=0.5 m/dB) | "SISMF" (IJSS 2026 eksen kontrolcüsü)
USE_KALMAN = SIGNAL_FILTER in ("KALMAN", "KALMAN+SISMF")
USE_SISMF  = SIGNAL_FILTER in ("SISMF", "KALMAN+SISMF")

# --- KALMAN PARAMETRELERİ (Rapor 2) ---
KALMAN_Q = 1.0
KALMAN_R = 20.0

# --- S-ISMF PARAMETRELERİ (N. Güzey, SCI_4.pdf, Mart 2026) ---
# s(t) = e(t) + λ · ∫ e(τ) dτ
# ẋ(t) = k · tanh(s(t) / φ) + κ · ∫ e(τ) dτ
# Ayrık LoRa telemetri frekansı (dt ≈ 1-2 sn) için optimize edilmiş değerler:
SISMF_LAMBDA = 0.02   # λ: Kayan yüzey integral kazancı
SISMF_K      = 0.3    # k: Kayan yüzey sign/tanh hız kazancı (dB/s)
SISMF_KAPPA  = 0.008  # κ: Düzeltici integral kazancı
SISMF_PHI    = 2.5    # φ: Sınır katmanı kalınlığı (chattering sönümleyici)
SISMF_INTEGRAL_MAX = 5.0 # Windup koruması (aşırı birikimi engelleme sınırı)

# ==========================================
# DRONE KALİBRASYON OFSETLERİ (donanım dengesizliğini düzeltir)
# ==========================================
# v2: kalibrasyon_sonucu_saha.json (P_ref = -78.72 dBm, n = 1.264, RMSE = 2.29 dB)
CALIBRATION_OFFSET_DB = {
    1:  0.00,   # D1 Kuzey (referans)
    2: -1.32,   # D2 Güney
    3:  2.38,   # D3 Doğu
    4:  1.10,   # D4 Batı
}

# ==========================================
# TEST MODU AYARI (PERVANESİZ MASAÜSTÜ TEST İÇİN)
# ==========================================
BENCH_TEST_MODE = False

if BENCH_TEST_MODE:
    AIRBORNE_RELATIVE_ALT_THRESHOLD_M = -10.0
else:
    AIRBORNE_RELATIVE_ALT_THRESHOLD_M = 2.0

EARTH_RADIUS_M = 6378137.0

LORA_BAUD = 9600
PIXHAWK_BAUD = 57600

LORA_RECONNECT_DELAY_S = 2.0
MAVLINK_HEARTBEAT_TIMEOUT_S = 10.0
MAVLINK_RECONNECT_DELAY_S = 3.0

# --- Hatırlatma (kod değil) ---
# Bilgisayar çöker veya telemetri anteni sökülürse bu script komut
# göndermeyi durdurur. ArduPilot tarafında Mission Planner üzerinden GCS
# Failsafe AYARLANMALIDIR (FS_GCS_ENABLE=1, FS_GCS_TIMEOUT=5, Aksiyon=RTL).
# Bu script bu parametreleri OTOMATİK YAZMAZ.


def compute_checksum(payload: str) -> int:
    """Node (Check1.py) tarafındaki ile birebir aynı algoritma."""
    return sum(ord(c) for c in payload) % 256


class ScalarKalmanFilter:
    """
    Tek durumlu (skaler) Kalman filtresi - Rapor 2'de tarif edilen
    adımların birebir uygulaması. Durum = filtrelenmiş RSSI (dBm).

    F = 1 (durum geçiş matrisi, sabit sinyal varsayımı: "mevcut RSSI
          değeri bir sonraki adımın da en iyi tahminidir")
    H = 1 (ölçüm matrisi, doğrudan RSSI'yı ölçüyoruz)
    """
    def __init__(self, initial_value, q=KALMAN_Q, r=KALMAN_R):
        self.x = float(initial_value)   # X(k|k) - durum tahmini
        self.p = float(r)               # P(k|k) - başlangıç belirsizliği
        self.q = float(q)
        self.r = float(r)

    def update(self, measurement):
        # 1) Tahmin (Prediction)
        x_pred = self.x                  # X(k+1|k) = X(k|k)  [F=I]
        p_pred = self.p + self.q         # P(k+1|k) = P(k|k) + Q

        # 2) Kalman Kazancı (Measurement Weighting)
        w = p_pred / (p_pred + self.r)   # W(k+1) = P(k+1|k) / (P(k+1|k)+R)  [H=1]

        # 3) Güncelleme (Update / Innovation)
        self.x = x_pred + w * (measurement - x_pred)

        # 4) Kovaryans Güncelleme
        self.p = (1.0 - w) * p_pred

        return self.x


# ==========================================
# SCI_4 (MART 2026) SMOOTHER INTEGRAL SLIDING MODE FILTER (SISMF)
# ==========================================
class SmootherIntegralSlidingModeFilter2026:
    """
    N. Güzey, "A smoother integral sliding mode filter approach for
    distributed moving target tracking", International Journal of Systems
    Science, March 2026 (SCI_4.pdf).

    Skaler RSSI Sinyal Filtresi (Durum Gözlemcisi):
      e(t) = measurement - state
      I_e(t) = ∫ e(τ) dτ
      s(t) = e(t) + λ · I_e(t)               (Denklem 3 & 5)
      ẋ(t) = k · tanh(s(t) / φ) + κ · I_e(t) (Denklem 4 & 6)
      state(t + dt) = state(t) + ẋ(t) · dt
    """
    def __init__(self, initial_value, lambda_param=SISMF_LAMBDA, k=SISMF_K,
                 kappa=SISMF_KAPPA, phi=SISMF_PHI, integral_max=SISMF_INTEGRAL_MAX):
        self.state = float(initial_value)
        self.lambda_param = float(lambda_param)
        self.k = float(k)
        self.kappa = float(kappa)
        self.phi = float(phi)
        self.integral_max = float(integral_max)
        self.error_integral = 0.0
        self.last_time = None

    def update(self, measurement, current_time=None):
        if current_time is None:
            current_time = time.time()

        if self.last_time is None:
            dt = 1.0
        else:
            dt = max(min(current_time - self.last_time, 5.0), 0.05)
        self.last_time = current_time

        error = float(measurement) - self.state
        self.error_integral += error * dt
        self.error_integral = max(min(self.error_integral, self.integral_max), -self.integral_max)

        # Kayan yüzey (SCI_4 Denklem 3 & 5)
        s = error + self.lambda_param * self.error_integral

        # Yumuşatılmış sign (Chattering önleyici)
        soft_sign = math.tanh(s / self.phi)

        # Türev ve durum güncellemesi (SCI_4 Denklem 4 & 6)
        state_dot = (self.k * soft_sign) + (self.kappa * self.error_integral)
        self.state += state_dot * dt
        return self.state

    def reset(self):
        self.error_integral = 0.0
        self.last_time = None


class AxisSISMFController:
    """
    SCI_4 (2026) Makalesindeki Denklem 3-6'nın Artı Formasyonu Eksenel
    (Kuzey-Güney ve Doğu-Batı) Diferansiyel Karar Motoruna Uygulanması.

    Hata sinyali: e = delta_dbm (örn: Kuzey - Güney)
    Kayan yüzey : s = e + λ · ∫ e dτ
    Hız komutu  : v = k · tanh(s / φ) + κ · ∫ e dτ
    Adım (m)    : shift = clip(v * dt, -5.0, 5.0)
    """
    def __init__(self, lambda_param=SISMF_LAMBDA, k=0.5, kappa=0.02,
                 phi=SISMF_PHI, integral_max=SISMF_INTEGRAL_MAX):
        self.lambda_param = float(lambda_param)
        self.k = float(k)           # Kayan yüzey bileşeni
        self.kappa = float(kappa)   # İntegral bileşeni
        self.phi = float(phi)
        self.integral_max = float(integral_max)
        self.error_integral = 0.0

    def update_shift(self, error, dt=5.0):
        if error is None:
            return 0.0
        
        self.error_integral += error * dt
        self.error_integral = max(min(self.error_integral, self.integral_max), -self.integral_max)

        # Kayan yüzey (SCI_4 Denklem 3 & 5)
        s = error + self.lambda_param * self.error_integral

        # Yumuşatılmış sign
        soft_sign = math.tanh(s / self.phi)

        # Hız ve Adım
        velocity = (self.k * soft_sign) + (self.kappa * self.error_integral)
        shift = max(min(velocity * dt, 5.0), -5.0)
        return shift

    def reset(self):
        self.error_integral = 0.0


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
        self.battery_remaining = 100
        self.last_heartbeat_time = 0.0
        self.last_position_time = 0.0

        self._stop = False

        if simulation:
            self.armed = True
            self.flightmode = "GUIDED"
            self.lat = connection.lat
            self.lon = connection.lon
            self.relative_alt_m = connection.relative_alt_m
            self.battery_remaining = 100
            self.last_heartbeat_time = time.time()
            self.last_position_time = time.time()
        else:
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()

    def _reader_loop(self):
        while not self._stop:
            try:
                msg = self.connection.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT", "SYS_STATUS"],
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
                
                elif msg.get_type() == "SYS_STATUS":
                    with self.lock:
                        if msg.battery_remaining != -1:
                            self.battery_remaining = msg.battery_remaining

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
            if self.battery_remaining < 20:
                if self.flightmode != "RTL" and not self.simulation:
                    print(f"[KRİTİK GÜVENLİK] Drone {self.drone_id} bataryası kritik (%{self.battery_remaining}). Lütfen RTL alın!")
                    # GCS üzerinde FS_GCS_ENABLE veya manuel müdahale beklenir.
                return False, f"batarya kritik (%{self.battery_remaining})"
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
        self.rssi_data = {}          # drone_id -> (filtrelenmis_rssi, son_zaman)
        self.window_samples = {}     # drone_id -> [bu 5sn penceresinde alinan filtrelenmis degerler]
        self.kalman_filters = {}     # drone_id -> ScalarKalmanFilter
        self.sismf_filters = {}      # drone_id -> SmootherIntegralSlidingModeFilter2026 (SCI_4)
        self.sismf_combo_filters = {}  # drone_id -> SISMF (Kalman çıktısını girdi olarak alır)

        # Artı formasyonu eksenel SISMF karar kontrolcüleri
        self.axis_sismf_y = AxisSISMFController()
        self.axis_sismf_x = AxisSISMFController()

        self.origin_lat = None
        self.origin_lon = None

        self.virtual_center_x = 0.0  # Doğu (East)
        self.virtual_center_y = 0.0  # Kuzey (North)

        self._lora_stop = False
        self._start_time = time.time()

        # Aktif filtre etiketi
        if USE_SISMF and USE_KALMAN:
            self.active_filter_label = "ZİNCİR COMBO (Kalman + SISMF)"
        elif USE_SISMF:
            self.active_filter_label = "SISMF (SCI_4.pdf 2026)"
        elif USE_KALMAN:
            self.active_filter_label = "Kalman (Rapor 2)"
        else:
            self.active_filter_label = "HAM (Filtresiz)"

        # --- DENEY LOG KAYDI (CSV) ---
        self.csv_filename = f"kalman_deney_log_{int(time.time())}.csv"
        try:
            self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "Timestamp", "Gecen_Sure_Sn", "Drone_ID",
                "Ham_RSSI", "Kalman_RSSI", "SISMF_RSSI", "KalmanSISMF_RSSI",
                "Aktif_Mod", "Sanal_Merkez_Kuzey_M", "Sanal_Merkez_Dogu_M"
            ])
            self.csv_file.flush()
            print(f"[LOG KAYIT] Deney verileri otomatik kaydediliyor -> {self.csv_filename}")
        except Exception as e:
            print(f"[UYARI LOG] CSV dosyası oluşturulamadı: {e}")
            self.csv_file = None
            self.csv_writer = None

        print("=" * 40)
        print("  🚁 OTONOM SÜRÜ YER İSTASYONU 🚁")
        if platform_label:
            print(f"  Platform: {platform_label}")
        print("=" * 40)
        print(f"Mod: {'SİMÜLASYON (TEST)' if simulation else 'GERÇEK DONANIM UÇUŞU'}")
        print(f"Aktif Filtre: {self.active_filter_label} | "
              f"SIGNAL_FILTER={SIGNAL_FILTER} | AXIS_CONTROLLER={AXIS_CONTROLLER}")
        print(f"RSSI_MAX_AGE_S={RSSI_MAX_AGE_S}s | "
              f"ADJUST_ORIGIN_FOR_DRONE1_POSITION={ADJUST_ORIGIN_FOR_DRONE1_POSITION} | "
              f"Geofence=±{MAX_CENTER_DRIFT_M}m")
        print(f"SISMF Parametreleri: λ={SISMF_LAMBDA}, k={SISMF_K}, κ={SISMF_KAPPA}, φ={SISMF_PHI}")
        print(f"Kalman Parametreleri: Q={KALMAN_Q}, R={KALMAN_R}")
        print(f"Kalibrasyon Ofsetleri (dB): {CALIBRATION_OFFSET_DB}")

        if BENCH_TEST_MODE:
            print("\n" + "!" * 60)
            print("!!  DİKKAT: BENCH_TEST_MODE = True                      !!")
            print("!!  İrtifa güvenlik kontrolü DEVRE DIŞI.                !!")
            print("!!  Formasyon kontrolü bir BİRİKTİRİCİDİR (integrator): !!")
            print("!!  İHA'lar fiziksel hareket etmediği için (pervanesiz) !!")
            print("!!  sanal merkez ±20m geofence sınırına dayanıp orada   !!")
            print("!!  durabilir - bu BEKLENEN bir davranıştır, hata       !!")
            print("!!  DEĞİLDİR (bkz. dosya başındaki açıklama).           !!")
            print("!!  GERÇEK UÇUŞTAN ÖNCE BENCH_TEST_MODE'u False YAPIN.  !!")
            print("!" * 60 + "\n")

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

        deadline = time.time() + 300.0
        while time.time() < deadline:
            with origin_link.lock:
                raw_lat, raw_lon = origin_link.lat, origin_link.lon
            if raw_lat is not None and raw_lon is not None:
                if ADJUST_ORIGIN_FOR_DRONE1_POSITION:
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

        print("[UYARI] 5 dakika içinde Drone 1'den GPS konumu alınamadı. "
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
        """
        print(f"[RADYO-MONITOR] {line}")

        try:
            if not line.startswith("N"):
                return
            parts = line[1:].split(",")

            try:
                drone_id = int(parts[0])
            except ValueError:
                return

            if len(parts) != 3:
                return
            
            # Checksum'ı orijinal ham string üzerinden hesapla!
            body = f"N{parts[0]},{parts[1]}"
            received_checksum = int(parts[2])
            expected_checksum = compute_checksum(body)

            if received_checksum != expected_checksum:
                print(f"[UYARI GÜVENLİK] Checksum uyuşmadı, paket reddedildi: '{line}'")
                return

            rssi = float(parts[1])
            drone_id = int(parts[0])

            # ── PTT / SİNYAL KOPMA KORUMASI (v2.1) ──
            # Telsiz mandalına basılmadığında alıcı dip gürültüsünü (-120 dBm) okur.
            # Dronların biri -120 dBm okurken diğeri -85 dBm okursa oluşacak yapay 35 dB'lik
            # farkın otopilotu ve filtreleri sarsmaması için bu paketler elenir.
            MIN_VALID_RSSI_DBM = -115.0
            if rssi <= MIN_VALID_RSSI_DBM:
                return

            if drone_id not in self.pixhawk_ports and not self.simulation:
                print(f"[UYARI] Tanınmayan drone ID'sinden paket, reddedildi: '{line}'")
                return

            with self.rssi_lock:
                # --- KALİBRASYON (donanım yanlılığını düzeltir) ---
                # Checksum doğrulaması yukarıda HAM rssi ile yapıldı, bu
                # yüzden kalibrasyon burada, checksum'dan SONRA uygulanır.
                calibrated_rssi = rssi + CALIBRATION_OFFSET_DB.get(drone_id, 0.0)

                # --- KALMAN FİLTRESİ HESAPLAMASI ---
                if drone_id not in self.kalman_filters:
                    self.kalman_filters[drone_id] = ScalarKalmanFilter(initial_value=calibrated_rssi)
                kalman_val = self.kalman_filters[drone_id].update(calibrated_rssi)

                # --- SCI_4 (2026) SMOOTHER INTEGRAL SLIDING MODE FILTER (SISMF) ---
                # Tek başına SISMF: Ham kalibre edilmiş RSSI'yı giriş olarak alır
                if drone_id not in self.sismf_filters:
                    self.sismf_filters[drone_id] = SmootherIntegralSlidingModeFilter2026(initial_value=calibrated_rssi)
                sismf_val = self.sismf_filters[drone_id].update(calibrated_rssi)

                # --- KALMAN + SISMF ZİNCİR FİLTRE (COMBO) ---
                # Kalman çıktısını SISMF'ye girdi olarak vererek iki filtrenin
                # birlikte ne kadar etki ettiğini ölçeriz
                if drone_id not in self.sismf_combo_filters:
                    self.sismf_combo_filters[drone_id] = SmootherIntegralSlidingModeFilter2026(initial_value=kalman_val)
                combo_val = self.sismf_combo_filters[drone_id].update(kalman_val)

                # Aktif filtre seçimi (formasyon kontrolünde hangisi kullanılacak)
                if USE_SISMF and USE_KALMAN:
                    final_rssi = combo_val
                    active_mode = "KALMAN+SISMF"
                elif USE_SISMF:
                    final_rssi = sismf_val
                    active_mode = "SISMF"
                elif USE_KALMAN:
                    final_rssi = kalman_val
                    active_mode = "KALMAN"
                else:
                    final_rssi = calibrated_rssi
                    active_mode = "HAM"
                # ------------------------------------------------

                self.rssi_data[drone_id] = (final_rssi, time.time())
                self.window_samples.setdefault(drone_id, []).append(final_rssi)

                # --- CSV'YE ANLIK DENEY VERİSİNİ KAYDET ---
                # 4 senaryo tek seferde: Ham, Kalman, SISMF, Kalman+SISMF
                if getattr(self, "csv_writer", None):
                    try:
                        elapsed = time.time() - self._start_time
                        self.csv_writer.writerow([
                            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            f"{elapsed:.2f}",
                            drone_id,
                            f"{calibrated_rssi:.2f}",
                            f"{kalman_val:.2f}",
                            f"{sismf_val:.2f}",
                            f"{combo_val:.2f}",
                            active_mode,
                            f"{self.virtual_center_y:.2f}",
                            f"{self.virtual_center_x:.2f}"
                        ])
                        self.csv_file.flush()
                    except Exception:
                        pass

                # --- RADAR GUI ICIN UDP YAYINI ---
                try:
                    if not hasattr(self, 'udp_sock'):
                        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    msg = json.dumps({d: int(v[0]) for d, v in self.rssi_data.items()})
                    self.udp_sock.sendto(msg.encode('utf-8'), ("127.0.0.1", 5555))
                except Exception:
                    pass
                # ---------------------------------

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

    def get_windowed_rssi_snapshot(self):
        """
        Son karar döngüsünden (5sn) bu yana alınan TÜM Kalman-filtrelenmiş
        örnekleri drone başına ORTALAYIP döndürür - tek bir anlık değere
        (o örnek tesadüfen biraz sapmış olsa bile) bel bağlamak yerine.
        Bu, 5 saniyelik pencerede gelen tüm paketleri kullanarak "o
        pencerede gerçekte ne oldu"nu yansıtır, tek bir gürültülü örneğin
        yanlış yön/mesafe göstermesini (false flag) engeller.

        Bu pencerede hiç yeni paket almayan ama yine de RSSI_MAX_AGE_S
        içinde olan bir drone için, en son bilinen değeri kullanır (o
        drone'un ekseni tamamen durmasın diye).
        """
        now = time.time()
        snapshot = {}
        sample_counts = {}
        with self.rssi_lock:
            for drone_id in (1, 2, 3, 4):
                samples = self.window_samples.get(drone_id, [])
                if samples:
                    snapshot[drone_id] = sum(samples) / len(samples)
                    sample_counts[drone_id] = len(samples)
                elif drone_id in self.rssi_data:
                    rssi, ts = self.rssi_data[drone_id]
                    if now - ts <= RSSI_MAX_AGE_S:
                        snapshot[drone_id] = rssi
                        sample_counts[drone_id] = 0  # bu pencerede yeni paket yok, eski deger kullanildi

            # Bir sonraki 5sn'lik pencere için biriktiriciyi sıfırla
            self.window_samples = {}

        if sample_counts:
            counts_str = ", ".join(f"D{d}:{c}" for d, c in sorted(sample_counts.items()))
            print(f"[PENCERE] Bu döngüde ortalanan örnek sayısı -> {counts_str}")

        return snapshot

    # -------------------------------------------------
    # KARAR MOTORU (Güzey vd. 2022 - hata SIFIR olana kadar sürülen
    # formasyon kontrolü; ölçümler Kalman-filtrelenmiş)
    # -------------------------------------------------
    def calculate_virtual_center_shift(self, rssi_data):
        """
        Kuzey-Güney ve Doğu-Batı gruplarının Kalman-filtrelenmiş güç
        farklarından bir "adım" (shift) hesaplar. Bu adım
        send_mavlink_commands() içinde sanal merkeze BİRİKTİRİLİR
        (integral kontrol) - rapordaki "hatalar sıfır olacak şekilde
        sürülür" ifadesiyle birebir örtüşür. Fiziksel olarak İHA'lar
        hedefe yaklaştıkça güç farkı gerçekten küçülür ve sistem doğal
        olarak durur.

        Eksen bağımsız tazelik kontrolü: bir eksenin çifti (örn. Drone
        1/2) taze değilse sadece O eksen bu döngüde sabit kalır, diğer
        eksen (örn. Drone 3/4) yine de tepki vermeye devam eder.
        """
        have_ns = 1 in rssi_data and 2 in rssi_data
        have_ew = 3 in rssi_data and 4 in rssi_data

        if not have_ns and not have_ew:
            missing = [d for d in [1, 2, 3, 4] if d not in rssi_data]
            print(f"[UYARI] Ne Kuzey-Güney ne de Doğu-Batı çiftinden taze veri var "
                  f"({len(rssi_data)}/4). Eksik Dronelar: {missing}. Merkez sabit tutuluyor.")
            return 0.0, 0.0

        K_GAIN = 0.5
        shift_y = 0.0
        shift_x = 0.0
        delta_y_dbm = None
        delta_x_dbm = None

        if have_ns:
            delta_y_dbm = rssi_data[1] - rssi_data[2]   # Kuzey - Güney
            if AXIS_CONTROLLER == "SISMF":
                shift_y = self.axis_sismf_y.update_shift(delta_y_dbm)
            else:
                shift_y = max(min(delta_y_dbm * K_GAIN, 5.0), -5.0)
        else:
            print("[UYARI] Kuzey-Güney çifti (Drone 1/2) taze değil, bu eksen sabit tutuluyor.")

        if have_ew:
            delta_x_dbm = rssi_data[3] - rssi_data[4]   # Doğu - Batı
            if AXIS_CONTROLLER == "SISMF":
                shift_x = self.axis_sismf_x.update_shift(delta_x_dbm)
            else:
                shift_x = max(min(delta_x_dbm * K_GAIN, 5.0), -5.0)
        else:
            print("[UYARI] Doğu-Batı çifti (Drone 3/4) taze değil, bu eksen sabit tutuluyor.")

        mode_str = f"filtre={SIGNAL_FILTER}, kontrolcü={AXIS_CONTROLLER}"
        ns_txt = f"K/G: {delta_y_dbm:.1f}dB" if delta_y_dbm is not None else "K/G: N/A"
        ew_txt = f"D/B: {delta_x_dbm:.1f}dB" if delta_x_dbm is not None else "D/B: N/A"
        print(f"[KARAR MOTORU] [{mode_str}] Güç Farkları -> {ns_txt}, {ew_txt}")
        print(f"[KARAR MOTORU] Adım (biriktirilecek) -> KUZEYE: {shift_y:.1f}m, DOĞUYA: {shift_x:.1f}m")

        return shift_x, shift_y

    # -------------------------------------------------
    # MAVLINK KOMUT GÖNDERİMİ
    # -------------------------------------------------
    def send_mavlink_commands(self, shift_x, shift_y):
        if self.origin_lat is None or self.origin_lon is None:
            print("[UYARI] Origin henüz ayarlanmadı, komut gönderilmiyor.")
            return

        # BİRİKTİRİCİ (integrator): rapordaki "hata sıfır olana kadar
        # sürülür" ifadesiyle birebir örtüşen tasarım. Gerçek uçuşta
        # İHA'lar fiziksel olarak yaklaştıkça delta_dbm küçülür ve bu
        # birikim doğal olarak durur. PERVANESİZ BENCH TESTİNDE bu
        # birikim geofence sınırına dayanıp orada kalabilir - bu
        # beklenen bir durumdur (dosya başındaki notu görün).
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

        # --- ÇARPIŞMA KORUMASI (GÜVENLİK) ---
        MIN_DRONE_DIST_M = 3.0
        for id1, (n1, e1) in offsets.items():
            for id2, (n2, e2) in offsets.items():
                if id1 >= id2: continue
                dist = math.hypot(n1 - n2, e1 - e2)
                if dist < MIN_DRONE_DIST_M:
                    print(f"[KRİTİK GÜVENLİK] Drone {id1} ve {id2} hedefleri çok yakın ({dist:.1f}m < {MIN_DRONE_DIST_M}m)! Komutlar durduruldu.")
                    return

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
            type_mask = 0x0BF8
            fixed_yaw_rad = 0.0

            if hasattr(connection, "mav"):
                connection.mav.set_position_target_global_int_send(
                    0,
                    connection.target_system, connection.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask,
                    int(target_lat * 1e7),
                    int(target_lon * 1e7),
                    FORMATION_ALT_M,
                    0, 0, 0,
                    0, 0, 0,
                    fixed_yaw_rad, 0
                )
                print(f"[MAVLINK] Drone {drone_id} -> LAT: {target_lat:.6f}, LON: {target_lon:.6f}, "
                      f"ALT: {FORMATION_ALT_M}m, YAW: 0°")
            else:
                connection.set_position_target_global_int_send(
                    0, 0, 0, 6, type_mask,
                    int(target_lat * 1e7), int(target_lon * 1e7), FORMATION_ALT_M,
                    0, 0, 0, 0, 0, 0, fixed_yaw_rad, 0
                )

    # -------------------------------------------------
    # ANA DÖNGÜ
    # -------------------------------------------------
    def run(self):
        print("\n[BİLGİ] Görev Döngüsü Başlıyor (Pasif Dinleme + 1Hz Komut). Durdurmak için Ctrl+C.")
        loop_period = 1.0 / COMMAND_LOOP_HZ

        while True:
            loop_start = time.monotonic()

            rssi_snapshot = self.get_windowed_rssi_snapshot()

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
        if getattr(self, "csv_file", None):
            try:
                self.csv_file.close()
                print(f"[LOG KAYIT] Deney verisi kaydedildi ve kapatıldı: {self.csv_filename}")
            except Exception:
                pass
