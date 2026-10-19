"""
ground_station_core_sphere_v2.py   (v2 - simülasyonla doğrulanmış düzeltmeler)

KÜRE FORMASYONU YER İSTASYONU

v2 DEĞİŞİKLİKLERİ (her biri sphere_sim/ simülasyonunda test edildi):
  1. D2/D4 konum formülü düzeltildi. Raporlardaki yazım hatası
     (D2 = c - R(sin b, 0, cos b), D4 = c + R(cos b, 0, sin b)) iki dronu 2R
     aralıkta tutmuyordu ve v2 vektörüyle tutarsızdı. Doğrusu:
     D2/D4 = c -/+ R * b,  b = v2/R.
  2. VERTICAL_PLANE = "aligned": dikey çember, D1-D3 eksenine dik olacak şekilde
     yaw'da döndürülür: b = cos(b)(-sin a, cos a, 0) + sin(b)(0,0,1).
     Makaledeki sabit x-z düzleminde, hedef kuzey/güneydeyken D1-D2 aynı noktaya
     gelir (simülasyonda 0.0 m mesafe = ÇARPIŞMA). "aligned" ile dronlar arası
     mesafe her zaman >= R*sqrt(2). "fixed_xz" ile makale geometrisi korunabilir.
  3. Açı hatası, kalibrasyondan tahmin edilen tam ölçek farka bölünür
     (NORMALIZE_ERROR). Ham dB farkı uzaklıkla ~1/d küçülür (R=5 m, d=70 m'de
     en fazla ~0.76 dB!); normalize edilince kontrol kazancı uzaklıktan bağımsız olur.
  4. Yakınsama ölçütü: |e| < 1 dB (3 döngü) yerine, en az MIN_DWELL_S beklendikten
     sonra son CONV_WINDOW_CYCLES döngünün ORTALAMA hatası < eşik. Eski 1 dB eşiği,
     uzak hedefte e'nin tam ölçeğinden büyük olduğu için "anında yakınsıyordu".
  5. Doğru, açıların son pencere ortalamasından (eksenel ortalama) hesaplanır.
  6. FLIP_PAIRS: her merkezde çiftler yer değiştirip ikinci kez yakınsanır; dron
     başına sabit kalibrasyon/anten farkı (offset hatası) iki doğrunun ortalamasında
     birinci dereceden sıfırlanır.
  7. Doğru, açıların yakınsadığı merkezde kaydedilir (v1 kaydırmadan SONRAKİ
     merkezi kaydediyordu). Merkez doğru boyunca 3 m kaydırılmaz (ardışık doğrular
     neredeyse paralel olup kesişim kötü koşullanıyordu); bunun yerine yanal taban
     çizgisi (CENTER_PLAN = "baseline") veya sabit liste kullanılır. RSSI eğilimine
     dayalı işaret tahmini kaldırıldı (doğru yönsüzdür, gerek yoktur).
  8. Sonuç: tüm doğruların en küçük kareler kesişimi (IJSS 2026 Denk. 8),
     isteğe bağlı yer düzlemi kısıtı (GROUND_PRIOR).
  9. SIMULATION_MODE artık rastgele RSSI değil, fiziksel bir sanal dünya kullanır:
     sanal dronlar komut edilen noktalara uçar, RSSI kalibre edilmiş yol kaybı
     modelinden + gürültüden üretilir (SIM_* ayarları). Kod uçmadan uçtan uca
     denenebilir; SIM_TIME_SCALE ile hızlandırılır.
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
import csv
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None   # v2: simülasyon modunda gerekmez; gerçek donanımda connect_hardware uyarır

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


# ==========================================
# AYARLAR (CONFIG) - v2.1 GÜVENLİ UÇUŞ & SAHA AYARLARI
# ==========================================
FORMATION_RADIUS_M = 10.0      # R: küre yarıçapı (dronlar arası 2R = 20m). v2.1: R=10m sinyal gradyanı ve güvenlik için en ideal uzlaşma.
FORMATION_ALT_M    = 18.0      # Dronların uçuş irtifası (merkez z_c). 18m - 10m = 8m >= MIN_SAFE_ALT_M
MIN_SAFE_ALT_M     = 5.0       # Dikey çemberin en alt noktası bunun altına inemez

MAX_CENTER_DRIFT_M = 50.0      # Merkezin origin'den max sapması

RSSI_MAX_AGE_S = 10.0
RSSI_MAX_AGE_S = max(RSSI_MAX_AGE_S, 6.0)

ADJUST_ORIGIN_FOR_DRONE1_POSITION = True

COMMAND_LOOP_HZ = 0.2          # 5 saniyede bir döngü (dt = 5s)

# ==========================================
# v2.1: GEOMETRİ, YAKINSAMA, MERKEZ PLANI
# ==========================================
VERTICAL_PLANE = "aligned"      # "aligned" (önerilen, çarpışmasız) | "fixed_xz" (makale)
NORMALIZE_ERROR = True          # e / E_max(d_tahmini) -> uzaklıktan bağımsız kazanç
MIN_DWELL_S = 120.0             # v2.1: her merkezde en az bekleme (2 dakika) -> Batarya dostu (~8 dk toplam görev)
MAX_DWELL_S = 180.0             # v2.1: her merkezde en fazla bekleme (3 dakika timeout)
CONV_WINDOW_CYCLES = 15         # v2.1: yakınsama/ortalama penceresi (15 döngü x 5 s = 75 s)
FLIP_PAIRS = False              # v2.1: Uçuşta False (uçuş süresini yarı yarıya düşürür, bataryayı korur)
CENTER_PLAN = "baseline"        # "baseline": c0, c0 + B*dik, c0 - B*dik | "fixed": FIXED_CENTERS
BASELINE_M = 20.0               # yanal taban çizgisi (geofence MAX_CENTER_DRIFT_M içinde kalmalı)
FIXED_CENTERS = [(0.0, 0.0), (20.0, 0.0), (0.0, 20.0)]   # (Doğu, Kuzey) m, CENTER_PLAN="fixed"
GROUND_PRIOR = True             # hedef yerde (z = 0) varsayımıyla kesişim
MIN_DRONE_DIST_M = 3.0          # komut edilen iki nokta arasında en az mesafe

# --- PTT / SİNYAL KOPMA KORUMASI (v2.1) ---
MIN_VALID_RSSI_DBM = -115.0     # -115 dBm ve altındaki paketler 'telsiz mandalı basılı değil' sayılarak elenir

# ==========================================
# FİLTRE VE AÇI KONTROLCÜSÜ SEÇİMİ (AÇMA/KAPAMA ANAHTARLARI)
# ==========================================
# USE_KALMAN     : True ise RSSI ham verileri Kalman Filtresinden geçirilir (Rapor 2).
#
# AÇI KONTROLÜ / SMF MODU (CONTROLLER_MODE):
#   "SISMF_2026" : N. Güzey, SCI_4 (Mart 2026) Makalesi: s = e + λ·∫e dτ
#                  (Türevsiz yeni kayan yüzey, chattering tamamen sıfır, en kararlı)
#   "ISMF_2024"  : N. Güzey, IEEE Access (2024) Makalesi: s = ė + λ·e
#   "PID"        : Klasik PID kontrolcüsü
USE_KALMAN      = True
CONTROLLER_MODE = "SISMF_2026"  # "SISMF_2026", "ISMF_2024", "PID"

# ==========================================
# KALMAN FİLTRESİ AYARLARI (Rapor 2, Denklem 24-28)
# ==========================================
KALMAN_Q = 1.0     # Süreç gürültüsü kovaryansı
KALMAN_R = 20.0    # Ölçüm gürültüsü kovaryansı

# ==========================================
# SCI_4 (MART 2026) SISMF PARAMETRELERİ (N. Güzey, IJSS 2026)
# ==========================================
# s = e + λ · ∫ e dτ
# α̇ = k · tanh(s / φ) + κ · ∫ e dτ
# v2: NORMALIZE_ERROR = True için simülasyonda ızgara aramasıyla seçilen kazançlar
# (sphere_sim/tune.py). Makaledeki λ=1, κ=0.05 (açı birimli hata için) dB hatasıyla
# limit çevrime (±100° salınım) giriyordu. NORMALIZE_ERROR = False ise eski değerleri
# kullanın: λ=1.0, k=0.2, κ=0.05, φ=2.0.
SISMF_2026_LAMBDA = 0.01  # λ: Kayan yüzey integral kazancı
SISMF_2026_K      = 0.2   # k: Sign/tanh kazancı (rad/s)
SISMF_2026_KAPPA  = 0.0016  # κ: Düzeltici integral kazancı
SISMF_2026_PHI    = 1.0   # φ: Sınır katmanı kalınlığı (normalize hata biriminde)
SISMF_2026_INTEGRAL_MAX = 30.0

# ==========================================
# S-ISMF PARAMETRELERİ (Güzey 2024, IEEE Access)
# ==========================================
ISMF_KS     = 0.2     # rad/s (sign kazancı)          [v2 ayarı; eski 0.08]
ISMF_KI     = 0.005   # integral kazancı               [v2 ayarı; eski 0.008]
ISMF_LAMBDA = 1.0     # kayan yüzey eğimi
ISMF_PHI    = 1.0     # sınır katmanı                  [v2 ayarı; eski 2.0 sabit]
ISMF_INTEGRAL_MAX = 20.0

# ==========================================
# PID KONTROL PARAMETRELERİ (Güzey 2022, Denklem 32)
# ==========================================
PID_KP = 0.1      # [v2 ayarı; eski 0.02]
PID_KI = 0.0016   # [v2 ayarı; eski 0.005]
PID_KD = 0.0      # [v2: türev gürültüyü büyütüyordu; eski 0.01]
PID_INTEGRAL_MAX = 50.0

# v2: son CONV_WINDOW_CYCLES döngünün ORTALAMA |e| değeri bu eşiğin altında ise
# (ve MIN_DWELL_S geçtiyse) "yakınsadı" sayılır (dB).
CONVERGENCE_THRESHOLD_DBM = 0.15

# ==========================================
# KALİBRASYON OFSETLERİ (kalibrasyon_sonucu_saha.json)
# ==========================================
# Saha kalibrasyon verileri (RMSE = 2.29 dB, en düşük hata)
# P_ref = -78.72 dBm, n = 1.264
CALIBRATION_OFFSET_DB = {
    1:  0.00,    # D1 Kuzey (referans)
    2: -1.32,    # D2 Güney
    3:  2.38,    # D3 Doğu
    4:  1.10,    # D4 Batı
}

# ==========================================
# TEST MODU
# ==========================================
BENCH_TEST_MODE = False          # v2: SIMULATION_MODE fiziksel simülasyon kullanır;
                                 # pervanesiz masa testi için True yapın.

# ==========================================
# v2: FİZİKSEL SİMÜLASYON (SIMULATION_MODE = True iken)
# ==========================================
SIM_TARGET_ENU = (55.0, 45.0, 0.0)  # sanal telsiz: Doğu, Kuzey, Yukarı (m, origin'e göre)
SIM_TIME_SCALE = 20.0           # 20 -> 1 saatlik görev ~3 dakikada
SIM_P_REF = -78.72              # saha kalibrasyonu
SIM_N = 2.000                   # v2.1: Havada görüş hattı (Line of Sight) boş uzay yayılımı (n ≈ 2.0)
SIM_NOISE_STD_DB = 1.2          # zamansal gürültü (en sakin saha logu ~1.2 dB)
SIM_NOISE_RHO = 0.3             # ardışık örnek korelasyonu
SIM_BIAS_STD_DB = 0.5           # kalibrasyon sonrası kalan dron farkı
SIM_DRONE_SPEED_MS = 3.0
SIM_SAMPLE_PERIOD_S = 2.0       # dron başına RSSI örnek periyodu (LoRa TDMA)

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

# ==========================================
# SCI_4 (MART 2026) SMOOTHER INTEGRAL SLIDING MODE FILTER (SISMF)
# ==========================================
class SmootherIntegralSlidingModeFilter2026:
    """
    N. Güzey, "A smoother integral sliding mode filter approach for
    distributed moving target tracking", International Journal of Systems
    Science, March 2026 (SCI_4.pdf).

    Makaledeki Denklemler (Eq 3 - 6):
      e1 = P1,+ - P1,-  (veya P3 - P1)
      e2 = P2,+ - P2,-  (veya P4 - P2)

      Yeni Kayan Yüzey (Türevsiz, Doğrudan Hata İntegrali ile, Denklem 3 & 5):
        s1(t) = e1(t) + λ · ∫ e1(τ) dτ
        s2(t) = e2(t) + λ · ∫ e2(τ) dτ

      Yumuşatılmış Açı Dinamiği (Denklem 4 & 6):
        α̇(t) = k · tanh(s1 / φ) + κ · ∫ e1(τ) dτ
        β̇(t) = k · tanh(s2 / φ) + κ · ∫ e2(τ) dτ

      Açı Güncellemesi:
        α(t + dt) = α(t) + α̇ · dt
        β(t + dt) = β(t) + β̇ · dt
    """
    def __init__(self, lambda_param=SISMF_2026_LAMBDA, k=SISMF_2026_K,
                 kappa=SISMF_2026_KAPPA, phi=SISMF_2026_PHI,
                 integral_max=SISMF_2026_INTEGRAL_MAX):
        self.lambda_param = float(lambda_param)
        self.k = float(k)
        self.kappa = float(kappa)
        self.phi = float(phi)
        self.integral_max = float(integral_max)
        self.error_integral = 0.0

    def compute(self, error, dt):
        """error = e (dBm), dt = döngü periyodu (s). Döndürür: açı türevi (rad/s)."""
        self.error_integral += error * dt
        self.error_integral = max(min(self.error_integral, self.integral_max), -self.integral_max)

        # Denklem 3 & 5: Türevsiz doğrudan integral kayan yüzey
        s = error + self.lambda_param * self.error_integral

        # Chattering önleyici sınır katmanı
        soft_sign = math.tanh(s / self.phi)

        # Denklem 4 & 6: Yumuşatılmış açı türevi
        angle_dot = (self.k * soft_sign) + (self.kappa * self.error_integral)

        # Güvenlik sınırı (max 0.5 rad/s)
        max_rad_per_sec = 0.5
        angle_dot = max(min(angle_dot, max_rad_per_sec), -max_rad_per_sec)
        return angle_dot

    def reset(self):
        self.error_integral = 0.0


class SmootherIntegralSlidingModeFilter:
    """
    N. Güzey, "Smoother Integral Sliding Mode Filter for RSSI-Based
    Localization", IEEE Access, vol. 12, pp. 172410-172418, 2024.
    DOI: 10.1109/ACCESS.2024.3467994

    Makaledeki denklemler:
      Güç farkları (Denklem 6):
        e1 = P3 - P1,  e2 = P4 - P2

      Hata türevleri (Denklem 7):
        ė1 = Ṗ3 - Ṗ1,  ė2 = Ṗ4 - Ṗ2

      Kayan yüzey (Denklem 8):
        s1 = ė1 + λ·e1
        s2 = ė2 + λ·e2

      Klasik SMF - chattering sorunlu (Denklem 9):
        α̇ = ks · sign(s1)
        β̇ = ks · sign(s2)

      Yumuşatılmış S-ISMF - chattering YOK (Denklem 10):
        α̇ = ks · sign(s1) + ki · ∫ e1(τ) dτ
        β̇ = ks · sign(s2) + ki · ∫ e2(τ) dτ

      Açı güncellemesi (Denklem 11):
        α(t) = α(t₀) + ∫ α̇(τ) dτ    (sayısal: α += α̇ · dt)
        β(t) = β(t₀) + ∫ β̇(τ) dτ

    Girdi:  error = e (dBm cinsinden güç farkı)
    Çıktı:  açı türevi α̇ veya β̇ (rad/s)
    """
    def __init__(self, k_s=ISMF_KS, k_i=ISMF_KI, lambda_param=ISMF_LAMBDA,
                 integral_max=ISMF_INTEGRAL_MAX):
        self.k_s = k_s                     # ks: sign kazancı
        self.k_i = k_i                     # ki: integral kazancı
        self.lambda_param = lambda_param    # λ: kayan yüzey eğimi
        self.integral_max = integral_max    # Windup koruma sınırı
        self.error_integral = 0.0           # ∫ e(τ) dτ (birikim)
        self.prev_error = None              # İlk döngü sıçramasını önlemek için None

    def compute(self, error, dt):
        """error = e (dBm), dt = döngü periyodu (s). Döndürür: açı türevi (rad/s)."""
        # İlk döngüde türev sıçramasını önle
        if self.prev_error is None:
            self.prev_error = error

        # ── Denklem 7: Hata türevi (sayısal yaklaşım) ──
        if dt > 0:
            e_dot = (error - self.prev_error) / dt
        else:
            e_dot = 0.0

        # ── Denklem 8: Kayan yüzey ──
        s = e_dot + self.lambda_param * error

        # ── Sınır katmanlı sign(s) fonksiyonu (Chattering önleyici) ──
        # sign(s) yerine tanh(s / phi) kullanarak dt=5s olan ayrık zamanda
        # eksen etrafında titreşimi (chattering) engelliyoruz.
        soft_sign = math.tanh(s / ISMF_PHI)

        # ── Denklem 10: Hata integrali (windup korumalı) ──
        self.error_integral += error * dt
        self.error_integral = max(min(self.error_integral, self.integral_max),
                                  -self.integral_max)

        # ── Denklem 10: Yumuşatılmış açı türevi ──
        angle_dot = (self.k_s * soft_sign) + (self.k_i * self.error_integral)

        # Güvenlik: Döngü başına maksimum dönüş hızını sınırla (örn: max 0.5 rad/s)
        max_rad_per_sec = 0.5
        angle_dot = max(min(angle_dot, max_rad_per_sec), -max_rad_per_sec)

        # ── Sonraki adım için hatayı kaydet ──
        self.prev_error = error
        return angle_dot

    def reset(self):
        """Yeni iterasyon (merkez kaydırma) öncesi sıfırla."""
        self.error_integral = 0.0
        self.prev_error = None


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
class SimWorld:
    """Sanal dünya: komut edilen noktalara uçan 4 dron + kalibre yol kaybı modeli.
    Konumlar origin'e göre (Doğu, Kuzey, Yukarı) metre. Zaman SIM_TIME_SCALE ile hızlı."""
    def __init__(self):
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.last_t = 0.0
        self.pos = {i: [0.0, 0.0, FORMATION_ALT_M] for i in (1, 2, 3, 4)}
        self.cmd = {i: list(p) for i, p in self.pos.items()}
        self.bias = {i: random.gauss(0.0, SIM_BIAS_STD_DB) for i in (1, 2, 3, 4)}
        self.ar = {i: random.gauss(0.0, SIM_NOISE_STD_DB) for i in (1, 2, 3, 4)}

    def sim_time(self):
        return (time.time() - self.t0) * SIM_TIME_SCALE

    def _advance(self):
        t = self.sim_time()
        dt = t - self.last_t
        self.last_t = t
        lim = SIM_DRONE_SPEED_MS * dt
        for i in (1, 2, 3, 4):
            p, c = self.pos[i], self.cmd[i]
            d = math.sqrt(sum((c[k] - p[k]) ** 2 for k in range(3)))
            f = 1.0 if d <= lim else lim / d
            self.pos[i] = [p[k] + f * (c[k] - p[k]) for k in range(3)]

    def command(self, drone_id, east, north, up):
        with self.lock:
            self._advance()
            self.cmd[drone_id] = [east, north, up]

    def position(self, drone_id):
        with self.lock:
            self._advance()
            return list(self.pos[drone_id])

    def raw_rssi(self, drone_id):
        """Dron üzerindeki SDR'nin göreceği HAM tam sayı RSSI (ofset uygulanmamış)."""
        with self.lock:
            self._advance()
            p = self.pos[drone_id]
            d = max(math.sqrt(sum((p[k] - SIM_TARGET_ENU[k]) ** 2 for k in range(3))), 0.5)
            rho = SIM_NOISE_RHO
            self.ar[drone_id] = rho * self.ar[drone_id] + math.sqrt(1 - rho * rho) * random.gauss(0, SIM_NOISE_STD_DB)
            true_cal = SIM_P_REF - 10.0 * SIM_N * math.log10(d) + self.bias[drone_id] + self.ar[drone_id]
            return int(round(true_cal - CALIBRATION_OFFSET_DB.get(drone_id, 0.0)))


SIM_WORLD = None


class MockSerial:
    """LoRa yerine: her dron için SIM_SAMPLE_PERIOD_S'de bir 'N{id},{rssi},{chk}' paketi."""
    def __init__(self):
        self._next_id = 1
        self._next_t = 0.0

    def read(self, size):
        # Paketler SANAL zamana göre üretilir (TDMA: 4 dron sırayla, dron başına
        # SIM_SAMPLE_PERIOD_S'de bir) - yazdırma/CSV yükü örnek hızını bozmaz.
        while SIM_WORLD.sim_time() < self._next_t:
            time.sleep(0.002)
        self._next_t += SIM_SAMPLE_PERIOD_S / 4.0
        drone_id = self._next_id
        self._next_id = (self._next_id % 4) + 1
        rssi = SIM_WORLD.raw_rssi(drone_id)
        body = f"N{drone_id},{rssi}"
        checksum = compute_checksum(body)
        reply = f"{body},{checksum}\n"
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
        last_gcs_heartbeat = 0.0
        while not self._stop:
            try:
                # ── GCS Heartbeat Gönderimi (Failsafe önleme) ──
                now = time.time()
                if now - last_gcs_heartbeat > 1.0:
                    try:
                        self.connection.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0
                        )
                        last_gcs_heartbeat = now
                    except Exception:
                        pass

                msg = self.connection.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT", "SYS_STATUS"],
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
                elif msg.get_type() == "SYS_STATUS":
                    with self.lock:
                        if not hasattr(self, 'battery_remaining'):
                            self.battery_remaining = 100
                        self.battery_remaining = msg.battery_remaining
            except Exception as e:
                print(f"[UYARI MAVLINK] Drone {self.drone_id}: {e}")
                time.sleep(1.0)

    def trigger_rtl(self):
        """Drone'u RTL veya LAND moduna alır (Kritik batarya durumu)."""
        try:
            print(f"[ACİL DURUM] Drone {self.drone_id} RTL moduna geçiriliyor!")
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6  # ArduCopter RTL mode
            )
        except Exception as e:
            print(f"[UYARI MAVLINK] Drone {self.drone_id} RTL tetiklenemedi: {e}")

    def is_ready_for_commands(self):
        with self.lock:
            if time.time() - self.last_heartbeat_time > MAVLINK_HEARTBEAT_TIMEOUT_S:
                return False, "heartbeat kaybı"
                
            # Batarya Kontrolü
            battery = getattr(self, 'battery_remaining', 100)
            if battery != -1 and battery < 20:
                if not getattr(self, 'rtl_triggered', False):
                    self.trigger_rtl()
                    self.rtl_triggered = True
                return False, f"KRİTİK BATARYA (%{battery}) - RTL TETİKLENDİ"
                
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
    STATE_SETUP     = "SETUP"
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

        # ── Açı Kontrolcüsü Seçimi ──
        if CONTROLLER_MODE == "SISMF_2026":
            self.angle_controller_alpha = SmootherIntegralSlidingModeFilter2026(
                lambda_param=SISMF_2026_LAMBDA, k=SISMF_2026_K, kappa=SISMF_2026_KAPPA, phi=SISMF_2026_PHI)
            self.angle_controller_beta  = SmootherIntegralSlidingModeFilter2026(
                lambda_param=SISMF_2026_LAMBDA, k=SISMF_2026_K, kappa=SISMF_2026_KAPPA, phi=SISMF_2026_PHI)
            angle_ctrl = "SISMF (SCI_4.pdf Mart 2026 - Türevsiz Yeni Kayan Yüzey)"
        elif CONTROLLER_MODE == "ISMF_2024":
            self.angle_controller_alpha = SmootherIntegralSlidingModeFilter(
                k_s=ISMF_KS, k_i=ISMF_KI, lambda_param=ISMF_LAMBDA)
            self.angle_controller_beta  = SmootherIntegralSlidingModeFilter(
                k_s=ISMF_KS, k_i=ISMF_KI, lambda_param=ISMF_LAMBDA)
            angle_ctrl = "S-ISMF (Güzey 2024, IEEE Access)"
        else:
            self.angle_controller_alpha = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            self.angle_controller_beta  = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            angle_ctrl = "PID (Klasik)"

        # Durum makinesi
        self.state = self.STATE_SETUP  # Formasyon Kurulum Fazı ile başla
        self.convergence_count = 0  # Kaç ardışık döngüde yakınsadı

        # v2: yakınsama penceresi, çift değiştirme (flip) ve merkez planı
        self.loop_sign_a = -1.0          # e1 = P3 - P1, D3 = c + R a için kararlı dal
        self.loop_sign_b = -1.0
        self.flip_phase = 0              # 0: normal, 1: çiftler yer değiştirmiş
        self.flip_first_dir = None
        self.dwell_start_t = None
        self.e_hist = []
        self.ab_hist = []
        self.plan = [(self.center_x, self.center_y)]
        if CENTER_PLAN == "fixed":
            self.plan = [tuple(c) for c in FIXED_CENTERS]
            self.center_x, self.center_y = self.plan[0]
        self.plan_index = 0
        self.STATE_DONE = "DONE"
        if FORMATION_ALT_M - FORMATION_RADIUS_M < MIN_SAFE_ALT_M and not BENCH_TEST_MODE:
            print(f"[KRİTİK] FORMATION_ALT_M - R = {FORMATION_ALT_M - FORMATION_RADIUS_M:.1f} m < "
                  f"{MIN_SAFE_ALT_M} m. Dikey çember yere çok yaklaşır. Ayarları düzeltin.")
            sys.exit(1)

        # Hedef doğruları (iterasyon bazında biriktirme)
        self.direction_lines = []   # [(C, A), ...]

        self._lora_stop = False
        self._start_time = time.time()

        # --- DENEY LOG KAYDI (CSV) ---
        self.csv_filename = f"sphere_deney_log_{int(time.time())}.csv"
        try:
            self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "Timestamp", "Gecen_Sure_Sn", "Drone_ID",
                "Ham_RSSI", "Kalman_RSSI", "Kontrolcu_Modu",
                "Alpha_Deg", "Beta_Deg",
                "Merkez_Kuzey_M", "Merkez_Dogu_M", "Merkez_Irtifa_M"
            ])
            self.csv_file.flush()
            print(f"[LOG KAYIT] Küre deney verileri kaydediliyor -> {self.csv_filename}")
        except Exception as e:
            print(f"[UYARI LOG] CSV dosyası oluşturulamadı: {e}")
            self.csv_file = None
            self.csv_writer = None

        # ── Başlangıç Mesajları ──
        kalman_status = "✅ AÇIK" if USE_KALMAN else "❌ KAPALI"

        print("=" * 60)
        print("  🌐 KÜRE FORMASYONU YER İSTASYONU 🌐")
        print("  TÜBİTAK 123E294 - Güzey vd. (2022 + 2024 + 2026)")
        if platform_label:
            print(f"  Platform: {platform_label}")
        print("=" * 60)
        print(f"Mod: {'SİMÜLASYON' if simulation else 'GERÇEK UÇUŞ'}")
        print(f"R={FORMATION_RADIUS_M}m | ALT={FORMATION_ALT_M}m | "
              f"Geofence=±{MAX_CENTER_DRIFT_M}m")
        print(f"")
        print(f"  FİLTRELER VE KONTROLCÜ:")
        print(f"    Kalman Filtresi (RSSI Gürültü):  {kalman_status} (Q={KALMAN_Q}, R={KALMAN_R})")
        print(f"    Açı Kontrolcüsü / SMF:           {angle_ctrl}")
        if CONTROLLER_MODE == "SISMF_2026":
            print(f"      SISMF_2026: λ={SISMF_2026_LAMBDA}, k={SISMF_2026_K}, κ={SISMF_2026_KAPPA}, φ={SISMF_2026_PHI}")
        elif CONTROLLER_MODE == "ISMF_2024":
            print(f"      ISMF_2024:  ks={ISMF_KS}, ki={ISMF_KI}, λ={ISMF_LAMBDA}")
        else:
            print(f"      PID:        Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
        print(f"")
        print(f"  Kalibrasyon (Saha): {CALIBRATION_OFFSET_DB}")
        print(f"  Yakınsama: pencere ort. |e| < {CONVERGENCE_THRESHOLD_DBM} dB, "
              f"{MIN_DWELL_S:.0f}-{MAX_DWELL_S:.0f} s/merkez, flip={FLIP_PAIRS}")
        print(f"  Dikey düzlem: {VERTICAL_PLANE} | Normalize: {NORMALIZE_ERROR} | "
              f"Merkez planı: {CENTER_PLAN} (B={BASELINE_M} m)")

        if BENCH_TEST_MODE:
            print("\n" + "!" * 60)
            print("!!  BENCH_TEST_MODE = True                               !!")
            print("!!  β açısı DONDURULDU (dikey dönüş yok).                !!")
            print("!!  Tüm dronlar sabit irtifada kalacak.                  !!")
            print("!!  Sadece α (yatay) ekseni aktif.                       !!")
            print("!!  GERÇEK UÇUŞTAN ÖNCE False YAPIN.                     !!")
            print("!" * 60 + "\n")

        if self.simulation:
            global SIM_WORLD
            SIM_WORLD = SimWorld()
            print(f"[SİM] Fiziksel simülasyon: hedef ENU={SIM_TARGET_ENU}, "
                  f"gürültü={SIM_NOISE_STD_DB} dB, hız x{SIM_TIME_SCALE}")
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

        if serial is None:
            print("[HATA] pyserial kütüphanesi eksik. 'pip install pyserial'")
            sys.exit(1)
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
            
            # Checksum'ı orijinal ham string (parts) üzerinden doğrula!
            body = f"N{parts[0]},{parts[1]}"
            chk = int(parts[2])
            if chk != compute_checksum(body):
                print(f"[UYARI] Checksum hatalı: '{line}'")
                return

            drone_id = int(parts[0])
            rssi = float(parts[1])

            # ── PTT / SİNYAL KOPMA KORUMASI (v2.1) ──
            # Telsiz mandalına basılmadığında alıcı dip gürültüsünü (-120 dBm) okur.
            # Dronların biri -120 dBm okurken diğeri -85 dBm okursa oluşacak yapay 35 dB'lik
            # farkın otopilotu ve filtreleri sarsmaması için bu paketler elenir.
            if rssi <= MIN_VALID_RSSI_DBM:
                return

            if drone_id not in self.pixhawk_ports and not self.simulation:
                return

            with self.rssi_lock:
                cal = rssi + CALIBRATION_OFFSET_DB.get(drone_id, 0.0)

                # ── Kalman Filtresi (açma/kapama) ──
                if USE_KALMAN:
                    if drone_id not in self.kalman_filters:
                        self.kalman_filters[drone_id] = ScalarKalmanFilter(cal)
                    filt = self.kalman_filters[drone_id].update(cal)
                else:
                    filt = cal   # Ham kalibrasyon uygulanmış değer

                self.rssi_data[drone_id] = (filt, time.time())
                self.window_samples.setdefault(drone_id, []).append(filt)

                # CSV'ye anlık veri kaydet
                if getattr(self, "csv_writer", None):
                    try:
                        elapsed = time.time() - self._start_time
                        self.csv_writer.writerow([
                            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            f"{elapsed:.2f}",
                            drone_id,
                            f"{cal:.2f}",
                            f"{filt:.2f}",
                            CONTROLLER_MODE,
                            f"{math.degrees(self.alpha):.2f}",
                            f"{math.degrees(self.beta):.2f}",
                            f"{self.center_y:.2f}",
                            f"{self.center_x:.2f}",
                            f"{self.center_z:.2f}"
                        ])
                        self.csv_file.flush()
                    except Exception:
                        pass

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
    def _axes(self, alpha=None, beta=None):
        """Birim eksenler: a (D1-D3, yatay) ve b (D2-D4, dikey düzlem)."""
        a_ang = self.alpha if alpha is None else alpha
        b_ang = self.beta if beta is None else beta
        a = (math.cos(a_ang), math.sin(a_ang), 0.0)
        if VERTICAL_PLANE == "aligned":
            b = (-math.cos(b_ang) * math.sin(a_ang), math.cos(b_ang) * math.cos(a_ang), math.sin(b_ang))
        else:  # "fixed_xz" (makale, Denk. 15)
            b = (math.cos(b_ang), 0.0, math.sin(b_ang))
        return a, b

    def compute_drone_positions(self):
        """
        v2 - DÜZELTİLMİŞ geometri (x = Doğu, y = Kuzey, z = irtifa):
          D1 = c - R a,  D3 = c + R a,   a = (cos α, sin α, 0)
          D2 = c - R b,  D4 = c + R b,   b = v2 / R
        Raporlardaki D2 = c - R(sin β, 0, cos β) yazım hatası 2R aralığını bozuyordu.
        Dönüş: {id: (kuzey_m, doğu_m, irtifa_m)}
        """
        R = FORMATION_RADIUS_M
        cx, cy, cz = self.center_x, self.center_y, self.center_z
        a, b = self._axes()
        pts = {
            1: (cx - R * a[0], cy - R * a[1], cz - R * a[2]),
            3: (cx + R * a[0], cy + R * a[1], cz + R * a[2]),
            2: (cx - R * b[0], cy - R * b[1], cz - R * b[2]),
            4: (cx + R * b[0], cy + R * b[1], cz + R * b[2]),
        }
        out = {}
        for i, (e, n, z) in pts.items():
            if BENCH_TEST_MODE:
                z = cz
            out[i] = (n, e, max(z, MIN_SAFE_ALT_M))
        return out

    def compute_direction_vector(self, alpha=None, beta=None):
        """v3 = v1 x v2 (IEEE Access 2024 Denk. 16): merkezden hedeften geçen doğrunun
        birim yönü. Doğru yönsüzdür (±v3 aynı doğru)."""
        a, b = self._axes(alpha, beta)
        v3 = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
        mag = math.sqrt(sum(c * c for c in v3))
        if mag < 1e-9:
            return 0.0, 0.0, 0.0
        return tuple(c / mag for c in v3)

    def _axial_mean(self, vectors):
        """Yönsüz doğruların ortalaması (sum v v^T'nin en büyük özvektörü)."""
        import numpy as np
        M = np.zeros((3, 3))
        for v in vectors:
            v = np.asarray(v, float)
            n = np.linalg.norm(v)
            if n > 1e-9:
                M += np.outer(v / n, v / n)
        w, V = np.linalg.eigh(M)
        return tuple(float(x) for x in V[:, -1])

    def _predicted_emax(self, mean_rssi):
        """Çift ekseni hedefe bakarken beklenen en büyük |P3-P1| (dB). Sadece kazanç ölçekleme."""
        p_ref = globals().get("SIM_P_REF", -78.72)
        n = globals().get("SIM_N", 1.264)
        R = FORMATION_RADIUS_M
        d_hat = max(10.0 ** ((p_ref - mean_rssi) / (10.0 * n)), 1.5 * R)
        return max(10.0 * n * math.log10((d_hat + R) / (d_hat - R)), 0.05)

    def update_rotation_angles(self, rssi_snap, dt):
        """
        Açı güncelleme (Rapor 2 Denk. 30-33 / IEEE Access Denk. 6-11 / IJSS Denk. 3-6).
          e1 = P3 - P1  -> α,   e2 = P4 - P2 -> β
        v2: e, NORMALIZE_ERROR ise E_max(d) ile bölünür; işaret kararlı daldan seçilir;
        yakınsama pencere ortalamasıyla değerlendirilir.
        """
        have_g1 = 1 in rssi_snap and 3 in rssi_snap
        have_g2 = 2 in rssi_snap and 4 in rssi_snap
        if not (have_g1 and have_g2):
            missing = [d for d in (1, 2, 3, 4) if d not in rssi_snap]
            print(f"[UYARI] Eksik RSSI: {missing}. Açılar sabit, döngü sayılmıyor.")
            return None, None

        e1 = rssi_snap[3] - rssi_snap[1]
        e2 = rssi_snap[4] - rssi_snap[2]
        if NORMALIZE_ERROR:
            emax = self._predicted_emax(sum(rssi_snap.values()) / len(rssi_snap))
            u1, u2 = e1 / emax, e2 / emax
        else:
            emax, u1, u2 = 1.0, e1, e2

        MAX_ANGLE_STEP_RAD = math.radians(8)
        da = self.loop_sign_a * self.angle_controller_alpha.compute(u1, dt) * dt
        db = self.loop_sign_b * self.angle_controller_beta.compute(u2, dt) * dt
        da = max(min(da, MAX_ANGLE_STEP_RAD), -MAX_ANGLE_STEP_RAD)
        db = max(min(db, MAX_ANGLE_STEP_RAD), -MAX_ANGLE_STEP_RAD)
        self.alpha += da
        if not BENCH_TEST_MODE:
            self.beta += db

        self.e_hist.append((e1, e2))
        self.ab_hist.append((self.alpha, self.beta))
        print(f"[KÜRE] α={math.degrees(self.alpha):.1f}° β={math.degrees(self.beta):.1f}° | "
              f"e1={e1:+.2f} e2={e2:+.2f} dB (E_max≈{emax:.2f} dB) | flip={self.flip_phase}")
        return e1, e2

    def is_converged(self, sim_now):
        """MIN_DWELL_S geçti ve pencere ortalaması eşik altında -> True. MAX_DWELL_S -> True."""
        dwell = sim_now - self.dwell_start_t
        if dwell >= MAX_DWELL_S:
            print(f"[KÜRE] MAX_DWELL_S ({MAX_DWELL_S:.0f} s) doldu; pencere ortalamasıyla devam.")
            return True
        w = CONV_WINDOW_CYCLES
        if dwell < MIN_DWELL_S or len(self.e_hist) < w:
            return False
        m1 = sum(e[0] for e in self.e_hist[-w:]) / w
        m2 = sum(e[1] for e in self.e_hist[-w:]) / w
        return abs(m1) < CONVERGENCE_THRESHOLD_DBM and (abs(m2) < CONVERGENCE_THRESHOLD_DBM or BENCH_TEST_MODE)

    def window_direction(self):
        w = min(CONV_WINDOW_CYCLES, len(self.ab_hist))
        if w == 0:
            return self.compute_direction_vector()
        return self._axial_mean([self.compute_direction_vector(a, b) for a, b in self.ab_hist[-w:]])

    # ─────────────────────────────────────────────
    # MAVLINK KOMUT GÖNDERİMİ
    # ─────────────────────────────────────────────
    def send_mavlink_commands(self):
        if self.origin_lat is None or self.origin_lon is None:
            print("[UYARI] Origin yok, komut gönderilmiyor.")
            return

        positions = self.compute_drone_positions()

        # ── Güvenlik: Minimum mesafe / çarpışma kontrolü ──
        points = list(positions.values())
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                d1 = points[i]
                d2 = points[j]
                dist = math.sqrt((d1[0]-d2[0])**2 + (d1[1]-d2[1])**2 + (d1[2]-d2[2])**2)
                if dist < MIN_DRONE_DIST_M:
                    print(f"[KRİTİK GÜVENLİK] ÇARPIŞMA RİSKİ! İki hedef arası mesafe sadece {dist:.1f}m. Komutlar IPTAL!")
                    # TODO: Otonom modda LOITER'e geçirilebilir, şimdilik tehlikeli pozisyona gitmesini engelliyoruz.
                    return

        for drone_id, (north_m, east_m, alt_m) in positions.items():
            link = self.drone_links.get(drone_id)
            if link is None:
                continue

            if self.simulation and SIM_WORLD is not None:
                SIM_WORLD.command(drone_id, east_m, north_m, alt_m)
                continue

            ready, reason = link.is_ready_for_commands()
            if not ready and not self.simulation:
                print(f"[GÜVENLİK] Drone {drone_id}: {reason}, ATLANIYOR.")
                continue

            target_lat, target_lon = offset_latlon(
                self.origin_lat, self.origin_lon, north_m, east_m
            )

            # MAVLink type_mask: x, y, z VE YAW aktif (diğerleri iptal).
            # 0b0000101111111000 = 0x0BF8 = 3064
            # Bit 11=0 (yaw'ı kullan), Bit 12=1 (yaw_rate yoksay)
            type_mask = 0x0BF8
            fixed_yaw_rad = 0.0  # Sabit kuzeye bak (anten polarizasyonunu sabit tut)

            conn = link.connection
            if hasattr(conn, "mav"):
                conn.mav.set_position_target_global_int_send(
                    0,
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask,
                    int(target_lat * 1e7),
                    int(target_lon * 1e7),
                    alt_m,
                    0, 0, 0, 0, 0, 0, fixed_yaw_rad, 0
                )
                print(f"[MAVLINK] Drone {drone_id} -> "
                      f"LAT:{target_lat:.6f} LON:{target_lon:.6f} ALT:{alt_m:.1f}m "
                      f"(N:{north_m:.1f}m E:{east_m:.1f}m) YAW:0°")
            else:
                conn.set_position_target_global_int_send(
                    0, 0, 0, 6, type_mask,
                    int(target_lat * 1e7), int(target_lon * 1e7), alt_m,
                    0, 0, 0, 0, 0, 0, fixed_yaw_rad, 0
                )

    # ─────────────────────────────────────────────
    # MERKEZ KAYDIRMA (hedef doğrusu boyunca)
    # ─────────────────────────────────────────────
    def finish_center(self, sim_now):
        """Bir merkezdeki yakınsama bitti: flip fazına geç veya doğruyu kaydedip
        plandaki sonraki merkeze git."""
        v = self.window_direction()
        if FLIP_PAIRS and self.flip_phase == 0:
            self.flip_first_dir = v
            self.flip_phase = 1
            # Çiftleri yer değiştir: a -> -a; b -> -b
            self.alpha += math.pi
            if VERTICAL_PLANE == "aligned":
                self.beta = -self.beta          # b(α+π, -β) = -b(α, β)
                self.loop_sign_a, self.loop_sign_b = +1.0, -1.0
            else:
                self.beta += math.pi
                self.loop_sign_a, self.loop_sign_b = +1.0, +1.0
            print("[FLIP] Çiftler yer değiştiriyor; aynı merkezde ikinci yakınsama.")
            self._reset_dwell(sim_now)
            return
        if FLIP_PAIRS:
            v = self._axial_mean([self.flip_first_dir, v])
            # yön değişmeden normal yerleşime dön
            self.alpha -= math.pi
            if VERTICAL_PLANE == "aligned":
                self.beta = -self.beta
            else:
                self.beta -= math.pi
            self.flip_phase = 0
            self.loop_sign_a = self.loop_sign_b = -1.0

        # Doğru, AÇILARIN YAKINSADIĞI merkezde kaydedilir (v1 hatası düzeltildi)
        self.direction_lines.append({
            'center': (self.center_x, self.center_y, self.center_z),
            'direction': v,
            'alpha_deg': math.degrees(self.alpha),
            'beta_deg': math.degrees(self.beta),
        })
        print("*" * 55)
        print(f"[DOĞRU #{len(self.direction_lines)}] C=({self.center_x:.1f}, {self.center_y:.1f}, "
              f"{self.center_z:.1f}) yön=({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f})")

        if CENTER_PLAN == "baseline" and len(self.plan) == 1:
            h = math.hypot(v[0], v[1])
            px, py = ((-v[1] / h, v[0] / h) if h > 1e-6 else (1.0, 0.0))
            c0 = self.plan[0]
            self.plan += [(c0[0] + BASELINE_M * px, c0[1] + BASELINE_M * py),
                          (c0[0] - BASELINE_M * px, c0[1] - BASELINE_M * py)]
        self.plan_index += 1
        est = self.compute_target_least_squares()
        if est is not None:
            print(f"[HEDEF ARA KESTİRİM] Doğu={est[0]:.1f} m, Kuzey={est[1]:.1f} m, z={est[2]:.1f} m")
        if self.plan_index >= len(self.plan):
            print("[GÖREV] Tüm merkezler tamamlandı. Dronlar son konumda bekliyor (DONE).")
            self.state = self.STATE_DONE
        else:
            nx, ny = self.plan[self.plan_index]
            nx = max(min(nx, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
            ny = max(min(ny, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
            print(f"[PLAN] Sonraki merkez: Doğu={nx:.1f} m, Kuzey={ny:.1f} m")
            self.center_x, self.center_y = nx, ny
            self.state = self.STATE_SETUP
            self._setup_start = time.time()
        print("*" * 55)
        self._reset_dwell(sim_now)

    def _reset_dwell(self, sim_now):
        self.angle_controller_alpha.reset()
        self.angle_controller_beta.reset()
        self.e_hist, self.ab_hist = [], []
        self.dwell_start_t = sim_now

    def check_formation_reached(self):
        """
        Dronların fiziksel olarak atanmış hedeflerine (slotlarına)
        ulaşıp ulaşmadığını kontrol eder.
        """
        if self.origin_lat is None or self.origin_lon is None:
            return False

        positions = self.compute_drone_positions()
        if self.simulation and SIM_WORLD is not None:
            for drone_id, (north_m, east_m, alt_m) in positions.items():
                p = SIM_WORLD.position(drone_id)
                if math.sqrt((p[0] - east_m) ** 2 + (p[1] - north_m) ** 2 + (p[2] - alt_m) ** 2) > 1.0:
                    return False
            return True
        TOLERANCE_M = 3.5  # Slot'a ulaşma toleransı (m) - GPS gürültüsü düşünülerek esnetildi
        
        all_reached = True
        for drone_id, (north_m, east_m, alt_m) in positions.items():
            link = self.drone_links.get(drone_id)
            if not link or not link.armed or link.lat is None:
                all_reached = False
                break
            
            target_lat, target_lon = offset_latlon(
                self.origin_lat, self.origin_lon, north_m, east_m
            )
            
            # 3D mesafe hesabı (haversine + irtifa)
            dlat = (link.lat - target_lat) * math.pi / 180.0
            dlon = (link.lon - target_lon) * math.pi / 180.0
            a = math.sin(dlat/2)**2 + math.cos(target_lat * math.pi / 180.0) * \
                math.cos(link.lat * math.pi / 180.0) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            dist_xy = EARTH_RADIUS_M * c
            dist_z = alt_m - link.relative_alt_m
            
            dist_3d = math.sqrt(dist_xy**2 + dist_z**2)
            
            if dist_3d > TOLERANCE_M:
                print(f"[SETUP] Drone {drone_id} hedefine uzağında: {dist_3d:.1f}m (Tolerans: {TOLERANCE_M}m)")
                all_reached = False
                
        return all_reached

    # ─────────────────────────────────────────────
    # ANA DÖNGÜ
    # ─────────────────────────────────────────────
    def _now(self):
        """Görev zamanı (s). Simülasyonda hızlandırılmış sanal zaman."""
        if self.simulation and SIM_WORLD is not None:
            return SIM_WORLD.sim_time()
        return time.time()

    def run(self):
        print("\n[BİLGİ] Küre Formasyonu v2 döngüsü başlıyor. Ctrl+C ile durdurun.")
        dt = 1.0 / COMMAND_LOOP_HZ  # 5 saniye (görev zamanı)
        scale = SIM_TIME_SCALE if self.simulation else 1.0
        self._setup_start = time.time()
        SETUP_TIMEOUT_S = 120

        while self.state != self.STATE_DONE:
            loop_start = time.monotonic()
            rssi = self.get_windowed_rssi()
            print("\n" + "=" * 55)
            print(f"[DURUM] {self.state} | Merkez: E:{self.center_x:.1f}m N:{self.center_y:.1f}m "
                  f"ALT:{self.center_z:.1f}m | t={self._now() - self._mission_t0 if hasattr(self, '_mission_t0') else 0:.0f}s")

            if self.state == self.STATE_SETUP:
                self.send_mavlink_commands()
                if self.check_formation_reached():
                    print("[SETUP] Formasyon tamam -> ORBIT")
                    self.state = self.STATE_ORBIT
                    self._reset_dwell(self._now())
                    if not hasattr(self, "_mission_t0"):
                        self._mission_t0 = self._now()
                elif (time.time() - self._setup_start) * scale > SETUP_TIMEOUT_S:
                    print(f"[UYARI] SETUP {SETUP_TIMEOUT_S}s aşıldı; ORBIT'e geçiliyor.")
                    self.state = self.STATE_ORBIT
                    self._reset_dwell(self._now())
                    if not hasattr(self, "_mission_t0"):
                        self._mission_t0 = self._now()

            elif self.state == self.STATE_ORBIT:
                e1, e2 = self.update_rotation_angles(rssi, dt)
                self.send_mavlink_commands()
                if e1 is not None and self.is_converged(self._now()):
                    self.finish_center(self._now())

            print("=" * 55 + "\n")
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, dt / scale - elapsed))
        self.report()

    def compute_target_least_squares(self):
        """Doğruların en küçük kareler kesişimi (IJSS 2026 Denk. 8).
        GROUND_PRIOR: hedef yerde (z = 0, kalkış noktası seviyesi) varsayılır."""
        if len(self.direction_lines) < 2:
            return None
        import numpy as np
        M = np.zeros((3, 3))
        b = np.zeros(3)
        for dl in self.direction_lines:
            c = np.array(dl['center'], float)
            v = np.array(dl['direction'], float)
            v = v / np.linalg.norm(v)
            P = np.eye(3) - np.outer(v, v)
            M += P
            b += P @ c
        try:
            if GROUND_PRIOR:
                xy = np.linalg.lstsq(M[:2, :2], b[:2], rcond=None)[0]
                return np.array([xy[0], xy[1], 0.0])
            return np.linalg.lstsq(M, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None

    def report(self):
        print("\n" + "=" * 55)
        print("[RAPOR] Hesaplanan hedef doğruları:")
        for i, dl in enumerate(self.direction_lines, 1):
            c, d = dl['center'], dl['direction']
            print(f"  #{i}: C=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) Yön=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})")
        target = self.compute_target_least_squares()
        if target is not None:
            print(f"[HEDEF] Doğu(X)={target[0]:.2f} m  Kuzey(Y)={target[1]:.2f} m  Z={target[2]:.2f} m")
            if self.simulation:
                err = math.hypot(target[0] - SIM_TARGET_ENU[0], target[1] - SIM_TARGET_ENU[1])
                print(f"[SİM] Gerçek hedef ENU={SIM_TARGET_ENU} -> yatay hata = {err:.2f} m")
        print("=" * 55)

    def shutdown(self):
        self._lora_stop = True
        for link in self.drone_links.values():
            link.stop()
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass

        # CSV dosyasını kapat
        if getattr(self, "csv_file", None):
            try:
                self.csv_file.flush()
                self.csv_file.close()
                print(f"[LOG KAYIT] Deney verileri başarıyla kapatıldı -> {self.csv_filename}")
            except Exception:
                pass

        # Son rapor
        if self.direction_lines and self.state != getattr(self, "STATE_DONE", "DONE"):
            self.report()
        if False:
            print("\n" + "=" * 55)
            print("[RAPOR] Hesaplanan hedef doğruları:")
            for i, dl in enumerate(self.direction_lines, 1):
                c = dl['center']
                d = dl['direction']
                print(f"  #{i}: C=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) "
                      f"Yön=({d[0]:.3f},{d[1]:.3f},{d[2]:.3f}) "
                      f"α={dl['alpha_deg']:.1f}° β={dl['beta_deg']:.1f}°")
                      
            target = self.compute_target_least_squares()
            if target is not None:
                print(f"\n[HEDEF] En Küçük Kareler Kestirimi (Least Squares):")
                print(f"        Doğu(X): {target[0]:.2f}m")
                print(f"        Kuzey(Y): {target[1]:.2f}m")
                print(f"        İrtifa(Z): {target[2]:.2f}m")
            print("=" * 55)
