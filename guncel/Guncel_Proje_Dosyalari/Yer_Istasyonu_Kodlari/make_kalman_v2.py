"""Builds ground_station_core_kalman_v2.py (Scenario I, '+' formation) from the repo file.
Explicit replacements only; stops if an anchor is missing."""
import sys

SRC, DST = sys.argv[1], sys.argv[2]
s = open(SRC, encoding="utf-8").read()


def rep(old, new):
    global s
    n = s.count(old)
    if n != 1:
        raise SystemExit(f"anchor found {n}x:\n{old[:160]}")
    s = s.replace(old, new)


# 1. header note
first_doc_end = s.index('"""', 3)
s = s[:first_doc_end] + '''
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
''' + s[first_doc_end:]

# 2. switches
rep('''USE_KALMAN = True  # True ise Kalman Filtresi devrede
USE_SISMF  = True   # True ise SCI_4 (2026) Smoother Integral Sliding Mode Filter devrede''',
    '''# v2: filtre ve kontrolcü ayrı seçilir (aşağıdaki eski USE_* bayrakları bunlardan türetilir)
SIGNAL_FILTER   = "KALMAN"   # "KALMAN" | "SISMF" | "KALMAN+SISMF" | "HAM"
AXIS_CONTROLLER = "P"        # "P" (Rapor 2, K=0.5 m/dB) | "SISMF" (IJSS 2026 eksen kontrolcüsü)
USE_KALMAN = SIGNAL_FILTER in ("KALMAN", "KALMAN+SISMF")
USE_SISMF  = SIGNAL_FILTER in ("SISMF", "KALMAN+SISMF")''')

# 3. calibration
rep('''CALIBRATION_OFFSET_DB = {
    1: -2.1,   # Drone 1 (Kuzey) diğerlerinden ~2dB güçlü okuyordu -> düşürülüyor
    2: 1.9,    # Drone 2 (Güney) diğerlerinden ~2dB zayıf okuyordu -> yükseltiliyor
    3: -0.1,
    4: 0.2,
}''', '''# v2: kalibrasyon_sonucu_saha.json (P_ref = -78.72 dBm, n = 1.264, RMSE = 2.29 dB)
CALIBRATION_OFFSET_DB = {
    1:  0.00,   # D1 Kuzey (referans)
    2: -1.32,   # D2 Güney
    3:  2.38,   # D3 Doğu
    4:  1.10,   # D4 Batı
}''')

# 4. controller selection uses AXIS_CONTROLLER
rep('''            if USE_SISMF:
                shift_y = self.axis_sismf_y.update_shift(delta_y_dbm)''',
    '''            if AXIS_CONTROLLER == "SISMF":
                shift_y = self.axis_sismf_y.update_shift(delta_y_dbm)''')
rep('''            if USE_SISMF:
                shift_x = self.axis_sismf_x.update_shift(delta_x_dbm)''',
    '''            if AXIS_CONTROLLER == "SISMF":
                shift_x = self.axis_sismf_x.update_shift(delta_x_dbm)''')

# 5. log line precedence bug
rep('''        mode_str = "SISMF (SCI_4)" if USE_SISMF else ("Kalman" if USE_KALMAN else "HAM")
        print(f"[KARAR MOTORU] [{mode_str}] Güç Farkları -> "
              f"K/G: {delta_y_dbm:.1f}dB" if delta_y_dbm is not None else "K/G: N/A", end="")
        print(f", D/B: {delta_x_dbm:.1f}dB" if delta_x_dbm is not None else ", D/B: N/A")''',
    '''        mode_str = f"filtre={SIGNAL_FILTER}, kontrolcü={AXIS_CONTROLLER}"
        ns_txt = f"K/G: {delta_y_dbm:.1f}dB" if delta_y_dbm is not None else "K/G: N/A"
        ew_txt = f"D/B: {delta_x_dbm:.1f}dB" if delta_x_dbm is not None else "D/B: N/A"
        print(f"[KARAR MOTORU] [{mode_str}] Güç Farkları -> {ns_txt}, {ew_txt}")''')

rep('''              f"USE_SISMF={USE_SISMF} | USE_KALMAN={USE_KALMAN}")''',
    '''              f"SIGNAL_FILTER={SIGNAL_FILTER} | AXIS_CONTROLLER={AXIS_CONTROLLER}")''')

open(DST, "w", encoding="utf-8").write(s)
print("written", DST)
