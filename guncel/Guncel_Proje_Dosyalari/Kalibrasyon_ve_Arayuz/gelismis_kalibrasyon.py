"""
gelismis_kalibrasyon.py

TÜBİTAK 123E294 - Gelişmiş RSSI Kalibrasyon ve Konum Modeli Aracı
===================================================================
13 farklı bilinen noktadan RSSI verisi toplayarak:
  1. Drone donanım ofsetlerini (CALIBRATION_OFFSET_DB)
  2. Ortam yayılım katsayısını (path loss exponent, n)
  3. Referans RSSI değerini (RSSI_0, 1m mesafedeki sinyal gücü)
hesaplar.

Bu parametrelerle hem Radar GUI hem de yer istasyonu karar motoru
hedefin yönünü VE yaklaşık mesafesini doğru hesaplayabilir.

ÖLÇÜM NOKTALARI (13 adet):
  1. MERKEZ         → tüm dronlara 5m
  2-5. Drone arkası → D1,D2,D3,D4'ün hemen arkasında (~1m)
  6-9. 5m ötesi     → Her dronun 5m arkasında (merkezden 10m)
  10-13. 10m ötesi  → Her dronun 10m arkasında (merkezden 15m)

KURULUM:
  D1=Kuzey(0,+5), D2=Güney(0,-5), D3=Doğu(+5,0), D4=Batı(-5,0)
  Dronlar arası 10m, merkeze 5m.
"""

import time
import sys
import math
import threading
import json
import os

try:
    import serial
except ImportError:
    print("pyserial kütüphanesi eksik. 'pip install pyserial'")
    sys.exit(1)

LORA_BAUD = 9600
SAMPLE_TIME_S = 15.0   # Her nokta için veri toplama süresi

# Drone pozisyonları (metre, origin = merkez)
DRONE_POS = {
    1: (0.0,  5.0),   # Kuzey
    2: (0.0, -5.0),   # Güney
    3: (5.0,  0.0),   # Doğu
    4: (-5.0, 0.0),   # Batı
}

# 13 ölçüm noktasının tanımları: (isim, açıklama, x, y)
MEASUREMENT_POINTS = [
    ("MERKEZ",        "Vericiyi dronların TAM ORTASINA koyun (her drona 5m).",
     0.0, 0.0),

    ("D1 ARKASI",     "Vericiyi D1'in (Kuzey) hemen arkasına koyun (~1m dışarıda, merkezden 6m kuzeyde).",
     0.0, 6.0),
    ("D2 ARKASI",     "Vericiyi D2'nin (Güney) hemen arkasına koyun (~1m dışarıda, merkezden 6m güneyde).",
     0.0, -6.0),
    ("D3 ARKASI",     "Vericiyi D3'ün (Doğu) hemen arkasına koyun (~1m dışarıda, merkezden 6m doğuda).",
     6.0, 0.0),
    ("D4 ARKASI",     "Vericiyi D4'ün (Batı) hemen arkasına koyun (~1m dışarıda, merkezden 6m batıda).",
     -6.0, 0.0),

    ("D1 5M ÖTESİ",   "Vericiyi D1'in 5m arkasına koyun (merkezden 10m kuzeyde).",
     0.0, 10.0),
    ("D2 5M ÖTESİ",   "Vericiyi D2'nin 5m arkasına koyun (merkezden 10m güneyde).",
     0.0, -10.0),
    ("D3 5M ÖTESİ",   "Vericiyi D3'ün 5m arkasına koyun (merkezden 10m doğuda).",
     10.0, 0.0),
    ("D4 5M ÖTESİ",   "Vericiyi D4'ün 5m arkasına koyun (merkezden 10m batıda).",
     -10.0, 0.0),

    ("D1 10M ÖTESİ",  "Vericiyi D1'in 10m arkasına koyun (merkezden 15m kuzeyde).",
     0.0, 15.0),
    ("D2 10M ÖTESİ",  "Vericiyi D2'nin 10m arkasına koyun (merkezden 15m güneyde).",
     0.0, -15.0),
    ("D3 10M ÖTESİ",  "Vericiyi D3'ün 10m arkasına koyun (merkezden 15m doğuda).",
     15.0, 0.0),
    ("D4 10M ÖTESİ",  "Vericiyi D4'ün 10m arkasına koyun (merkezden 15m batıda).",
     -15.0, 0.0),
]


def compute_checksum(payload):
    return sum(ord(c) for c in payload) % 256


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class AdvancedCalibration:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.is_running = False
        self.collecting = False
        self.current_samples = {1: [], 2: [], 3: [], 4: []}

        # Her ölçüm noktasının sonuçları: {isim: {drone_id: median_rssi}}
        self.results = {}

        # Hesaplanan parametreler
        self.offsets = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        self.path_loss_n = 2.0      # Başlangıç tahmini
        self.rssi_ref_1m = -40.0    # 1m referans RSSI

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, LORA_BAUD, timeout=0.1)
            print(f"  [OK] {self.port} bağlandı.\n")
            self.is_running = True
            threading.Thread(target=self._reader, daemon=True).start()
        except Exception as e:
            print(f"  [HATA] {self.port}: {e}")
            sys.exit(1)

    def _reader(self):
        buf = ""
        while self.is_running:
            try:
                data = self.ser.read(256)
                if data:
                    buf += data.decode(errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line and self.collecting:
                            self._parse(line)
            except Exception:
                pass
            time.sleep(0.01)

    def _parse(self, line):
        try:
            if not line.startswith("N"):
                return
            parts = line[1:].split(",")
            if len(parts) < 2:
                return
            did = int(parts[0])
            rssi = int(parts[1])
            if len(parts) == 3:
                chk = int(parts[2])
                body = f"N{did},{rssi}"
                if chk != compute_checksum(body):
                    return
            if did in (1, 2, 3, 4):
                self.current_samples[did].append(rssi)
        except (ValueError, IndexError):
            pass

    def collect_point(self, name, description):
        print(f"  {'━' * 55}")
        print(f"  📍 {name}")
        print(f"  {'━' * 55}")
        print(f"  {description}")
        print()
        input("  Hazır olduğunuzda ENTER'a basın...")

        self.current_samples = {1: [], 2: [], 3: [], 4: []}
        self.collecting = True

        start = time.time()
        while time.time() - start < SAMPLE_TIME_S:
            remaining = SAMPLE_TIME_S - (time.time() - start)
            counts = " ".join(f"D{d}:{len(self.current_samples[d])}" for d in [1, 2, 3, 4])
            sys.stdout.write(f"\r  ⏱ {remaining:.0f}s [{counts}]   ")
            sys.stdout.flush()
            time.sleep(0.3)

        self.collecting = False
        print()

        medians = {}
        for d in [1, 2, 3, 4]:
            s = self.current_samples[d]
            if not s:
                print(f"  ⚠️  D{d}: VERİ YOK!")
                medians[d] = None
            else:
                s_sorted = sorted(s)
                n = len(s_sorted)
                med = s_sorted[n // 2] if n % 2 else (s_sorted[n // 2 - 1] + s_sorted[n // 2]) / 2.0
                avg = sum(s) / n
                print(f"  D{d}: Medyan={med:.1f} | Ort={avg:.1f} | {n} paket")
                medians[d] = med

        self.results[name] = medians
        print()

    # ─────────────────────────────────────────────
    # HESAPLAMALAR
    # ─────────────────────────────────────────────
    def calculate_all(self):
        print("\n" + "=" * 55)
        print("  📊 HESAPLAMALAR")
        print("=" * 55)

        # ── 1) Donanım Ofsetleri (Merkez ölçümünden) ──
        center = self.results.get("MERKEZ")
        if not center:
            print("  Merkez verisi yok!")
            return False

        valid = {d: v for d, v in center.items() if v is not None}
        if len(valid) < 3:
            print("  Yetersiz merkez verisi!")
            return False

        ref = sum(valid.values()) / len(valid)
        for d in [1, 2, 3, 4]:
            self.offsets[d] = round(ref - center[d], 2) if center[d] is not None else 0.0

        print(f"\n  [1/3] DONANIM OFSETLERİ (merkez ölçümünden):")
        print(f"  Referans RSSI: {ref:.1f} dBm")
        for d in [1, 2, 3, 4]:
            print(f"    D{d}: ham={center[d]:.1f} → ofset={self.offsets[d]:+.2f} dBm")

        # ── 2) Path Loss Modeli ──
        # Tüm ölçüm noktalarından (mesafe, kalibreli_rssi) çiftleri topla
        print(f"\n  [2/3] YAYILIM MODELİ (path loss):")
        print(f"  RSSI = RSSI₀ - 10·n·log₁₀(d)")

        data_points = []  # (true_distance_m, calibrated_rssi)

        for point_name, point_desc, px, py in MEASUREMENT_POINTS:
            meas = self.results.get(point_name)
            if not meas:
                continue
            for d in [1, 2, 3, 4]:
                if meas[d] is None:
                    continue
                dx, dy = DRONE_POS[d]
                dist = distance(px, py, dx, dy)
                if dist < 0.5:
                    dist = 0.5  # Sıfıra çok yakın mesafeler için alt sınır
                cal_rssi = meas[d] + self.offsets[d]
                data_points.append((dist, cal_rssi))

        if len(data_points) < 5:
            print("  Yetersiz veri noktası!")
            self.path_loss_n = 2.0
            self.rssi_ref_1m = ref
            return True

        # En küçük kareler ile n ve RSSI_0 fit et
        # RSSI = RSSI_0 - 10*n*log10(d)
        # y = a + b*x  where y=RSSI, x=log10(d), a=RSSI_0, b=-10*n
        sum_x = 0.0
        sum_y = 0.0
        sum_xx = 0.0
        sum_xy = 0.0
        N = len(data_points)

        for dist, rssi in data_points:
            x = math.log10(dist)
            y = rssi
            sum_x += x
            sum_y += y
            sum_xx += x * x
            sum_xy += x * y

        denom = N * sum_xx - sum_x * sum_x
        if abs(denom) < 1e-10:
            print("  Fit edilemedi (dejenere veri).")
            self.path_loss_n = 2.0
            self.rssi_ref_1m = ref
            return True

        b = (N * sum_xy - sum_x * sum_y) / denom
        a = (sum_y - b * sum_x) / N

        self.rssi_ref_1m = round(a, 1)
        self.path_loss_n = round(-b / 10.0, 2)

        # n'yi makul aralıkta tut (1.5-5.0)
        if self.path_loss_n < 1.5:
            self.path_loss_n = 1.5
        elif self.path_loss_n > 5.0:
            self.path_loss_n = 5.0

        print(f"  Kullanılan veri noktası: {N}")
        print(f"  ┌──────────────────────────────────────────┐")
        print(f"  │ RSSI₀ (1m referans) = {self.rssi_ref_1m:>6.1f} dBm       │")
        print(f"  │ n (yayılım katsayısı) = {self.path_loss_n:>5.2f}           │")
        print(f"  │                                          │")
        print(f"  │ Formül: RSSI = {self.rssi_ref_1m:.0f} - {10*self.path_loss_n:.0f}·log₁₀(d)    │")
        print(f"  └──────────────────────────────────────────┘")

        # n'nin anlamı
        if self.path_loss_n < 2.0:
            print(f"  Not: n={self.path_loss_n:.2f} < 2.0 → Yönlü anten etkisi veya yansıma kazancı olabilir.")
        elif self.path_loss_n < 2.5:
            print(f"  Not: n={self.path_loss_n:.2f} → Açık alan, serbest yayılıma yakın. Normal.")
        elif self.path_loss_n < 3.5:
            print(f"  Not: n={self.path_loss_n:.2f} → Hafif engellenmiş ortam (çim, ağaç, araç). Beklenen.")
        else:
            print(f"  Not: n={self.path_loss_n:.2f} → Çok engellenmiş ortam. Dikkat!")

        # Fit kalitesi kontrolü: her noktadaki hata
        print(f"\n  Fit kalitesi (her ölçüm noktasındaki hata):")
        errors = []
        for dist, rssi in data_points:
            predicted = self.rssi_ref_1m - 10 * self.path_loss_n * math.log10(dist)
            err = rssi - predicted
            errors.append(err)

        mean_err = sum(errors) / len(errors)
        rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))
        print(f"  Ortalama hata: {mean_err:+.1f} dBm | RMSE: {rmse:.1f} dBm")
        if rmse < 3.0:
            print(f"  ✅ Model kalitesi: MÜKEMMEL")
        elif rmse < 6.0:
            print(f"  ⚠️  Model kalitesi: KABUL EDİLEBİLİR")
        else:
            print(f"  🔴 Model kalitesi: ZAYIF (ortam çok gürültülü veya engeller var)")

        # ── 3) Yön Doğruluğu ──
        print(f"\n  [3/3] YÖN DOĞRULUĞU TESTİ:")
        direction_tests = [
            ("D1 ARKASI",    "Kuzey",     1, 2),
            ("D2 ARKASI",    "Güney",     2, 1),
            ("D3 ARKASI",    "Doğu",      3, 4),
            ("D4 ARKASI",    "Batı",      4, 3),
            ("D1 5M ÖTESİ",  "Kuzey (5m)", 1, 2),
            ("D2 5M ÖTESİ",  "Güney (5m)", 2, 1),
            ("D3 5M ÖTESİ",  "Doğu (5m)",  3, 4),
            ("D4 5M ÖTESİ",  "Batı (5m)",  4, 3),
            ("D1 10M ÖTESİ", "Kuzey (10m)", 1, 2),
            ("D2 10M ÖTESİ", "Güney (10m)", 2, 1),
            ("D3 10M ÖTESİ", "Doğu (10m)",  3, 4),
            ("D4 10M ÖTESİ", "Batı (10m)",  4, 3),
        ]

        ok = 0
        total = 0
        for point_name, label, near_d, far_d in direction_tests:
            meas = self.results.get(point_name)
            if not meas or meas[near_d] is None or meas[far_d] is None:
                continue
            total += 1
            cal_near = meas[near_d] + self.offsets[near_d]
            cal_far = meas[far_d] + self.offsets[far_d]
            if cal_near > cal_far:
                print(f"  ✅ {label:>15s}: D{near_d}={cal_near:.0f} > D{far_d}={cal_far:.0f} "
                      f"(fark={cal_near - cal_far:.0f}dB) DOĞRU")
                ok += 1
            else:
                print(f"  ❌ {label:>15s}: D{near_d}={cal_near:.0f} ≤ D{far_d}={cal_far:.0f} YANLIŞ!")

        if total > 0:
            pct = ok / total * 100
            print(f"\n  Yön doğruluğu: {ok}/{total} ({pct:.0f}%)")
            if pct == 100:
                print(f"  🎯 TÜM YÖNLER DOĞRU!")
            elif pct >= 75:
                print(f"  ⚠️  Bazı yönlerde sorun var.")
            else:
                print(f"  🔴 Ciddi sorunlar var, antenler kontrol edilmeli!")

        # ── Mesafe tahmin örnekleri ──
        print(f"\n  📏 MESAFE TAHMİNİ ÖRNEKLERİ:")
        print(f"  (Gerçek mesafe vs. model tahmini)")
        example_points = [
            ("MERKEZ",       5.0),     # merkez → herhangi drone'a 5m
            ("D1 ARKASI",    1.0),     # D1 arkasında → D1'e ~1m
            ("D1 5M ÖTESİ",  10.0),   # → D1'e 5m, ama merkezden 10m
            ("D1 10M ÖTESİ", 15.0),   # → D1'e 10m, merkezden 15m
        ]
        for point_name, approx_near_dist in example_points:
            meas = self.results.get(point_name)
            if not meas:
                continue
            for d in [1, 2, 3, 4]:
                if meas[d] is None:
                    continue
                cal = meas[d] + self.offsets[d]
                est_dist = self.rssi_to_distance(cal)
                true_dist = distance(
                    dict(MEASUREMENT_POINTS)[point_name] if False else 0, 0,
                    *DRONE_POS[d]
                )
                # Gerçek mesafeyi hesapla
                for mp_name, mp_desc, mp_x, mp_y in MEASUREMENT_POINTS:
                    if mp_name == point_name:
                        dx, dy = DRONE_POS[d]
                        true_dist = distance(mp_x, mp_y, dx, dy)
                        break
                print(f"    {point_name:>15s} → D{d}: "
                      f"gerçek={true_dist:.1f}m, tahmin={est_dist:.1f}m, "
                      f"hata={abs(true_dist - est_dist):.1f}m")

        return True

    def rssi_to_distance(self, rssi):
        """Kalibreli RSSI → tahmini mesafe (metre)."""
        if rssi >= self.rssi_ref_1m:
            return 0.5
        exponent = (self.rssi_ref_1m - rssi) / (10.0 * self.path_loss_n)
        d = 10.0 ** exponent
        return min(d, 200.0)  # 200m üst sınır

    def print_final_output(self):
        print("\n" + "╔" + "═" * 55 + "╗")
        print("║    KALİBRASYON SONUÇLARI — KOD İÇİN KOPYALAYIN      ║")
        print("╠" + "═" * 55 + "╣")
        print("║                                                       ║")
        print(f"║  CALIBRATION_OFFSET_DB = {{                            ║")
        print(f"║      1: {self.offsets[1]:>+6.2f},  # Kuzey (D1)                 ║")
        print(f"║      2: {self.offsets[2]:>+6.2f},  # Güney (D2)                 ║")
        print(f"║      3: {self.offsets[3]:>+6.2f},  # Doğu  (D3)                 ║")
        print(f"║      4: {self.offsets[4]:>+6.2f},  # Batı  (D4)                 ║")
        print(f"║  }}                                                    ║")
        print("║                                                       ║")
        print(f"║  PATH_LOSS_N = {self.path_loss_n:<5.2f}  # Ortam yayılım katsayısı ║")
        print(f"║  RSSI_REF_1M = {self.rssi_ref_1m:<5.1f}  # 1m referans RSSI (dBm) ║")
        print("║                                                       ║")
        print("╚" + "═" * 55 + "╝")

        # JSON olarak da kaydet
        output = {
            "calibration_offsets": self.offsets,
            "path_loss_n": self.path_loss_n,
            "rssi_ref_1m": self.rssi_ref_1m,
            "raw_data": {}
        }
        for name, medians in self.results.items():
            output["raw_data"][name] = {
                str(d): v for d, v in medians.items()
            }

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "kalibrasyon_sonuclari.json")
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(output, f, indent=2, ensure_ascii=False)
            print(f"\n  💾 Ham veriler kaydedildi: {save_path}")
        except Exception as e:
            print(f"\n  ⚠️  Kayıt hatası: {e}")

    def shutdown(self):
        self.is_running = False
        if self.ser:
            self.ser.close()

    # ─────────────────────────────────────────────
    # CANLI TRİLATERASYON TESTİ
    # ─────────────────────────────────────────────
    def run_live_test(self):
        print("\n" + "=" * 55)
        print("  🔴 CANLI TRİLATERASYON TESTİ")
        print("=" * 55)
        print("  Vericiyi istediğiniz yere taşıyın.")
        print("  Kalibreli RSSI → Mesafe → Konum tahmin edilecek.")
        print("  Ctrl+C ile çıkın.\n")

        self.collecting = True  # Sürekli veri al

        try:
            while True:
                time.sleep(2.0)
                # Son verileri al
                cal_rssi = {}
                for d in [1, 2, 3, 4]:
                    s = self.current_samples[d]
                    if s:
                        # Son 10 paketin medyanı
                        recent = s[-10:]
                        recent_sorted = sorted(recent)
                        n = len(recent_sorted)
                        med = recent_sorted[n // 2]
                        cal_rssi[d] = med + self.offsets[d]

                if len(cal_rssi) < 2:
                    sys.stdout.write(f"\r  ⏳ Veri bekleniyor ({len(cal_rssi)}/4)...   ")
                    sys.stdout.flush()
                    continue

                # Mesafe tahminleri
                distances = {}
                for d in [1, 2, 3, 4]:
                    if d in cal_rssi:
                        distances[d] = self.rssi_to_distance(cal_rssi[d])

                # Ağırlıklı trilaterasyon
                # w_i = 1/d_i^2 (yakın drone'a daha çok güven)
                est_x = 0.0
                est_y = 0.0
                total_w = 0.0
                for d, est_d in distances.items():
                    dx, dy = DRONE_POS[d]
                    w = 1.0 / (est_d ** 2 + 0.01)
                    # Hedef, drone'dan est_d uzakta.
                    # Ağırlıklı olarak drone pozisyonlarından
                    # hedefin yönüne kayma yapıyoruz.
                    total_w += w

                # Basit ağırlıklı yön hesabı (güç farkına dayalı)
                if 1 in cal_rssi and 2 in cal_rssi:
                    delta_ns = cal_rssi[1] - cal_rssi[2]
                else:
                    delta_ns = 0.0
                if 3 in cal_rssi and 4 in cal_rssi:
                    delta_ew = cal_rssi[3] - cal_rssi[4]
                else:
                    delta_ew = 0.0

                # Yön (derece)
                angle_deg = 0.0
                mag = math.sqrt(delta_ns ** 2 + delta_ew ** 2)
                if mag > 1.0:
                    angle_deg = math.degrees(math.atan2(delta_ew, delta_ns)) % 360

                # Mesafe tahmini: en yakın drone'un mesafesi × yön
                if distances:
                    nearest_d = min(distances, key=distances.get)
                    nearest_dist = distances[nearest_d]
                else:
                    nearest_d = 1
                    nearest_dist = 0

                # Yön ismi
                dirs = [
                    (0, "⬆️  KUZEY"), (45, "↗️  KD"), (90, "➡️  DOĞU"),
                    (135, "↘️  GD"), (180, "⬇️  GÜNEY"), (225, "↙️  GB"),
                    (270, "⬅️  BATI"), (315, "↖️  KB"),
                ]
                dir_name = "⭕ MERKEZ"
                if mag > 2.0:
                    for center_a, name in dirs:
                        diff = abs(angle_deg - center_a)
                        if diff > 180:
                            diff = 360 - diff
                        if diff <= 22.5:
                            dir_name = name
                            break

                # Ekrana yaz
                print("\033[2J\033[H")
                print("  🔴 CANLI TEST (Ctrl+C = çıkış)")
                print("  " + "─" * 50)
                for d in [1, 2, 3, 4]:
                    if d in cal_rssi:
                        r = cal_rssi[d]
                        dist = distances.get(d, 0)
                        bar_len = max(0, min(30, int((r + 100) / 2)))
                        bar = "█" * bar_len + "░" * (30 - bar_len)
                        print(f"  D{d}: [{bar}] {r:.0f}dBm → {dist:.1f}m")
                    else:
                        print(f"  D{d}: [{'░' * 30}]  N/A")
                print()
                print(f"  ΔK-G: {delta_ns:+.0f}dBm | ΔD-B: {delta_ew:+.0f}dBm")
                print(f"  🧭 Yön: {dir_name} ({angle_deg:.0f}°)")
                print(f"  📏 En yakın: D{nearest_d} → {nearest_dist:.1f}m")
                print(f"  Model: RSSI₀={self.rssi_ref_1m:.0f}, n={self.path_loss_n:.2f}")
                print("  " + "─" * 50)

        except KeyboardInterrupt:
            self.collecting = False
            print("\n\n  Canlı test sonlandırıldı.")


# ==========================================
# ANA PROGRAM
# ==========================================
if __name__ == "__main__":
    print()
    print("╔" + "═" * 55 + "╗")
    print("║  🛰️  GELİŞMİŞ KALİBRASYON & KONUM MODELİ ARACI  🛰️  ║")
    print("║  TÜBİTAK 123E294 — 13 Nokta Kalibrasyonu              ║")
    print("╚" + "═" * 55 + "╝")
    print()
    print("  KURULUM:")
    print("  Dronları artı (+) şeklinde, karşılıklı 10m mesafeye koyun.")
    print()
    print("                D1 (Kuzey)")
    print("                  ↑ 5m")
    print("   D4 (Batı) ← MERKEZ → D3 (Doğu)")
    print("           5m     ↓ 5m     5m")
    print("                D2 (Güney)")
    print()
    print("  13 noktada ölçüm alınacak:")
    print("  • Merkez (1)")
    print("  • Her dronun hemen arkası (4)")
    print("  • Her dronun 5m ötesi (4)")
    print("  • Her dronun 10m ötesi (4)")
    print(f"  Her nokta {SAMPLE_TIME_S:.0f} sn → Toplam ~{13 * SAMPLE_TIME_S / 60:.0f} dakika")
    print()

    import platform
    is_linux = platform.system() != "Windows"
    default_port = "/dev/ttyUSB0" if is_linux else "COM5"

    port = input(f"  LoRa portu [{default_port}]: ").strip()
    if not port:
        port = default_port
    if is_linux and not port.startswith("/"):
        port = "/dev/" + port

    tool = AdvancedCalibration(port)
    tool.connect()

    try:
        for name, desc, px, py in MEASUREMENT_POINTS:
            tool.collect_point(name, desc)

        success = tool.calculate_all()

        if success:
            tool.print_final_output()
            print()
            choice = input("  Canlı trilaterasyon testine geçmek ister misiniz? [E/h]: ").strip().lower()
            if choice != "h":
                tool.run_live_test()

    except KeyboardInterrupt:
        print("\n\n  İşlem iptal edildi.")
    finally:
        tool.shutdown()
        print("  Program sonlandırıldı. İyi uçuşlar! ✈️")
