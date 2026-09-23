"""
otomatik_kalibrasyon.py

TÜBİTAK 123E294 Projesi - RSSI Kalibrasyon ve Doğrulama Aracı
==============================================================
Bu araç, dronların anten ve donanım kaynaklı RSSI sapmalarını ölçer,
kalibrasyon ofsetlerini hesaplar ve ardından CANLI TEST ile sonucu
doğrulamanızı sağlar.

KULLANIM:
1. 4 dronu artı (+) şeklinde, karşılıklı 10'ar metre mesafeye dizin.
   D1=Kuzey, D2=Güney, D3=Doğu, D4=Batı. Merkez tüm dronlara 5m.
2. Bu programı çalıştırın: python otomatik_kalibrasyon.py
3. Ekrandaki 5 aşamayı takip edin (Merkez → D1 → D2 → D3 → D4).
4. Kalibrasyon bittikten sonra canlı test moduna geçecek —
   vericiyi istediğiniz yere taşıyıp sistemin doğru yönü
   gösterip göstermediğini anında test edin.

ANTEN NOTU:
SDR antenlerini çıkardıysanız sorun yok. Antensiz SDR yakın mesafede
daha hassas yön ayrımı verir. Önemli olan 4'ünün de aynı koşulda
olmasıdır (hepsi antensiz veya hepsi antenli).
"""

import time
import sys
import math
import threading
try:
    import serial
except ImportError:
    print("pyserial kütüphanesi eksik. 'pip install pyserial'")
    sys.exit(1)

# ==========================================
# AYARLAR
# ==========================================
LORA_BAUD = 9600
SAMPLE_TIME_S = 15.0   # Her aşama için veri toplama süresi (saniye)
LIVE_WINDOW_S = 3.0     # Canlı testte kaç saniyelik pencere kullanılsın


def compute_checksum(payload: str) -> int:
    return sum(ord(c) for c in payload) % 256


class CalibrationTool:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.is_running = False

        # Her aşamada toplanan ham veriler
        self.current_samples = {1: [], 2: [], 3: [], 4: []}
        self.collecting = False

        # Canlı test için sürekli akan veriler (timestamp ile)
        self.live_buffer = {1: [], 2: [], 3: [], 4: []}
        self.live_mode = False

        # Aşama sonuçları
        self.results = {}

        # Hesaplanan ofsetler
        self.offsets = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

    # ─────────────────────────────────────────────
    # BAĞLANTI
    # ─────────────────────────────────────────────
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, LORA_BAUD, timeout=0.1)
            print(f"[OK] {self.port} bağlandı.")
            self.is_running = True
            threading.Thread(target=self._reader_loop, daemon=True).start()
        except Exception as e:
            print(f"[HATA] {self.port} açılamadı: {e}")
            sys.exit(1)

    def _reader_loop(self):
        rx_buf = ""
        while self.is_running:
            try:
                data = self.ser.read(256)
                if data:
                    rx_buf += data.decode(errors="ignore")
                    while "\n" in rx_buf:
                        line, rx_buf = rx_buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._parse_line(line)
            except Exception:
                pass
            time.sleep(0.01)

    def _parse_line(self, line):
        try:
            if not line.startswith("N"):
                return
            parts = line[1:].split(",")
            if len(parts) != 3:
                return
            drone_id = int(parts[0])
            rssi = int(parts[1])
            chk = int(parts[2])

            body = f"N{drone_id},{rssi}"
            if chk != compute_checksum(body):
                return
            if drone_id not in (1, 2, 3, 4):
                return

            # Kalibrasyon aşaması verisi
            if self.collecting:
                self.current_samples[drone_id].append(rssi)

            # Canlı test verisi (zaman damgalı)
            if self.live_mode:
                now = time.time()
                self.live_buffer[drone_id].append((now, rssi))
                # Eski verileri temizle
                cutoff = now - LIVE_WINDOW_S * 2
                self.live_buffer[drone_id] = [
                    (t, v) for t, v in self.live_buffer[drone_id] if t > cutoff
                ]
        except (ValueError, IndexError):
            pass

    # ─────────────────────────────────────────────
    # KALİBRASYON AŞAMASI
    # ─────────────────────────────────────────────
    def run_phase(self, phase_name, description):
        print(f"\n{'━' * 55}")
        print(f"  📍 AŞAMA: {phase_name}")
        print(f"{'━' * 55}")
        print(f"  {description}")
        print()
        input("  Hazır olduğunuzda ENTER'a basın...")

        self.current_samples = {1: [], 2: [], 3: [], 4: []}
        self.collecting = True

        print(f"\n  Veri toplanıyor ({SAMPLE_TIME_S:.0f} saniye, vericiyi hareket ettirmeyin)...")

        start = time.time()
        while time.time() - start < SAMPLE_TIME_S:
            remaining = SAMPLE_TIME_S - (time.time() - start)
            counts = " | ".join(
                f"D{d}:{len(self.current_samples[d])}" for d in [1, 2, 3, 4]
            )
            sys.stdout.write(f"\r  ⏱ Kalan: {remaining:.0f}s  [{counts}]   ")
            sys.stdout.flush()
            time.sleep(0.3)

        self.collecting = False
        print("\n  ✅ Veri toplama tamamlandı.\n")

        averages = {}
        for d in [1, 2, 3, 4]:
            samples = self.current_samples[d]
            if not samples:
                print(f"  ⚠️  Drone {d}: VERİ YOK! LoRa bağlantısını kontrol edin.")
                averages[d] = None
            else:
                # Medyan kullan (aşırı değerlere karşı dayanıklı)
                sorted_s = sorted(samples)
                n = len(sorted_s)
                if n % 2 == 0:
                    median = (sorted_s[n // 2 - 1] + sorted_s[n // 2]) / 2.0
                else:
                    median = sorted_s[n // 2]
                avg = sum(samples) / len(samples)
                std = (sum((s - avg) ** 2 for s in samples) / len(samples)) ** 0.5
                averages[d] = median
                print(f"  Drone {d}: Medyan={median:.1f} dBm | "
                      f"Ort={avg:.1f} dBm | Std={std:.1f} | "
                      f"{len(samples)} paket")

        self.results[phase_name] = averages

    # ─────────────────────────────────────────────
    # OFSET HESAPLAMA
    # ─────────────────────────────────────────────
    def calculate_offsets(self):
        print("\n" + "=" * 55)
        print("  📊 KALİBRASYON ANALİZİ")
        print("=" * 55)

        center = self.results.get("MERKEZ")
        if not center:
            print("  Merkez verisi yok!")
            return False

        # Geçerli merkez verileri
        valid = {d: v for d, v in center.items() if v is not None}
        if len(valid) < 3:
            print("  Yeterli veri yok (en az 3 drone gerekli).")
            return False

        # Ortalama RSSI (referans seviye)
        ref_rssi = sum(valid.values()) / len(valid)

        print(f"\n  Referans RSSI (4 dronun ortalaması): {ref_rssi:.1f} dBm\n")

        # Her drone'un ofseti = referans - ölçülen
        # Pozitif ofset = drone zayıf okuyor, güçlendirmek lazım
        # Negatif ofset = drone güçlü okuyor, azaltmak lazım
        print("  ┌─────────┬───────────┬──────────┬─────────────────────────────────┐")
        print("  │ Drone   │ Ham RSSI  │ Ofset    │ Teşhis                          │")
        print("  ├─────────┼───────────┼──────────┼─────────────────────────────────┤")

        for d in [1, 2, 3, 4]:
            if center[d] is not None:
                offset = ref_rssi - center[d]
                self.offsets[d] = round(offset, 2)
                if abs(offset) < 1.0:
                    diag = "✅ Normal"
                elif abs(offset) < 3.0:
                    diag = "⚠️  Hafif sapma"
                elif offset > 0:
                    diag = "🔴 Zayıf alıcı (sağır)"
                else:
                    diag = "🔴 Aşırı hassas"
                print(f"  │ D{d:<6} │ {center[d]:>7.1f}   │ {offset:>+7.2f}  │ {diag:<31} │")
            else:
                self.offsets[d] = 0.0
                print(f"  │ D{d:<6} │   N/A     │  0.00    │ ❌ Veri yok                      │")

        print("  └─────────┴───────────┴──────────┴─────────────────────────────────┘")

        # ── Simetri Testi ──
        print("\n  🔄 SİMETRİ TESTİ (Kenar Ölçümleri):")

        # Kuzey'e koyunca: D1 en yakın, D2 en uzak, D3≈D4 (eşit)
        north = self.results.get("KUZEY (D1)")
        south = self.results.get("GÜNEY (D2)")
        east = self.results.get("DOĞU (D3)")
        west = self.results.get("BATI (D4)")

        if north and north[3] is not None and north[4] is not None:
            cal3 = north[3] + self.offsets[3]
            cal4 = north[4] + self.offsets[4]
            diff = abs(cal3 - cal4)
            status = "✅" if diff < 3.0 else "⚠️"
            print(f"  {status} Kuzey'de D3 vs D4 farkı: {diff:.1f} dBm "
                  f"(kalibreli D3={cal3:.1f}, D4={cal4:.1f})")

        if south and south[3] is not None and south[4] is not None:
            cal3 = south[3] + self.offsets[3]
            cal4 = south[4] + self.offsets[4]
            diff = abs(cal3 - cal4)
            status = "✅" if diff < 3.0 else "⚠️"
            print(f"  {status} Güney'de D3 vs D4 farkı: {diff:.1f} dBm "
                  f"(kalibreli D3={cal3:.1f}, D4={cal4:.1f})")

        if east and east[1] is not None and east[2] is not None:
            cal1 = east[1] + self.offsets[1]
            cal2 = east[2] + self.offsets[2]
            diff = abs(cal1 - cal2)
            status = "✅" if diff < 3.0 else "⚠️"
            print(f"  {status} Doğu'da D1 vs D2 farkı: {diff:.1f} dBm "
                  f"(kalibreli D1={cal1:.1f}, D2={cal2:.1f})")

        if west and west[1] is not None and west[2] is not None:
            cal1 = west[1] + self.offsets[1]
            cal2 = west[2] + self.offsets[2]
            diff = abs(cal1 - cal2)
            status = "✅" if diff < 3.0 else "⚠️"
            print(f"  {status} Batı'da D1 vs D2 farkı: {diff:.1f} dBm "
                  f"(kalibreli D1={cal1:.1f}, D2={cal2:.1f})")

        # ── Kenar noktalarında yön doğruluğu testi ──
        print("\n  🧭 YÖN DOĞRULUĞU TESTİ (kenar ölçümlerinden):")
        directions = {
            "KUZEY (D1)": (1, 2, "Kuzey", "D1 en güçlü, D2 en zayıf olmalı"),
            "GÜNEY (D2)": (2, 1, "Güney", "D2 en güçlü, D1 en zayıf olmalı"),
            "DOĞU (D3)":  (3, 4, "Doğu",  "D3 en güçlü, D4 en zayıf olmalı"),
            "BATI (D4)":  (4, 3, "Batı",  "D4 en güçlü, D3 en zayıf olmalı"),
        }

        direction_ok_count = 0
        for phase, (near_d, far_d, label, expect) in directions.items():
            data = self.results.get(phase)
            if data and data[near_d] is not None and data[far_d] is not None:
                cal_near = data[near_d] + self.offsets[near_d]
                cal_far = data[far_d] + self.offsets[far_d]
                if cal_near > cal_far:
                    print(f"  ✅ {label}: DOĞRU (yakın D{near_d}={cal_near:.1f} > "
                          f"uzak D{far_d}={cal_far:.1f}, fark={cal_near - cal_far:.1f})")
                    direction_ok_count += 1
                else:
                    print(f"  ❌ {label}: YANLIŞ! (D{near_d}={cal_near:.1f} <= "
                          f"D{far_d}={cal_far:.1f}) → {expect}")

        print(f"\n  Yön doğruluğu: {direction_ok_count}/4")
        if direction_ok_count == 4:
            print("  🎯 MÜKEMMEL! Kalibrasyon başarılı.")
        elif direction_ok_count >= 3:
            print("  ⚠️  Kabul edilebilir. Bir eksende sorun var, anten yönünü kontrol edin.")
        else:
            print("  🔴 Sorunlu! Antenler/donanım kontrol edilmeli.")

        # ── Sonuç ──
        print("\n" + "╔" + "═" * 53 + "╗")
        print("║  KALİBRASYON DEĞERLERİNİZ (bu satırları kopyalayın):  ║")
        print("╠" + "═" * 53 + "╣")
        print("║                                                       ║")
        print(f"║  CALIBRATION_OFFSET_DB = {{                            ║")
        print(f"║      1: {self.offsets[1]:>+6.2f},  # Kuzey (D1)                 ║")
        print(f"║      2: {self.offsets[2]:>+6.2f},  # Güney (D2)                 ║")
        print(f"║      3: {self.offsets[3]:>+6.2f},  # Doğu  (D3)                 ║")
        print(f"║      4: {self.offsets[4]:>+6.2f},  # Batı  (D4)                 ║")
        print(f"║  }}                                                    ║")
        print("║                                                       ║")
        print("╚" + "═" * 53 + "╝")

        return True

    # ─────────────────────────────────────────────
    # CANLI TEST MODU
    # ─────────────────────────────────────────────
    def run_live_test(self):
        """
        Kalibrasyon sonrası canlı doğrulama.
        Vericiyi istediğiniz yere taşıyın, sistem kalibreli RSSI
        farkını ve tahmini yönü anlık gösterir.
        """
        print("\n" + "=" * 55)
        print("  🔴 CANLI TEST MODU")
        print("=" * 55)
        print("  Kalibre edilmiş ofsetler uygulanıyor.")
        print("  Vericiyi istediğiniz konuma taşıyın.")
        print("  Sistem kalibreli RSSI + tahmini yön gösterecek.")
        print("  Çıkmak için Ctrl+C basın.\n")

        self.live_mode = True
        self.live_buffer = {1: [], 2: [], 3: [], 4: []}

        try:
            while True:
                time.sleep(2.0)

                now = time.time()
                calibrated = {}
                for d in [1, 2, 3, 4]:
                    recent = [
                        v for t, v in self.live_buffer[d]
                        if now - t <= LIVE_WINDOW_S
                    ]
                    if recent:
                        avg = sum(recent) / len(recent)
                        calibrated[d] = avg + self.offsets[d]

                if len(calibrated) < 2:
                    sys.stdout.write(f"\r  ⏳ Veri bekleniyor... ({len(calibrated)}/4 drone)   ")
                    sys.stdout.flush()
                    continue

                # RSSI çubuğu gösterimi
                lines = []
                for d in [1, 2, 3, 4]:
                    if d in calibrated:
                        val = calibrated[d]
                        # -100 ile -40 arası bar
                        bar_len = max(0, min(30, int((val + 100) / 2)))
                        bar = "█" * bar_len + "░" * (30 - bar_len)
                        lines.append(f"  D{d}: [{bar}] {val:>6.1f} dBm")
                    else:
                        lines.append(f"  D{d}: [{'░' * 30}]   N/A")

                # Artı formasyonu yön hesabı
                # Kuzey-Güney ekseni: e_ns = P1 - P2 (pozitif → kuzey)
                # Doğu-Batı ekseni:   e_ew = P3 - P4 (pozitif → doğu)
                e_ns = None
                e_ew = None
                if 1 in calibrated and 2 in calibrated:
                    e_ns = calibrated[1] - calibrated[2]
                if 3 in calibrated and 4 in calibrated:
                    e_ew = calibrated[3] - calibrated[4]

                # Yön hesapla
                direction = ""
                angle_deg = None
                if e_ns is not None and e_ew is not None:
                    angle_rad = math.atan2(e_ew, e_ns)
                    angle_deg = math.degrees(angle_rad)

                    # Pusula yönü
                    if abs(e_ns) < 1.0 and abs(e_ew) < 1.0:
                        direction = "⭕ MERKEZ (eşit güç)"
                    else:
                        # 8 yön
                        a = angle_deg % 360
                        if a < 22.5 or a >= 337.5:
                            direction = "⬆️  KUZEY"
                        elif a < 67.5:
                            direction = "↗️  KUZEYDOĞU"
                        elif a < 112.5:
                            direction = "➡️  DOĞU"
                        elif a < 157.5:
                            direction = "↘️  GÜNEYDOĞU"
                        elif a < 202.5:
                            direction = "⬇️  GÜNEY"
                        elif a < 247.5:
                            direction = "↙️  GÜNEYBATI"
                        elif a < 292.5:
                            direction = "⬅️  BATI"
                        else:
                            direction = "↖️  KUZEYBATI"

                        # Güç farkının büyüklüğü ≈ uzaklık tahmini
                        magnitude = math.sqrt(e_ns ** 2 + e_ew ** 2)
                        if magnitude < 3.0:
                            direction += " (yakın)"
                        elif magnitude < 8.0:
                            direction += " (orta)"
                        else:
                            direction += " (uzak)"

                # Ekranı temizleyip yaz
                print("\033[2J\033[H")  # Terminal temizle
                print("  🔴 CANLI TEST (Ctrl+C = çıkış)")
                print("  " + "─" * 50)
                print("  KALİBRELİ RSSI DEĞERLERİ:")
                for l in lines:
                    print(l)
                print()
                if e_ns is not None:
                    print(f"  Kuzey-Güney farkı (P1-P2): {e_ns:>+6.1f} dBm "
                          f"({'↑ Kuzey' if e_ns > 0 else '↓ Güney' if e_ns < 0 else '= Eşit'})")
                if e_ew is not None:
                    print(f"  Doğu-Batı farkı  (P3-P4): {e_ew:>+6.1f} dBm "
                          f"({'→ Doğu' if e_ew > 0 else '← Batı' if e_ew < 0 else '= Eşit'})")
                if direction:
                    print(f"\n  🧭 TAHMİNİ YÖN: {direction}")
                    if angle_deg is not None:
                        print(f"     Açı: {angle_deg:.0f}° (Kuzey=0°, Doğu=90°)")
                print("  " + "─" * 50)

        except KeyboardInterrupt:
            self.live_mode = False
            print("\n\n  Canlı test sonlandırıldı.")

    # ─────────────────────────────────────────────
    def shutdown(self):
        self.is_running = False
        self.live_mode = False
        if self.ser:
            self.ser.close()


# ==========================================
# ANA PROGRAM
# ==========================================
if __name__ == "__main__":
    print()
    print("╔" + "═" * 53 + "╗")
    print("║     🛰️  RSSI OTOMATİK KALİBRASYON ARACI  🛰️       ║")
    print("║     TÜBİTAK 123E294 - Artı (+) Formasyonu          ║")
    print("╚" + "═" * 53 + "╝")
    print()
    print("  Dronlar artı (+) şeklinde karşılıklı 10m mesafede")
    print("  olmalı. Merkez noktası her drona 5m uzaklıkta.")
    print()
    print("       D1 (Kuzey)")
    print("         ↑")
    print("  D4 ← MERKEZ → D3")
    print("  (Batı)   ↓    (Doğu)")
    print("       D2 (Güney)")
    print()

    import platform
    is_linux = platform.system() != "Windows"
    default_port = "/dev/ttyUSB0" if is_linux else "COM5"

    port = input(f"  LoRa portunu girin [{default_port}]: ").strip()
    if not port:
        port = default_port

    # Linux'ta sadece "ttyUSB3" yazılırsa "/dev/ttyUSB3" yap
    if is_linux and not port.startswith("/"):
        port = "/dev/" + port

    print(f"  Port: {port}\n")

    tool = CalibrationTool(port)
    tool.connect()

    try:
        # ── 5 Aşamalı Kalibrasyon ──
        tool.run_phase(
            "MERKEZ",
            "Vericiyi dronların TAM ORTASINA koyun (her drona 5m)."
        )
        tool.run_phase(
            "KUZEY (D1)",
            "Vericiyi D1'in (Kuzey) hemen yanına koyun (~1m)."
        )
        tool.run_phase(
            "GÜNEY (D2)",
            "Vericiyi D2'nin (Güney) hemen yanına koyun (~1m)."
        )
        tool.run_phase(
            "DOĞU (D3)",
            "Vericiyi D3'ün (Doğu) hemen yanına koyun (~1m)."
        )
        tool.run_phase(
            "BATI (D4)",
            "Vericiyi D4'ün (Batı) hemen yanına koyun (~1m)."
        )

        # ── Hesaplama ──
        success = tool.calculate_offsets()

        # ── Canlı Test ──
        if success:
            print()
            choice = input("  Canlı test moduna geçmek ister misiniz? [E/h]: ").strip().lower()
            if choice != "h":
                tool.run_live_test()

    except KeyboardInterrupt:
        print("\n\n  İşlem iptal edildi.")
    finally:
        tool.shutdown()
        print("  Program sonlandırıldı. İyi uçuşlar! ✈️")
