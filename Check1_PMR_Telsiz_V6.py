import numpy as np
import serial
import serial.tools.list_ports
import time
import sys
import signal
import logging

try:
    from rtlsdr import RtlSdr
except ImportError:
    print("pyrtlsdr eksik. 'pip install pyrtlsdr' calistirin.")
    sys.exit(1)

# ==========================================
# SÜRÜ AYARLARI (TDMA)
# ==========================================
NODE_ID = 1               # LUTFEN KENDI DRONUNUZA GORE DEGISTIRIN (1, 2, 3, 4)
TOTAL_NODES = 4
SLOT_DURATION = 0.5
CYCLE_DURATION = TOTAL_NODES * SLOT_DURATION
LORA_BAUD = 9600

# ==========================================
# TELSIZ (PMR) KESKİN NİŞANCI AYARLARI
# ==========================================
SDR_FREQ = 446.0e6          # SDR Merkez frekans (DC Spike'tan kacis)
TARGET_FREQ = 446.148e6     # Telsizin TAM frekansi
SDR_SAMPLE_RATE = 2.048e6
SDR_TARGET_GAIN_DB = 10.0   # Telsiz asiri guclu, gain dusuk tutuldu

# ==========================================
# KALİBRASYON AYARI (HER DRONE İÇİN FARKLI)
# ==========================================
# Tum dronlari telsizden esit uzakliga (ornegin 3 metre) koyun.
# Hepsinin ayni dB degerini gostermesi gerekir.
# Eger Drone 2 digerlerinden 4 dB fazla okuyorsa,
# Drone 2'nin kodunda bu degeri -4 yapin.
# Eger Drone 3 digerlerinden 3 dB az okuyorsa,
# Drone 3'un kodunda bu degeri +3 yapin.
# Varsayilan olarak 0 birakin (kalibrasyon yapilmamis).
CALIBRATION_OFFSET_DB = 0

# ==========================================
# KIRPILMA (CLIPPING) DEDEKTORU AYARLARI
# ==========================================
# Bu esik degerinin altindaki kirpilma oranlari normal kabul edilir.
# Uzerindekiler ise duzeltme formulune girer.
CLIP_THRESHOLD = 0.01  # %1'den fazla kirpilma varsa duzeltme uygula

LORA_RETRY_DELAY = 2
SDR_RETRY_DELAY = 3

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def compute_checksum(payload: str) -> int:
    return sum(ord(c) for c in payload) % 256

class LocalizationNode:
    def __init__(self):
        self.node_id = NODE_ID
        self.lora = None
        self.sdr = None
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    def find_working_lora_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "ttyS" in p.device: continue
            try:
                test_ser = serial.Serial(p.device, LORA_BAUD, timeout=0.5)
                test_ser.close()
                return p.device
            except Exception: pass
        return None

    def connect_lora(self):
        while self.running:
            port = self.find_working_lora_port()
            if not port:
                time.sleep(LORA_RETRY_DELAY)
                continue
            try:
                self.lora = serial.Serial(port, LORA_BAUD, timeout=0.1)
                logging.info(f"LoRa baglandi: {port}")
                return True
            except Exception as e:
                self.safe_close_lora()
                time.sleep(LORA_RETRY_DELAY)
        return False

    def connect_sdr(self):
        while self.running:
            try:
                self.sdr = RtlSdr()
                self.sdr.sample_rate = SDR_SAMPLE_RATE
                self.sdr.center_freq = SDR_FREQ

                self.sdr.set_manual_gain_enabled(True)
                try:
                    valid_gains = self.sdr.valid_gains_db
                    closest_gain = min(valid_gains, key=lambda g: abs(g - SDR_TARGET_GAIN_DB))
                    self.sdr.gain = closest_gain
                except Exception:
                    self.sdr.gain = SDR_TARGET_GAIN_DB

                logging.info(f"RTL-SDR baglandi. Gain: {self.sdr.gain} dB | Kalibrasyon: {CALIBRATION_OFFSET_DB} dB")
                time.sleep(0.5)
                try:
                    _ = self.sdr.read_bytes(8192)
                except Exception:
                    pass
                return True
            except Exception as e:
                logging.warning(f"SDR baglantisi basarisiz. Tekrar deneniyor...")
                self.safe_close_sdr()
                time.sleep(SDR_RETRY_DELAY)
        return False

    def get_filtered_rssi(self):
        """
        V6 KESKIN NISANCI + KIRPILMA DEDEKTORU

        Adim 1: SDR'den ham (raw) baytlari oku. Her bayt 0-255 arasi.
        Adim 2: Kac tanesinin 0 veya 255'e yapistigini say (Kirpilma Orani).
        Adim 3: Ham baytlari IQ (karmasik sayi) formatina cevir ve FFT yap.
        Adim 4: Hedef frekans penceresindeki en buyuk tepeyi (peak) al.
        Adim 5: Eger kirpilma varsa, gercek gucu matematiksel olarak hesapla.
        Adim 6: Kalibrasyon duzeltmesini uygula.
        """
        readings = []
        try:
            for _ in range(2):
                # ----- HAM BAYT OKUMA (Kirpilma tespiti icin) -----
                num_samples = 16384
                num_bytes = num_samples * 2  # I ve Q ayri ayri

                # Kararsiz (stale) veriyi at
                try:
                    _ = self.sdr.read_bytes(8192)
                except Exception:
                    continue

                raw_data = np.array(self.sdr.read_bytes(num_bytes), dtype=np.uint8)

                if len(raw_data) < num_bytes:
                    continue

                # ----- KIRPILMA ORANI HESAPLA -----
                clipped_count = int(np.sum((raw_data == 0) | (raw_data == 255)))
                total_count = len(raw_data)
                clip_ratio = clipped_count / total_count

                # ----- HAM BAYTI IQ'YA CEVIR -----
                iq_float = raw_data.astype(np.float64)
                iq_float = (iq_float - 127.5) / 127.5  # [-1, +1] normalize
                samples = iq_float[0::2] + 1j * iq_float[1::2]  # I + jQ

                if not np.all(np.isfinite(samples)):
                    continue

                # ----- FFT -----
                samples = samples - np.mean(samples)
                window = np.hanning(len(samples))
                windowed = samples * window

                fft_vals = np.abs(np.fft.fft(windowed))
                fft_freqs = np.fft.fftfreq(len(windowed), 1.0 / SDR_SAMPLE_RATE)

                # Telsiz (PMR) Filtresi +-15kHz
                target_offset_hz = TARGET_FREQ - SDR_FREQ
                window_width_hz = 15e3
                mask = (fft_freqs > (target_offset_hz - window_width_hz)) & \
                       (fft_freqs < (target_offset_hz + window_width_hz))

                if not np.any(mask):
                    readings.append(-120)
                    continue

                peak_val = np.max(fft_vals[mask])
                base_dbm = 10 * np.log10(peak_val + 1e-12)

                # ----- KIRPILMA DUZELTMESI -----
                if clip_ratio > CLIP_THRESHOLD:
                    clip_correction_db = 10 * np.log10(1.0 / (1.0 - clip_ratio + 1e-9))
                    final_dbm = base_dbm + clip_correction_db
                    logging.debug(f"Kirpilma: %{clip_ratio*100:.1f} | Duzeltme: +{clip_correction_db:.1f} dB")
                else:
                    final_dbm = base_dbm

                # ----- KALIBRASYON UYGULA -----
                final_dbm = final_dbm + CALIBRATION_OFFSET_DB

                readings.append(final_dbm)

            if len(readings) == 0:
                return 0

            return int(np.median(readings))

        except Exception as e:
            logging.error(f"SDR Hatasi: {e}")
            self.safe_close_sdr()
            self.connect_sdr()
            return 0

    def transmit(self, rssi):
        body = f"N{self.node_id},{rssi}"
        checksum = compute_checksum(body)
        payload = f"{body},{checksum}\n"

        for attempt in range(2):
            try:
                if self.lora is None or not self.lora.is_open: self.connect_lora()
                self.lora.write(payload.encode("ascii"))
                self.lora.flush()
                logging.info(f"TX -> {payload.strip()}")
                return True
            except Exception:
                self.safe_close_lora()
                time.sleep(0.1)
        return False

    def safe_close_lora(self):
        try:
            if self.lora: self.lora.close()
        except Exception: pass
        self.lora = None

    def safe_close_sdr(self):
        try:
            if self.sdr: self.sdr.close()
        except Exception: pass
        self.sdr = None

    def shutdown(self, signum, frame):
        self.running = False
        self.safe_close_sdr()
        self.safe_close_lora()
        sys.exit(0)

    def run(self):
        logging.info(f"V6 KIRPILMA DEDEKTORLU TDMA Basladi. Dongu: {CYCLE_DURATION}s")
        last_tx_cycle = -1

        while self.running:
            now = time.time()
            current_cycle = int(now // CYCLE_DURATION)
            target_time = (current_cycle * CYCLE_DURATION) + \
                          ((self.node_id - 1) * SLOT_DURATION) + \
                          (SLOT_DURATION * 0.5)

            if now > target_time:
                current_cycle += 1
                target_time = (current_cycle * CYCLE_DURATION) + \
                              ((self.node_id - 1) * SLOT_DURATION) + \
                              (SLOT_DURATION * 0.5)

            sleep_time = target_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

            cycle_index = int(time.time() // CYCLE_DURATION)
            if cycle_index != last_tx_cycle:
                rssi = self.get_filtered_rssi()
                self.transmit(rssi)
                last_tx_cycle = cycle_index

if __name__ == "__main__":
    node = LocalizationNode()
    node.connect_lora()
    node.connect_sdr()
    node.run()
