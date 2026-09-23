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

# YENI GUVENLI HIZ: 0.5 Saniye (Toplam dongu 2 saniye surer, havada hicbir paket carpismaz)
SLOT_DURATION = 0.5       
CYCLE_DURATION = TOTAL_NODES * SLOT_DURATION
LORA_BAUD = 9600

# ==========================================
# TELSIZ (PMR) KESKİN NİŞANCI AYARLARI
# ==========================================
SDR_FREQ = 446.0e6          # SDR Merkez frekans (DC Spike'tan kacis)
TARGET_FREQ = 446.148e6     # Telsizin TAM frekansi
SDR_SAMPLE_RATE = 2.048e6
SDR_TARGET_GAIN_DB = 10.0   # Telsiz asiri guclu oldugu icin gain kisildi

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
                
                logging.info(f"RTL-SDR baglandi. Gain: {self.sdr.gain} dB")
                time.sleep(0.5)
                return True
            except Exception as e:
                logging.warning(f"SDR baglantisi basarisiz. Tekrar deneniyor...")
                self.safe_close_sdr()
                time.sleep(SDR_RETRY_DELAY)
        return False

    def get_filtered_rssi(self):
        readings = []
        try:
            for _ in range(2): # Hizlanmasi icin 3 yerine 2 ornekleme alindi
                _ = self.sdr.read_samples(4096)
                samples = self.sdr.read_samples(16384)

                if len(samples) < 16384 or not np.all(np.isfinite(samples)):
                    continue

                samples = samples - np.mean(samples)
                window = np.hanning(len(samples))
                windowed = samples * window

                fft_vals = np.abs(np.fft.fft(windowed))
                fft_freqs = np.fft.fftfreq(len(windowed), 1.0 / SDR_SAMPLE_RATE)

                # Telsiz (PMR) Filtresi +-15kHz (NFM Yayin)
                target_offset_hz = TARGET_FREQ - SDR_FREQ
                window_width_hz = 15e3 
                mask = (fft_freqs > (target_offset_hz - window_width_hz)) & (fft_freqs < (target_offset_hz + window_width_hz))

                if not np.any(mask):
                    readings.append(-120)
                    continue

                peak_val = np.max(fft_vals[mask])
                dbm = 10 * np.log10(peak_val + 1e-12)
                readings.append(dbm)

            if len(readings) == 0: return 0
            return int(np.median(readings))

        except Exception as e:
            logging.error(f"SDR Hatasi: {e}")
            self.reconnect_sdr()
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

    # TAMAMEN YENILENMIS, KUSURSUZ ZAMANLAMALI TDMA DONGUSU
    def run(self):
        logging.info(f"YENI KUSURSUZ TDMA Basladi. Dongu Hizi: {CYCLE_DURATION}s")
        last_tx_cycle = -1
        
        while self.running:
            now = time.time()
            current_cycle = int(now // CYCLE_DURATION)
            
            # Bu dronun slotunun tam ortasini matematiksel olarak bul
            target_time = (current_cycle * CYCLE_DURATION) + ((self.node_id - 1) * SLOT_DURATION) + (SLOT_DURATION * 0.5)

            # Eger sira bittiyse bir sonraki turu (donguyu) bekle
            if now > target_time:
                current_cycle += 1
                target_time = (current_cycle * CYCLE_DURATION) + ((self.node_id - 1) * SLOT_DURATION) + (SLOT_DURATION * 0.5)

            # Sirasi gelene kadar uyu
            sleep_time = target_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Sira geldi! Okuma yap ve Lora'dan gonder.
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
