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
    print("pyrtlsdr kütüphanesi eksik. Lütfen 'pip install pyrtlsdr' komutunu çalıştırın.")
    sys.exit(1)


# ==========================================
# SÜRÜ AYARLARI (SWARM CONFIGURATION)
# ==========================================
NODE_ID = 1               # Bu drone'un kimlik numarası (Her drone'da FARKLI olmalı: 1,2,3,4)
TOTAL_NODES = 4
SLOT_DURATION = 1.0
CYCLE_DURATION = TOTAL_NODES * SLOT_DURATION

LORA_BAUD = 9600

# HC-12 standart bandı 433 MHz. (Önceki 446.450 MHz PMR telsiz bandı için
# unutulmuş bir test değeriydi - HC-12 sinyalini hiç görmüyordu.)
SDR_FREQ = 433.0e6
SDR_SAMPLE_RATE = 2.048e6

# Sabit kazanç (dB). AGC ('auto') her cihazda farklı ve zamanla değişken
# davranır; 4 SDR'ın ölçümlerinin birbiriyle kıyaslanabilir olması için
# TÜM node'larda AYNI sabit değer kullanılmalı. Bağlı SDR'ın desteklediği
# kazanç kademelerine göre en yakın stabil değere yuvarlanır (aşağıda).
SDR_TARGET_GAIN_DB = 40.0

# --- RETRY SETTINGS ---
LORA_RETRY_DELAY = 2
SDR_RETRY_DELAY = 3

# --- LOGGING SETUP ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)


def compute_checksum(payload: str) -> int:
    """
    Basit ama yeterince ayırt edici bir checksum: ASCII değerlerin toplamı mod 256.
    Kriptografik güvenlik sağlamaz (bunun için HMAC + paylaşılan anahtar gerekir),
    ama rastgele RF gürültüsünden / bozuk paketlerden kaynaklanan hatalı verinin
    sisteme sızmasını engeller.
    """
    return sum(ord(c) for c in payload) % 256


class LocalizationNode:

    def __init__(self):
        self.node_id = NODE_ID
        self.running = True
        self.sdr = None
        self.lora = None

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        self.connect_hardware()

    # -------------------------------------------------
    # PORT SEARCH
    # -------------------------------------------------
    def list_serial_candidates(self):
        candidates = []
        ports = list(serial.tools.list_ports.comports())

        for port in ports:
            device = port.device
            desc = (port.description or "").upper()
            hwid = (port.hwid or "").upper()

            logging.info(f"Detected port: {device} | {desc} | {hwid}")

            if (
                "TTYUSB" in device.upper()
                or "TTYACM" in device.upper()
                or "USB" in desc
                or "SERIAL" in desc
                or "CH340" in desc
                or "CP210" in desc
                or "FTDI" in desc
                or "UART" in desc
            ):
                candidates.append(device)

        candidates = list(dict.fromkeys(candidates))
        return candidates

    def find_working_lora_port(self):
        candidates = self.list_serial_candidates()

        if not candidates:
            logging.warning("No serial port candidate found.")
            return None

        for device in candidates:
            try:
                logging.info(f"Trying LoRa port: {device}")
                test = serial.Serial(device, LORA_BAUD, timeout=0.2)
                test.close()
                logging.info(f"Working LoRa port found: {device}")
                return device
            except Exception as e:
                logging.warning(f"Port failed: {device} | {e}")

        return None

    # -------------------------------------------------
    # HARDWARE CONNECT / RECONNECT
    # -------------------------------------------------
    def connect_lora(self):
        while self.running:
            port = self.find_working_lora_port()

            if not port:
                logging.warning(f"No LoRa port available. Retrying in {LORA_RETRY_DELAY}s...")
                time.sleep(LORA_RETRY_DELAY)
                continue

            try:
                self.lora = serial.Serial(port, LORA_BAUD, timeout=0.1)
                logging.info(f"LoRa connected on {port}")
                return True
            except Exception as e:
                logging.warning(f"Could not open LoRa on {port}: {e}")
                self.safe_close_lora()
                time.sleep(LORA_RETRY_DELAY)

        return False

    def connect_sdr(self):
        while self.running:
            try:
                self.sdr = RtlSdr()
                self.configure_sdr()
                logging.info("RTL-SDR connected and configured.")
                return True
            except Exception as e:
                logging.warning(f"SDR connection failed: {e}. Retrying in {SDR_RETRY_DELAY}s...")
                self.safe_close_sdr()
                time.sleep(SDR_RETRY_DELAY)

        return False

    def connect_hardware(self):
        logging.info("Waiting for hardware...")
        self.connect_lora()
        self.connect_sdr()
        logging.info(f"Localization Node {self.node_id} armed.")

    def reconnect_lora(self):
        logging.warning("Reconnecting LoRa...")
        self.safe_close_lora()
        return self.connect_lora()

    def reconnect_sdr(self):
        logging.warning("Reconnecting SDR...")
        self.safe_close_sdr()
        return self.connect_sdr()

    # -------------------------------------------------
    # SDR
    # -------------------------------------------------
    def configure_sdr(self):
        self.sdr.sample_rate = SDR_SAMPLE_RATE
        self.sdr.center_freq = SDR_FREQ

        # AGC KAPALI: 4 node arasında dBm değerlerinin birebir kıyaslanabilir
        # olması için sabit kazanç şart. Cihazın desteklediği en yakın
        # kademeye yuvarlanıyor (RTL-SDR'lar keyfi ondalık kazanç kabul etmez).
        try:
            valid_gains = self.sdr.valid_gains_db
            closest_gain = min(valid_gains, key=lambda g: abs(g - SDR_TARGET_GAIN_DB))
            self.sdr.gain = closest_gain
            logging.info(f"SDR gain sabitlendi: {closest_gain} dB (hedef: {SDR_TARGET_GAIN_DB} dB)")
        except Exception as e:
            logging.warning(f"Kazanç kademeleri okunamadı, doğrudan hedef değer deneniyor: {e}")
            self.sdr.gain = SDR_TARGET_GAIN_DB

    def get_filtered_rssi(self):
        readings = []

        try:
            for _ in range(3):
                _ = self.sdr.read_samples(256)
                samples = self.sdr.read_samples(16384)

                power = np.mean(np.abs(samples) ** 2)
                dbm = 10 * np.log10(power + 1e-12)
                readings.append(dbm)

            return int(np.median(readings))

        except Exception as e:
            logging.error(f"SDR Read Error: {e}")
            self.reconnect_sdr()
            return -120

    # -------------------------------------------------
    # LORA TX
    # -------------------------------------------------
    def transmit(self, rssi):
        # Checksum, "N{id},{rssi}" gövdesi üzerinden hesaplanır ve pakete eklenir.
        # Format: N{id},{rssi},{checksum}\n
        body = f"N{self.node_id},{rssi}"
        checksum = compute_checksum(body)
        payload = f"{body},{checksum}\n"

        for attempt in range(2):
            try:
                if self.lora is None or not self.lora.is_open:
                    self.reconnect_lora()

                self.lora.write(payload.encode("ascii"))
                self.lora.flush()
                logging.info(f"TX -> {payload.strip()}")
                return True

            except Exception as e:
                logging.error(f"LoRa TX Failure: {e}")
                self.reconnect_lora()

        logging.error("LoRa TX failed after reconnect attempt.")
        return False

    # -------------------------------------------------
    # SAFE CLOSE
    # -------------------------------------------------
    def safe_close_lora(self):
        try:
            if self.lora:
                self.lora.close()
        except Exception:
            pass
        self.lora = None

    def safe_close_sdr(self):
        try:
            if self.sdr:
                self.sdr.close()
        except Exception:
            pass
        self.sdr = None

    # -------------------------------------------------
    # SHUTDOWN
    # -------------------------------------------------
    def shutdown(self, signum, frame):
        logging.info("Initiating graceful teardown...")
        self.running = False
        self.safe_close_sdr()
        self.safe_close_lora()
        logging.info("Hardware released. Node disarmed.")
        sys.exit(0)

    # -------------------------------------------------
    # MAIN LOOP - Absolute-Time TDMA
    # -------------------------------------------------
    def run(self):
        logging.info(f"ABSOLUTE-TIME TDMA Started. Cycle: {CYCLE_DURATION}s")

        last_tx_cycle = -1

        while self.running:
            now = time.time()
            slot_start = (self.node_id - 1) * SLOT_DURATION
            slot_tx_offset = slot_start + 0.20

            current_cycle = int(now // CYCLE_DURATION)
            target_time = (current_cycle * CYCLE_DURATION) + slot_tx_offset

            if now >= target_time:
                current_cycle += 1
                target_time = (current_cycle * CYCLE_DURATION) + slot_tx_offset

            sleep_time = target_time - time.time()
            if sleep_time > 0.05:
                time.sleep(sleep_time - 0.02)

            while time.time() < target_time:
                time.sleep(0.002)

            cycle_index = int(time.time() // CYCLE_DURATION)

            if cycle_index != last_tx_cycle:
                rssi = self.get_filtered_rssi()
                self.transmit(rssi)
                last_tx_cycle = cycle_index

            time.sleep(0.10)


if __name__ == "__main__":
    node = LocalizationNode()
    node.run()
