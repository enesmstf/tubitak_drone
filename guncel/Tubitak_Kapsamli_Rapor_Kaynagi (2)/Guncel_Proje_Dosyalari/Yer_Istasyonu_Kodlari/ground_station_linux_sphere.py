"""
ground_station_linux_sphere.py

Linux'ta KÜRE FORMASYONU yer istasyonunu çalıştırır.
Port yollarını `ls /dev/ttyUSB*` ile doğrulayın.

KULLANIM:
    python3 ground_station_linux_sphere.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_station_core_sphere import GroundStation

# ==========================================
# LINUX PORT AYARLARI
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "/dev/ttyUSB3"

PIXHAWK_PORTS = {
    1: "/dev/ttyUSB2",
    2: "/dev/ttyUSB1",
    3: "/dev/ttyUSB0",
    4: "/dev/ttyUSB4",
}

if __name__ == "__main__":
    gs = GroundStation(
        lora_port=LORA_PORT,
        pixhawk_ports=PIXHAWK_PORTS,
        simulation=SIMULATION_MODE,
        platform_label="Linux (Küre)"
    )
    try:
        gs.run()
    except KeyboardInterrupt:
        print("\n[BİLGİ] Sistem kapatıldı.")
        gs.shutdown()
