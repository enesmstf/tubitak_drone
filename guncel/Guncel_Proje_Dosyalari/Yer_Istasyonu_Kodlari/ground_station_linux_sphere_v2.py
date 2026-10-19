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
from ground_station_core_sphere_v2 import GroundStation

# ==========================================
# LINUX PORT AYARLARI
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "/dev/ttyUSB4"

PIXHAWK_PORTS = {
    1: "/dev/ttyUSB0",
    2: "/dev/ttyUSB2",
    3: "/dev/ttyUSB1",
    4: "/dev/ttyUSB3",
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
