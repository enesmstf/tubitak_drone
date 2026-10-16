"""
ground_station_windows_sphere.py

Windows'ta KÜRE FORMASYONU yer istasyonunu çalıştırır.
Port numaralarını Aygıt Yöneticisi'nden doğrulayın.

KULLANIM:
    python ground_station_windows_sphere.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_station_core_sphere import GroundStation

# ==========================================
# WINDOWS PORT AYARLARI
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "COM5"

PIXHAWK_PORTS = {
    1: "COM6",
    2: "COM7",
    3: "COM8",
    4: "COM9",
}

if __name__ == "__main__":
    gs = GroundStation(
        lora_port=LORA_PORT,
        pixhawk_ports=PIXHAWK_PORTS,
        simulation=SIMULATION_MODE,
        platform_label="Windows (Küre)"
    )
    try:
        gs.run()
    except KeyboardInterrupt:
        print("\n[BİLGİ] Sistem kapatıldı.")
        gs.shutdown()
