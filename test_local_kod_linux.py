import sys
import os

# Çekirdek kodu (ground_station_core) içeri aktaralım
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_station_core import GroundStation

# ==========================================
# LINUX CANLI TEST AYARLARI (SADECE 3 DRONE İÇİN)
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "/dev/ttyUSB0"

PIXHAWK_PORTS = {
    1: "/dev/ttyUSB3",
    2: "/dev/ttyUSB1",
    3: "/dev/ttyUSB4",
    4: "/dev/ttyUSB2",
}

if __name__ == "__main__":
    print("==================================================")
    print("LINUX YEREL KOD TESTİ BAŞLATILIYOR (Drone 1, 2, 4)")
    print("==================================================")
    
    gs = GroundStation(
        lora_port=LORA_PORT,
        pixhawk_ports=PIXHAWK_PORTS,
        simulation=SIMULATION_MODE,
        platform_label="LinuxTest"
    )
    
    try:
        gs.run()
    except KeyboardInterrupt:
        print("\n[BİLGİ] Test sistemi kullanıcı tarafından kapatıldı.")
        gs.shutdown()
