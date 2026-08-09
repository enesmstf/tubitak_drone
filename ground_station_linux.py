"""
ground_station_linux.py

Linux bilgisayarında çalıştırın. Tek farkı port isimlendirmesi -
mantığın tamamı ground_station_core.py içindedir.

KULLANIM:
    1. Aşağıdaki /dev/ttyUSB* değerlerini gerçek portlarınızla eşleştirin
       (python3 -m serial.tools.list_ports -v ile doğrulayabilirsiniz).
    2. python3 ground_station_linux.py

NOT: USB reset sonrası port numaraları değişebilir (ttyUSB0 -> ttyUSB1 gibi);
gerekirse udev kuralıyla sabit isim (symlink) atamanız önerilir.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_station_core import GroundStation

# ==========================================
# LINUX PORT AYARLARI
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "/dev/ttyUSB0"

PIXHAWK_PORTS = {
    1: "/dev/ttyUSB3",  # Drone 1 (ORIGIN referansı bu drone'dan alınır)
    2: "/dev/ttyUSB1",  # Drone 2
    3: "/dev/ttyUSB4",  # Drone 3
    4: "/dev/ttyUSB2",  # Drone 4
}


if __name__ == "__main__":
    gs = GroundStation(
        lora_port=LORA_PORT,
        pixhawk_ports=PIXHAWK_PORTS,
        simulation=SIMULATION_MODE,
        platform_label="Linux"
    )
    try:
        gs.run()
    except KeyboardInterrupt:
        print("\n[BİLGİ] Sistem kullanıcı tarafından kapatıldı. Emniyetli uçuşlar!")
        gs.shutdown()
