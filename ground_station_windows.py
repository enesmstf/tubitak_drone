"""
ground_station_windows.py

Windows bilgisayarında çalıştırın. Tek farkı port isimlendirmesi -
mantığın tamamı ground_station_core.py içindedir.

KULLANIM:
    1. Aşağıdaki COM_* değerlerini Windows Aygıt Yöneticisi'nden (Device
       Manager) gördüğünüz gerçek port numaralarıyla değiştirin.
    2. python ground_station_windows.py

NOT: Bu script LoRa yer modülünün COM portunu KENDİSİ AÇAR. Aynı anda
RealTerm ile bu portu ayrıca açamazsınız (Windows tek bir process'e izin
verir). Bu script her ham LoRa satırını "[RADYO-MONITOR] ..." olarak
kendi konsoluna bastığı için RealTerm'e ihtiyacınız kalmaz - izleme
işlevini bu script üstlenir. Aynı mantık Mission Planner için de geçerlidir:
bu script çalışırken Pixhawk telemetri COM portlarına Mission Planner ile
CONNECT olamazsınız.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_station_core import GroundStation

# ==========================================
# WINDOWS PORT AYARLARI - Aygıt Yöneticisi'nden doğrulayın
# ==========================================
SIMULATION_MODE = False

LORA_PORT = "COM5"

PIXHAWK_PORTS = {
    1: "COM6",   # Drone 1 (ORIGIN referansı bu drone'dan alınır)
    2: "COM7",   # Drone 2
    3: "COM8",   # Drone 3
    4: "COM9",   # Drone 4
}


if __name__ == "__main__":
    gs = GroundStation(
        lora_port=LORA_PORT,
        pixhawk_ports=PIXHAWK_PORTS,
        simulation=SIMULATION_MODE,
        platform_label="Windows"
    )
    try:
        gs.run()
    except KeyboardInterrupt:
        print("\n[BİLGİ] Sistem kullanıcı tarafından kapatıldı. Emniyetli uçuşlar!")
        gs.shutdown()
