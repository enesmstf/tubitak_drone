import re

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'r') as f:
    content = f.read()

# Increase RSSI_MAX_AGE_S
content = re.sub(r'RSSI_MAX_AGE_S\s*=\s*8\.0', 'RSSI_MAX_AGE_S = 30.0', content)

# Change the error message to show WHICH drones are missing
old_calc = """    def calculate_virtual_center_shift(self, rssi_data):
        if len(rssi_data) < 4:
            print(f"[UYARI] Tüm dronelardan güncel veri alınamadı ({len(rssi_data)}/4, "
                  f"tazelik sınırı: {RSSI_MAX_AGE_S}s). Merkez son bilinen konumda sabit tutuluyor.")
            return 0.0, 0.0"""

new_calc = """    def calculate_virtual_center_shift(self, rssi_data):
        if len(rssi_data) < 4:
            missing = [d for d in [1, 2, 3, 4] if d not in rssi_data]
            print(f"[UYARI] Tüm dronelardan güncel veri alınamadı ({len(rssi_data)}/4). "
                  f"Eksik Dronelar: {missing} (Son {RSSI_MAX_AGE_S}s içinde veri atmadılar). Merkez sabit tutuluyor.")
            return 0.0, 0.0"""

content = content.replace(old_calc, new_calc)

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'w') as f:
    f.write(content)

