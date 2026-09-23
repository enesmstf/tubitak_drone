import re

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'r') as f:
    content = f.read()

bad_block = """            # --- ÖZEL İSTİSNA (BYPASS): DRONE 3 ---
            # Drone 3'e HDMI arızası yüzünden yeni kod atılamadığı için, 
            # onun şifresiz (2 parçalı) verisini kabul ediyoruz.
            if drone_id == 3 and len(parts) == 2:
                rssi = int(parts[1])
                received_checksum = 0
                expected_checksum = 0
            # --------------------------------------
            elif len(parts) == 3:
                rssi = int(parts[1])
                received_checksum = int(parts[2])
                body = f"N{drone_id},{rssi}"
                expected_checksum = compute_checksum(body)
            else:
                return"""

good_block = """            if len(parts) != 3:
                return
            rssi = int(parts[1])
            received_checksum = int(parts[2])
            body = f"N{drone_id},{rssi}"
            expected_checksum = compute_checksum(body)"""

content = content.replace(bad_block, good_block)

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'w') as f:
    f.write(content)
