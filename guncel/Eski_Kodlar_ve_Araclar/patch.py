import re

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'r') as f:
    content = f.read()

udp_code = """
                self.rssi_data[drone_id] = (smoothed_rssi, time.time())
                
                # --- RADAR GUI ICIN UDP YAYINI ---
                import socket, json
                try:
                    if not hasattr(self, 'udp_sock'):
                        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    msg = json.dumps({d: int(v[0]) for d, v in self.rssi_data.items()})
                    self.udp_sock.sendto(msg.encode('utf-8'), ("127.0.0.1", 5555))
                except Exception:
                    pass
                # ---------------------------------
"""

content = content.replace("self.rssi_data[drone_id] = (smoothed_rssi, time.time())", udp_code)

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'w') as f:
    f.write(content)
