import re

with open('/home/enes/Belgeler/tubitak_drone/Radar_Kalibrasyon_GUI.py', 'r') as f:
    content = f.read()

import_code = """import tkinter as tk
import math
import socket
import json
import threading"""

content = content.replace("import tkinter as tk\nimport math", import_code)

init_code = """        self.target_visible = True  # Yanıp sönme animasyonu için

        # UDP Dinleyici Başlat
        self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.udp_thread.start()"""

content = content.replace("self.target_visible = True  # Yanıp sönme animasyonu için", init_code)

listener_method = """    def udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 5555))
        sock.settimeout(1.0)
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                for d_id, val in msg.items():
                    d_id = int(d_id)
                    if d_id in self.rssi:
                        self.rssi[d_id] = val
                        # Slider'ı da güncelle
                        if d_id in self.sliders:
                            self.sliders[d_id].set(val)
                self.root.after(10, self.update_radar)
            except socket.timeout:
                pass
            except Exception as e:
                pass

    def setup_ui(self):"""

content = content.replace("def setup_ui(self):", listener_method)

with open('/home/enes/Belgeler/tubitak_drone/Radar_Kalibrasyon_GUI.py', 'w') as f:
    f.write(content)
