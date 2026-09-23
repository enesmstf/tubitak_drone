import re

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_kalman.py', 'r') as f:
    content = f.read()

# Add kalman initialization
init_old = "self.rssi_data = {}  # {drone_id: (smoothed_rssi, timestamp)}"
init_new = """self.rssi_data = {}  # {drone_id: (smoothed_rssi, timestamp)}
        self.kalman_states = {} # {drone_id: {'x_hat': float, 'P': float}}
        self.KALMAN_Q = 0.1   # Süreç Gürültüsü (Process Noise)
        self.KALMAN_R = 5.0   # Ölçüm Gürültüsü (Measurement Noise)"""

content = content.replace(init_old, init_new)

# Replace EMA with Kalman in _handle_line
ema_block = """            with self.rssi_lock:
                if drone_id in self.rssi_data:
                    old_rssi, _ = self.rssi_data[drone_id]
                    # EMA Filtresi: %40 Yeni Veri, %60 Eski Veri (Pürüzsüzleştirme)
                    smoothed_rssi = (0.4 * rssi) + (0.6 * old_rssi)
                else:
                    smoothed_rssi = rssi

                self.rssi_data[drone_id] = (smoothed_rssi, time.time())"""

kalman_block = """            with self.rssi_lock:
                if drone_id not in self.kalman_states:
                    self.kalman_states[drone_id] = {'x_hat': float(rssi), 'P': 1.0}
                    kalman_rssi = float(rssi)
                else:
                    # KALMAN FİLTRESİ ADIMLARI (Rapordaki Denklem 4-8)
                    state = self.kalman_states[drone_id]
                    x_hat = state['x_hat']
                    P = state['P']

                    # 1. Tahmin (Prediction) -> Denklem 4, 5
                    x_predict = x_hat
                    P_predict = P + self.KALMAN_Q

                    # 2. Kalman Kazancı (Measurement Weighting) -> Denklem 6
                    K = P_predict / (P_predict + self.KALMAN_R)

                    # 3. Durum Güncellemesi (State Correction) -> Denklem 7
                    x_new = x_predict + K * (rssi - x_predict)

                    # 4. Kovaryans Güncellemesi (Updated Uncertainty) -> Denklem 8
                    P_new = (1 - K) * P_predict

                    self.kalman_states[drone_id] = {'x_hat': x_new, 'P': P_new}
                    kalman_rssi = x_new

                self.rssi_data[drone_id] = (kalman_rssi, time.time())"""

content = content.replace(ema_block, kalman_block)

# Also replace the intro text
intro_old = "Bu dosya YER İSTASYONU mantığının TAMAMINI içerir"
intro_new = "Bu dosya YER İSTASYONU mantığının TAMAMINI içerir\nKALMAN FİLTRESİ EKLENMİŞ ÖZEL SÜRÜM."
content = content.replace(intro_old, intro_new)

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_kalman.py', 'w') as f:
    f.write(content)

