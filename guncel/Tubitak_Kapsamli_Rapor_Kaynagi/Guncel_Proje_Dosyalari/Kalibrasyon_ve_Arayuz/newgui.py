import tkinter as tk
import math
import platform
import serial
import threading
import time

# LoRa modülünün baud rate'i (ground_station_core ile aynı olmalı)
LORA_BAUD = 9600


class SwarmRadar:
    def __init__(self, root):
        self.root = root
        self.root.title("TÜBİTAK Otonom Sürü - Canlı Radar Ekranı")
        self.root.geometry("900x680")
        self.root.configure(bg="#1a1a2e")

        self.OFFSET_M = 5.0
        self.K_GAIN = 0.5                   # near-field gradient-to-meters scaling (triangulation)
        self.SCALE = 20
        self.CANVAS_SIZE = 600
        self.CENTER = self.CANVAS_SIZE // 2

        # --- Distance estimation (Friis / log-distance path-loss model) ---
        # Bu değerler ARTIK PLACEHOLDER DEĞİL - 13 noktalı kalibrasyon
        # oturumundan (araç etkisi giderilmiş, temiz veri, RMSE=2.29dB)
        # ortak regresyonla fit edildi:
        #   RSSI(d) = RSSI_REF_1M - 10*PATH_LOSS_EXPONENT*log10(d) + kalibrasyon_ofseti
        self.RSSI_REF_1M = -78.72           # 1 metrede beklenen RSSI (dBm)
        self.PATH_LOSS_EXPONENT = 1.264     # path-loss üsteli (n<2, açık alan/zemin yansıması nedeniyle beklenen)
        self.CALIBRATION_OFFSET_DB = {      # her drone'un donanım yanlılığı (ham RSSI'ya eklenir)
            1: 0.0,
            2: -1.32,
            3: 2.38,
            4: 1.10,
        }
        self.MIN_RANGE_M = 0.3              # log10(0) tekilliğinden kaçınmak için alt sınır
        self.NO_SIGNAL_RSSI = -125          # bu değerin altı "hiç veri yok" sayılır
        self.DISPLAY_RING_M = 13            # radar'ın GÖRSEL yarıçapı (gerçek menzilden bağımsız, sadece çizim için)

        self.drones = {
            1: {"name": "Drone 1 (Kuzey)", "x": 0,              "y": self.OFFSET_M,  "color": "#3498db"},
            2: {"name": "Drone 2 (Güney)", "x": 0,              "y": -self.OFFSET_M, "color": "#e67e22"},
            3: {"name": "Drone 3 (Doğu)",  "x": self.OFFSET_M,  "y": 0,              "color": "#9b59b6"},
            4: {"name": "Drone 4 (Batı)",  "x": -self.OFFSET_M, "y": 0,              "color": "#1abc9c"}
        }

        self.rssi = {1: -90, 2: -90, 3: -90, 4: -90}

        # Filtered RSSI used for all distance/direction math. Raw self.rssi
        # (above) is still what's shown on sliders/log for transparency, but
        # a single corrupted/noisy LoRa packet on one drone (e.g. a spurious
        # -119 dBm reading while the other 3 drones read -80ish) should NOT
        # be allowed to single-handedly flip the near/far mode or swing the
        # displayed distance by several meters. Two layers of protection:
        #   1) MAX_STEP_DB: caps how far one packet can move the estimate
        #   2) EMA_ALPHA: smooths on top of that
        self.filtered_rssi = dict(self.rssi)
        self.MAX_STEP_DB = 10                # tighter clamp than before
        self.EMA_ALPHA = 0.2                 # heavier smoothing than before

        # Second smoothing pass, applied to the FINAL computed target position
        # (not RSSI). Even smoothed RSSI can still cause small jitter in the
        # near/far mode switch or the direction vector; smoothing the actual
        # x/y position removes the last bit of visible jumpiness in both the
        # dot on screen AND the distance/direction numbers (which are derived
        # from this same smoothed vector).
        self.smoothed_target_x = 0.0
        self.smoothed_target_y = 0.0
        self.POS_SMOOTHING_ALPHA = 0.2

        self.target_visible = True
        self.serial_port = None
        self.is_reading_serial = False
        self.packet_count = 0
        self.last_update = {1: "---", 2: "---", 3: "---", 4: "---"}

        self.setup_ui()
        self.update_radar()
        self.blink_target()

    def default_port_for_os(self):
        """Return a sensible placeholder port name depending on the OS.
        This is only a starting suggestion in an editable field — the user
        can type any port name (COM3, /dev/ttyUSB1, /dev/tty.usbserial-*, etc.)."""
        system = platform.system()
        if system == "Windows":
            return "COM3"
        elif system == "Darwin":  # macOS
            return "/dev/tty.usbserial-0001"
        else:  # Linux and others
            return "/dev/ttyUSB0"

    def setup_ui(self):
        self.canvas = tk.Canvas(self.root, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE, bg="#16213e", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)

        control_frame = tk.Frame(self.root, bg="#1a1a2e")
        control_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- Serial Connection ---
        ser_frame = tk.Frame(control_frame, bg="#0f3460", padx=8, pady=8)
        ser_frame.pack(fill=tk.X, pady=5)
        tk.Label(ser_frame, text="LoRa Port:", fg="white", bg="#0f3460", font=("Arial", 10, "bold")).pack(anchor="w")

        port_row = tk.Frame(ser_frame, bg="#0f3460")
        port_row.pack(fill=tk.X, pady=3)
        self.com_entry = tk.Entry(port_row, width=18, font=("Arial", 11))
        self.com_entry.insert(0, self.default_port_for_os())
        self.com_entry.pack(side=tk.LEFT, padx=(0, 5))

        self.btn_connect = tk.Button(port_row, text="BAĞLAN", bg="#27ae60", fg="white",
                                     font=("Arial", 9, "bold"), command=self.toggle_serial)
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.status_label = tk.Label(ser_frame, text="⚪ Bağlantı Yok", fg="#95a5a6", bg="#0f3460", font=("Arial", 9))
        self.status_label.pack(anchor="w", pady=2)

        # --- Log Monitor ---
        log_frame = tk.Frame(control_frame, bg="#0f3460", padx=5, pady=5)
        log_frame.pack(fill=tk.X, pady=5)
        tk.Label(log_frame, text="Ham Veri (LoRa Çıkışı):", fg="white", bg="#0f3460", font=("Arial", 9, "bold")).pack(anchor="w")
        self.log_text = tk.Text(log_frame, height=5, width=25, bg="#0a0a23", fg="#2ecc71",
                                font=("Courier", 9), state=tk.DISABLED)
        self.log_text.pack(fill=tk.X)

        # --- RSSI Bars ---
        tk.Label(control_frame, text="Canlı Sinyal (dBm)", fg="white", bg="#1a1a2e", font=("Arial", 12, "bold")).pack(pady=5)

        self.sliders = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(control_frame, bg="#1a1a2e")
            frame.pack(fill=tk.X, pady=2)
            tk.Label(frame, text=data["name"], fg=data["color"], bg="#1a1a2e", font=("Arial", 9, "bold")).pack(anchor="w")
            slider = tk.Scale(frame, from_=-60, to=-120, orient=tk.HORIZONTAL, bg="#16213e", fg="white",
                              troughcolor="#7f8c8d", highlightthickness=0,
                              command=lambda val, i=d_id: self.on_slider_change(i, val))
            slider.set(self.rssi[d_id])
            slider.pack(fill=tk.X)
            self.sliders[d_id] = slider

        # --- Reset button (previously defined but never wired up) ---
        self.btn_reset = tk.Button(control_frame, text="SİNYALLERİ SIFIRLA", bg="#7f8c8d", fg="white",
                                    font=("Arial", 9, "bold"), command=self.reset_signals)
        self.btn_reset.pack(fill=tk.X, pady=(2, 8))

        # --- Öne Çıkan Başlık: hangi drone'un kaç metre ilerisinde ---
        self.headline_label = tk.Label(control_frame, text="Hedef Aranıyor...", fg="#e74c3c", bg="#1a1a2e",
                                        font=("Arial", 15, "bold"), justify=tk.LEFT, wraplength=380)
        self.headline_label.pack(pady=(8, 4), anchor="w")

        # --- Info ---
        self.info_label = tk.Label(control_frame, text="Hedef Aranıyor...", fg="#f1c40f", bg="#1a1a2e",
                                   font=("Arial", 11), justify=tk.LEFT)
        self.info_label.pack(pady=10, anchor="w")

    def _update_filtered_rssi(self, drone_id, raw_val):
        """Slew-rate limit + EMA smoothing so one noisy/corrupted packet
        can't single-handedly swing the distance estimate or flip near/far mode."""
        prev = self.filtered_rssi.get(drone_id, raw_val)
        clamped = max(min(raw_val, prev + self.MAX_STEP_DB), prev - self.MAX_STEP_DB)
        self.filtered_rssi[drone_id] = self.EMA_ALPHA * clamped + (1 - self.EMA_ALPHA) * prev

    def toggle_serial(self):
        if not self.is_reading_serial:
            port = self.com_entry.get().strip()
            try:
                self.serial_port = serial.Serial(port, LORA_BAUD, timeout=1)
                self.is_reading_serial = True
                self.packet_count = 0
                self.btn_connect.config(text="KES", bg="#c0392b")
                self.status_label.config(text="🟢 Bağlı - Veri bekleniyor...", fg="#2ecc71")
                threading.Thread(target=self.serial_reader_loop, daemon=True).start()
            except Exception as e:
                self.status_label.config(text=f"🔴 HATA: {e}", fg="#e74c3c")
        else:
            self.is_reading_serial = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.btn_connect.config(text="BAĞLAN", bg="#27ae60")
            self.status_label.config(text="⚪ Bağlantı Yok", fg="#95a5a6")

    def serial_reader_loop(self):
        buffer = ""
        while self.is_reading_serial:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    break
                raw = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not raw:
                    continue
                buffer += raw.decode('ascii', errors='ignore')

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    # Log'a yaz
                    self.root.after(0, self.append_log, line)

                    # Parse: N{id},{rssi} veya N{id},{rssi},{checksum}
                    if line.startswith("N"):
                        parts = line[1:].split(",")
                        if len(parts) >= 2:
                            try:
                                drone_id = int(parts[0])
                                rssi_val = int(parts[1])
                                if drone_id in self.rssi:
                                    self.packet_count += 1
                                    self.root.after(0, self.update_from_serial, drone_id, rssi_val)
                            except ValueError:
                                pass
            except serial.SerialException:
                self.root.after(0, self.serial_error)
                break
            except Exception:
                pass

    def serial_error(self):
        self.is_reading_serial = False
        self.btn_connect.config(text="BAĞLAN", bg="#27ae60")
        self.status_label.config(text="🔴 Bağlantı Koptu!", fg="#e74c3c")

    def append_log(self, line):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        # Max 50 satır tut
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 50:
            self.log_text.delete('1.0', '2.0')
        self.log_text.config(state=tk.DISABLED)

    def update_from_serial(self, drone_id, rssi_val):
        self.rssi[drone_id] = rssi_val  # ham değer - şeffaflık için slider/log'da olduğu gibi gösterilir
        calibrated = rssi_val + self.CALIBRATION_OFFSET_DB.get(drone_id, 0.0)
        self._update_filtered_rssi(drone_id, calibrated)
        self.last_update[drone_id] = time.strftime("%H:%M:%S")
        if drone_id in self.sliders:
            self.sliders[drone_id].set(rssi_val)
        self.status_label.config(text=f"🟢 Bağlı - {self.packet_count} paket alındı", fg="#2ecc71")
        self.update_radar()

    def on_slider_change(self, drone_id, value):
        if not self.is_reading_serial:
            self.rssi[drone_id] = int(value)
            calibrated = float(value) + self.CALIBRATION_OFFSET_DB.get(drone_id, 0.0)
            self.filtered_rssi[drone_id] = calibrated
            self.update_radar()

    def reset_signals(self):
        for d_id in self.sliders:
            self.sliders[d_id].set(-90)
            self.rssi[d_id] = -90
            self.filtered_rssi[d_id] = -90
        self.smoothed_target_x = 0.0
        self.smoothed_target_y = 0.0
        self.update_radar()

    def update_radar(self):
        self.canvas.delete("all")

        # Grid
        for i in range(0, self.CANVAS_SIZE, self.SCALE):
            self.canvas.create_line(i, 0, i, self.CANVAS_SIZE, fill="#1a1a2e")
            self.canvas.create_line(0, i, self.CANVAS_SIZE, i, fill="#1a1a2e")

        # Axes
        self.canvas.create_line(self.CENTER, 0, self.CENTER, self.CANVAS_SIZE, fill="#7f8c8d", dash=(4, 4))
        self.canvas.create_line(0, self.CENTER, self.CANVAS_SIZE, self.CENTER, fill="#7f8c8d", dash=(4, 4))

        # Compass labels
        self.canvas.create_text(self.CENTER, 15, text="KUZEY (N)", fill="#ecf0f1", font=("Arial", 10, "bold"))
        self.canvas.create_text(self.CENTER, self.CANVAS_SIZE - 15, text="GÜNEY (S)", fill="#ecf0f1", font=("Arial", 10, "bold"))
        self.canvas.create_text(self.CANVAS_SIZE - 35, self.CENTER, text="DOĞU (E)", fill="#ecf0f1", font=("Arial", 9, "bold"))
        self.canvas.create_text(35, self.CENTER, text="BATI (W)", fill="#ecf0f1", font=("Arial", 9, "bold"))

        # Virtual center
        self.canvas.create_oval(self.CENTER - 5, self.CENTER - 5, self.CENTER + 5, self.CENTER + 5, fill="white", outline="#ecf0f1")
        self.canvas.create_text(self.CENTER + 35, self.CENTER + 15, text="Sanal Merkez", fill="#bdc3c7", font=("Arial", 8))

        # Drones
        for d_id, data in self.drones.items():
            cx = self.CENTER + (data["x"] * self.SCALE)
            cy = self.CENTER - (data["y"] * self.SCALE)

            # Drone circle
            self.canvas.create_oval(cx - 12, cy - 12, cx + 12, cy + 12, fill=data["color"], outline="white", width=2)
            self.canvas.create_text(cx, cy, text=str(d_id), fill="white", font=("Arial", 9, "bold"))
            self.canvas.create_text(cx, cy + 22, text=data["name"], fill=data["color"], font=("Arial", 8, "bold"))
            self.canvas.create_text(cx, cy - 22, text=f"{self.rssi[d_id]} dBm", fill="white", font=("Arial", 9, "bold"))

        # --- Range ring (visible radar radius — just a display boundary, not the real max range) ---
        ring_r = self.DISPLAY_RING_M * self.SCALE
        self.canvas.create_oval(self.CENTER - ring_r, self.CENTER - ring_r,
                                 self.CENTER + ring_r, self.CENTER + ring_r,
                                 outline="#34495e", dash=(2, 3))
        self.canvas.create_text(self.CENTER, self.CENTER - ring_r - 10,
                                 text=f"Görüntüleme Sınırı ~{self.DISPLAY_RING_M:.0f} m",
                                 fill="#7f8c8d", font=("Arial", 8))

        # Direction comes from the RSSI imbalance between opposing drones (gradient).
        # Uses FILTERED values (slew-limited + smoothed) so a single corrupted
        # packet on one drone can't swing the estimate — see _update_filtered_rssi.
        delta_y = self.filtered_rssi[1] - self.filtered_rssi[2]
        delta_x = self.filtered_rssi[3] - self.filtered_rssi[4]
        avg_rssi = sum(self.filtered_rssi.values()) / len(self.filtered_rssi)

        dir_mag = math.sqrt(delta_x ** 2 + delta_y ** 2)
        if dir_mag > 1e-6:
            dir_x, dir_y = delta_x / dir_mag, delta_y / dir_mag
        else:
            dir_x, dir_y = 0.0, 0.0

        has_signal = avg_rssi > (self.NO_SIGNAL_RSSI + 1)
        nearest_drone = None
        min_dist = None

        if has_signal:
            # Two independent estimates:
            #  - near_dist: gradient/triangulation, precise INSIDE the drone cluster,
            #    meaningless once the target is far (all 4 drones read almost the same RSSI)
            #  - far_dist: average-RSSI path-loss model, the only signal that carries
            #    real information once the target is beyond the cluster
            near_dist = min(self.K_GAIN * dir_mag, self.OFFSET_M)

            # Gerçek fit edilmiş Friis/log-distance path-loss modeli
            # (13 noktalı kalibrasyon oturumundan): avg_rssi zaten yukarıda
            # kalibrasyon ofseti uygulanmış+Kalman/EMA-filtrelenmiş
            # değerlerin ortalaması, o yüzden doğrudan formüle sokulabilir.
            #   d = 10 ^ ((RSSI_REF_1M - RSSI) / (10 * PATH_LOSS_EXPONENT))
            far_dist = 10 ** ((self.RSSI_REF_1M - avg_rssi) / (10 * self.PATH_LOSS_EXPONENT))
            far_dist = max(far_dist, self.MIN_RANGE_M)

            inside_cluster = far_dist <= self.OFFSET_M
            if inside_cluster:
                # Trust the fine-grained triangulation near the drones
                real_distance_m = near_dist
                mode_label = "Yakın Alan (Üçgenleme)"
            else:
                # Trust the absolute RSSI distance model out here
                real_distance_m = far_dist
                mode_label = "Uzak Alan (RSSI Mesafe Tahmini)"

            out_of_range = real_distance_m > self.DISPLAY_RING_M
            display_dist = min(real_distance_m, self.DISPLAY_RING_M)
            raw_target_x_m = dir_x * display_dist
            raw_target_y_m = dir_y * display_dist

            # Smooth the position vector itself (on top of the RSSI smoothing
            # above). This is what finally removes the last bit of jitter —
            # both the drawn dot AND the distance/direction text below are
            # derived from this smoothed vector, so they move in lockstep
            # and stay calm together instead of the number and dot jittering
            # independently.
            a = self.POS_SMOOTHING_ALPHA
            self.smoothed_target_x = a * raw_target_x_m + (1 - a) * self.smoothed_target_x
            self.smoothed_target_y = a * raw_target_y_m + (1 - a) * self.smoothed_target_y
            target_x_m, target_y_m = self.smoothed_target_x, self.smoothed_target_y

            # Displayed distance/direction now come from the SMOOTHED vector
            smoothed_display_dist = math.sqrt(target_x_m ** 2 + target_y_m ** 2)
            # Rescale real_distance_m proportionally so the printed meters
            # stays consistent with what's actually drawn (avoids a smoothed
            # dot next to an unsmoothed, jumpy number).
            if display_dist > 1e-6:
                real_distance_m = real_distance_m * (smoothed_display_dist / display_dist)

            t_px = self.CENTER + (target_x_m * self.SCALE)
            t_py = self.CENTER - (target_y_m * self.SCALE)

            # Nearest drone uses the REAL (uncapped) estimated position
            real_x_m, real_y_m = dir_x * real_distance_m, dir_y * real_distance_m
            min_dist = float('inf')
            nearest_drone_id = None
            for d_id, data in self.drones.items():
                dx = real_x_m - data["x"]
                dy = real_y_m - data["y"]
                dist = math.sqrt(dx ** 2 + dy ** 2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_drone = data["name"]
                    nearest_drone_id = d_id

            dot_color = "#f39c12" if out_of_range else "#e74c3c"
            headline = f"Drone {nearest_drone_id} ({nearest_drone.split('(')[-1].rstrip(')')}) → {min_dist:.1f}m İLERİDE"
            label = f"📻 {headline}" + (" (MENZİL DIŞI)" if out_of_range else "")

            if self.target_visible:
                self.canvas.create_oval(t_px - 14, t_py - 14, t_px + 14, t_py + 14, fill="", outline=dot_color, width=2)
                self.canvas.create_oval(t_px - 8, t_py - 8, t_px + 8, t_py + 8, fill=dot_color, outline="white", width=2)
                self.canvas.create_text(t_px, t_py - 22, text=label, fill=dot_color, font=("Arial", 9, "bold"))

            self.canvas.create_line(self.CENTER, self.CENTER, t_px, t_py, fill=dot_color, dash=(3, 3), width=2)

            if smoothed_display_dist > 0.5:
                direction = self.angle_to_direction(math.degrees(math.atan2(target_y_m, target_x_m)))
            else:
                direction = "MERKEZ"

            info_text = (
                f"🎯 {headline}\n\n"
                f"📍 Hedef Yönü: {direction}\n"
                f"   Mod: {mode_label}\n"
                f"   Tahmini Mesafe (merkeze göre): {real_distance_m:.1f} m\n\n"
                f"🚀 En Yakın: {nearest_drone}\n"
                f"   Mesafe: {min_dist:.1f} m\n\n"
                f"📊 Delta K-G: {delta_y:+.1f} dBm  |  Delta D-B: {delta_x:+.1f} dBm\n"
                f"   Ort. RSSI (kalibre): {avg_rssi:.0f} dBm"
            )
            self.headline_label.config(
                text=("⚠️ " + headline if out_of_range else "🎯 " + headline),
                fg=("#f39c12" if out_of_range else "#e74c3c")
            )
        else:
            info_text = "📻 Hedef bekleniyor...\n   (Henüz sinyal alınmadı)"
            self.headline_label.config(text="📻 Hedef Aranıyor...", fg="#95a5a6")

        self.info_label.config(text=info_text)

    def angle_to_direction(self, deg):
        if -22.5 <= deg < 22.5:
            return "→ DOĞU"
        elif 22.5 <= deg < 67.5:
            return "↗ KUZEY-DOĞU"
        elif 67.5 <= deg < 112.5:
            return "↑ KUZEY"
        elif 112.5 <= deg < 157.5:
            return "↖ KUZEY-BATI"
        elif deg >= 157.5 or deg < -157.5:
            return "← BATI"
        elif -157.5 <= deg < -112.5:
            return "↙ GÜNEY-BATI"
        elif -112.5 <= deg < -67.5:
            return "↓ GÜNEY"
        elif -67.5 <= deg < -22.5:
            return "↘ GÜNEY-DOĞU"
        return "?"

    def blink_target(self):
        self.target_visible = not self.target_visible
        self.update_radar()
        self.root.after(500, self.blink_target)


if __name__ == "__main__":
    root = tk.Tk()
    app = SwarmRadar(root)
    root.mainloop()