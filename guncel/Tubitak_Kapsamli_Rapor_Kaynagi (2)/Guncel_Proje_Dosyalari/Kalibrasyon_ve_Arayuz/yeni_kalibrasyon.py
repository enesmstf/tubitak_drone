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

        # --- Distance estimation ---
        # IMPORTANT: RSSI_STRONG / RSSI_WEAK / REAL_MAX_RANGE_M below are
        # PLACEHOLDERS. Calibrate them with real field readings from YOUR
        # hardware: note the avg RSSI at a few known distances (e.g. 1m,
        # 10m, 50m, 100m) and adjust these three numbers to match.
        self.RSSI_STRONG = -60              # avg RSSI when target is essentially on top of the cluster
        self.RSSI_WEAK = -125               # avg RSSI at your true max operating range (no more usable signal)
        self.MIN_RANGE_M = 0.5              # real distance at RSSI_STRONG
        self.REAL_MAX_RANGE_M = 150         # real distance at RSSI_WEAK (calibration anchor, NOT a display limit)
        self.NO_SIGNAL_RSSI = self.RSSI_WEAK - 3   # floor below which we treat it as no data at all
        self.DISPLAY_RING_M = 13            # radar's VISUAL radius on the canvas (independent of real range)

        self.drones = {
            1: {"name": "Drone 1 (Kuzey)", "x": 0,              "y": self.OFFSET_M,  "color": "#3498db"},
            2: {"name": "Drone 2 (Güney)", "x": 0,              "y": -self.OFFSET_M, "color": "#e67e22"},
            3: {"name": "Drone 3 (Doğu)",  "x": self.OFFSET_M,  "y": 0,              "color": "#9b59b6"},
            4: {"name": "Drone 4 (Batı)",  "x": -self.OFFSET_M, "y": 0,              "color": "#1abc9c"}
        }

        self.rssi = {1: -90, 2: -90, 3: -90, 4: -90}
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

        # --- Info ---
        self.info_label = tk.Label(control_frame, text="Hedef Aranıyor...", fg="#f1c40f", bg="#1a1a2e",
                                   font=("Arial", 11), justify=tk.LEFT)
        self.info_label.pack(pady=10, anchor="w")

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
        self.rssi[drone_id] = rssi_val
        self.last_update[drone_id] = time.strftime("%H:%M:%S")
        if drone_id in self.sliders:
            self.sliders[drone_id].set(rssi_val)
        self.status_label.config(text=f"🟢 Bağlı - {self.packet_count} paket alındı", fg="#2ecc71")
        self.update_radar()

    def on_slider_change(self, drone_id, value):
        if not self.is_reading_serial:
            self.rssi[drone_id] = int(value)
            self.update_radar()

    def reset_signals(self):
        for d_id in self.sliders:
            self.sliders[d_id].set(-90)
            self.rssi[d_id] = -90
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
        delta_y = self.rssi[1] - self.rssi[2]
        delta_x = self.rssi[3] - self.rssi[4]
        avg_rssi = sum(self.rssi.values()) / len(self.rssi)

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

            # Interpolate in log10(distance) space between the two calibration
            # anchors: RSSI_STRONG -> MIN_RANGE_M and RSSI_WEAK -> REAL_MAX_RANGE_M.
            # This is calibrated against your REAL operating range (e.g. 150m),
            # separate from DISPLAY_RING_M which only controls how it's drawn.
            # So a target at 100m now computes an honest ~100m estimate instead
            # of being compressed toward whatever the old 13m display cap was.
            frac = (self.RSSI_STRONG - avg_rssi) / (self.RSSI_STRONG - self.RSSI_WEAK)
            frac = min(max(frac, 0.0), 1.0)
            log_d = math.log10(self.MIN_RANGE_M) + frac * (math.log10(self.REAL_MAX_RANGE_M) - math.log10(self.MIN_RANGE_M))
            far_dist = 10 ** log_d

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
            target_x_m = dir_x * display_dist
            target_y_m = dir_y * display_dist

            t_px = self.CENTER + (target_x_m * self.SCALE)
            t_py = self.CENTER - (target_y_m * self.SCALE)

            # Nearest drone uses the REAL (uncapped) estimated position
            real_x_m, real_y_m = dir_x * real_distance_m, dir_y * real_distance_m
            min_dist = float('inf')
            for d_id, data in self.drones.items():
                dx = real_x_m - data["x"]
                dy = real_y_m - data["y"]
                dist = math.sqrt(dx ** 2 + dy ** 2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_drone = data["name"]

            dot_color = "#f39c12" if out_of_range else "#e74c3c"
            label = "📻 HDF (MENZİL DIŞI)" if out_of_range else "📻 HDF"

            if self.target_visible:
                self.canvas.create_oval(t_px - 14, t_py - 14, t_px + 14, t_py + 14, fill="", outline=dot_color, width=2)
                self.canvas.create_oval(t_px - 8, t_py - 8, t_px + 8, t_py + 8, fill=dot_color, outline="white", width=2)
                self.canvas.create_text(t_px, t_py - 22, text=label, fill=dot_color, font=("Arial", 9, "bold"))

            self.canvas.create_line(self.CENTER, self.CENTER, t_px, t_py, fill=dot_color, dash=(3, 3), width=2)

            if dir_mag > 0.5:
                direction = self.angle_to_direction(math.degrees(math.atan2(dir_y, dir_x)))
            else:
                direction = "MERKEZ"

            info_text = (
                f"📍 Hedef Yönü: {direction}\n"
                f"   Mod: {mode_label}\n"
                f"   Tahmini Mesafe: {real_distance_m:.1f} m\n\n"
                f"🚀 En Yakın: {nearest_drone}\n"
                f"   Mesafe: {min_dist:.1f} m\n\n"
                f"📊 Delta K-G: {delta_y:+d} dBm  |  Delta D-B: {delta_x:+d} dBm\n"
                f"   Ort. RSSI: {avg_rssi:.0f} dBm"
            )
        else:
            info_text = "📻 Hedef bekleniyor...\n   (Henüz sinyal alınmadı)"

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