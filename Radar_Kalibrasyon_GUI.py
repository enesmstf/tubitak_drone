import tkinter as tk
import math
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
        self.K_GAIN = 0.5
        self.SCALE = 20
        self.CANVAS_SIZE = 600
        self.CENTER = self.CANVAS_SIZE // 2

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
        self.com_entry.insert(0, "/dev/ttyUSB0")
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
                except:
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

        # Target calculation (same algorithm as ground_station_core)
        delta_y = self.rssi[1] - self.rssi[2]
        delta_x = self.rssi[3] - self.rssi[4]

        target_x_m = delta_x * self.K_GAIN
        target_y_m = delta_y * self.K_GAIN

        # Clamp (max 5m per step, same as real code)
        mag = math.sqrt(target_x_m ** 2 + target_y_m ** 2)
        if mag > 5.0:
            target_x_m = (target_x_m / mag) * 5.0
            target_y_m = (target_y_m / mag) * 5.0

        t_px = self.CENTER + (target_x_m * self.SCALE)
        t_py = self.CENTER - (target_y_m * self.SCALE)

        # Nearest drone
        min_dist = float('inf')
        nearest_drone = None
        for d_id, data in self.drones.items():
            dx = target_x_m - data["x"]
            dy = target_y_m - data["y"]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist < min_dist:
                min_dist = dist
                nearest_drone = data["name"]

        # Target blinking dot
        if self.target_visible:
            # Glow effect
            self.canvas.create_oval(t_px - 14, t_py - 14, t_px + 14, t_py + 14, fill="", outline="#e74c3c", width=2)
            self.canvas.create_oval(t_px - 8, t_py - 8, t_px + 8, t_py + 8, fill="#e74c3c", outline="white", width=2)
            self.canvas.create_text(t_px, t_py - 22, text="📻 HDF", fill="#e74c3c", font=("Arial", 9, "bold"))

        # Line from center to target
        self.canvas.create_line(self.CENTER, self.CENTER, t_px, t_py, fill="#e74c3c", dash=(3, 3), width=2)

        # Direction arrow indicator
        if mag > 0.5:
            angle = math.atan2(target_y_m, target_x_m)
            angle_deg = math.degrees(angle)
            direction = self.angle_to_direction(angle_deg)
        else:
            direction = "MERKEZ"

        # Info panel
        info_text = (
            f"📍 Hedef Yönü: {direction}\n"
            f"   X: {target_x_m:+.1f} m  |  Y: {target_y_m:+.1f} m\n\n"
            f"🚀 En Yakın: {nearest_drone}\n"
            f"   Mesafe: {min_dist:.1f} m\n\n"
            f"📊 Delta K-G: {delta_y:+d} dBm\n"
            f"   Delta D-B: {delta_x:+d} dBm"
        )
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
