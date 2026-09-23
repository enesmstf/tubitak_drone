import tkinter as tk
import math
import serial
import threading
import time

LORA_BAUD = 9600

# ==========================================
# KALİBRASYON OFSETLERİ
# ==========================================
# otomatik_kalibrasyon.py çıktısını buraya yapıştırın
CALIBRATION_OFFSET_DB = {
    1: -2.1,   # Kuzey (D1)
    2:  1.9,   # Güney (D2)
    3: -0.1,   # Doğu  (D3)
    4:  0.2,   # Batı  (D4)
}


class SwarmRadar:
    def __init__(self, root):
        self.root = root
        self.root.title("TÜBİTAK Otonom Sürü - Radar (İntegrator)")
        self.root.geometry("950x700")
        self.root.configure(bg="#1a1a2e")

        self.OFFSET_M = 5.0          # Dronların merkeze mesafesi
        self.CANVAS_SIZE = 600
        self.CENTER = self.CANVAS_SIZE // 2

        # Radar ölçeği: başlangıçta 1m = 20px, zoom ile değişir
        self.SCALE = 20.0
        self.MIN_SCALE = 4.0          # En uzak zoom (1m = 4px → ~75m görüş)
        self.MAX_SCALE = 40.0         # En yakın zoom (1m = 40px)

        # Integrator hızı (metre/adım per dBm fark)
        self.INTEGRATOR_GAIN = 0.3    # Her güncelleme: shift = fark × gain
        self.MAX_STEP_M = 2.0         # Tek adımda max kayma

        # Geofence (metre) - sanal merkez buradan öteye gidemez
        self.GEOFENCE_M = 50.0

        self.drones = {
            1: {"name": "D1 Kuzey", "x":  0.0, "y":  self.OFFSET_M, "color": "#3498db"},
            2: {"name": "D2 Güney", "x":  0.0, "y": -self.OFFSET_M, "color": "#e67e22"},
            3: {"name": "D3 Doğu",  "x":  self.OFFSET_M, "y": 0.0,  "color": "#9b59b6"},
            4: {"name": "D4 Batı",  "x": -self.OFFSET_M, "y": 0.0,  "color": "#1abc9c"},
        }

        # Ham ve kalibreli RSSI
        self.rssi_raw = {1: -90, 2: -90, 3: -90, 4: -90}
        self.rssi_cal = {1: -90, 2: -90, 3: -90, 4: -90}

        # İntegrator durumu: sanal merkezin birikimli konumu
        self.virtual_x = 0.0   # Doğu+
        self.virtual_y = 0.0   # Kuzey+

        self.target_visible = True
        self.serial_port = None
        self.is_reading_serial = False
        self.packet_count = 0

        # İz çizgisi (trail)
        self.trail = []
        self.MAX_TRAIL = 200

        self.setup_ui()
        self.update_radar()
        self.blink_target()

    def setup_ui(self):
        self.canvas = tk.Canvas(
            self.root, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE,
            bg="#16213e", highlightthickness=0
        )
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)

        # Mouse wheel zoom
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)       # Windows
        self.canvas.bind("<Button-4>", self._on_mousewheel_linux)   # Linux up
        self.canvas.bind("<Button-5>", self._on_mousewheel_linux)   # Linux down

        control = tk.Frame(self.root, bg="#1a1a2e")
        control.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ── Test Modu Butonu ──
        self.bench_mode = True
        self.btn_mode = tk.Button(
            control, text="Mod: MASAÜSTÜ (Anlık Gösterim)", bg="#8e44ad", fg="white",
            font=("Arial", 10, "bold"), command=self.toggle_mode
        )
        self.btn_mode.pack(fill=tk.X, pady=5)
        
        tk.Label(control, text="Masaüstü testi için üstteki butonu kullanın.\nUçuş modunda dronlar hareket etmezse\nhedef sonsuza kayar.", 
                 fg="#95a5a6", bg="#1a1a2e", font=("Arial", 8)).pack(pady=2)

        # ── Serial ──
        ser_frame = tk.Frame(control, bg="#0f3460", padx=8, pady=8)
        ser_frame.pack(fill=tk.X, pady=5)
        tk.Label(ser_frame, text="LoRa Port:", fg="white", bg="#0f3460",
                 font=("Arial", 10, "bold")).pack(anchor="w")
        port_row = tk.Frame(ser_frame, bg="#0f3460")
        port_row.pack(fill=tk.X, pady=3)
        self.com_entry = tk.Entry(port_row, width=18, font=("Arial", 11))
        self.com_entry.insert(0, "/dev/ttyUSB0")
        self.com_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.btn_connect = tk.Button(
            port_row, text="BAĞLAN", bg="#27ae60", fg="white",
            font=("Arial", 9, "bold"), command=self.toggle_serial
        )
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status_label = tk.Label(
            ser_frame, text="⚪ Bağlantı Yok", fg="#95a5a6", bg="#0f3460",
            font=("Arial", 9)
        )
        self.status_label.pack(anchor="w", pady=2)

        # ── Log ──
        log_frame = tk.Frame(control, bg="#0f3460", padx=5, pady=5)
        log_frame.pack(fill=tk.X, pady=5)
        tk.Label(log_frame, text="Ham Veri:", fg="white", bg="#0f3460",
                 font=("Arial", 9, "bold")).pack(anchor="w")
        self.log_text = tk.Text(
            log_frame, height=4, width=28, bg="#0a0a23", fg="#2ecc71",
            font=("Courier", 9), state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.X)

        # ── RSSI Barlar ──
        tk.Label(control, text="Kalibreli Sinyal (dBm)", fg="white",
                 bg="#1a1a2e", font=("Arial", 12, "bold")).pack(pady=5)

        self.sliders = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(control, bg="#1a1a2e")
            frame.pack(fill=tk.X, pady=1)
            tk.Label(frame, text=data["name"], fg=data["color"],
                     bg="#1a1a2e", font=("Arial", 9, "bold")).pack(anchor="w")
            slider = tk.Scale(
                frame, from_=-40, to=-120, orient=tk.HORIZONTAL,
                bg="#16213e", fg="white", troughcolor="#7f8c8d",
                highlightthickness=0,
                command=lambda val, i=d_id: self.on_slider_change(i, val)
            )
            slider.set(self.rssi_raw[d_id])
            slider.pack(fill=tk.X)
            self.sliders[d_id] = slider

        # ── Sıfırla butonu ──
        btn_frame = tk.Frame(control, bg="#1a1a2e")
        btn_frame.pack(fill=tk.X, pady=5)
        tk.Button(
            btn_frame, text="🔄 Merkezi Sıfırla", bg="#c0392b", fg="white",
            font=("Arial", 9, "bold"), command=self.reset_center
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Button(
            btn_frame, text="🗑 İzi Temizle", bg="#7f8c8d", fg="white",
            font=("Arial", 9, "bold"), command=self.clear_trail
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # ── Bilgi paneli ──
        self.info_label = tk.Label(
            control, text="Veri bekleniyor...", fg="#f1c40f", bg="#1a1a2e",
            font=("Arial", 10), justify=tk.LEFT, anchor="w"
        )
        self.info_label.pack(pady=5, fill=tk.X)

    def toggle_mode(self):
        self.bench_mode = not self.bench_mode
        if self.bench_mode:
            self.btn_mode.config(text="Mod: MASAÜSTÜ (Anlık Gösterim)", bg="#8e44ad")
        else:
            self.btn_mode.config(text="Mod: UÇUŞ (İntegrator Birikimli)", bg="#d35400")
        self.reset_center()

    # ─────────────────────────────────────────────
    # ZOOM
    # ─────────────────────────────────────────────
    def _on_mousewheel(self, event):
        if event.delta > 0:
            self.SCALE = min(self.MAX_SCALE, self.SCALE * 1.15)
        else:
            self.SCALE = max(self.MIN_SCALE, self.SCALE / 1.15)
        self.update_radar()

    def _on_mousewheel_linux(self, event):
        if event.num == 4:
            self.SCALE = min(self.MAX_SCALE, self.SCALE * 1.15)
        else:
            self.SCALE = max(self.MIN_SCALE, self.SCALE / 1.15)
        self.update_radar()

    # ─────────────────────────────────────────────
    # SERİ PORT
    # ─────────────────────────────────────────────
    def toggle_serial(self):
        if not self.is_reading_serial:
            port = self.com_entry.get().strip()
            if not port.startswith("/") and not port.startswith("COM"):
                port = "/dev/" + port
            try:
                self.serial_port = serial.Serial(port, LORA_BAUD, timeout=1)
                self.is_reading_serial = True
                self.packet_count = 0
                self.btn_connect.config(text="KES", bg="#c0392b")
                self.status_label.config(text="🟢 Bağlı", fg="#2ecc71")
                threading.Thread(target=self._serial_loop, daemon=True).start()
            except Exception as e:
                self.status_label.config(text=f"🔴 {e}", fg="#e74c3c")
        else:
            self.is_reading_serial = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            self.btn_connect.config(text="BAĞLAN", bg="#27ae60")
            self.status_label.config(text="⚪ Bağlantı Yok", fg="#95a5a6")

    def _serial_loop(self):
        buf = ""
        while self.is_reading_serial:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    break
                raw = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not raw:
                    continue
                buf += raw.decode("ascii", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        self.root.after(0, self._append_log, line)
                        if line.startswith("N"):
                            parts = line[1:].split(",")
                            if len(parts) >= 2:
                                try:
                                    did = int(parts[0])
                                    rssi = int(parts[1])
                                    if did in self.rssi_raw:
                                        # Checksum kontrolü (varsa)
                                        if len(parts) == 3:
                                            chk = int(parts[2])
                                            body = f"N{did},{rssi}"
                                            expected = sum(ord(c) for c in body) % 256
                                            if chk != expected:
                                                continue
                                        self.packet_count += 1
                                        self.root.after(0, self._update_rssi, did, rssi)
                                except ValueError:
                                    pass
            except serial.SerialException:
                self.root.after(0, self._serial_error)
                break
            except:
                pass

    def _serial_error(self):
        self.is_reading_serial = False
        self.btn_connect.config(text="BAĞLAN", bg="#27ae60")
        self.status_label.config(text="🔴 Bağlantı Koptu!", fg="#e74c3c")

    def _append_log(self, line):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        n = int(self.log_text.index("end-1c").split(".")[0])
        if n > 40:
            self.log_text.delete("1.0", "2.0")
        self.log_text.config(state=tk.DISABLED)

    def _update_rssi(self, did, rssi):
        self.rssi_raw[did] = rssi
        self.rssi_cal[did] = rssi + CALIBRATION_OFFSET_DB.get(did, 0.0)
        if did in self.sliders:
            self.sliders[did].set(rssi)
        self.status_label.config(
            text=f"🟢 {self.packet_count} paket", fg="#2ecc71"
        )
        # Her yeni veri geldiğinde integrator'ı güncelle
        self._integrator_step()
        self.update_radar()

    def on_slider_change(self, did, value):
        if not self.is_reading_serial:
            self.rssi_raw[did] = int(value)
            self.rssi_cal[did] = int(value) + CALIBRATION_OFFSET_DB.get(did, 0.0)
            self._integrator_step()
            self.update_radar()

    # ─────────────────────────────────────────────
    # İNTEGRATOR (BİRİKTİRİCİ) KONTROL
    # ─────────────────────────────────────────────
    def _integrator_step(self):
        p1 = self.rssi_cal[1]
        p2 = self.rssi_cal[2]
        p3 = self.rssi_cal[3]
        p4 = self.rssi_cal[4]

        delta_ew = p3 - p4   # Doğu-Batı farkı
        delta_ns = p1 - p2   # Kuzey-Güney farkı

        DEAD_ZONE_DB = 2.0
        if abs(delta_ew) < DEAD_ZONE_DB:
            delta_ew = 0.0
        if abs(delta_ns) < DEAD_ZONE_DB:
            delta_ns = 0.0

        if self.bench_mode:
            # MASAÜSTÜ MODU (Anlık Konum)
            # Biriktirme yapmaz, anlık güç farkını doğrudan mesafeye çevirir.
            # Ekranda hedefin tam o anki göreceli yerini görmek içindir.
            self.virtual_x = delta_ew * 1.5  # 1 dBm = 1.5 metre görsel çarpan
            self.virtual_y = delta_ns * 1.5
            
            # Sınırla
            MAX_VISUAL = 25.0
            mag = math.sqrt(self.virtual_x ** 2 + self.virtual_y ** 2)
            if mag > MAX_VISUAL:
                self.virtual_x = (self.virtual_x / mag) * MAX_VISUAL
                self.virtual_y = (self.virtual_y / mag) * MAX_VISUAL
        else:
            # UÇUŞ MODU (Birikimli Integrator)
            if delta_ew == 0.0 and delta_ns == 0.0:
                return

            shift_x = delta_ew * self.INTEGRATOR_GAIN
            shift_y = delta_ns * self.INTEGRATOR_GAIN

            mag = math.sqrt(shift_x ** 2 + shift_y ** 2)
            if mag > self.MAX_STEP_M:
                shift_x = (shift_x / mag) * self.MAX_STEP_M
                shift_y = (shift_y / mag) * self.MAX_STEP_M

            self.virtual_x += shift_x
            self.virtual_y += shift_y

            self.virtual_x = max(-self.GEOFENCE_M, min(self.GEOFENCE_M, self.virtual_x))
            self.virtual_y = max(-self.GEOFENCE_M, min(self.GEOFENCE_M, self.virtual_y))
        
        self.trail.append((self.virtual_x, self.virtual_y))
        if len(self.trail) > self.MAX_TRAIL:
            self.trail.pop(0)

    # ─────────────────────────────────────────────
    # RADAR ÇİZİMİ
    # ─────────────────────────────────────────────
    def update_radar(self):
        c = self.canvas
        c.delete("all")
        S = self.SCALE
        C = self.CENTER
        CS = self.CANVAS_SIZE

        # Görüş alanı (metre)
        view_range = CS / (2 * S)

        # ── Izgara ──
        grid_step = 5  # metre
        i = 0
        while i <= view_range:
            px = C + i * S
            mx = C - i * S
            py = C + i * S
            my = C - i * S
            if px <= CS:
                c.create_line(px, 0, px, CS, fill="#1a1a2e")
            if mx >= 0:
                c.create_line(mx, 0, mx, CS, fill="#1a1a2e")
            if py <= CS:
                c.create_line(0, py, CS, py, fill="#1a1a2e")
            if my >= 0:
                c.create_line(0, my, CS, my, fill="#1a1a2e")
            i += grid_step

        # ── Eksenler ──
        c.create_line(C, 0, C, CS, fill="#7f8c8d", dash=(4, 4))
        c.create_line(0, C, CS, C, fill="#7f8c8d", dash=(4, 4))

        # ── Mesafe halkaları ──
        for r_m in [5, 10, 15, 20, 30, 50]:
            r_px = r_m * S
            if r_px > 10 and r_px < CS:
                c.create_oval(
                    C - r_px, C - r_px, C + r_px, C + r_px,
                    outline="#1e3a5f", dash=(2, 4)
                )
                c.create_text(C + r_px - 12, C + 10, text=f"{r_m}m",
                              fill="#3a5f8f", font=("Arial", 7))

        # ── Pusula ──
        c.create_text(C, 12, text="KUZEY", fill="#ecf0f1", font=("Arial", 10, "bold"))
        c.create_text(C, CS - 12, text="GÜNEY", fill="#ecf0f1", font=("Arial", 10, "bold"))
        c.create_text(CS - 30, C, text="DOĞU", fill="#ecf0f1", font=("Arial", 9, "bold"))
        c.create_text(30, C, text="BATI", fill="#ecf0f1", font=("Arial", 9, "bold"))

        # ── Geofence ──
        gf_px = self.GEOFENCE_M * S
        if gf_px < CS:
            c.create_oval(
                C - gf_px, C - gf_px, C + gf_px, C + gf_px,
                outline="#e74c3c", dash=(6, 3), width=2
            )

        # ── Origin noktası ──
        c.create_oval(C - 4, C - 4, C + 4, C + 4, fill="white", outline="#ecf0f1")

        # ── Dronlar ──
        for d_id, data in self.drones.items():
            cx = C + data["x"] * S
            cy = C - data["y"] * S
            if 0 <= cx <= CS and 0 <= cy <= CS:
                c.create_oval(cx - 10, cy - 10, cx + 10, cy + 10,
                              fill=data["color"], outline="white", width=2)
                c.create_text(cx, cy, text=str(d_id), fill="white",
                              font=("Arial", 9, "bold"))
                c.create_text(cx, cy - 18,
                              text=f"{self.rssi_cal[d_id]:.0f}dBm",
                              fill="white", font=("Arial", 8, "bold"))

        # ── İz çizgisi (trail) ──
        if len(self.trail) >= 2:
            for i in range(1, len(self.trail)):
                x0 = C + self.trail[i - 1][0] * S
                y0 = C - self.trail[i - 1][1] * S
                x1 = C + self.trail[i][0] * S
                y1 = C - self.trail[i][1] * S
                # Yaşına göre solan renk
                alpha = int(80 + (175 * i / len(self.trail)))
                color = f"#{alpha:02x}{40:02x}{40:02x}"
                c.create_line(x0, y0, x1, y1, fill=color, width=1)

        # ── Sanal merkez (hedef tahmini) ──
        t_px = C + self.virtual_x * S
        t_py = C - self.virtual_y * S

        # Merkez→Hedef çizgisi
        c.create_line(C, C, t_px, t_py, fill="#e74c3c", dash=(3, 3), width=2)

        if self.target_visible:
            # Yanıp sönen hedef noktası
            if 0 <= t_px <= CS and 0 <= t_py <= CS:
                c.create_oval(t_px - 14, t_py - 14, t_px + 14, t_py + 14,
                              fill="", outline="#e74c3c", width=2)
                c.create_oval(t_px - 7, t_py - 7, t_px + 7, t_py + 7,
                              fill="#e74c3c", outline="white", width=2)
                c.create_text(t_px, t_py - 22, text="📻 HDF",
                              fill="#e74c3c", font=("Arial", 9, "bold"))
            else:
                # Ekran dışındaysa ok ile göster
                angle = math.atan2(-(t_py - C), t_px - C)
                edge_x = C + (CS / 2 - 20) * math.cos(angle)
                edge_y = C - (CS / 2 - 20) * math.sin(angle)
                dist = math.sqrt(self.virtual_x ** 2 + self.virtual_y ** 2)
                c.create_text(edge_x, edge_y,
                              text=f"📻 {dist:.0f}m →",
                              fill="#e74c3c", font=("Arial", 10, "bold"))

        # ── Yön hesabı ──
        dist = math.sqrt(self.virtual_x ** 2 + self.virtual_y ** 2)
        if dist > 0.5:
            angle = math.atan2(self.virtual_x, self.virtual_y)  # atan2(east, north)
            angle_deg = math.degrees(angle) % 360
            direction = self._angle_to_dir(angle_deg)
        else:
            direction = "⭕ MERKEZ"
            angle_deg = 0

        # ── Güç farkları ──
        delta_ns = self.rssi_cal[1] - self.rssi_cal[2]
        delta_ew = self.rssi_cal[3] - self.rssi_cal[4]

        info = (
            f"🧭 Yön: {direction}\n"
            f"📍 Konum: D:{self.virtual_x:+.1f}m  K:{self.virtual_y:+.1f}m\n"
            f"📏 Mesafe: {dist:.1f}m\n\n"
            f"📊 ΔK-G (P1-P2): {delta_ns:+.1f} dBm\n"
            f"   ΔD-B (P3-P4): {delta_ew:+.1f} dBm\n\n"
            f"🔍 Zoom: {CS / (2 * S):.0f}m görüş"
        )
        self.info_label.config(text=info)

    def _angle_to_dir(self, deg):
        """0°=Kuzey, 90°=Doğu, saat yönünde."""
        dirs = [
            (  0, "⬆️  KUZEY"),
            ( 45, "↗️  KUZEYDOĞU"),
            ( 90, "➡️  DOĞU"),
            (135, "↘️  GÜNEYDOĞU"),
            (180, "⬇️  GÜNEY"),
            (225, "↙️  GÜNEYBATI"),
            (270, "⬅️  BATI"),
            (315, "↖️  KUZEYBATI"),
        ]
        for center, name in dirs:
            diff = abs(deg - center)
            if diff > 180:
                diff = 360 - diff
            if diff <= 22.5:
                return name
        return "⬆️  KUZEY"

    # ─────────────────────────────────────────────
    # KONTROLLER
    # ─────────────────────────────────────────────
    def reset_center(self):
        self.virtual_x = 0.0
        self.virtual_y = 0.0
        self.trail.clear()
        self.update_radar()

    def clear_trail(self):
        self.trail.clear()
        self.update_radar()

    def blink_target(self):
        self.target_visible = not self.target_visible
        self.update_radar()
        self.root.after(600, self.blink_target)


if __name__ == "__main__":
    root = tk.Tk()
    app = SwarmRadar(root)
    root.mainloop()
