#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TÜBİTAK 123E294 - Kıyaslamalı Radar Arayüzü (Calibrated vs Raw)
Bu arayüz, kalibre edilmiş verilerle hesaplanan konumu (KIRMIZI) ve 
hiçbir kalibrasyon/ofset uygulanmamış ham konumu (SARI) aynı ekranda gösterir.
"""

import tkinter as tk
import math
import serial
import threading
import time

LORA_BAUD = 9600

# ==============================================================
# KALİBRASYON DEĞERLERİ
# ==============================================================
P_REF     = -78.72     # 1 metre referans gücü (dBm)
PATH_N    = 1.264      # Ortam sönümleme katsayısı
OFFSETS   = {          # Donanım ofsetleri (dBm)
    1:  0.00,          # D1 Kuzey (referans)
    2: -1.32,          # D2 Güney
    3:  2.38,          # D3 Doğu
    4:  1.10,          # D4 Batı
}

DRONE_SPACING = 5.0     # Dronların merkeze mesafesi (metre)


def rssi_to_distance(rssi_val, p_ref=P_REF, n=PATH_N):
    if n <= 0: return 999.0
    diff = p_ref - rssi_val
    if diff < 0: return 0.1
    d = 10.0 ** (diff / (10.0 * n))
    return min(d, 999.0)


def calculate_xy(p1, p2, p3, p4, d1, d2, d3, d4):
    """ Diferansiyel güç farkı ve mesafelerle hedef X, Y konumunu hesaplar """
    delta_ns = p1 - p2
    delta_ew = p3 - p4

    DEAD_ZONE = 1.5
    if abs(delta_ns) < DEAD_ZONE: delta_ns = 0.0
    if abs(delta_ew) < DEAD_ZONE: delta_ew = 0.0

    min_dist = min(d1, d2, d3, d4)

    if delta_ns == 0 and delta_ew == 0:
        return 0.0, 0.0
    
    # Mesafe farklarından temel vektör
    r_ns = (d2 - d1) / 2.0  # Pozitif = Kuzeye yakın
    r_ew = (d4 - d3) / 2.0  # Pozitif = Doğuya yakın

    tx = r_ew
    ty = r_ns

    # Dronların dışına çıkabilmesi için ölçeklendirme
    if min_dist > DRONE_SPACING:
        mag = math.sqrt(tx**2 + ty**2)
        if mag > 0.1:
            scale_factor = min(min_dist / mag, 5.0)
            tx *= scale_factor
            ty *= scale_factor

    # Geofence sınırlaması (50m)
    mag = math.sqrt(tx**2 + ty**2)
    if mag > 50.0:
        tx = (tx / mag) * 50.0
        ty = (ty / mag) * 50.0

    return tx, ty


class KiyaslamaliRadar:
    def __init__(self, root):
        self.root = root
        self.root.title("TÜBİTAK 123E294 - Kıyaslamalı Radar (Kalibreli vs Ham)")
        self.root.geometry("1150x800")
        self.root.configure(bg="#0d1117")

        self.CANVAS_SIZE = 700
        self.CENTER = self.CANVAS_SIZE // 2

        self.SCALE = 15.0         
        self.MIN_SCALE = 3.0
        self.MAX_SCALE = 50.0

        self.drones = {
            1: {"name": "D1 Kuzey", "x":  0.0, "y":  DRONE_SPACING, "color": "#58a6ff"},
            2: {"name": "D2 Güney", "x":  0.0, "y": -DRONE_SPACING, "color": "#f0883e"},
            3: {"name": "D3 Doğu",  "x":  DRONE_SPACING, "y": 0.0,  "color": "#bc8cff"},
            4: {"name": "D4 Batı",  "x": -DRONE_SPACING, "y": 0.0,  "color": "#39d353"},
        }

        # Veri yapıları (Ham ve Kalibreli ayrı ayrı)
        self.rssi_raw = {1: -90, 2: -90, 3: -90, 4: -90}
        self.rssi_cal = {1: -90, 2: -90, 3: -90, 4: -90}
        self.dist_raw = {1: 10.0, 2: 10.0, 3: 10.0, 4: 10.0}
        self.dist_cal = {1: 10.0, 2: 10.0, 3: 10.0, 4: 10.0}

        self.target_cal_x = 0.0
        self.target_cal_y = 0.0
        self.target_raw_x = 0.0
        self.target_raw_y = 0.0

        self.trail_cal = []
        self.trail_raw = []
        self.MAX_TRAIL = 100
        
        self.target_pulse = 0

        self.serial_port = None
        self.is_reading = False
        self.packet_count = 0

        self.kalman = {}
        for d_id in self.drones:
            self.kalman[d_id] = {"x": -90.0, "P": 10.0, "Q": 0.5, "R": 8.0}

        self.setup_ui()
        self.update_radar()
        self.animate()

    def kalman_update(self, d_id, measurement):
        k = self.kalman[d_id]
        P_pred = k["P"] + k["Q"]
        K = P_pred / (P_pred + k["R"])
        k["x"] = k["x"] + K * (measurement - k["x"])
        k["P"] = (1 - K) * P_pred
        return k["x"]

    def setup_ui(self):
        left = tk.Frame(self.root, bg="#0d1117")
        left.pack(side=tk.LEFT, padx=10, pady=10)

        self.canvas = tk.Canvas(
            left, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE,
            bg="#0d1117", highlightthickness=1, highlightbackground="#30363d"
        )
        self.canvas.pack()

        self.zoom_label = tk.Label(left, text="", fg="#8b949e", bg="#0d1117", font=("Arial", 9))
        self.zoom_label.pack(pady=2)

        self.canvas.bind("<Button-4>", lambda e: self._zoom(1.15))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(1/1.15))
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(1.15 if e.delta > 0 else 1/1.15))

        right = tk.Frame(self.root, bg="#0d1117", width=400)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        tk.Label(right, text="⚖️ KIYASLAMALI RADAR", fg="#58a6ff", bg="#0d1117", font=("Arial", 14, "bold")).pack(pady=(0, 5))

        # Legend (Renk Açıklamaları)
        leg_frame = tk.Frame(right, bg="#161b22", padx=8, pady=6)
        leg_frame.pack(fill=tk.X, pady=3)
        tk.Label(leg_frame, text="🔴 Kalibreli Konum (Düzeltilmiş)", fg="#f85149", bg="#161b22", font=("Arial", 10, "bold")).pack(anchor="w")
        tk.Label(leg_frame, text="🟡 Kalibresiz Konum (0 Ofset, Ham)", fg="#e3b341", bg="#161b22", font=("Arial", 10, "bold")).pack(anchor="w")

        ser_frame = tk.Frame(right, bg="#161b22", padx=8, pady=6)
        ser_frame.pack(fill=tk.X, pady=3)
        port_row = tk.Frame(ser_frame, bg="#161b22")
        port_row.pack(fill=tk.X, pady=3)
        self.com_entry = tk.Entry(port_row, width=12, font=("Courier", 11), bg="#0d1117", fg="white", insertbackground="white")
        self.com_entry.insert(0, "/dev/ttyUSB0")
        self.com_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.btn_connect = tk.Button(port_row, text="BAĞLAN", bg="#238636", fg="white", font=("Arial", 9, "bold"), command=self.toggle_serial)
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.status_label = tk.Label(ser_frame, text="⚪ Bağlantı Yok", fg="#8b949e", bg="#161b22", font=("Arial", 9))
        self.status_label.pack(anchor="w", pady=2)

        # Drone Sinyalleri
        tk.Label(right, text="📊 Drone Sinyalleri (Ham -> Kalibreli)", fg="white", bg="#0d1117", font=("Arial", 11, "bold")).pack(pady=(8, 3))
        self.drone_labels = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(right, bg="#161b22", padx=6, pady=3)
            frame.pack(fill=tk.X, pady=1)
            tk.Label(frame, text=f"  {data['name']}", fg=data["color"], bg="#161b22", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            lbl = tk.Label(frame, text="-90 -> -90 dB", fg="#8b949e", bg="#161b22", font=("Courier", 10))
            lbl.pack(side=tk.RIGHT)
            self.drone_labels[d_id] = lbl

        # Sliderlar
        tk.Label(right, text="🎚️ Manuel Test", fg="#8b949e", bg="#0d1117", font=("Arial", 9)).pack(pady=(8, 2))
        self.sliders = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(right, bg="#0d1117")
            frame.pack(fill=tk.X, pady=0)
            tk.Label(frame, text=f"D{d_id}", fg=data["color"], bg="#0d1117", font=("Arial", 8, "bold"), width=3).pack(side=tk.LEFT)
            slider = tk.Scale(
                frame, from_=-40, to=-120, orient=tk.HORIZONTAL,
                bg="#161b22", fg="#8b949e", troughcolor="#30363d",
                highlightthickness=0, length=250, showvalue=False,
                command=lambda val, i=d_id: self.on_slider(i, val)
            )
            slider.set(-90)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders[d_id] = slider

        btn_frame = tk.Frame(right, bg="#0d1117")
        btn_frame.pack(fill=tk.X, pady=8)
        tk.Button(btn_frame, text="🗑 İzi Temizle", bg="#30363d", fg="white", font=("Arial", 9), command=self.clear_trail).pack(fill=tk.X)

        self.info_label = tk.Label(right, text="Veri bekleniyor...", fg="#8b949e", bg="#0d1117", font=("Courier", 10), justify=tk.LEFT, anchor="w")
        self.info_label.pack(pady=5, fill=tk.X)

    def _zoom(self, factor):
        self.SCALE = max(self.MIN_SCALE, min(self.MAX_SCALE, self.SCALE * factor))
        self.update_radar()

    def toggle_serial(self):
        if not self.is_reading:
            port = self.com_entry.get().strip()
            if not port.startswith("/") and not port.startswith("COM"):
                port = "/dev/" + port
            try:
                self.serial_port = serial.Serial(port, LORA_BAUD, timeout=1)
                self.is_reading = True
                self.packet_count = 0
                self.btn_connect.config(text="KES", bg="#da3633")
                self.status_label.config(text="🟢 Bağlı", fg="#39d353")
                threading.Thread(target=self._serial_loop, daemon=True).start()
            except Exception as e:
                self.status_label.config(text=f"🔴 Hata", fg="#f85149")
        else:
            self.is_reading = False
            if self.serial_port:
                try: self.serial_port.close()
                except: pass
            self.btn_connect.config(text="BAĞLAN", bg="#238636")
            self.status_label.config(text="⚪ Bağlantı Yok", fg="#8b949e")

    def _serial_loop(self):
        buf = ""
        while self.is_reading:
            try:
                if not self.serial_port or not self.serial_port.is_open: break
                raw = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not raw: continue
                buf += raw.decode("ascii", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("N"):
                        parts = line[1:].split(",")
                        if len(parts) >= 2:
                            try:
                                did = int(parts[0])
                                rssi = int(parts[1])
                                if did in self.rssi_raw:
                                    self.packet_count += 1
                                    self.root.after(0, self._process_rssi, did, rssi)
                            except ValueError: pass
            except: pass

    def _process_rssi(self, did, rssi):
        filtered = self.kalman_update(did, rssi)
        
        # 1. HAM (0 Kalibrasyonlu) Hesaplama
        self.rssi_raw[did] = filtered
        self.dist_raw[did] = rssi_to_distance(self.rssi_raw[did])
        
        # 2. KALİBRELİ Hesaplama
        self.rssi_cal[did] = filtered + OFFSETS.get(did, 0.0)
        self.dist_cal[did] = rssi_to_distance(self.rssi_cal[did])
        
        if did in self.sliders:
            self.sliders[did].set(rssi)
            
        self.status_label.config(text=f"🟢 {self.packet_count} paket", fg="#39d353")
        self._compute_positions()
        self.update_radar()

    def on_slider(self, did, value):
        if not self.is_reading:
            rssi = int(value)
            filtered = self.kalman_update(did, rssi)
            self.rssi_raw[did] = filtered
            self.dist_raw[did] = rssi_to_distance(self.rssi_raw[did])
            self.rssi_cal[did] = filtered + OFFSETS.get(did, 0.0)
            self.dist_cal[did] = rssi_to_distance(self.rssi_cal[did])
            self._compute_positions()
            self.update_radar()

    def _compute_positions(self):
        # Ham Konum Hesaplama
        self.target_raw_x, self.target_raw_y = calculate_xy(
            self.rssi_raw[1], self.rssi_raw[2], self.rssi_raw[3], self.rssi_raw[4],
            self.dist_raw[1], self.dist_raw[2], self.dist_raw[3], self.dist_raw[4]
        )
        
        # Kalibreli Konum Hesaplama
        self.target_cal_x, self.target_cal_y = calculate_xy(
            self.rssi_cal[1], self.rssi_cal[2], self.rssi_cal[3], self.rssi_cal[4],
            self.dist_cal[1], self.dist_cal[2], self.dist_cal[3], self.dist_cal[4]
        )

        self.trail_raw.append((self.target_raw_x, self.target_raw_y))
        if len(self.trail_raw) > self.MAX_TRAIL: self.trail_raw.pop(0)

        self.trail_cal.append((self.target_cal_x, self.target_cal_y))
        if len(self.trail_cal) > self.MAX_TRAIL: self.trail_cal.pop(0)

    def update_radar(self):
        c = self.canvas
        c.delete("all")
        S = self.SCALE
        C = self.CENTER
        CS = self.CANVAS_SIZE
        view_range = CS / (2 * S)

        # Izgara
        grid_step = 5
        i = grid_step
        while i <= view_range:
            px, mx = C + i * S, C - i * S
            if px <= CS: c.create_line(px, 0, px, CS, fill="#21262d")
            if mx >= 0: c.create_line(mx, 0, mx, CS, fill="#21262d")
            py, my = C + i * S, C - i * S
            if py <= CS: c.create_line(0, py, CS, py, fill="#21262d")
            if my >= 0: c.create_line(0, my, CS, my, fill="#21262d")
            i += grid_step

        c.create_line(C, 0, C, CS, fill="#30363d", dash=(4, 4))
        c.create_line(0, C, CS, C, fill="#30363d", dash=(4, 4))

        for r_m in [5, 10, 15, 20, 30, 50]:
            r_px = r_m * S
            if 10 < r_px < CS:
                c.create_oval(C - r_px, C - r_px, C + r_px, C + r_px, outline="#1f3044", dash=(2, 4))
                c.create_text(C + r_px - 14, C + 12, text=f"{r_m}m", fill="#3a5f8f", font=("Arial", 7))

        # Dronlar
        for d_id, data in self.drones.items():
            cx = C + data["x"] * S
            cy = C - data["y"] * S
            c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12, fill=data["color"], outline="white", width=2)
            c.create_text(cx, cy, text=str(d_id), fill="white", font=("Arial", 10, "bold"))
            
            # Label
            txt = f"  Ham:{self.rssi_raw[d_id]:.1f} | Kal:{self.rssi_cal[d_id]:.1f}"
            self.drone_labels[d_id].config(text=txt)

        # İzler
        if len(self.trail_raw) >= 2:
            for i in range(1, len(self.trail_raw)):
                x0, y0 = C + self.trail_raw[i - 1][0] * S, C - self.trail_raw[i - 1][1] * S
                x1, y1 = C + self.trail_raw[i][0] * S, C - self.trail_raw[i][1] * S
                c.create_line(x0, y0, x1, y1, fill="#e3b341", width=1, dash=(2,2)) # Sarı kesik çizgi

        if len(self.trail_cal) >= 2:
            for i in range(1, len(self.trail_cal)):
                x0, y0 = C + self.trail_cal[i - 1][0] * S, C - self.trail_cal[i - 1][1] * S
                x1, y1 = C + self.trail_cal[i][0] * S, C - self.trail_cal[i][1] * S
                c.create_line(x0, y0, x1, y1, fill="#f85149", width=2) # Kırmızı düz çizgi

        # Hedefleri Çiz
        # HAM (Sarı)
        r_px = C + self.target_raw_x * S
        r_py = C - self.target_raw_y * S
        c.create_oval(r_px - 6, r_py - 6, r_px + 6, r_py + 6, fill="#e3b341", outline="white", width=1)
        c.create_text(r_px, r_py - 15, text="Ham", fill="#e3b341", font=("Arial", 8, "bold"))

        # KALİBRELİ (Kırmızı)
        c_px = C + self.target_cal_x * S
        c_py = C - self.target_cal_y * S
        
        pulse_r = 12 + self.target_pulse
        c.create_oval(c_px - pulse_r, c_py - pulse_r, c_px + pulse_r, c_py + pulse_r, fill="", outline="#7a2824", width=2)
        c.create_oval(c_px - 8, c_py - 8, c_px + 8, c_py + 8, fill="#f85149", outline="white", width=2)
        c.create_text(c_px, c_py - 22, text="Kalibreli", fill="#f85149", font=("Arial", 9, "bold"))

        # Bilgi Paneli
        dist_cal = math.sqrt(self.target_cal_x**2 + self.target_cal_y**2)
        dist_raw = math.sqrt(self.target_raw_x**2 + self.target_raw_y**2)
        
        info = (
            f"🔴 KALİBRELİ:\n"
            f"   Mesafe: {dist_cal:.1f}m | X: {self.target_cal_x:+.1f} Y: {self.target_cal_y:+.1f}\n\n"
            f"🟡 KALİBRESİZ (HAM):\n"
            f"   Mesafe: {dist_raw:.1f}m | X: {self.target_raw_x:+.1f} Y: {self.target_raw_y:+.1f}\n\n"
            f"🔍 Görüş Alanı: {view_range:.0f}m"
        )
        self.info_label.config(text=info)
        self.zoom_label.config(text=f"Zoom: {view_range:.0f}m görüş | Ölçek: 1m = {S:.0f}px")

    def clear_trail(self):
        self.trail_cal.clear()
        self.trail_raw.clear()
        self.update_radar()

    def animate(self):
        self.target_pulse = (self.target_pulse + 1) % 6
        self.update_radar()
        self.root.after(500, self.animate)

if __name__ == "__main__":
    root = tk.Tk()
    app = KiyaslamaliRadar(root)
    root.mainloop()
