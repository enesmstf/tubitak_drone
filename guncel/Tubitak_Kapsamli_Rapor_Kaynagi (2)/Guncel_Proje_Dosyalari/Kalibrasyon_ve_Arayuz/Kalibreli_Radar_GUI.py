#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TÜBİTAK 123E294 - Kalibre Edilmiş Radar Arayüzü
Kalibrasyon sonuçlarından (kalibrasyon_sonucu.json) elde edilen
P_ref, n ve ofset değerlerini kullanarak telsizin dronlara göre
gerçek metrik konumunu gösteren arayüz.
"""

import tkinter as tk
import math
import serial
import threading
import time
import json
import os

LORA_BAUD = 9600

# ==============================================================
# KALİBRASYON DEĞERLERİ (kalibrasyon_sonucu.json'dan)
# ==============================================================
P_REF     = -78.72     # 1 metre referans gücü (dBm)
PATH_N    = 1.264      # Ortam sönümleme katsayısı
OFFSETS   = {           # Donanım ofsetleri (dBm)
    1:  0.0,            # D1 Kuzey (referans)
    2: -1.32,           # D2 Güney
    3:  2.38,           # D3 Doğu
    4:  1.10,           # D4 Batı
}

# Dron konumları (metre, merkeze göre)
DRONE_SPACING = 5.0     # Dronların merkeze mesafesi


def rssi_to_distance(rssi_cal, p_ref=P_REF, n=PATH_N):
    """Kalibre edilmiş RSSI'dan mesafe (metre) hesaplar.
    Formül: d = 10^((P_ref - RSSI) / (10 * n))
    """
    if n <= 0:
        return 999.0
    diff = p_ref - rssi_cal
    if diff < 0:
        return 0.1  # Sinyal referanstan güçlü: çok yakın
    d = 10.0 ** (diff / (10.0 * n))
    return min(d, 999.0)


def trilaterate_2d(drones_xy, distances):
    """4 dronun (x,y) konumları ve tahmin edilen mesafeleriyle
    ağırlıklı en küçük kareler konumlandırma yapar.
    Ağırlıklar: 1/d^2 (yakın drone daha güvenilir)
    """
    # Ağırlıklı ortalama yöntemi (basit, hızlı, güvenilir)
    wx_sum = 0.0
    wy_sum = 0.0
    w_sum  = 0.0
    for (dx, dy), dist in zip(drones_xy, distances):
        if dist < 0.1:
            dist = 0.1
        w = 1.0 / (dist ** 2)
        wx_sum += dx * w
        wy_sum += dy * w
        w_sum  += w

    if w_sum < 1e-12:
        return 0.0, 0.0

    # Ağırlıklı ortalama = dronların konumu
    cx = wx_sum / w_sum
    cy = wy_sum / w_sum

    # Trilaterasyon sonucu ters çevirme:
    # Hedef, ağırlıklı ortalamanın KARŞI tarafında.
    # Eğer sinyal D1'de güçlüyse, ağırlıklı merkez D1'e kayar
    # ama hedef aslında D1'in ÖTESINDE olabilir.
    # Bunu düzeltmek için fark vektörünü kullanıyoruz:
    # En yakın drona doğru merkezden kaydırma yapıyoruz.

    # Ama burada farklı bir yaklaşım uyguluyoruz:
    # Her drondan hedefe olan mesafe ile dronun merkezden uzaklığını
    # karşılaştırarak hedefin iç mi dış mı olduğunu anlıyoruz.

    # Yön vektörü: Diferansiyel güç farkından
    # (Bu kısım integratör mantığından bağımsız, sadece yön gösterir)
    return cx, cy


class KalibreliRadar:
    def __init__(self, root):
        self.root = root
        self.root.title("TÜBİTAK 123E294 - Kalibreli Radar Arayüzü")
        self.root.geometry("1100x750")
        self.root.configure(bg="#0d1117")

        self.CANVAS_SIZE = 650
        self.CENTER = self.CANVAS_SIZE // 2

        # Ölçek ve zoom
        self.SCALE = 15.0         # 1m = 15px
        self.MIN_SCALE = 3.0
        self.MAX_SCALE = 50.0

        # Dron pozisyonları
        self.drones = {
            1: {"name": "D1 Kuzey", "x":  0.0, "y":  DRONE_SPACING, "color": "#58a6ff"},
            2: {"name": "D2 Güney", "x":  0.0, "y": -DRONE_SPACING, "color": "#f0883e"},
            3: {"name": "D3 Doğu",  "x":  DRONE_SPACING, "y": 0.0,  "color": "#bc8cff"},
            4: {"name": "D4 Batı",  "x": -DRONE_SPACING, "y": 0.0,  "color": "#39d353"},
        }

        # RSSI verileri
        self.rssi_raw = {1: -90, 2: -90, 3: -90, 4: -90}
        self.rssi_cal = {1: -90, 2: -90, 3: -90, 4: -90}
        self.distances = {1: 10.0, 2: 10.0, 3: 10.0, 4: 10.0}

        # Hedef konumu
        self.target_x = 0.0
        self.target_y = 0.0

        # İz (trail) ve animasyon
        self.trail = []
        self.MAX_TRAIL = 300
        self.target_visible = True
        self.target_pulse = 0

        # Seri port
        self.serial_port = None
        self.is_reading = False
        self.packet_count = 0
        self.last_update = time.time()

        # Kalman filtreleri (her drone için)
        self.kalman = {}
        for d_id in self.drones:
            self.kalman[d_id] = {"x": -90.0, "P": 10.0, "Q": 0.5, "R": 8.0}

        self.setup_ui()
        self.update_radar()
        self.animate()

    def kalman_update(self, d_id, measurement):
        """Basit Skaler Kalman Filtresi"""
        k = self.kalman[d_id]
        # Tahmin
        P_pred = k["P"] + k["Q"]
        # Güncelleme
        K = P_pred / (P_pred + k["R"])
        k["x"] = k["x"] + K * (measurement - k["x"])
        k["P"] = (1 - K) * P_pred
        return k["x"]

    def setup_ui(self):
        # Sol taraf: Radar kanvası
        left = tk.Frame(self.root, bg="#0d1117")
        left.pack(side=tk.LEFT, padx=10, pady=10)

        self.canvas = tk.Canvas(
            left, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE,
            bg="#0d1117", highlightthickness=1, highlightbackground="#30363d"
        )
        self.canvas.pack()

        # Zoom bilgisi
        self.zoom_label = tk.Label(
            left, text="", fg="#8b949e", bg="#0d1117", font=("Arial", 9)
        )
        self.zoom_label.pack(pady=2)

        # Mouse wheel zoom
        self.canvas.bind("<Button-4>", lambda e: self._zoom(1.15))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(1/1.15))
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(1.15 if e.delta > 0 else 1/1.15))

        # Sağ taraf: Kontrol paneli
        right = tk.Frame(self.root, bg="#0d1117", width=400)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Başlık
        tk.Label(right, text="🛰️ KALİBRELİ RADAR", fg="#58a6ff",
                 bg="#0d1117", font=("Arial", 14, "bold")).pack(pady=(0, 5))

        # Kalibrasyon bilgisi
        cal_frame = tk.Frame(right, bg="#161b22", padx=8, pady=6)
        cal_frame.pack(fill=tk.X, pady=3)
        tk.Label(cal_frame, text="📐 Kalibrasyon Değerleri", fg="#f0883e",
                 bg="#161b22", font=("Arial", 10, "bold")).pack(anchor="w")
        tk.Label(cal_frame,
                 text=f"P_ref = {P_REF:.2f} dBm | n = {PATH_N:.3f}\n"
                      f"D1: {OFFSETS[1]:+.2f}  D2: {OFFSETS[2]:+.2f}  "
                      f"D3: {OFFSETS[3]:+.2f}  D4: {OFFSETS[4]:+.2f}",
                 fg="#8b949e", bg="#161b22", font=("Courier", 9),
                 justify=tk.LEFT).pack(anchor="w")

        # Seri port bağlantısı
        ser_frame = tk.Frame(right, bg="#161b22", padx=8, pady=6)
        ser_frame.pack(fill=tk.X, pady=3)
        tk.Label(ser_frame, text="📡 LoRa Bağlantısı", fg="#58a6ff",
                 bg="#161b22", font=("Arial", 10, "bold")).pack(anchor="w")
        port_row = tk.Frame(ser_frame, bg="#161b22")
        port_row.pack(fill=tk.X, pady=3)
        self.com_entry = tk.Entry(port_row, width=16, font=("Courier", 11),
                                  bg="#0d1117", fg="white", insertbackground="white")
        self.com_entry.insert(0, "/dev/ttyUSB0")
        self.com_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.btn_connect = tk.Button(
            port_row, text="BAĞLAN", bg="#238636", fg="white",
            font=("Arial", 9, "bold"), command=self.toggle_serial,
            activebackground="#2ea043"
        )
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status_label = tk.Label(
            ser_frame, text="⚪ Bağlantı Yok", fg="#8b949e", bg="#161b22",
            font=("Arial", 9)
        )
        self.status_label.pack(anchor="w", pady=2)

        # Log penceresi
        log_frame = tk.Frame(right, bg="#161b22", padx=5, pady=5)
        log_frame.pack(fill=tk.X, pady=3)
        tk.Label(log_frame, text="📋 Ham Veri:", fg="#8b949e",
                 bg="#161b22", font=("Arial", 9, "bold")).pack(anchor="w")
        self.log_text = tk.Text(
            log_frame, height=3, width=35, bg="#0d1117", fg="#39d353",
            font=("Courier", 9), state=tk.DISABLED, borderwidth=0
        )
        self.log_text.pack(fill=tk.X)

        # Drone RSSI ve Mesafe Paneli
        tk.Label(right, text="📊 Drone Sinyalleri", fg="white",
                 bg="#0d1117", font=("Arial", 11, "bold")).pack(pady=(8, 3))

        self.drone_labels = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(right, bg="#161b22", padx=6, pady=3)
            frame.pack(fill=tk.X, pady=1)
            tk.Label(frame, text=f"  {data['name']}", fg=data["color"],
                     bg="#161b22", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            lbl = tk.Label(frame, text="  -90 dBm  →  ?.?m",
                          fg="#8b949e", bg="#161b22", font=("Courier", 10))
            lbl.pack(side=tk.RIGHT)
            self.drone_labels[d_id] = lbl

        # Slider'lar (test için)
        tk.Label(right, text="🎚️ Manuel Test (Port yokken)", fg="#8b949e",
                 bg="#0d1117", font=("Arial", 9)).pack(pady=(8, 2))
        self.sliders = {}
        for d_id, data in self.drones.items():
            frame = tk.Frame(right, bg="#0d1117")
            frame.pack(fill=tk.X, pady=0)
            tk.Label(frame, text=f"D{d_id}", fg=data["color"],
                     bg="#0d1117", font=("Arial", 8, "bold"), width=3).pack(side=tk.LEFT)
            slider = tk.Scale(
                frame, from_=-40, to=-120, orient=tk.HORIZONTAL,
                bg="#161b22", fg="#8b949e", troughcolor="#30363d",
                highlightthickness=0, length=250, showvalue=False,
                command=lambda val, i=d_id: self.on_slider(i, val)
            )
            slider.set(-90)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders[d_id] = slider

        # Butonlar
        btn_frame = tk.Frame(right, bg="#0d1117")
        btn_frame.pack(fill=tk.X, pady=8)
        tk.Button(btn_frame, text="🗑 İzi Temizle", bg="#30363d", fg="white",
                  font=("Arial", 9), command=self.clear_trail,
                  activebackground="#484f58").pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Büyük bilgi paneli
        self.info_label = tk.Label(
            right, text="Veri bekleniyor...", fg="#f0883e", bg="#0d1117",
            font=("Arial", 11), justify=tk.LEFT, anchor="w"
        )
        self.info_label.pack(pady=5, fill=tk.X)

    def _zoom(self, factor):
        self.SCALE = max(self.MIN_SCALE, min(self.MAX_SCALE, self.SCALE * factor))
        self.update_radar()

    # ─────────────────────────────────────────────
    # SERİ PORT
    # ─────────────────────────────────────────────
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
                self.status_label.config(text=f"🔴 {e}", fg="#f85149")
        else:
            self.is_reading = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            self.btn_connect.config(text="BAĞLAN", bg="#238636")
            self.status_label.config(text="⚪ Bağlantı Yok", fg="#8b949e")

    def _serial_loop(self):
        buf = ""
        while self.is_reading:
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
                                        if len(parts) == 3:
                                            chk = int(parts[2])
                                            body = f"N{did},{rssi}"
                                            expected = sum(ord(c) for c in body) % 256
                                            if chk != expected:
                                                continue
                                        self.packet_count += 1
                                        self.root.after(0, self._process_rssi, did, rssi)
                                except ValueError:
                                    pass
            except serial.SerialException:
                self.root.after(0, self._serial_error)
                break
            except:
                pass

    def _serial_error(self):
        self.is_reading = False
        self.btn_connect.config(text="BAĞLAN", bg="#238636")
        self.status_label.config(text="🔴 Bağlantı Koptu!", fg="#f85149")

    def _append_log(self, line):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        n = int(self.log_text.index("end-1c").split(".")[0])
        if n > 30:
            self.log_text.delete("1.0", "2.0")
        self.log_text.config(state=tk.DISABLED)

    # ─────────────────────────────────────────────
    # RSSI İŞLEME VE KONUM HESAPLAMA
    # ─────────────────────────────────────────────
    def _process_rssi(self, did, rssi):
        # Kalman filtresi uygula
        filtered = self.kalman_update(did, rssi)
        self.rssi_raw[did] = rssi
        # Ofset uygulayarak kalibre et
        self.rssi_cal[did] = filtered + OFFSETS.get(did, 0.0)
        # Mesafe hesapla
        self.distances[did] = rssi_to_distance(self.rssi_cal[did])
        # Slider güncelle
        if did in self.sliders:
            self.sliders[did].set(rssi)
        self.status_label.config(
            text=f"🟢 {self.packet_count} paket", fg="#39d353"
        )
        self._compute_position()
        self.update_radar()

    def on_slider(self, did, value):
        if not self.is_reading:
            rssi = int(value)
            filtered = self.kalman_update(did, rssi)
            self.rssi_raw[did] = rssi
            self.rssi_cal[did] = filtered + OFFSETS.get(did, 0.0)
            self.distances[did] = rssi_to_distance(self.rssi_cal[did])
            self._compute_position()
            self.update_radar()

    def _compute_position(self):
        """Diferansiyel güç farkı + kalibreli mesafe ile konum hesapla."""
        p1 = self.rssi_cal[1]  # Kuzey
        p2 = self.rssi_cal[2]  # Güney
        p3 = self.rssi_cal[3]  # Doğu
        p4 = self.rssi_cal[4]  # Batı

        d1 = self.distances[1]
        d2 = self.distances[2]
        d3 = self.distances[3]
        d4 = self.distances[4]

        # Diferansiyel yön (hangi tarafta daha güçlü?)
        delta_ns = p1 - p2  # Pozitif = Kuzeyde güçlü = hedef Kuzeyde
        delta_ew = p3 - p4  # Pozitif = Doğuda güçlü  = hedef Doğuda

        # Ölü bölge
        DEAD_ZONE = 1.5
        if abs(delta_ns) < DEAD_ZONE:
            delta_ns = 0.0
        if abs(delta_ew) < DEAD_ZONE:
            delta_ew = 0.0

        # Mesafe tahmini: en yakın dronun mesafesini kullan
        min_dist = min(d1, d2, d3, d4)

        # Yön: güç farkından açı hesapla
        if delta_ns == 0 and delta_ew == 0:
            self.target_x = 0.0
            self.target_y = 0.0
        else:
            angle = math.atan2(delta_ew, delta_ns)  # radyan
            # Mesafe: Diferansiyel mesafe hesabı
            # Karşılıklı dronlardan yarıçap farkı
            r_ns = (d2 - d1) / 2.0  # Pozitif = Kuzeye yakın
            r_ew = (d4 - d3) / 2.0  # Pozitif = Doğuya yakın

            # Mesafe vektörü
            self.target_x = r_ew
            self.target_y = r_ns

            # Dronların dışına çıkabilmesi için:
            # Eğer en yakın drone'a olan mesafe, drone-merkez mesafesinden büyükse
            # hedef dışarıdadır.
            if min_dist > DRONE_SPACING:
                # Hedef formasyonun dışında, yönü koru ama mesafeyi uzat
                mag = math.sqrt(self.target_x**2 + self.target_y**2)
                if mag > 0.1:
                    scale_factor = min_dist / max(mag, 0.1)
                    scale_factor = min(scale_factor, 5.0)  # Aşırı büyümeyi sınırla
                    self.target_x *= scale_factor
                    self.target_y *= scale_factor

        # Geofence sınırlaması
        GEOFENCE = 50.0
        mag = math.sqrt(self.target_x**2 + self.target_y**2)
        if mag > GEOFENCE:
            self.target_x = (self.target_x / mag) * GEOFENCE
            self.target_y = (self.target_y / mag) * GEOFENCE

        # Trail güncelle
        self.trail.append((self.target_x, self.target_y))
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
        view_range = CS / (2 * S)

        # Izgara (5m aralıklı)
        grid_step = 5
        i = grid_step
        while i <= view_range:
            px = C + i * S
            mx = C - i * S
            if px <= CS:
                c.create_line(px, 0, px, CS, fill="#21262d")
            if mx >= 0:
                c.create_line(mx, 0, mx, CS, fill="#21262d")
            py = C + i * S
            my = C - i * S
            if py <= CS:
                c.create_line(0, py, CS, py, fill="#21262d")
            if my >= 0:
                c.create_line(0, my, CS, my, fill="#21262d")
            i += grid_step

        # Eksenler
        c.create_line(C, 0, C, CS, fill="#30363d", dash=(4, 4))
        c.create_line(0, C, CS, C, fill="#30363d", dash=(4, 4))

        # Mesafe halkaları
        for r_m in [5, 10, 15, 20, 30, 50]:
            r_px = r_m * S
            if 10 < r_px < CS:
                c.create_oval(C - r_px, C - r_px, C + r_px, C + r_px,
                              outline="#1f3044", dash=(2, 4))
                c.create_text(C + r_px - 14, C + 12, text=f"{r_m}m",
                              fill="#3a5f8f", font=("Arial", 7))

        # Pusula yönleri
        c.create_text(C, 14, text="K U Z E Y", fill="#58a6ff",
                      font=("Arial", 10, "bold"))
        c.create_text(C, CS - 14, text="G Ü N E Y", fill="#f0883e",
                      font=("Arial", 10, "bold"))
        c.create_text(CS - 32, C, text="DOĞU", fill="#bc8cff",
                      font=("Arial", 9, "bold"))
        c.create_text(32, C, text="BATI", fill="#39d353",
                      font=("Arial", 9, "bold"))

        # Geofence çizgisi (50m)
        gf_px = 50.0 * S
        if gf_px < CS:
            c.create_oval(C - gf_px, C - gf_px, C + gf_px, C + gf_px,
                          outline="#7a2824", dash=(6, 3), width=2)

        # Merkez noktası
        c.create_oval(C - 3, C - 3, C + 3, C + 3, fill="#8b949e", outline="")

        # Dronlar ve mesafe daireleri
        for d_id, data in self.drones.items():
            cx = C + data["x"] * S
            cy = C - data["y"] * S

            # Her dronun kalibreli mesafe dairesi (yarı saydam)
            d_m = self.distances[d_id]
            d_px = d_m * S
            if d_px > 3 and d_px < CS * 2:
                c.create_oval(cx - d_px, cy - d_px, cx + d_px, cy + d_px,
                              outline=data["color"], dash=(3, 6), width=1)

            # Drone ikonu
            if 0 <= cx <= CS and 0 <= cy <= CS:
                c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12,
                              fill=data["color"], outline="white", width=2)
                c.create_text(cx, cy, text=str(d_id), fill="white",
                              font=("Arial", 10, "bold"))
                # RSSI ve mesafe etiketi
                c.create_text(cx, cy - 20,
                              text=f"{self.rssi_cal[d_id]:.0f}dB | {self.distances[d_id]:.1f}m",
                              fill="white", font=("Arial", 8, "bold"))

        # İz çizgisi (trail)
        if len(self.trail) >= 2:
            for i in range(1, len(self.trail)):
                x0 = C + self.trail[i - 1][0] * S
                y0 = C - self.trail[i - 1][1] * S
                x1 = C + self.trail[i][0] * S
                y1 = C - self.trail[i][1] * S
                ratio = i / len(self.trail)
                r = int(200 * ratio)
                g = int(60 * ratio)
                b = int(60 * ratio)
                color = f"#{r:02x}{g:02x}{b:02x}"
                c.create_line(x0, y0, x1, y1, fill=color, width=1)

        # Hedef noktası
        t_px = C + self.target_x * S
        t_py = C - self.target_y * S
        dist = math.sqrt(self.target_x ** 2 + self.target_y ** 2)

        # Merkez -> Hedef çizgisi
        c.create_line(C, C, t_px, t_py, fill="#f85149", dash=(3, 3), width=2)

        if self.target_visible:
            if 0 <= t_px <= CS and 0 <= t_py <= CS:
                # Pulse animasyonu
                pulse_r = 14 + self.target_pulse
                c.create_oval(t_px - pulse_r, t_py - pulse_r,
                              t_px + pulse_r, t_py + pulse_r,
                              fill="", outline="#7a2824", width=2)
                c.create_oval(t_px - 8, t_py - 8, t_px + 8, t_py + 8,
                              fill="#f85149", outline="white", width=2)
                c.create_text(t_px, t_py - 24, text=f"📻 {dist:.1f}m",
                              fill="#f85149", font=("Arial", 10, "bold"))
            else:
                # Ekran dışı: ok ile göster
                angle = math.atan2(-(t_py - C), t_px - C)
                edge_x = C + (CS / 2 - 25) * math.cos(angle)
                edge_y = C - (CS / 2 - 25) * math.sin(angle)
                c.create_text(edge_x, edge_y,
                              text=f"📻 {dist:.1f}m →",
                              fill="#f85149", font=("Arial", 11, "bold"))

        # Yön hesabı
        if dist > 0.5:
            angle_deg = math.degrees(math.atan2(self.target_x, self.target_y)) % 360
            direction = self._angle_to_dir(angle_deg)
        else:
            direction = "⭕ MERKEZ"
            angle_deg = 0

        # Güç farkları
        delta_ns = self.rssi_cal[1] - self.rssi_cal[2]
        delta_ew = self.rssi_cal[3] - self.rssi_cal[4]

        info = (
            f"🧭 Yön: {direction}\n"
            f"📍 Konum: D:{self.target_x:+.1f}m  K:{self.target_y:+.1f}m\n"
            f"📏 Mesafe: {dist:.1f}m\n\n"
            f"📊 ΔK-G: {delta_ns:+.1f} dB | ΔD-B: {delta_ew:+.1f} dB\n"
            f"🔍 Görüş: {view_range:.0f}m"
        )
        self.info_label.config(text=info)

        # Drone paneli güncelle
        for d_id in self.drones:
            txt = f"  {self.rssi_cal[d_id]:+.0f} dBm → {self.distances[d_id]:.1f}m"
            self.drone_labels[d_id].config(text=txt)

        self.zoom_label.config(
            text=f"Zoom: {view_range:.0f}m görüş | Ölçek: 1m = {S:.0f}px"
        )

    def _angle_to_dir(self, deg):
        dirs = [
            (0, "⬆️  KUZEY"), (45, "↗️  KD"), (90, "➡️  DOĞU"),
            (135, "↘️  GD"), (180, "⬇️  GÜNEY"), (225, "↙️  GB"),
            (270, "⬅️  BATI"), (315, "↖️  KB"),
        ]
        for center, name in dirs:
            diff = abs(deg - center)
            if diff > 180:
                diff = 360 - diff
            if diff <= 22.5:
                return name
        return "⬆️  KUZEY"

    def clear_trail(self):
        self.trail.clear()
        self.update_radar()

    def animate(self):
        self.target_visible = not self.target_visible
        self.target_pulse = (self.target_pulse + 1) % 6
        self.update_radar()
        self.root.after(500, self.animate)


if __name__ == "__main__":
    root = tk.Tk()
    app = KalibreliRadar(root)
    root.mainloop()
