"""
kalibrasyon_oturumu.py

13 BİLİNEN KONUMLU KALİBRASYON OTURUMU

Formasyon merkezinde 4 drone '+' şeklinde (her biri merkezden 5m) dururken,
hedefi sırayla 13 BİLİNEN noktaya yerleştirip her noktada birkaç saniye
canlı RSSI ortalaması alınır. Noktalar:

  - Merkez (4 drone'a da eşit uzaklıkta, 5m)
  - Her drone için 3 mesafe kademesi (drone+1m, +5m, +10m -> merkeze göre
    6m, 10m, 15m), o drone'un TAM ARKASINDAN (merkeze göre radyal yönde)

Toplam 13 nokta x 4 drone = 52 ölçüm noktası. Bu veri, TEK bir eşit-uzaklık
testinden çok daha güçlü bir kalibrasyon sağlar: hem her drone'un sabit
donanım yanlılığını (kalibrasyon ofseti) HEM DE gerçek RSSI-mesafe
modelini (Friis: P_ref, path-loss üsteli n) ORTAK bir en küçük kareler
regresyonuyla aynı anda tahmin eder.

Sonuçlar, ground_station_core.py'deki CALIBRATION_OFFSET_DB ve
swarm_radar.py'deki mesafe modeli sabitlerine doğrudan aktarılabilir
biçimde ekrana basılır ve bir CSV/JSON dosyasına kaydedilir.
"""

import tkinter as tk
from tkinter import messagebox
import math
import platform
import serial
import threading
import time
import json
import csv
import datetime

import numpy as np

LORA_BAUD = 9600
FORMATION_DISTANCE_M = 5.0   # mevcut '+' formasyonuyla aynı (merkezden her drone'a mesafe)
RECORD_DURATION_S = 5.0      # her noktada kaç saniye ortalama alınacak

DRONE_NAMES = {1: "Kuzey", 2: "Güney", 3: "Doğu", 4: "Batı"}
DRONE_LOCAL_POS = {  # (Kuzey_m, Doğu_m) - mevcut '+' formasyonla aynı
    1: (FORMATION_DISTANCE_M, 0.0),
    2: (-FORMATION_DISTANCE_M, 0.0),
    3: (0.0, FORMATION_DISTANCE_M),
    4: (0.0, -FORMATION_DISTANCE_M),
}
DRONE_DIRECTION = {1: (1, 0), 2: (-1, 0), 3: (0, 1), 4: (0, -1)}


def build_calibration_points(tier_offsets_m=(1, 5, 10)):
    """13 kalibrasyon noktasını (etiket, (Kuzey_m,Doğu_m)) olarak üretir."""
    points = [("Merkez", (0.0, 0.0))]
    for d in (1, 2, 3, 4):
        dn, de = DRONE_DIRECTION[d]
        for tier in tier_offsets_m:
            dist_from_center = FORMATION_DISTANCE_M + tier
            points.append((f"{DRONE_NAMES[d]}_{tier}m", (dn * dist_from_center, de * dist_from_center)))
    return points


CALIBRATION_POINTS = build_calibration_points()


def compute_checksum(payload: str) -> int:
    return sum(ord(c) for c in payload) % 256


def fit_calibration(results):
    """
    results: {label: {drone_id: mean_rssi, ...}, ...}
    Ortak regresyon: RSSI(d,p) = P_ref - 10*n*log10(dist(d,p)) + offset(d)
    (offset(1) = 0, referans olarak sabitlenir)
    unknowns = [P_ref, n, offset2, offset3, offset4]
    """
    A_mat, b_vec, meta = [], [], []
    for label, (pn, pe) in CALIBRATION_POINTS:
        if label not in results:
            continue
        for d, (dn, de) in DRONE_LOCAL_POS.items():
            if d not in results[label]:
                continue
            dist = math.dist((pn, pe), (dn, de))
            if dist <= 0:
                continue
            rssi = results[label][d]
            row = [1.0, -10 * math.log10(dist),
                   1.0 if d == 2 else 0.0,
                   1.0 if d == 3 else 0.0,
                   1.0 if d == 4 else 0.0]
            A_mat.append(row)
            b_vec.append(rssi)
            meta.append((label, d, dist, rssi))

    if len(A_mat) < 6:
        return None

    A_mat = np.array(A_mat)
    b_vec = np.array(b_vec)
    sol, residuals, rank, sv = np.linalg.lstsq(A_mat, b_vec, rcond=None)
    P_ref, n, off2, off3, off4 = sol
    offsets = {1: 0.0, 2: float(off2), 3: float(off3), 4: float(off4)}

    # ortalama kalıntı (fit kalitesi göstergesi)
    predicted = A_mat @ sol
    resid = b_vec - predicted
    rmse = float(np.sqrt(np.mean(resid ** 2)))

    return {
        "P_ref": float(P_ref),
        "n": float(n),
        "offsets": offsets,
        "rmse_db": rmse,
        "n_measurements": len(A_mat),
    }


class CalibrationSession:
    def __init__(self, root):
        self.root = root
        self.root.title("Kalibrasyon Oturumu - 13 Nokta")
        self.root.geometry("980x700")
        self.root.configure(bg="#1a1a2e")

        self.serial_port = None
        self.is_reading = False
        self.current_index = 0
        self.recording = False
        self.record_samples = {1: [], 2: [], 3: [], 4: []}
        self.results = {}  # label -> {drone_id: mean_rssi}
        self.live_rssi = {1: None, 2: None, 3: None, 4: None}

        self.setup_ui()
        self.update_point_display()
        self.redraw_canvas()

    def default_port(self):
        system = platform.system()
        if system == "Windows":
            return "COM3"
        elif system == "Darwin":
            return "/dev/tty.usbserial-0001"
        return "/dev/ttyUSB0"

    def setup_ui(self):
        canvas_frame = tk.Frame(self.root, bg="#1a1a2e")
        canvas_frame.pack(side=tk.LEFT, padx=10, pady=10)
        self.canvas = tk.Canvas(canvas_frame, width=480, height=480, bg="#16213e", highlightthickness=0)
        self.canvas.pack()

        control = tk.Frame(self.root, bg="#1a1a2e")
        control.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        conn_frame = tk.Frame(control, bg="#0f3460", padx=8, pady=8)
        conn_frame.pack(fill=tk.X, pady=5)
        tk.Label(conn_frame, text="LoRa Port:", fg="white", bg="#0f3460", font=("Arial", 10, "bold")).pack(anchor="w")
        row = tk.Frame(conn_frame, bg="#0f3460")
        row.pack(fill=tk.X, pady=3)
        self.port_entry = tk.Entry(row, width=18, font=("Arial", 11))
        self.port_entry.insert(0, self.default_port())
        self.port_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.btn_connect = tk.Button(row, text="BAĞLAN", bg="#27ae60", fg="white",
                                      font=("Arial", 9, "bold"), command=self.toggle_serial)
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status_label = tk.Label(conn_frame, text="⚪ Bağlantı Yok", fg="#95a5a6", bg="#0f3460", font=("Arial", 9))
        self.status_label.pack(anchor="w", pady=2)

        point_frame = tk.Frame(control, bg="#0f3460", padx=8, pady=8)
        point_frame.pack(fill=tk.X, pady=5)
        self.point_title = tk.Label(point_frame, text="", fg="#f1c40f", bg="#0f3460", font=("Arial", 13, "bold"))
        self.point_title.pack(anchor="w")
        self.point_instr = tk.Label(point_frame, text="", fg="white", bg="#0f3460", font=("Arial", 10),
                                     wraplength=420, justify=tk.LEFT)
        self.point_instr.pack(anchor="w", pady=4)

        live_frame = tk.Frame(control, bg="#1a1a2e")
        live_frame.pack(fill=tk.X, pady=5)
        tk.Label(live_frame, text="Canlı RSSI (dBm)", fg="white", bg="#1a1a2e", font=("Arial", 11, "bold")).pack(anchor="w")
        self.live_labels = {}
        for d in (1, 2, 3, 4):
            lbl = tk.Label(live_frame, text=f"Drone {d} ({DRONE_NAMES[d]}): ---",
                            fg="#2ecc71", bg="#1a1a2e", font=("Courier", 10))
            lbl.pack(anchor="w")
            self.live_labels[d] = lbl

        self.record_btn = tk.Button(control, text=f"KAYDET ({RECORD_DURATION_S:.0f}sn ORTALAMA)",
                                     bg="#e67e22", fg="white", font=("Arial", 11, "bold"),
                                     command=self.start_recording)
        self.record_btn.pack(fill=tk.X, pady=8)
        self.progress_label = tk.Label(control, text="", fg="#f1c40f", bg="#1a1a2e", font=("Arial", 10))
        self.progress_label.pack()

        nav_frame = tk.Frame(control, bg="#1a1a2e")
        nav_frame.pack(fill=tk.X, pady=5)
        tk.Button(nav_frame, text="< Önceki", command=self.prev_point).pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(nav_frame, text="Sonraki >", command=self.next_point).pack(side=tk.LEFT, expand=True, fill=tk.X)

        self.progress_list = tk.Listbox(control, height=13, font=("Courier", 9), bg="#0a0a23", fg="#2ecc71")
        self.progress_list.pack(fill=tk.BOTH, expand=True, pady=8)
        self.refresh_progress_list()

        action_frame = tk.Frame(control, bg="#1a1a2e")
        action_frame.pack(fill=tk.X, pady=5)
        tk.Button(action_frame, text="ANALİZ ET (Kalibrasyonu Hesapla)", bg="#8e44ad", fg="white",
                  font=("Arial", 10, "bold"), command=self.run_analysis).pack(fill=tk.X, pady=2)
        tk.Button(action_frame, text="CSV'ye Kaydet", command=self.export_csv).pack(fill=tk.X, pady=2)

        self.result_text = tk.Text(control, height=8, bg="#0a0a23", fg="#ecf0f1", font=("Courier", 9))
        self.result_text.pack(fill=tk.BOTH, pady=5)

    # -------------------------------------------------
    def toggle_serial(self):
        if not self.is_reading:
            port = self.port_entry.get().strip()
            try:
                self.serial_port = serial.Serial(port, LORA_BAUD, timeout=1)
                self.is_reading = True
                self.btn_connect.config(text="KES", bg="#c0392b")
                self.status_label.config(text="🟢 Bağlı", fg="#2ecc71")
                threading.Thread(target=self.reader_loop, daemon=True).start()
            except Exception as e:
                self.status_label.config(text=f"🔴 HATA: {e}", fg="#e74c3c")
        else:
            self.is_reading = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.btn_connect.config(text="BAĞLAN", bg="#27ae60")
            self.status_label.config(text="⚪ Bağlantı Yok", fg="#95a5a6")

    def reader_loop(self):
        buffer = ""
        while self.is_reading:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    break
                raw = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not raw:
                    continue
                buffer += raw.decode("ascii", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line.startswith("N"):
                        parts = line[1:].split(",")
                        if len(parts) == 3:
                            try:
                                drone_id = int(parts[0])
                                rssi = int(parts[1])
                                chk = int(parts[2])
                                body = f"N{drone_id},{rssi}"
                                if compute_checksum(body) == chk and drone_id in self.live_rssi:
                                    self.live_rssi[drone_id] = rssi
                                    self.root.after(0, self.update_live_label, drone_id, rssi)
                                    if self.recording:
                                        self.record_samples[drone_id].append(rssi)
                            except ValueError:
                                pass
            except Exception:
                pass

    def update_live_label(self, drone_id, rssi):
        self.live_labels[drone_id].config(text=f"Drone {drone_id} ({DRONE_NAMES[drone_id]}): {rssi} dBm")

    # -------------------------------------------------
    def update_point_display(self):
        label, (pn, pe) = CALIBRATION_POINTS[self.current_index]
        done = " ✅ (kaydedildi)" if label in self.results else ""
        self.point_title.config(text=f"Nokta {self.current_index+1}/{len(CALIBRATION_POINTS)}: {label}{done}")
        if label == "Merkez":
            instr = "Hedefi formasyonun TAM MERKEZİNE yerleştirin (4 drone'a da 5m eşit uzaklıkta)."
        else:
            drone_name, tier = label.split("_")
            instr = (f"Hedefi Drone ({drone_name})'nin TAM ARKASINA, drone'dan {tier} öteye yerleştirin "
                     f"(merkeze göre toplam {round(math.dist((pn,pe),(0,0)),0):.0f}m).")
        self.point_instr.config(text=instr)
        self.redraw_canvas()

    def next_point(self):
        if self.current_index < len(CALIBRATION_POINTS) - 1:
            self.current_index += 1
            self.update_point_display()

    def prev_point(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.update_point_display()

    def refresh_progress_list(self):
        self.progress_list.delete(0, tk.END)
        for i, (label, _) in enumerate(CALIBRATION_POINTS):
            mark = "✅" if label in self.results else "⬜"
            self.progress_list.insert(tk.END, f"{mark} {i+1:2d}. {label}")

    # -------------------------------------------------
    def start_recording(self):
        if not self.is_reading:
            messagebox.showwarning("Uyarı", "Önce LoRa portuna bağlanın.")
            return
        if self.recording:
            return
        self.recording = True
        self.record_samples = {1: [], 2: [], 3: [], 4: []}
        self.record_btn.config(state=tk.DISABLED, bg="#7f8c8d")
        self._record_start = time.time()
        self._record_tick()

    def _record_tick(self):
        elapsed = time.time() - self._record_start
        remaining = max(0.0, RECORD_DURATION_S - elapsed)
        self.progress_label.config(text=f"Kaydediliyor... {remaining:.1f}sn kaldı")
        if remaining > 0:
            self.root.after(100, self._record_tick)
        else:
            self._finish_recording()

    def _finish_recording(self):
        self.recording = False
        label, _ = CALIBRATION_POINTS[self.current_index]
        means = {}
        counts = {}
        for d in (1, 2, 3, 4):
            samples = self.record_samples[d]
            if samples:
                means[d] = sum(samples) / len(samples)
                counts[d] = len(samples)
        if len(means) < 4:
            missing = [d for d in (1, 2, 3, 4) if d not in means]
            messagebox.showwarning("Eksik Veri",
                                    f"Bu noktada şu dronelardan hiç paket alınamadı: {missing}. "
                                    f"Tekrar deneyin.")
        else:
            self.results[label] = means
            self.progress_label.config(
                text=f"Kaydedildi: " + ", ".join(f"D{d}={means[d]:.1f}dBm(n={counts[d]})" for d in means))
        self.record_btn.config(state=tk.NORMAL, bg="#e67e22")
        self.refresh_progress_list()
        self.update_point_display()

    # -------------------------------------------------
    def redraw_canvas(self):
        self.canvas.delete("all")
        W, H = 480, 480
        cx, cy = W // 2, H // 2
        scale = 12  # px per meter

        for i in range(0, W, scale * 5):
            self.canvas.create_line(i, 0, i, H, fill="#1a1a2e")
            self.canvas.create_line(0, i, W, i, fill="#1a1a2e")
        self.canvas.create_line(cx, 0, cx, H, fill="#7f8c8d", dash=(4, 4))
        self.canvas.create_line(0, cy, W, cy, fill="#7f8c8d", dash=(4, 4))

        # tüm 13 nokta (zayıf renk), kaydedilenler yeşil, mevcut nokta sarı
        for i, (label, (pn, pe)) in enumerate(CALIBRATION_POINTS):
            px = cx + pe * scale
            py = cy - pn * scale
            if i == self.current_index:
                color = "#f1c40f"
                r = 7
            elif label in self.results:
                color = "#2ecc71"
                r = 5
            else:
                color = "#566573"
                r = 4
            self.canvas.create_oval(px - r, py - r, px + r, py + r, fill=color, outline="white")

        # drone'lar
        for d, (dn, de) in DRONE_LOCAL_POS.items():
            px = cx + de * scale
            py = cy - dn * scale
            self.canvas.create_oval(px - 10, py - 10, px + 10, py + 10, fill="#3498db", outline="white", width=2)
            self.canvas.create_text(px, py, text=str(d), fill="white", font=("Arial", 9, "bold"))
            self.canvas.create_text(px, py + 20, text=DRONE_NAMES[d], fill="#3498db", font=("Arial", 8, "bold"))

        self.canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="white")

    # -------------------------------------------------
    def run_analysis(self):
        if len(self.results) < 4:
            messagebox.showwarning("Yetersiz Veri", "Analiz için en az birkaç nokta kaydedilmiş olmalı.")
            return
        fit = fit_calibration(self.results)
        if fit is None:
            messagebox.showwarning("Yetersiz Veri", "Regresyon için yeterli ölçüm yok.")
            return

        self.result_text.delete("1.0", tk.END)
        lines = []
        lines.append(f"Kullanılan ölçüm sayısı: {fit['n_measurements']} / 52")
        lines.append(f"Fit RMSE: {fit['rmse_db']:.2f} dB (ne kadar küçükse o kadar iyi)")
        lines.append(f"P_ref (1m'de beklenen RSSI): {fit['P_ref']:.2f} dBm")
        lines.append(f"n (path-loss üsteli): {fit['n']:.3f}")
        lines.append("")
        lines.append("--- ground_station_core.py için CALIBRATION_OFFSET_DB ---")
        lines.append("CALIBRATION_OFFSET_DB = {")
        for d in (1, 2, 3, 4):
            lines.append(f"    {d}: {fit['offsets'][d]:.2f},")
        lines.append("}")
        lines.append("")
        lines.append("--- swarm_radar.py için mesafe modeli ---")
        lines.append(f"RSSI_REF (1m) = {fit['P_ref']:.1f}   PATH_LOSS_EXP = {fit['n']:.2f}")
        self.result_text.insert(tk.END, "\n".join(lines))

        self._last_fit = fit
        try:
            with open("kalibrasyon_sonucu.json", "w", encoding="utf-8") as f:
                json.dump({"fit": fit, "raw_results": self.results}, f, ensure_ascii=False, indent=2)
            self.progress_label.config(text="kalibrasyon_sonucu.json kaydedildi.")
        except Exception as e:
            print("JSON kaydetme hatası:", e)

    def export_csv(self):
        fname = f"kalibrasyon_veri_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(fname, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["nokta", "kuzey_m", "dogu_m", "drone_id", "gercek_mesafe_m", "ortalama_rssi_dbm"])
            for label, (pn, pe) in CALIBRATION_POINTS:
                if label not in self.results:
                    continue
                for d, mean_rssi in self.results[label].items():
                    dn, de = DRONE_LOCAL_POS[d]
                    dist = math.dist((pn, pe), (dn, de))
                    writer.writerow([label, pn, pe, d, round(dist, 2), round(mean_rssi, 2)])
        messagebox.showinfo("Kaydedildi", f"{fname} olarak kaydedildi.")


if __name__ == "__main__":
    root = tk.Tk()
    app = CalibrationSession(root)
    root.mainloop()
