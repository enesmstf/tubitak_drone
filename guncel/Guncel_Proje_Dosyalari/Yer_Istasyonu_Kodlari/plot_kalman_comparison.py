#!/usr/bin/env python3
"""
plot_kalman_comparison.py
Bu betik, yer istasyonu tarafından kaydedilen deney loglarını inceler.
Telsizin aktif olduğu (-70 ile -95 dBm arası) en kaliteli veriyi seçer veya
argüman olarak verilen CSV dosyasını işleyerek TÜBİTAK raporuna doğrudan
eklenebilecek akademik 4-drone grafiğini ve istatistik tablosunu üretir.
"""

import sys
import glob
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 300

def select_best_file():
    candidates = sorted(glob.glob("kalman_deney_log_*.csv") + glob.glob("/home/enes/Belgeler/tubitak_drone/kalman_deney_log_*.csv"))
    if not candidates:
        return None
    
    # -70 ile -100 dBm arasında en çok aktif telsiz verisi olan dosyayı bul
    best_file = None
    best_active_count = -1
    
    for f in set(candidates):
        try:
            with open(f, "r", encoding="utf-8") as fp:
                lines = fp.readlines()
                active = 0
                for line in lines[1:]:
                    parts = line.strip().split(",")
                    if len(parts) >= 5:
                        raw = float(parts[3])
                        if -105.0 < raw < -60.0:  # Gerçek telsiz sinyali
                            active += 1
                if active > best_active_count:
                    best_active_count = active
                    best_file = f
        except Exception:
            continue
            
    return best_file

def main():
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        csv_file = sys.argv[1]
    else:
        csv_file = select_best_file()
        if not csv_file:
            print("[HATA] Uygun 'kalman_deney_log_*.csv' dosyası bulunamadı.")
            return

    print(f"\n[BİLGİ] İncelenen Deney Dosyası: {os.path.basename(csv_file)}")
    print(f"[YOL] {csv_file}")

    # CSV verisini oku
    data = []
    with open(csv_file, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 7:
                try:
                    data.append({
                        "time": float(parts[1]),
                        "drone_id": int(parts[2]),
                        "raw_rssi": float(parts[3]),
                        "kalman_rssi": float(parts[4]),
                        "center_north": float(parts[5]),
                        "center_east": float(parts[6])
                    })
                except ValueError:
                    continue

    if not data:
        print("[HATA] Dosya boş veya okunamadı.")
        return

    # Sadece aktif telsiz sinyali (> -110 dBm) olan satırları filtrele (dip gürültü -120 dBm olan kısımları temizle)
    active_data = [d for d in data if d["raw_rssi"] > -110.0]
    if not active_data:
        print("[UYARI] Dosyada -110 dBm üzerinde aktif telsiz sinyali bulunamadı, tüm veriler kullanılıyor.")
        active_data = data

    drones = sorted(list(set(d["drone_id"] for d in active_data)))
    print(f"[BİLGİ] Aktif Telsiz Örneklemi: {len(active_data)} adet | Algılanan Dronlar: {drones}")

    # -------------------------------------------------------------
    # İSTATİSTİKSEL HESAPLAMALAR (DRONE BAZINDA)
    # -------------------------------------------------------------
    stats = {}
    print("\n" + "=" * 80)
    print("  📊 TÜBİTAK RAPORU İÇİN KALMAN FİLTRESİ PERFORMANS TABLOSU 📊")
    print("  (Telsiz Açıkken - Gerçek Sinyal Bölgesi: -76 dBm ile -100 dBm arası)")
    print("=" * 80)
    print(f"{'İHA / Metrik':<12} | {'Ham Varyans':<13} | {'Kalman Varyans':<15} | {'Varyans ↓':<11} | {'Titreme ↓':<11}")
    print("-" * 80)

    total_var_red = []
    total_jit_red = []

    for did in drones:
        d_raw = [d["raw_rssi"] for d in active_data if d["drone_id"] == did]
        d_kal = [d["kalman_rssi"] for d in active_data if d["drone_id"] == did]
        d_times = [d["time"] for d in active_data if d["drone_id"] == did]

        if len(d_raw) < 5:
            continue

        raw_var = np.var(d_raw)
        kal_var = np.var(d_kal)
        var_red = (1.0 - (kal_var / raw_var)) * 100.0 if raw_var > 0 else 0.0

        raw_diffs = np.abs(np.diff(d_raw))
        kal_diffs = np.abs(np.diff(d_kal))
        raw_jit = np.mean(raw_diffs) if len(raw_diffs) > 0 else 0.0
        kal_jit = np.mean(kal_diffs) if len(kal_diffs) > 0 else 0.0
        jit_red = (1.0 - (kal_jit / raw_jit)) * 100.0 if raw_jit > 0 else 0.0

        stats[did] = {
            "times": np.array(d_times) - d_times[0],
            "raw": d_raw,
            "kal": d_kal,
            "raw_var": raw_var,
            "kal_var": kal_var,
            "raw_std": np.std(d_raw),
            "kal_std": np.std(d_kal),
            "var_red": var_red,
            "raw_jit": raw_jit,
            "kal_jit": kal_jit,
            "jit_red": jit_red,
        }

        total_var_red.append(var_red)
        total_jit_red.append(jit_red)

        print(f"Drone {did:<6} | {raw_var:<13.2f} | {kal_var:<15.2f} | %{var_red:<10.1f} | %{jit_red:<10.1f}")

    print("-" * 80)
    avg_v = np.mean(total_var_red) if total_var_red else 0.0
    avg_j = np.mean(total_jit_red) if total_jit_red else 0.0
    print(f"{'GENEL ORT.':<12} | {'-':<13} | {'-':<15} | %{avg_v:<10.1f} | %{avg_j:<10.1f}")
    print("=" * 80 + "\n")

    # -------------------------------------------------------------
    # GRAFİK 1: 4 DRONE RSSI KARŞILAŞTIRMASI (ZAMAN SERİSİ)
    # -------------------------------------------------------------
    active_dids = [d for d in drones if d in stats]
    fig, axes = plt.subplots(len(active_dids), 1, figsize=(11, 2.8 * len(active_dids)), sharex=True)
    if len(active_dids) == 1:
        axes = [axes]

    for ax, did in zip(axes, active_dids):
        st = stats[did]
        ax.plot(st["times"], st["raw"], label=f"Ham RSSI (σ={st['raw_std']:.2f} dBm)",
                color="#1f77b4", alpha=0.55, linestyle="--", linewidth=1.2)
        ax.plot(st["times"], st["kal"], label=f"Kalman Filtrelenmiş (σ={st['kal_std']:.2f} dBm)",
                color="#d62728", linewidth=2.0)
        ax.set_ylabel("Sinyal (dBm)", fontsize=10, fontweight="bold")
        ax.set_title(f"Drone {did}  —  Gürültü Azaltma: %{st['var_red']:.1f}  |  Titreme Azaltma: %{st['jit_red']:.1f}",
                     fontsize=10, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(fontsize=9, loc="upper right")

    axes[-1].set_xlabel("Zaman (saniye)", fontsize=11, fontweight="bold")
    fig.suptitle("Skaler Kalman Filtresinin Ham Telsiz Sinyali Üzerindeki Filtreleme Başarımı",
                 fontsize=12, fontweight="bold", y=0.995)
    fig.tight_layout()

    out1 = "kalman_rssi_karsilastirma.png"
    plt.savefig(out1, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[BAŞARILI] 4-Drone Sinyal Grafiği Kaydedildi -> {out1}")

    # -------------------------------------------------------------
    # GRAFİK 2: SANAL MERKEZ HAREKETİ (X-Y DÜZLEMİ)
    # -------------------------------------------------------------
    c_north = [d["center_north"] for d in active_data]
    c_east = [d["center_east"] for d in active_data]

    plt.figure(figsize=(7, 7))
    plt.plot(c_east, c_north, label="Sürü Sanal Merkezi Yörüngesi", color="#2ca02c", marker="o", markersize=3, linewidth=1.5)
    plt.scatter([c_east[0]], [c_north[0]], color="#1f77b4", s=120, edgecolors="black", label="Başlangıç (Origin)", zorder=5)
    plt.scatter([c_east[-1]], [c_north[-1]], color="#d62728", s=120, edgecolors="black", label="Hedef Yönü", zorder=5)
    plt.title("Sürünün Hedefe Doğru Hesaplanan Sanal Merkez Hareketi", fontsize=11, fontweight="bold")
    plt.xlabel("Doğu Ekseni (Metre)", fontsize=10, fontweight="bold")
    plt.ylabel("Kuzey Ekseni (Metre)", fontsize=10, fontweight="bold")
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.axis("equal")
    plt.legend(fontsize=10)
    plt.tight_layout()

    out2 = "kalman_rota_karsilastirma.png"
    plt.savefig(out2, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[BAŞARILI] Sanal Merkez Yörünge Grafiği Kaydedildi -> {out2}")

if __name__ == "__main__":
    main()
