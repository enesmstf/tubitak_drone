#!/usr/bin/env python3
"""
plot_sismf_comparison.py
========================
Yer istasyonu tarafından kaydedilen deney loglarını okur ve 4 filtre
senaryosunu istatistiksel ve grafiksel olarak karşılaştırır:

  1) HAM RSSI         — Filtrelenmemiş ham kalibre sinyal
  2) KALMAN           — Skaler Kalman Filtresi (Rapor 2)
  3) SISMF            — SCI_4 (2026) Smoother Integral Sliding Mode Filter (tek başına)
  4) KALMAN + SISMF   — Kalman çıktısının SISMF'den de geçirilmiş hali (zincir filtre)

Üretilen Ayrı Grafikler:
  1) {csv_basename}_varyans_analizi.png    -> Sadece Varyans (Güç Dağılımı ve Azalma Oranı)
  2) {csv_basename}_titresim_analizi.png   -> Sadece Titreşim / Chattering (Jitter ve Pürüzsüzlük)
  3) {csv_basename}_zaman_serisi.png       -> 4 İHA × 4 Filtre Zaman Takip Eğrileri

Kullanım:
  python3 plot_sismf_comparison.py                          # en son CSV'yi otomatik bulur
  python3 plot_sismf_comparison.py kalman_deney_log_xxx.csv  # belirli bir CSV
"""

import sys
import glob
import os
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 300

SONUC_KLASORU = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sismf_test_sonuclari")


def select_latest_csv():
    candidates = sorted(
        glob.glob("kalman_deney_log_*.csv") +
        glob.glob("/home/enes/Belgeler/tubitak_drone/kalman_deney_log_*.csv")
    )
    if not candidates:
        return None
    return max(set(candidates), key=os.path.getctime)


def main():
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        csv_file = sys.argv[1]
    else:
        csv_file = select_latest_csv()
        if not csv_file:
            print("[HATA] İncelenecek deney CSV dosyası bulunamadı.")
            return

    csv_basename = os.path.splitext(os.path.basename(csv_file))[0]
    print(f"\n[BİLGİ] İncelenen Deney Dosyası: {os.path.basename(csv_file)}")
    print(f"[YOL] {csv_file}")

    # --- CSV Okuma ---
    data = []
    has_combo_column = False
    with open(csv_file, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        has_combo_column = "KalmanSISMF_RSSI" in header
        has_sismf_column = "SISMF_RSSI" in header

        for line in f:
            parts = line.strip().split(",")
            try:
                if has_combo_column and len(parts) >= 10:
                    row = {
                        "time": float(parts[1]),
                        "drone_id": int(parts[2]),
                        "raw_rssi": float(parts[3]),
                        "kalman_rssi": float(parts[4]),
                        "sismf_rssi": float(parts[5]),
                        "combo_rssi": float(parts[6]),
                        "active_mode": parts[7],
                    }
                elif has_sismf_column and len(parts) >= 9:
                    row = {
                        "time": float(parts[1]),
                        "drone_id": int(parts[2]),
                        "raw_rssi": float(parts[3]),
                        "kalman_rssi": float(parts[4]),
                        "sismf_rssi": float(parts[5]),
                        "combo_rssi": None,
                        "active_mode": parts[6],
                    }
                elif len(parts) >= 7:
                    row = {
                        "time": float(parts[1]),
                        "drone_id": int(parts[2]),
                        "raw_rssi": float(parts[3]),
                        "kalman_rssi": float(parts[4]),
                        "sismf_rssi": None,
                        "combo_rssi": None,
                        "active_mode": "ESKI_LOG",
                    }
                else:
                    continue
                data.append(row)
            except (ValueError, IndexError):
                continue

    if not data:
        print("[HATA] Dosya boş veya okunamadı.")
        return

    # --- Sadece aktif sinyal (> -110 dBm) bölgesini al ---
    active_data = [d for d in data if d["raw_rssi"] > -110.0]
    if not active_data:
        active_data = data

    print(f"[BİLGİ] Toplam satır: {len(data)} | Aktif sinyal (>-110 dBm): {len(active_data)}")

    # --- SISMF & Combo eksikse simüle et ---
    needs_sismf = any(d["sismf_rssi"] is None for d in active_data)
    needs_combo = any(d["combo_rssi"] is None for d in active_data)

    if needs_sismf or needs_combo:
        print("[BİLGİ] Bazı sütunlar eksik; optimizasyonlu parametre seti ile hesaplanıyor...")
        sismf_states, sismf_ints, sismf_lts = {}, {}, {}
        combo_states, combo_ints, combo_lts = {}, {}, {}

        for d in active_data:
            did = d["drone_id"]
            raw = d["raw_rssi"]
            kal = d["kalman_rssi"]
            t = d["time"]

            # Tek başına SISMF
            if needs_sismf and d["sismf_rssi"] is None:
                if did not in sismf_states:
                    sismf_states[did] = raw
                    sismf_ints[did] = 0.0
                    sismf_lts[did] = t
                    dt = 0.5
                else:
                    dt = max(min(t - sismf_lts[did], 5.0), 0.05)
                    sismf_lts[did] = t
                err = raw - sismf_states[did]
                sismf_ints[did] = np.clip(sismf_ints[did] + err * dt, -5.0, 5.0)
                s = err + 0.02 * sismf_ints[did]
                dot = 0.3 * np.tanh(s / 2.5) + 0.008 * sismf_ints[did]
                sismf_states[did] += dot * dt
                d["sismf_rssi"] = sismf_states[did]

            # Kalman + SISMF Combo
            if needs_combo and d["combo_rssi"] is None:
                if did not in combo_states:
                    combo_states[did] = kal
                    combo_ints[did] = 0.0
                    combo_lts[did] = t
                    dt2 = 0.5
                else:
                    dt2 = max(min(t - combo_lts[did], 5.0), 0.05)
                    combo_lts[did] = t
                err2 = kal - combo_states[did]
                combo_ints[did] = np.clip(combo_ints[did] + err2 * dt2, -5.0, 5.0)
                s2 = err2 + 0.02 * combo_ints[did]
                dot2 = 0.3 * np.tanh(s2 / 2.5) + 0.008 * combo_ints[did]
                combo_states[did] += dot2 * dt2
                d["combo_rssi"] = combo_states[did]

    drones = sorted(set(d["drone_id"] for d in active_data))
    print(f"[BİLGİ] Analiz Edilen Örneklem: {len(active_data)} | Dronlar: {drones}")

    # --- İSTATİSTİKSEL HESAPLAMALAR ---
    drone_stats = {}
    all_vars = {"raw": [], "kal": [], "sismf": [], "combo": []}
    all_jits = {"raw": [], "kal": [], "sismf": [], "combo": []}

    for did in drones:
        sub = [d for d in active_data if d["drone_id"] == did]
        if len(sub) < 5:
            continue

        raws = [d["raw_rssi"] for d in sub]
        kals = [d["kalman_rssi"] for d in sub]
        sismfs = [d["sismf_rssi"] for d in sub]
        combos = [d["combo_rssi"] for d in sub]
        times = [d["time"] for d in sub]

        raw_var = np.var(raws)
        kal_var = np.var(kals)
        sismf_var = np.var(sismfs)
        combo_var = np.var(combos)

        kal_red = (1 - kal_var / raw_var) * 100 if raw_var > 0 else 0
        sismf_red = (1 - sismf_var / raw_var) * 100 if raw_var > 0 else 0
        combo_red = (1 - combo_var / raw_var) * 100 if raw_var > 0 else 0

        raw_jit = np.mean(np.abs(np.diff(raws))) if len(raws) > 1 else 0
        kal_jit = np.mean(np.abs(np.diff(kals))) if len(kals) > 1 else 0
        sismf_jit = np.mean(np.abs(np.diff(sismfs))) if len(sismfs) > 1 else 0
        combo_jit = np.mean(np.abs(np.diff(combos))) if len(combos) > 1 else 0

        kal_jit_red = (1 - kal_jit / raw_jit) * 100 if raw_jit > 0 else 0
        sismf_jit_red = (1 - sismf_jit / raw_jit) * 100 if raw_jit > 0 else 0
        combo_jit_red = (1 - combo_jit / raw_jit) * 100 if raw_jit > 0 else 0

        drone_stats[did] = {
            "times": np.array(times) - times[0],
            "raw": raws, "kal": kals, "sismf": sismfs, "combo": combos,
            "raw_var": raw_var, "kal_var": kal_var, "sismf_var": sismf_var, "combo_var": combo_var,
            "kal_red": kal_red, "sismf_red": sismf_red, "combo_red": combo_red,
            "raw_jit": raw_jit, "kal_jit": kal_jit, "sismf_jit": sismf_jit, "combo_jit": combo_jit,
            "kal_jit_red": kal_jit_red, "sismf_jit_red": sismf_jit_red, "combo_jit_red": combo_jit_red,
            "raw_std": np.std(raws), "kal_std": np.std(kals),
            "sismf_std": np.std(sismfs), "combo_std": np.std(combos),
        }

        all_vars["raw"].append(raw_var)
        all_vars["kal"].append(kal_var)
        all_vars["sismf"].append(sismf_var)
        all_vars["combo"].append(combo_var)
        all_jits["raw"].append(raw_jit)
        all_jits["kal"].append(kal_jit)
        all_jits["sismf"].append(sismf_jit)
        all_jits["combo"].append(combo_jit)

    # Genel Ortalamalar
    m_raw_v = np.mean(all_vars["raw"])
    m_kal_v = np.mean(all_vars["kal"])
    m_sismf_v = np.mean(all_vars["sismf"])
    m_combo_v = np.mean(all_vars["combo"])
    m_kal_red = (1 - m_kal_v / m_raw_v) * 100
    m_sismf_red = (1 - m_sismf_v / m_raw_v) * 100
    m_combo_red = (1 - m_combo_v / m_raw_v) * 100

    m_raw_j = np.mean(all_jits["raw"])
    m_kal_j = np.mean(all_jits["kal"])
    m_sismf_j = np.mean(all_jits["sismf"])
    m_combo_j = np.mean(all_jits["combo"])
    m_kal_j_red = (1 - m_kal_j / m_raw_j) * 100
    m_sismf_j_red = (1 - m_sismf_j / m_raw_j) * 100
    m_combo_j_red = (1 - m_combo_j / m_raw_j) * 100

    print("\n" + "=" * 135)
    print("  📊 TÜBİTAK 123E294 — 4 SENARYO FİLTRE PERFORMANS VE TİTREŞİM ANALİZ TABLOSU 📊")
    print("=" * 135)
    header_fmt = (f"{'İHA':<8} | {'Ham σ²':<8} | {'Kalman σ²':<9} | {'SISMF σ²':<9} | {'K+S σ²':<8} | "
                  f"{'Kalman Var↓%':<12} | {'SISMF Var↓%':<11} | {'K+S Var↓%':<10} | "
                  f"{'Kalman Jit↓%':<12} | {'SISMF Jit↓%':<11} | {'K+S Jit↓%':<10}")
    print(header_fmt)
    print("-" * 135)

    for did in sorted(drone_stats.keys()):
        st = drone_stats[did]
        print(f"Drone {did:<4} | {st['raw_var']:<8.2f} | {st['kal_var']:<9.2f} | {st['sismf_var']:<9.2f} | {st['combo_var']:<8.2f} | "
              f"%{st['kal_red']:<11.1f} | %{st['sismf_red']:<10.1f} | %{st['combo_red']:<9.1f} | "
              f"%{st['kal_jit_red']:<11.1f} | %{st['sismf_jit_red']:<10.1f} | %{st['combo_jit_red']:<9.1f}")

    print("-" * 135)
    print(f"{'GENEL ORT':<8} | {m_raw_v:<8.2f} | {m_kal_v:<9.2f} | {m_sismf_v:<9.2f} | {m_combo_v:<8.2f} | "
          f"%{m_kal_red:<11.1f} | %{m_sismf_red:<10.1f} | %{m_combo_red:<9.1f} | "
          f"%{m_kal_j_red:<11.1f} | %{m_sismf_j_red:<10.1f} | %{m_combo_j_red:<9.1f}")
    print("=" * 135)

    # --- Klasörler ---
    os.makedirs(SONUC_KLASORU, exist_ok=True)
    csv_dir = os.path.dirname(os.path.abspath(csv_file))

    active_dids = sorted(drone_stats.keys())
    x_labels = [f"Drone {d}" for d in active_dids] + ["ORTALAMA"]
    x = np.arange(len(x_labels))
    w = 0.20

    # =================================================================
    # 📈 AYRI GRAFİK 1: VARYANS ANALİZİ (GÜÇ KARARLILIĞI)
    # =================================================================
    fig_var, (ax_v1, ax_v2) = plt.subplots(1, 2, figsize=(15, 5.5))

    # Panel 1: Mutlak Varyans (dBm²)
    v_raw = [drone_stats[d]["raw_var"] for d in active_dids] + [m_raw_v]
    v_kal = [drone_stats[d]["kal_var"] for d in active_dids] + [m_kal_v]
    v_sismf = [drone_stats[d]["sismf_var"] for d in active_dids] + [m_sismf_v]
    v_combo = [drone_stats[d]["combo_var"] for d in active_dids] + [m_combo_v]

    b1 = ax_v1.bar(x - 1.5*w, v_raw, w, label="Ham Sinyal", color="#A0AEC0", alpha=0.9)
    b2 = ax_v1.bar(x - 0.5*w, v_kal, w, label="Kalman", color="#3182CE", alpha=0.9)
    b3 = ax_v1.bar(x + 0.5*w, v_sismf, w, label="SISMF (2026)", color="#E53E3E", alpha=0.9)
    b4 = ax_v1.bar(x + 1.5*w, v_combo, w, label="Kalman+SISMF", color="#38A169", alpha=0.9)

    ax_v1.set_ylabel("Mutlak Varyans σ² (dBm²)", fontweight="bold")
    ax_v1.set_title("1. Sinyal Mutlak Varyansı (Düşük Olan Daha İyi)", fontweight="bold", fontsize=11)
    ax_v1.set_xticks(x)
    ax_v1.set_xticklabels(x_labels, fontweight="bold")
    ax_v1.legend(loc="upper right", fontsize=8.5)
    ax_v1.grid(axis="y", linestyle=":", alpha=0.5)

    for bars in [b1, b2, b3, b4]:
        for bar in bars:
            h = bar.get_height()
            ax_v1.annotate(f"{h:.1f}", xy=(bar.get_x() + bar.get_width()/2, h),
                           xytext=(0, 3), textcoords="offset points", ha='center', fontsize=7.5)

    # Panel 2: Varyans Azalma Yüzdesi (%↓)
    red_kal = [drone_stats[d]["kal_red"] for d in active_dids] + [m_kal_red]
    red_sismf = [drone_stats[d]["sismf_red"] for d in active_dids] + [m_sismf_red]
    red_combo = [drone_stats[d]["combo_red"] for d in active_dids] + [m_combo_red]

    w2 = 0.25
    b5 = ax_v2.bar(x - w2, red_kal, w2, label="Kalman", color="#3182CE", alpha=0.9)
    b6 = ax_v2.bar(x, red_sismf, w2, label="SISMF (2026)", color="#E53E3E", alpha=0.9)
    b7 = ax_v2.bar(x + w2, red_combo, w2, label="Kalman+SISMF", color="#38A169", alpha=0.9)

    ax_v2.set_ylabel("Varyans Azalma Oranı (%)", fontweight="bold")
    ax_v2.set_title("2. Ham Sinyale Göre Varyans Azalma Oranı (Yüksek = Başarılı)", fontweight="bold", fontsize=11)
    ax_v2.set_xticks(x)
    ax_v2.set_xticklabels(x_labels, fontweight="bold")
    ax_v2.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax_v2.legend(loc="upper right", fontsize=8.5)
    ax_v2.grid(axis="y", linestyle=":", alpha=0.5)

    for bars in [b5, b6, b7]:
        for bar in bars:
            h = bar.get_height()
            ax_v2.annotate(f"%{h:.1f}", xy=(bar.get_x() + bar.get_width()/2, h),
                           xytext=(0, 3), textcoords="offset points", ha='center', fontsize=7.5, fontweight="bold")

    fig_var.suptitle("TÜBİTAK 123E294 — SİNYAL VARYANS VE KARARLILIK ANALİZİ (Ham vs Kalman vs SISMF)",
                     fontsize=12, fontweight="bold")
    fig_var.tight_layout()

    out_var_name = f"{csv_basename}_varyans_analizi.png"
    out_var_path = os.path.join(SONUC_KLASORU, out_var_name)
    plt.savefig(out_var_path, dpi=300, bbox_inches="tight")
    plt.close()
    shutil.copy2(out_var_path, os.path.join(csv_dir, out_var_name))
    print(f"\n[BAŞARILI] 1. Varyans Analiz Grafiği -> {out_var_name}")

    # =================================================================
    # 📉 AYRI GRAFİK 2: TİTREŞİM VE CHATTERING ANALİZİ
    # =================================================================
    fig_jit, (ax_j1, ax_j2) = plt.subplots(1, 2, figsize=(15, 5.5))

    # Panel 1: Mutlak Titreşim (Ardışık Örnekler Arası Ortalama Sıçrama, dB)
    j_raw = [drone_stats[d]["raw_jit"] for d in active_dids] + [m_raw_j]
    j_kal = [drone_stats[d]["kal_jit"] for d in active_dids] + [m_kal_j]
    j_sismf = [drone_stats[d]["sismf_jit"] for d in active_dids] + [m_sismf_j]
    j_combo = [drone_stats[d]["combo_jit"] for d in active_dids] + [m_combo_j]

    jb1 = ax_j1.bar(x - 1.5*w, j_raw, w, label="Ham Sinyal", color="#A0AEC0", alpha=0.9)
    jb2 = ax_j1.bar(x - 0.5*w, j_kal, w, label="Kalman", color="#3182CE", alpha=0.9)
    jb3 = ax_j1.bar(x + 0.5*w, j_sismf, w, label="SISMF (2026)", color="#E53E3E", alpha=0.9)
    jb4 = ax_j1.bar(x + 1.5*w, j_combo, w, label="Kalman+SISMF", color="#38A169", alpha=0.9)

    ax_j1.set_ylabel("Ortalama Titreşim / Adım Başına Sıçrama (dB)", fontweight="bold")
    ax_j1.set_title("1. Mutlak Titreşim (Jitter) Miktarı (Düşük Olan Daha Pürüzsüz)", fontweight="bold", fontsize=11)
    ax_j1.set_xticks(x)
    ax_j1.set_xticklabels(x_labels, fontweight="bold")
    ax_j1.legend(loc="upper right", fontsize=8.5)
    ax_j1.grid(axis="y", linestyle=":", alpha=0.5)

    for bars in [jb1, jb2, jb3, jb4]:
        for bar in bars:
            h = bar.get_height()
            ax_j1.annotate(f"{h:.2f}dB", xy=(bar.get_x() + bar.get_width()/2, h),
                           xytext=(0, 3), textcoords="offset points", ha='center', fontsize=7.2)

    # Panel 2: Titreşim Azalma Yüzdesi (%↓)
    jred_kal = [drone_stats[d]["kal_jit_red"] for d in active_dids] + [m_kal_j_red]
    jred_sismf = [drone_stats[d]["sismf_jit_red"] for d in active_dids] + [m_sismf_j_red]
    jred_combo = [drone_stats[d]["combo_jit_red"] for d in active_dids] + [m_combo_j_red]

    jb5 = ax_j2.bar(x - w2, jred_kal, w2, label="Kalman", color="#3182CE", alpha=0.9)
    jb6 = ax_j2.bar(x, jred_sismf, w2, label="SISMF (2026)", color="#E53E3E", alpha=0.9)
    jb7 = ax_j2.bar(x + w2, jred_combo, w2, label="Kalman+SISMF", color="#38A169", alpha=0.9)

    ax_j2.set_ylabel("Titreşim / Chattering Azalma Oranı (%)", fontweight="bold")
    ax_j2.set_title("2. Titreşim Sönümleme Başarısı (Yüksek = Zikzaksız Düz Uçuş)", fontweight="bold", fontsize=11)
    ax_j2.set_xticks(x)
    ax_j2.set_xticklabels(x_labels, fontweight="bold")
    ax_j2.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax_j2.legend(loc="upper right", fontsize=8.5)
    ax_j2.grid(axis="y", linestyle=":", alpha=0.5)

    for bars in [jb5, jb6, jb7]:
        for bar in bars:
            h = bar.get_height()
            ax_j2.annotate(f"%{h:.1f}", xy=(bar.get_x() + bar.get_width()/2, h),
                           xytext=(0, 3), textcoords="offset points", ha='center', fontsize=7.5, fontweight="bold")

    fig_jit.suptitle("TÜBİTAK 123E294 — SİNYAL TİTREŞİMİ (JITTER / CHATTERING) SÖNÜMLEME ANALİZİ",
                     fontsize=12, fontweight="bold")
    fig_jit.tight_layout()

    out_jit_name = f"{csv_basename}_titresim_analizi.png"
    out_jit_path = os.path.join(SONUC_KLASORU, out_jit_name)
    plt.savefig(out_jit_path, dpi=300, bbox_inches="tight")
    plt.close()
    shutil.copy2(out_jit_path, os.path.join(csv_dir, out_jit_name))
    print(f"[BAŞARILI] 2. Titreşim Analiz Grafiği -> {out_jit_name}")

    # =================================================================
    # ⏱️ AYRI GRAFİK 3: ZAMAN SERİSİ TAKİP EĞRİLERİ
    # =================================================================
    fig_ts, axes = plt.subplots(len(active_dids), 1, figsize=(14, 3.2 * len(active_dids)), sharex=True)
    if len(active_dids) == 1:
        axes = [axes]

    for ax, did in zip(axes, active_dids):
        st = drone_stats[did]
        ax.plot(st["times"], st["raw"], label=f"Ham (σ={st['raw_std']:.1f} dB)",
                color="#A0AEC0", alpha=0.50, linestyle="--", linewidth=1.0)
        ax.plot(st["times"], st["kal"], label=f"Kalman (σ={st['kal_std']:.1f} dB)",
                color="#3182CE", linewidth=1.5, linestyle="-.")
        ax.plot(st["times"], st["sismf"], label=f"SISMF (σ={st['sismf_std']:.1f} dB)",
                color="#E53E3E", linewidth=2.0)
        ax.plot(st["times"], st["combo"], label=f"Kalman+SISMF (σ={st['combo_std']:.1f} dB)",
                color="#38A169", linewidth=2.2, linestyle="-")

        ax.set_ylabel("Sinyal (dBm)", fontsize=9, fontweight="bold")
        ax.set_title(f"Drone {did} — Varyans Azalması: SISMF %{st['sismf_red']:.0f} | Kalman %{st['kal_red']:.0f} | K+S %{st['combo_red']:.0f}",
                     fontsize=9.5, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(fontsize=7.5, loc="upper right", ncol=2)

    axes[-1].set_xlabel("Zaman (saniye)", fontsize=10, fontweight="bold")
    fig_ts.suptitle("TÜBİTAK 123E294 — 4 İHA Sinyal Takip Zaman Serisi (Ham vs Kalman vs SISMF vs K+S)",
                    fontsize=12, fontweight="bold", y=0.995)
    fig_ts.tight_layout()

    out_ts_name = f"{csv_basename}_zaman_serisi.png"
    out_ts_path = os.path.join(SONUC_KLASORU, out_ts_name)
    plt.savefig(out_ts_path, dpi=300, bbox_inches="tight")
    plt.close()
    shutil.copy2(out_ts_path, os.path.join(csv_dir, out_ts_name))
    print(f"[BAŞARILI] 3. Zaman Serisi Grafiği -> {out_ts_name}")

    print(f"\n[TAMAMLANDI] 3 Ayrı Grafik Başarıyla Oluşturuldu!")
    print(f"Klasörler: {SONUC_KLASORU} ve {csv_dir}")


if __name__ == "__main__":
    main()
