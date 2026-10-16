"""
algorithm2_sphere_localization.py

Proposal dosyasındaki (Yeni Bir Geometrik RSSI Yaklaşımıyla...) İKİNCİ
SENARYONUN ("ii) Hedefin Üzerine Yönelmeden Konumlandırma" / Güzey vd.
2022'nin küre-tabanlı yöntemi) BAĞIMSIZ, donanımdan ARINDIRILMIŞ bir
simülasyonudur.

Bu dosya ground_station_scenario2_core.py'nin YERİNE GEÇMEZ - o dosya
gerçek LoRa/MAVLink donanımına bağlanan, uçuş komutu gönderen sürümdür.
Bu dosya ise SADECE ALGORİTMANIN MATEMATİĞİNİ göstermek/anlamak/test
etmek için var: hiçbir donanıma bağlanmaz, sanal bir hedef pozisyonu
tanımlar ve algoritmanın o hedefi ne kadar doğru bulduğunu raporlar.

==========================================================================
ALGORİTMA 2 NE YAPIYOR? (özet)
==========================================================================
4 İHA, ortak bir merkez etrafında iki dik çember üzerinde döner:
    - D1 & D3: x-y düzleminde bir çember (açı: alpha)
    - D2 & D4: x-z düzleminde bir çember (açı: beta, yani bu ikisi
      irtifayı değiştirir)

Her çemberdeki iki İHA'nın aldığı sinyal gücü karşılaştırılır
(D3-D1 farkı, D4-D2 farkı) ve bu fark SIFIR olana kadar (yani ikisi de
hedefe eşit uzaklıkta olana kadar) İHA'lar PID kontrolcüyle çember
üzerinde döndürülür. Bu adım BÖLÜM 1 (rotate_until_converged).

Yakınsama sağlandığında, 4 İHA'nın konumundan, küreye VE 4 İHA'nın
hepsine eşit uzaklıkta olan bir "augmented point" (A) hesaplanır -
bu ADIM 2 (solve_augmented_point). Küre merkezi C'den A'dan geçen
doğru, hedefin yönünü verir (ama TEK bir doğru hedefi tam olarak
belirlemez - iki doğru/daha fazla lazım).

Bu yüzden İHA grubu birkaç farklı küre merkezine (baseline) taşınır,
her birinde aynı işlem tekrarlanır, ve elde edilen ÇOKLU doğrular
kesiştirilerek hedefin 3D konumu tahmin edilir - BÖLÜM 3
(estimate_target_from_spheres). solve_augmented_point iki olası nokta
döndürür (küre üzerinde iki simetrik çözüm) - hangisinin doğru olduğu
BELLİ DEĞİLDİR (makalenin kapalı-form denklem 9'u bunu doğrudan
seçmiyor). Bu belirsizliği çözmek için: tüm doğru kombinasyonları
denenir, ve TÜM doğruların birbirine en yakın kesiştiği (en az
"pairwise gap" veren) kombinasyon seçilir - gerçek hedef konumu
bilinmeden de çalışan, kendi kendine tutarlı bir strateji.

ÖNEMLİ: Makalenin kendi kapalı-form "augmented point" formülü (Denklem
9), belgeden çıkarılan haliyle güvenilir değildi (muhtemelen OCR/format
kaybı). Bu yüzden augmented point burada formülden DEĞİL, kendi
TANIMINDAN türetildi: "küreye VE 4 İHA'nın hepsine eşit uzaklıkta olan
nokta" -> bu bir lineer cebir problemi (4 küre denkleminin ortak
çözümü). Bu yaklaşım sentetik/bilinen hedeflerle test edildi ve
tutarlı çalıştığı doğrulandı.
==========================================================================
"""

import math
import itertools
import numpy as np


# ==========================================
# AYARLAR (simülasyon parametreleri)
# ==========================================
CIRCLE_RADIUS_M = 5.0          # r: her çemberin yarıçapı
PID_KP = 0.02
PID_KI = 0.002
PID_KD = 0.01
CONVERGENCE_TOLERANCE_DB = 0.5  # güç farkı bu değerin altına inince "yakınsadı" sayılır
MAX_PID_STEPS = 3000
DT_S = 1.0                      # simülasyon zaman adımı (saniye)

# RSSI simülasyon modeli (gerçek donanımda bunun yerine ölçülen RSSI kullanılır)
RSSI_REF_1M = -40.0
PATH_LOSS_EXPONENT = 2.5


# ==========================================
# BÖLÜM 0: YARDIMCI GEOMETRİ
# ==========================================
def circle_positions(center, r, alpha, beta):
    """
    Verilen merkez, yarıçap ve iki dönme açısına göre 4 İHA'nın 3D
    konumunu hesaplar.
      - D1, D3: x-y düzleminde (alpha)
      - D2, D4: x-z düzleminde (beta) -> irtifa değişir
    """
    xc, yc, zc = center
    return {
        1: (xc - r * math.cos(alpha), yc - r * math.sin(alpha), zc),
        3: (xc + r * math.cos(alpha), yc + r * math.sin(alpha), zc),
        2: (xc - r * math.sin(beta), yc, zc - r * math.cos(beta)),
        4: (xc + r * math.sin(beta), yc, zc + r * math.cos(beta)),
    }


def path_loss_rssi(pos, target, p_ref=RSSI_REF_1M, n=PATH_LOSS_EXPONENT, noise_std_db=0.0, rng=None):
    """
    SİMÜLASYON İÇİN: bir İHA konumunda, hedeften gelen sinyalin RSSI'sını
    (dBm) hesaplar. Gerçek donanımda bu fonksiyonun yerini LoRa'dan
    okunan GERÇEK RSSI alır - bu sadece test/demo amaçlıdır.
    noise_std_db > 0 verilirse, gürültülü/aykırı okumaları simüle etmek
    için Gauss gürültüsü eklenir (bkz. main() sonundaki gürültü testi).
    """
    dx = pos[0] - target[0]
    dy = pos[1] - target[1]
    dz = pos[2] - target[2]
    d = max(math.sqrt(dx * dx + dy * dy + dz * dz), 0.1)
    rssi = p_ref - 10 * n * math.log10(d)
    if noise_std_db > 0:
        rng = rng or np.random.default_rng()
        rssi += rng.normal(0, noise_std_db)
    return rssi


# ==========================================
# BÖLÜM 1: BİR KÜRE POZİSYONUNDA DÖNEREK YAKINSAMA (PID)
# ==========================================
def rotate_until_converged(center, target, r=CIRCLE_RADIUS_M, get_rssi=None,
                            tol=CONVERGENCE_TOLERANCE_DB, max_steps=MAX_PID_STEPS, dt=DT_S):
    """
    Belirtilen küre merkezinde, D1/D3 ve D2/D4 çiftlerinin güç farkı
    SIFIRA yaklaşana kadar (PID kontrolcüyle) alpha/beta açılarını
    döndürür. Yakınsayınca son İHA konumlarını döndürür.

    get_rssi(position, target) -> dBm: gerçek donanımda bu fonksiyon
    LoRa'dan okunan canlı RSSI'yı döndürecek şekilde değiştirilebilir.
    Simülasyonda varsayılan olarak path_loss_rssi kullanılır.
    """
    if get_rssi is None:
        get_rssi = path_loss_rssi

    alpha = beta = 0.0
    e1_int = e2_int = 0.0
    e1_prev = e2_prev = None

    for step in range(max_steps):
        pos = circle_positions(center, r, alpha, beta)
        p1 = get_rssi(pos[1], target)
        p2 = get_rssi(pos[2], target)
        p3 = get_rssi(pos[3], target)
        p4 = get_rssi(pos[4], target)

        e1 = p3 - p1   # D1-D3 güç farkı (x-y çemberi)
        e2 = p4 - p2   # D2-D4 güç farkı (x-z çemberi)

        e1_dot = 0.0 if e1_prev is None else (e1 - e1_prev) / dt
        e2_dot = 0.0 if e2_prev is None else (e2 - e2_prev) / dt
        e1_prev, e2_prev = e1, e2

        e1_int += e1 * dt
        e2_int += e2 * dt

        alpha_dot = PID_KP * e1 + PID_KI * e1_int + PID_KD * e1_dot
        beta_dot = PID_KP * e2 + PID_KI * e2_int + PID_KD * e2_dot

        alpha += alpha_dot * dt
        beta += beta_dot * dt

        if abs(e1) <= tol and abs(e2) <= tol and step > 5:
            return {"positions": pos, "alpha": alpha, "beta": beta, "steps": step, "converged": True}

    return {"positions": pos, "alpha": alpha, "beta": beta, "steps": max_steps, "converged": False}


# ==========================================
# BÖLÜM 2: AUGMENTED POINT (küreye + 4 İHA'ya eşit uzaklıkta nokta)
# ==========================================
def solve_augmented_point(positions, center, r):
    """
    positions: {1: (x,y,z), 2: ..., 3: ..., 4: ...} - 4 İHA'nın yakınsanmış konumu.
    center, r: küre merkezi ve yarıçapı.

    4 İHA'nın hepsine eşit uzaklıkta olan noktalar bir DOĞRU oluşturur
    (3 denklem, 3 bilinmeyen ama denklemler doğrusal olunca 1 serbestlik
    derecesi kalır). Bu doğrunun küreyle (|A-C|=r) kesişimi, 0, 1 veya
    2 nokta verir - geometrik olarak burada 2 çözüm beklenir (küre
    üzerinde simetrik iki nokta). Hangisinin "doğru" olduğu bu adımda
    BELİRLİ DEĞİL - bkz. BÖLÜM 3.
    """
    p1, p2, p3, p4 = positions[1], positions[2], positions[3], positions[4]

    # "pk'ye eşit uzaklık = p1'e eşit uzaklık" denklemlerinden lineer sistem kur
    A_mat, b_vec = [], []
    for pk in (p2, p3, p4):
        row = [2 * (pk[i] - p1[i]) for i in range(3)]
        rhs = sum(pk[i] ** 2 - p1[i] ** 2 for i in range(3))
        A_mat.append(row)
        b_vec.append(rhs)
    A_mat = np.array(A_mat, dtype=float)
    b_vec = np.array(b_vec, dtype=float)

    # En küçük normlu özel çözüm + null-space yönü (SVD ile)
    A0, *_ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
    U, S, Vt = np.linalg.svd(A_mat)
    null_vectors = list(Vt[len(S):]) + [Vt[i] for i in range(len(S)) if S[i] < 1e-6]
    if not null_vectors:
        return (tuple(A0),)  # dejenere durum: tek çözüm, doğru yok

    v = null_vectors[0]
    v = v / np.linalg.norm(v)

    # Doğru (A0 + t*v) ile küre (|A-C|=r) kesişimi -> ikinci dereceden denklem
    C = np.array(center, dtype=float)
    diff = A0 - C
    a_coef = np.dot(v, v)
    b_coef = 2 * np.dot(v, diff)
    c_coef = np.dot(diff, diff) - r * r
    disc = b_coef ** 2 - 4 * a_coef * c_coef
    if disc < 0:
        return (tuple(A0),)  # geometri tutarsızsa (olmamalı) yedek çözüm

    sqrt_disc = math.sqrt(disc)
    t1 = (-b_coef + sqrt_disc) / (2 * a_coef)
    t2 = (-b_coef - sqrt_disc) / (2 * a_coef)
    return (tuple(A0 + t1 * v), tuple(A0 + t2 * v))


# ==========================================
# BÖLÜM 3: BİRDEN ÇOK KÜRE POZİSYONUNDAN GELEN DOĞRULARI BİRLEŞTİRME
# ==========================================
def _closest_points_between_lines(p1, d1, p2, d2):
    """İki 3D doğru arasındaki en yakın nokta çiftini bulur (çakışmıyorlarsa)."""
    p1, p2 = np.array(p1, dtype=float), np.array(p2, dtype=float)
    d1, d2 = np.array(d1, dtype=float), np.array(d2, dtype=float)
    r_ = p1 - p2
    a, e = d1 @ d1, d2 @ d2
    f, c, b = d2 @ r_, d1 @ r_, d1 @ d2
    denom = a * e - b * b
    if abs(denom) < 1e-9:
        t, s = 0.0, (f / e if e > 1e-9 else 0.0)
    else:
        t = (b * f - c * e) / denom
        s = (a * f - b * c) / denom
    return p1 + t * d1, p2 + s * d2


def _line_gap(p1, d1, p2, d2):
    a, b = _closest_points_between_lines(p1, d1, p2, d2)
    return np.linalg.norm(a - b), (a + b) / 2


def estimate_target_from_spheres(sphere_results):
    """
    sphere_results: [(center, (candA, candB)), ...] - her küre pozisyonu
    için merkez ve solve_augmented_point'ten gelen iki aday nokta.

    Her küre için candA/candB'den birini seçen TÜM kombinasyonları
    dener (2^N kombinasyon), ve doğruların birbirine EN YAKIN kesiştiği
    (toplam pairwise-gap'i en küçük olan) kombinasyonu seçer. Bu, gerçek
    hedef konumu bilinmeden çalışan, kendiliğinden tutarlı bir stratejidir
    - GÜRÜLTÜLÜ/AYKIRI bir küre ölçümü varsa bile, geri kalan doğrular
    onu "ezer" (bkz. main() sonundaki gürültü testi).

    Dönüş: (tahmini_pozisyon, toplam_gap, seçilen_kombinasyon)
    """
    if len(sphere_results) < 2:
        return None, None, None

    best_gap, best_combo, best_est = None, None, None
    for choice in itertools.product([0, 1], repeat=len(sphere_results)):
        lines = []
        for (center, cands), ch in zip(sphere_results, choice):
            A = cands[ch] if ch < len(cands) else cands[0]
            direction = tuple(A[i] - center[i] for i in range(3))
            lines.append((center, direction))

        total_gap = 0.0
        midpoints = []
        for (p1, d1), (p2, d2) in itertools.combinations(lines, 2):
            gap, mid = _line_gap(p1, d1, p2, d2)
            total_gap += gap
            midpoints.append(mid)

        if best_gap is None or total_gap < best_gap:
            best_gap = total_gap
            best_combo = choice
            best_est = np.mean(midpoints, axis=0)

    return tuple(best_est), best_gap, best_combo


# ==========================================
# ANA SİMÜLASYON (main)
# ==========================================
def run_simulation(target, sphere_centers, noisy_sphere_index=None, noise_std_db=6.0,
                    tol=CONVERGENCE_TOLERANCE_DB, verbose=True):
    """
    target: gerçek (bilinmeyen varsayılan) hedef konumu (x, y, z) - sadece
            simülasyonda "doğruluk kontrolü" için kullanılır, algoritma
            bunu görmez.
    sphere_centers: [(x,y,z), ...] - İHA grubunun sırayla ziyaret edeceği
            küre merkezleri (baseline pozisyonları).
    noisy_sphere_index: verilirse, o indeksteki küre pozisyonunda RSSI
            ölçümlerine gürültü eklenir - "bir ölçüm yanlış olursa ne
            olur?" sorusunu test etmek için.
    """
    sphere_results = []

    for i, center in enumerate(sphere_centers):
        noise = noise_std_db if i == noisy_sphere_index else 0.0
        rng = np.random.default_rng(seed=None)

        def get_rssi(pos, tgt, _noise=noise, _rng=rng):
            return path_loss_rssi(pos, tgt, noise_std_db=_noise, rng=_rng)

        result = rotate_until_converged(center, target, get_rssi=get_rssi, tol=tol)
        cands = solve_augmented_point(result["positions"], center, CIRCLE_RADIUS_M)
        sphere_results.append((center, cands))

        if verbose:
            tag = "  <-- GÜRÜLTÜLÜ (test için)" if i == noisy_sphere_index else ""
            print(f"[Küre {i+1}] merkez={tuple(round(v,1) for v in center)}  "
                  f"yakınsadı={result['converged']} ({result['steps']} adım)  "
                  f"alpha={math.degrees(result['alpha']):.1f}°  beta={math.degrees(result['beta']):.1f}°{tag}")

    estimate, total_gap, combo = estimate_target_from_spheres(sphere_results)

    if verbose:
        print()
        print(f"[SONUÇ] Seçilen doğru kombinasyonu: {combo}  (toplam kesişim boşluğu: {total_gap:.2f} m)")
        print(f"[SONUÇ] Tahmini hedef konumu : {tuple(round(v,1) for v in estimate)}")
        print(f"[SONUÇ] Gerçek hedef konumu  : {tuple(round(v,1) for v in target)}")
        err = np.linalg.norm(np.array(estimate) - np.array(target))
        print(f"[SONUÇ] Lokalizasyon hatası  : {err:.2f} m")

    return estimate, total_gap, combo


if __name__ == "__main__":
    TRUE_TARGET = (60.0, 40.0, 35.0)
    BASE_ALT = 15.0
    SPHERE_CENTERS = [
        (0.0, 0.0, BASE_ALT),
        (30.0, 10.0, BASE_ALT),
        (0.0, 30.0, BASE_ALT),
    ]

    print("=" * 70)
    print("TEST 1: Gürültüsüz (temiz) ölçümlerle konumlandırma")
    print("=" * 70)
    run_simulation(TRUE_TARGET, SPHERE_CENTERS)

    print()
    print("=" * 70)
    print("TEST 2: 2. küre pozisyonunda GÜRÜLTÜLÜ/YANLIŞ ölçüm simülasyonu")
    print("        (bir konuşmada bahsedilen 'bir ölçüm yanlış olursa ne")
    print("         olur' sorusunun somut testi - diğer doğrular onu")
    print("         'ezip' tahmini yine de makul tutabiliyor mu?)")
    print("=" * 70)
    run_simulation(TRUE_TARGET, SPHERE_CENTERS, noisy_sphere_index=1, noise_std_db=6.0)

    print()
    print("=" * 70)
    print("TEST 3: YAKINSAMA TOLERANSININ DOĞRULUĞA ETKİSİ (önemli bulgu)")
    print("        Güç farkını 'sıfır' saymak için ne kadar sıkı bir eşik")
    print("        kullandığınız, augmented-point geometrisinde ÇOK")
    print("        büyütülerek final hataya yansıyor. Gerçek donanımda")
    print("        çok sıkı bir tolerans (örn. 0.02dB) pratik değildir")
    print("        (gürültü seviyesinin altında), bu yüzden bu, üzerinde")
    print("        gerçek sahada AYRICA çalışılması gereken bir")
    print("        mühendislik parametresidir - makale bunu belirtmiyor.")
    print("=" * 70)
    for tol in (1.0, 0.5, 0.1, 0.02):
        est, gap, combo = run_simulation(TRUE_TARGET, SPHERE_CENTERS, tol=tol, verbose=False)
        err = np.linalg.norm(np.array(est) - np.array(TRUE_TARGET))
        print(f"  Tolerans={tol:5.2f} dB  ->  Lokalizasyon hatası = {err:6.2f} m")

