"""Builds ground_station_core_sphere_v2.py from the repo's ground_station_core_sphere.py.
Every change is an explicit, reviewable text replacement; the script stops if any anchor
is not found, so it cannot silently produce a half-patched file."""
import re
import sys

SRC = sys.argv[1]
DST = sys.argv[2]
s = open(SRC, encoding="utf-8").read()


def rep(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        raise SystemExit(f"anchor found {n}x (expected {count}):\n{old[:200]}")
    s = s.replace(old, new)


def replace_method(name, new_code):
    """Replace a GroundStation method (4-space indented 'def name(') up to the next method."""
    global s
    m = re.search(rf"\n    def {name}\(", s)
    if not m:
        raise SystemExit(f"method {name} not found")
    start = m.start() + 1
    nxt = re.search(r"\n    (def |# ─)", s[m.end():])
    end = m.end() + nxt.start() + 1 if nxt else len(s)
    s = s[:start] + new_code.rstrip("\n") + "\n\n" + s[end:]


# ---------------------------------------------------------------------------
# 1. Header
# ---------------------------------------------------------------------------
rep('''"""
ground_station_core_sphere.py

KÜRE FORMASYONU YER İSTASYONU''', '''"""
ground_station_core_sphere_v2.py   (v2 - simülasyonla doğrulanmış düzeltmeler)

KÜRE FORMASYONU YER İSTASYONU

v2 DEĞİŞİKLİKLERİ (her biri sphere_sim/ simülasyonunda test edildi):
  1. D2/D4 konum formülü düzeltildi. Raporlardaki yazım hatası
     (D2 = c - R(sin b, 0, cos b), D4 = c + R(cos b, 0, sin b)) iki dronu 2R
     aralıkta tutmuyordu ve v2 vektörüyle tutarsızdı. Doğrusu:
     D2/D4 = c -/+ R * b,  b = v2/R.
  2. VERTICAL_PLANE = "aligned": dikey çember, D1-D3 eksenine dik olacak şekilde
     yaw'da döndürülür: b = cos(b)(-sin a, cos a, 0) + sin(b)(0,0,1).
     Makaledeki sabit x-z düzleminde, hedef kuzey/güneydeyken D1-D2 aynı noktaya
     gelir (simülasyonda 0.0 m mesafe = ÇARPIŞMA). "aligned" ile dronlar arası
     mesafe her zaman >= R*sqrt(2). "fixed_xz" ile makale geometrisi korunabilir.
  3. Açı hatası, kalibrasyondan tahmin edilen tam ölçek farka bölünür
     (NORMALIZE_ERROR). Ham dB farkı uzaklıkla ~1/d küçülür (R=5 m, d=70 m'de
     en fazla ~0.76 dB!); normalize edilince kontrol kazancı uzaklıktan bağımsız olur.
  4. Yakınsama ölçütü: |e| < 1 dB (3 döngü) yerine, en az MIN_DWELL_S beklendikten
     sonra son CONV_WINDOW_CYCLES döngünün ORTALAMA hatası < eşik. Eski 1 dB eşiği,
     uzak hedefte e'nin tam ölçeğinden büyük olduğu için "anında yakınsıyordu".
  5. Doğru, açıların son pencere ortalamasından (eksenel ortalama) hesaplanır.
  6. FLIP_PAIRS: her merkezde çiftler yer değiştirip ikinci kez yakınsanır; dron
     başına sabit kalibrasyon/anten farkı (offset hatası) iki doğrunun ortalamasında
     birinci dereceden sıfırlanır.
  7. Doğru, açıların yakınsadığı merkezde kaydedilir (v1 kaydırmadan SONRAKİ
     merkezi kaydediyordu). Merkez doğru boyunca 3 m kaydırılmaz (ardışık doğrular
     neredeyse paralel olup kesişim kötü koşullanıyordu); bunun yerine yanal taban
     çizgisi (CENTER_PLAN = "baseline") veya sabit liste kullanılır. RSSI eğilimine
     dayalı işaret tahmini kaldırıldı (doğru yönsüzdür, gerek yoktur).
  8. Sonuç: tüm doğruların en küçük kareler kesişimi (IJSS 2026 Denk. 8),
     isteğe bağlı yer düzlemi kısıtı (GROUND_PRIOR).
  9. SIMULATION_MODE artık rastgele RSSI değil, fiziksel bir sanal dünya kullanır:
     sanal dronlar komut edilen noktalara uçar, RSSI kalibre edilmiş yol kaybı
     modelinden + gürültüden üretilir (SIM_* ayarları). Kod uçmadan uçtan uca
     denenebilir; SIM_TIME_SCALE ile hızlandırılır.''')

# ---------------------------------------------------------------------------
# 2. Configuration
# ---------------------------------------------------------------------------
rep('''FORMATION_RADIUS_M = 5.0       # R: küre yarıçapı (dronlar arası 2R = 10m)
FORMATION_ALT_M    = 15.0      # Dronların uçuş irtifası (merkez z_c)''',
    '''FORMATION_RADIUS_M = 5.0       # R: küre yarıçapı (dronlar arası 2R = 10m)
                               # SİMÜLASYON: doğruluk ~R/d ile ölçeklenir. R=5 m ile 40-100 m
                               # uzaktaki hedefte saha gürültüsünde doğru hatası ~50-70°;
                               # güvenli ise R = 15-20 m önerilir (FORMATION_ALT_M >= R + 5).
FORMATION_ALT_M    = 15.0      # Dronların uçuş irtifası (merkez z_c)
MIN_SAFE_ALT_M     = 5.0       # Dikey çemberin en alt noktası bunun altına inemez''')

rep('''COMMAND_LOOP_HZ = 0.2          # 5 saniyede bir döngü (dt = 5s)''',
    '''COMMAND_LOOP_HZ = 0.2          # 5 saniyede bir döngü (dt = 5s)

# ==========================================
# v2: GEOMETRİ, YAKINSAMA, MERKEZ PLANI
# ==========================================
VERTICAL_PLANE = "aligned"      # "aligned" (önerilen, çarpışmasız) | "fixed_xz" (makale)
NORMALIZE_ERROR = True          # e / E_max(d_tahmini) -> uzaklıktan bağımsız kazanç
MIN_DWELL_S = 300.0             # her merkezde en az bekleme (gürültü ortalaması için)
MAX_DWELL_S = 600.0             # her merkezde en fazla bekleme (sonra pencere ortalamasıyla devam)
CONV_WINDOW_CYCLES = 30         # yakınsama/ortalama penceresi (döngü sayısı, 30 x 5 s = 150 s)
FLIP_PAIRS = True               # çift değiştirerek kalibrasyon/anten farkını iptal et
CENTER_PLAN = "baseline"        # "baseline": c0, c0 + B*dik, c0 - B*dik | "fixed": FIXED_CENTERS
BASELINE_M = 20.0               # yanal taban çizgisi (geofence MAX_CENTER_DRIFT_M içinde kalmalı)
FIXED_CENTERS = [(0.0, 0.0), (20.0, 0.0), (0.0, 20.0)]   # (Doğu, Kuzey) m, CENTER_PLAN="fixed"
GROUND_PRIOR = True             # hedef yerde (z = 0) varsayımıyla kesişim
MIN_DRONE_DIST_M = 3.0          # komut edilen iki nokta arasında en az mesafe''')

rep('''SISMF_2026_LAMBDA = 1.0   # λ: Kayan yüzey integral kazancı (SCI_4 Bölüm 4.1)
SISMF_2026_K      = 0.2   # k: Sign/tanh kazancı (rad/s)
SISMF_2026_KAPPA  = 0.05  # κ: Düzeltici integral kazancı
SISMF_2026_PHI    = 2.0   # φ: Sınır katmanı kalınlığı''',
    '''# v2: NORMALIZE_ERROR = True için simülasyonda ızgara aramasıyla seçilen kazançlar
# (sphere_sim/tune.py). Makaledeki λ=1, κ=0.05 (açı birimli hata için) dB hatasıyla
# limit çevrime (±100° salınım) giriyordu. NORMALIZE_ERROR = False ise eski değerleri
# kullanın: λ=1.0, k=0.2, κ=0.05, φ=2.0.
SISMF_2026_LAMBDA = 0.01  # λ: Kayan yüzey integral kazancı
SISMF_2026_K      = 0.2   # k: Sign/tanh kazancı (rad/s)
SISMF_2026_KAPPA  = 0.0016  # κ: Düzeltici integral kazancı
SISMF_2026_PHI    = 1.0   # φ: Sınır katmanı kalınlığı (normalize hata biriminde)''')

rep('''ISMF_KS     = 0.08    # rad/s (sign kazancı)
ISMF_KI     = 0.008   # rad/s per dBm·s (integral kazancı)
ISMF_LAMBDA = 1.0     # kayan yüzey eğimi''',
    '''ISMF_KS     = 0.2     # rad/s (sign kazancı)          [v2 ayarı; eski 0.08]
ISMF_KI     = 0.005   # integral kazancı               [v2 ayarı; eski 0.008]
ISMF_LAMBDA = 1.0     # kayan yüzey eğimi
ISMF_PHI    = 1.0     # sınır katmanı                  [v2 ayarı; eski 2.0 sabit]''')

rep('''PID_KP = 0.02     # rad/s per dBm fark
PID_KI = 0.005    # rad/s per dBm·s (integral)
PID_KD = 0.01     # rad/s per dBm/s (türev)''',
    '''PID_KP = 0.1      # [v2 ayarı; eski 0.02]
PID_KI = 0.0016   # [v2 ayarı; eski 0.005]
PID_KD = 0.0      # [v2: türev gürültüyü büyütüyordu; eski 0.01]''')

rep('''# Güç farkı bu eşiğin altına düştüğünde "yakınsadı" sayılır (dBm)
CONVERGENCE_THRESHOLD_DBM = 1.0''',
    '''# v2: son CONV_WINDOW_CYCLES döngünün ORTALAMA |e| değeri bu eşiğin altında ise
# (ve MIN_DWELL_S geçtiyse) "yakınsadı" sayılır (dB).
CONVERGENCE_THRESHOLD_DBM = 0.15''')

rep('''BENCH_TEST_MODE = True
''', '''BENCH_TEST_MODE = False          # v2: SIMULATION_MODE fiziksel simülasyon kullanır;
                                 # pervanesiz masa testi için True yapın.

# ==========================================
# v2: FİZİKSEL SİMÜLASYON (SIMULATION_MODE = True iken)
# ==========================================
SIM_TARGET_ENU = (55.0, 45.0, 0.0)  # sanal telsiz: Doğu, Kuzey, Yukarı (m, origin'e göre)
SIM_TIME_SCALE = 20.0           # 20 -> 1 saatlik görev ~3 dakikada
SIM_P_REF = -78.72              # saha kalibrasyonu
SIM_N = 1.264
SIM_NOISE_STD_DB = 1.2          # zamansal gürültü (en sakin saha logu ~1.2 dB)
SIM_NOISE_RHO = 0.3             # ardışık örnek korelasyonu
SIM_BIAS_STD_DB = 0.5           # kalibrasyon sonrası kalan dron farkı
SIM_DRONE_SPEED_MS = 3.0
SIM_SAMPLE_PERIOD_S = 2.0       # dron başına RSSI örnek periyodu (LoRa TDMA)
''')

rep('''try:
    import serial
except ImportError:
    print("pyserial kütüphanesi eksik. 'pip install pyserial'")
    sys.exit(1)''', '''try:
    import serial
except ImportError:
    serial = None   # v2: simülasyon modunda gerekmez; gerçek donanımda connect_hardware uyarır''')
rep('''        print(f"[SİSTEM] LoRa bağlanıyor: {self.lora_port}")''', '''        if serial is None:
            print("[HATA] pyserial kütüphanesi eksik. 'pip install pyserial'")
            sys.exit(1)
        print(f"[SİSTEM] LoRa bağlanıyor: {self.lora_port}")''')

# ---------------------------------------------------------------------------
# 3. ISMF phi parameter
# ---------------------------------------------------------------------------
rep('''        phi = 2.0  # Sınır katmanı kalınlığı
        soft_sign = math.tanh(s / phi)''', '''        soft_sign = math.tanh(s / ISMF_PHI)''')

# ---------------------------------------------------------------------------
# 4. Physical simulation world + mocks
# ---------------------------------------------------------------------------
rep('''class MockSerial:
    def __init__(self):
        self._next_id = 1

    def read(self, size):
        drone_id = self._next_id
        self._next_id = (self._next_id % 4) + 1
        fake_rssi = random.randint(-90, -60)
        body = f"N{drone_id},{fake_rssi}"
        checksum = compute_checksum(body)
        reply = f"{body},{checksum}\\n"
        time.sleep(0.25)
        return reply.encode("ascii")''',
    '''class SimWorld:
    """Sanal dünya: komut edilen noktalara uçan 4 dron + kalibre yol kaybı modeli.
    Konumlar origin'e göre (Doğu, Kuzey, Yukarı) metre. Zaman SIM_TIME_SCALE ile hızlı."""
    def __init__(self):
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.last_t = 0.0
        self.pos = {i: [0.0, 0.0, FORMATION_ALT_M] for i in (1, 2, 3, 4)}
        self.cmd = {i: list(p) for i, p in self.pos.items()}
        self.bias = {i: random.gauss(0.0, SIM_BIAS_STD_DB) for i in (1, 2, 3, 4)}
        self.ar = {i: random.gauss(0.0, SIM_NOISE_STD_DB) for i in (1, 2, 3, 4)}

    def sim_time(self):
        return (time.time() - self.t0) * SIM_TIME_SCALE

    def _advance(self):
        t = self.sim_time()
        dt = t - self.last_t
        self.last_t = t
        lim = SIM_DRONE_SPEED_MS * dt
        for i in (1, 2, 3, 4):
            p, c = self.pos[i], self.cmd[i]
            d = math.sqrt(sum((c[k] - p[k]) ** 2 for k in range(3)))
            f = 1.0 if d <= lim else lim / d
            self.pos[i] = [p[k] + f * (c[k] - p[k]) for k in range(3)]

    def command(self, drone_id, east, north, up):
        with self.lock:
            self._advance()
            self.cmd[drone_id] = [east, north, up]

    def position(self, drone_id):
        with self.lock:
            self._advance()
            return list(self.pos[drone_id])

    def raw_rssi(self, drone_id):
        """Dron üzerindeki SDR'nin göreceği HAM tam sayı RSSI (ofset uygulanmamış)."""
        with self.lock:
            self._advance()
            p = self.pos[drone_id]
            d = max(math.sqrt(sum((p[k] - SIM_TARGET_ENU[k]) ** 2 for k in range(3))), 0.5)
            rho = SIM_NOISE_RHO
            self.ar[drone_id] = rho * self.ar[drone_id] + math.sqrt(1 - rho * rho) * random.gauss(0, SIM_NOISE_STD_DB)
            true_cal = SIM_P_REF - 10.0 * SIM_N * math.log10(d) + self.bias[drone_id] + self.ar[drone_id]
            return int(round(true_cal - CALIBRATION_OFFSET_DB.get(drone_id, 0.0)))


SIM_WORLD = None


class MockSerial:
    """LoRa yerine: her dron için SIM_SAMPLE_PERIOD_S'de bir 'N{id},{rssi},{chk}' paketi."""
    def __init__(self):
        self._next_id = 1
        self._next_t = 0.0

    def read(self, size):
        # Paketler SANAL zamana göre üretilir (TDMA: 4 dron sırayla, dron başına
        # SIM_SAMPLE_PERIOD_S'de bir) - yazdırma/CSV yükü örnek hızını bozmaz.
        while SIM_WORLD.sim_time() < self._next_t:
            time.sleep(0.002)
        self._next_t += SIM_SAMPLE_PERIOD_S / 4.0
        drone_id = self._next_id
        self._next_id = (self._next_id % 4) + 1
        rssi = SIM_WORLD.raw_rssi(drone_id)
        body = f"N{drone_id},{rssi}"
        checksum = compute_checksum(body)
        reply = f"{body},{checksum}\\n"
        return reply.encode("ascii")''')

# ---------------------------------------------------------------------------
# 5. GroundStation: __init__ additions
# ---------------------------------------------------------------------------
rep('''        # Durum makinesi
        self.state = self.STATE_SETUP  # Formasyon Kurulum Fazı ile başla
        self.convergence_count = 0  # Kaç ardışık döngüde yakınsadı''',
    '''        # Durum makinesi
        self.state = self.STATE_SETUP  # Formasyon Kurulum Fazı ile başla
        self.convergence_count = 0  # Kaç ardışık döngüde yakınsadı

        # v2: yakınsama penceresi, çift değiştirme (flip) ve merkez planı
        self.loop_sign_a = -1.0          # e1 = P3 - P1, D3 = c + R a için kararlı dal
        self.loop_sign_b = -1.0
        self.flip_phase = 0              # 0: normal, 1: çiftler yer değiştirmiş
        self.flip_first_dir = None
        self.dwell_start_t = None
        self.e_hist = []
        self.ab_hist = []
        self.plan = [(self.center_x, self.center_y)]
        if CENTER_PLAN == "fixed":
            self.plan = [tuple(c) for c in FIXED_CENTERS]
            self.center_x, self.center_y = self.plan[0]
        self.plan_index = 0
        self.STATE_DONE = "DONE"
        if FORMATION_ALT_M - FORMATION_RADIUS_M < MIN_SAFE_ALT_M and not BENCH_TEST_MODE:
            print(f"[KRİTİK] FORMATION_ALT_M - R = {FORMATION_ALT_M - FORMATION_RADIUS_M:.1f} m < "
                  f"{MIN_SAFE_ALT_M} m. Dikey çember yere çok yaklaşır. Ayarları düzeltin.")
            sys.exit(1)''')

rep('''        self.connect_hardware()
        self.start_lora_listener()
        self.capture_origin()''',
    '''        if self.simulation:
            global SIM_WORLD
            SIM_WORLD = SimWorld()
            print(f"[SİM] Fiziksel simülasyon: hedef ENU={SIM_TARGET_ENU}, "
                  f"gürültü={SIM_NOISE_STD_DB} dB, hız x{SIM_TIME_SCALE}")
        self.connect_hardware()
        self.start_lora_listener()
        self.capture_origin()''')

rep('''        print(f"  Yakınsama eşiği: ±{CONVERGENCE_THRESHOLD_DBM} dBm")''',
    '''        print(f"  Yakınsama: pencere ort. |e| < {CONVERGENCE_THRESHOLD_DBM} dB, "
              f"{MIN_DWELL_S:.0f}-{MAX_DWELL_S:.0f} s/merkez, flip={FLIP_PAIRS}")
        print(f"  Dikey düzlem: {VERTICAL_PLANE} | Normalize: {NORMALIZE_ERROR} | "
              f"Merkez planı: {CENTER_PLAN} (B={BASELINE_M} m)")''')

# ---------------------------------------------------------------------------
# 6. Methods
# ---------------------------------------------------------------------------
replace_method("compute_drone_positions", '''    def _axes(self, alpha=None, beta=None):
        """Birim eksenler: a (D1-D3, yatay) ve b (D2-D4, dikey düzlem)."""
        a_ang = self.alpha if alpha is None else alpha
        b_ang = self.beta if beta is None else beta
        a = (math.cos(a_ang), math.sin(a_ang), 0.0)
        if VERTICAL_PLANE == "aligned":
            b = (-math.cos(b_ang) * math.sin(a_ang), math.cos(b_ang) * math.cos(a_ang), math.sin(b_ang))
        else:  # "fixed_xz" (makale, Denk. 15)
            b = (math.cos(b_ang), 0.0, math.sin(b_ang))
        return a, b

    def compute_drone_positions(self):
        """
        v2 - DÜZELTİLMİŞ geometri (x = Doğu, y = Kuzey, z = irtifa):
          D1 = c - R a,  D3 = c + R a,   a = (cos α, sin α, 0)
          D2 = c - R b,  D4 = c + R b,   b = v2 / R
        Raporlardaki D2 = c - R(sin β, 0, cos β) yazım hatası 2R aralığını bozuyordu.
        Dönüş: {id: (kuzey_m, doğu_m, irtifa_m)}
        """
        R = FORMATION_RADIUS_M
        cx, cy, cz = self.center_x, self.center_y, self.center_z
        a, b = self._axes()
        pts = {
            1: (cx - R * a[0], cy - R * a[1], cz - R * a[2]),
            3: (cx + R * a[0], cy + R * a[1], cz + R * a[2]),
            2: (cx - R * b[0], cy - R * b[1], cz - R * b[2]),
            4: (cx + R * b[0], cy + R * b[1], cz + R * b[2]),
        }
        out = {}
        for i, (e, n, z) in pts.items():
            if BENCH_TEST_MODE:
                z = cz
            out[i] = (n, e, max(z, MIN_SAFE_ALT_M))
        return out''')

replace_method("compute_direction_vector", '''    def compute_direction_vector(self, alpha=None, beta=None):
        """v3 = v1 x v2 (IEEE Access 2024 Denk. 16): merkezden hedeften geçen doğrunun
        birim yönü. Doğru yönsüzdür (±v3 aynı doğru)."""
        a, b = self._axes(alpha, beta)
        v3 = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
        mag = math.sqrt(sum(c * c for c in v3))
        if mag < 1e-9:
            return 0.0, 0.0, 0.0
        return tuple(c / mag for c in v3)

    def _axial_mean(self, vectors):
        """Yönsüz doğruların ortalaması (sum v v^T'nin en büyük özvektörü)."""
        import numpy as np
        M = np.zeros((3, 3))
        for v in vectors:
            v = np.asarray(v, float)
            n = np.linalg.norm(v)
            if n > 1e-9:
                M += np.outer(v / n, v / n)
        w, V = np.linalg.eigh(M)
        return tuple(float(x) for x in V[:, -1])

    def _predicted_emax(self, mean_rssi):
        """Çift ekseni hedefe bakarken beklenen en büyük |P3-P1| (dB). Sadece kazanç ölçekleme."""
        p_ref = globals().get("SIM_P_REF", -78.72)
        n = globals().get("SIM_N", 1.264)
        R = FORMATION_RADIUS_M
        d_hat = max(10.0 ** ((p_ref - mean_rssi) / (10.0 * n)), 1.5 * R)
        return max(10.0 * n * math.log10((d_hat + R) / (d_hat - R)), 0.05)''')

replace_method("update_rotation_angles", '''    def update_rotation_angles(self, rssi_snap, dt):
        """
        Açı güncelleme (Rapor 2 Denk. 30-33 / IEEE Access Denk. 6-11 / IJSS Denk. 3-6).
          e1 = P3 - P1  -> α,   e2 = P4 - P2 -> β
        v2: e, NORMALIZE_ERROR ise E_max(d) ile bölünür; işaret kararlı daldan seçilir;
        yakınsama pencere ortalamasıyla değerlendirilir.
        """
        have_g1 = 1 in rssi_snap and 3 in rssi_snap
        have_g2 = 2 in rssi_snap and 4 in rssi_snap
        if not (have_g1 and have_g2):
            missing = [d for d in (1, 2, 3, 4) if d not in rssi_snap]
            print(f"[UYARI] Eksik RSSI: {missing}. Açılar sabit, döngü sayılmıyor.")
            return None, None

        e1 = rssi_snap[3] - rssi_snap[1]
        e2 = rssi_snap[4] - rssi_snap[2]
        if NORMALIZE_ERROR:
            emax = self._predicted_emax(sum(rssi_snap.values()) / len(rssi_snap))
            u1, u2 = e1 / emax, e2 / emax
        else:
            emax, u1, u2 = 1.0, e1, e2

        MAX_ANGLE_STEP_RAD = math.radians(8)
        da = self.loop_sign_a * self.angle_controller_alpha.compute(u1, dt) * dt
        db = self.loop_sign_b * self.angle_controller_beta.compute(u2, dt) * dt
        da = max(min(da, MAX_ANGLE_STEP_RAD), -MAX_ANGLE_STEP_RAD)
        db = max(min(db, MAX_ANGLE_STEP_RAD), -MAX_ANGLE_STEP_RAD)
        self.alpha += da
        if not BENCH_TEST_MODE:
            self.beta += db

        self.e_hist.append((e1, e2))
        self.ab_hist.append((self.alpha, self.beta))
        print(f"[KÜRE] α={math.degrees(self.alpha):.1f}° β={math.degrees(self.beta):.1f}° | "
              f"e1={e1:+.2f} e2={e2:+.2f} dB (E_max≈{emax:.2f} dB) | flip={self.flip_phase}")
        return e1, e2

    def is_converged(self, sim_now):
        """MIN_DWELL_S geçti ve pencere ortalaması eşik altında -> True. MAX_DWELL_S -> True."""
        dwell = sim_now - self.dwell_start_t
        if dwell >= MAX_DWELL_S:
            print(f"[KÜRE] MAX_DWELL_S ({MAX_DWELL_S:.0f} s) doldu; pencere ortalamasıyla devam.")
            return True
        w = CONV_WINDOW_CYCLES
        if dwell < MIN_DWELL_S or len(self.e_hist) < w:
            return False
        m1 = sum(e[0] for e in self.e_hist[-w:]) / w
        m2 = sum(e[1] for e in self.e_hist[-w:]) / w
        return abs(m1) < CONVERGENCE_THRESHOLD_DBM and (abs(m2) < CONVERGENCE_THRESHOLD_DBM or BENCH_TEST_MODE)

    def window_direction(self):
        w = min(CONV_WINDOW_CYCLES, len(self.ab_hist))
        if w == 0:
            return self.compute_direction_vector()
        return self._axial_mean([self.compute_direction_vector(a, b) for a, b in self.ab_hist[-w:]])''')

# send_mavlink_commands: safety distance from config, simulation hook
rep('''        # ── Güvenlik: Minimum mesafe / çarpışma kontrolü ──
        MIN_DRONE_DIST_M = 3.0  # Dronlar arası olması gereken minimum mesafe (m)
        points''', '''        # ── Güvenlik: Minimum mesafe / çarpışma kontrolü ──
        points''')
rep('''            ready, reason = link.is_ready_for_commands()
            if not ready and not self.simulation:''', '''            if self.simulation and SIM_WORLD is not None:
                SIM_WORLD.command(drone_id, east_m, north_m, alt_m)
                continue

            ready, reason = link.is_ready_for_commands()
            if not ready and not self.simulation:''')

replace_method("shift_center_toward_target", '''    def finish_center(self, sim_now):
        """Bir merkezdeki yakınsama bitti: flip fazına geç veya doğruyu kaydedip
        plandaki sonraki merkeze git."""
        v = self.window_direction()
        if FLIP_PAIRS and self.flip_phase == 0:
            self.flip_first_dir = v
            self.flip_phase = 1
            # Çiftleri yer değiştir: a -> -a; b -> -b
            self.alpha += math.pi
            if VERTICAL_PLANE == "aligned":
                self.beta = -self.beta          # b(α+π, -β) = -b(α, β)
                self.loop_sign_a, self.loop_sign_b = +1.0, -1.0
            else:
                self.beta += math.pi
                self.loop_sign_a, self.loop_sign_b = +1.0, +1.0
            print("[FLIP] Çiftler yer değiştiriyor; aynı merkezde ikinci yakınsama.")
            self._reset_dwell(sim_now)
            return
        if FLIP_PAIRS:
            v = self._axial_mean([self.flip_first_dir, v])
            # yön değişmeden normal yerleşime dön
            self.alpha -= math.pi
            if VERTICAL_PLANE == "aligned":
                self.beta = -self.beta
            else:
                self.beta -= math.pi
            self.flip_phase = 0
            self.loop_sign_a = self.loop_sign_b = -1.0

        # Doğru, AÇILARIN YAKINSADIĞI merkezde kaydedilir (v1 hatası düzeltildi)
        self.direction_lines.append({
            'center': (self.center_x, self.center_y, self.center_z),
            'direction': v,
            'alpha_deg': math.degrees(self.alpha),
            'beta_deg': math.degrees(self.beta),
        })
        print("*" * 55)
        print(f"[DOĞRU #{len(self.direction_lines)}] C=({self.center_x:.1f}, {self.center_y:.1f}, "
              f"{self.center_z:.1f}) yön=({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f})")

        if CENTER_PLAN == "baseline" and len(self.plan) == 1:
            h = math.hypot(v[0], v[1])
            px, py = ((-v[1] / h, v[0] / h) if h > 1e-6 else (1.0, 0.0))
            c0 = self.plan[0]
            self.plan += [(c0[0] + BASELINE_M * px, c0[1] + BASELINE_M * py),
                          (c0[0] - BASELINE_M * px, c0[1] - BASELINE_M * py)]
        self.plan_index += 1
        est = self.compute_target_least_squares()
        if est is not None:
            print(f"[HEDEF ARA KESTİRİM] Doğu={est[0]:.1f} m, Kuzey={est[1]:.1f} m, z={est[2]:.1f} m")
        if self.plan_index >= len(self.plan):
            print("[GÖREV] Tüm merkezler tamamlandı. Dronlar son konumda bekliyor (DONE).")
            self.state = self.STATE_DONE
        else:
            nx, ny = self.plan[self.plan_index]
            nx = max(min(nx, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
            ny = max(min(ny, MAX_CENTER_DRIFT_M), -MAX_CENTER_DRIFT_M)
            print(f"[PLAN] Sonraki merkez: Doğu={nx:.1f} m, Kuzey={ny:.1f} m")
            self.center_x, self.center_y = nx, ny
            self.state = self.STATE_SETUP
            self._setup_start = time.time()
        print("*" * 55)
        self._reset_dwell(sim_now)

    def _reset_dwell(self, sim_now):
        self.angle_controller_alpha.reset()
        self.angle_controller_beta.reset()
        self.e_hist, self.ab_hist = [], []
        self.dwell_start_t = sim_now''')

rep('''        positions = self.compute_drone_positions()
        TOLERANCE_M = 3.5''', '''        positions = self.compute_drone_positions()
        if self.simulation and SIM_WORLD is not None:
            for drone_id, (north_m, east_m, alt_m) in positions.items():
                p = SIM_WORLD.position(drone_id)
                if math.sqrt((p[0] - east_m) ** 2 + (p[1] - north_m) ** 2 + (p[2] - alt_m) ** 2) > 1.0:
                    return False
            return True
        TOLERANCE_M = 3.5''')

replace_method("run", '''    def _now(self):
        """Görev zamanı (s). Simülasyonda hızlandırılmış sanal zaman."""
        if self.simulation and SIM_WORLD is not None:
            return SIM_WORLD.sim_time()
        return time.time()

    def run(self):
        print("\\n[BİLGİ] Küre Formasyonu v2 döngüsü başlıyor. Ctrl+C ile durdurun.")
        dt = 1.0 / COMMAND_LOOP_HZ  # 5 saniye (görev zamanı)
        scale = SIM_TIME_SCALE if self.simulation else 1.0
        self._setup_start = time.time()
        SETUP_TIMEOUT_S = 120

        while self.state != self.STATE_DONE:
            loop_start = time.monotonic()
            rssi = self.get_windowed_rssi()
            print("\\n" + "=" * 55)
            print(f"[DURUM] {self.state} | Merkez: E:{self.center_x:.1f}m N:{self.center_y:.1f}m "
                  f"ALT:{self.center_z:.1f}m | t={self._now() - self._mission_t0 if hasattr(self, '_mission_t0') else 0:.0f}s")

            if self.state == self.STATE_SETUP:
                self.send_mavlink_commands()
                if self.check_formation_reached():
                    print("[SETUP] Formasyon tamam -> ORBIT")
                    self.state = self.STATE_ORBIT
                    self._reset_dwell(self._now())
                    if not hasattr(self, "_mission_t0"):
                        self._mission_t0 = self._now()
                elif (time.time() - self._setup_start) * scale > SETUP_TIMEOUT_S:
                    print(f"[UYARI] SETUP {SETUP_TIMEOUT_S}s aşıldı; ORBIT'e geçiliyor.")
                    self.state = self.STATE_ORBIT
                    self._reset_dwell(self._now())
                    if not hasattr(self, "_mission_t0"):
                        self._mission_t0 = self._now()

            elif self.state == self.STATE_ORBIT:
                e1, e2 = self.update_rotation_angles(rssi, dt)
                self.send_mavlink_commands()
                if e1 is not None and self.is_converged(self._now()):
                    self.finish_center(self._now())

            print("=" * 55 + "\\n")
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, dt / scale - elapsed))
        self.report()''')

replace_method("compute_target_least_squares", '''    def compute_target_least_squares(self):
        """Doğruların en küçük kareler kesişimi (IJSS 2026 Denk. 8).
        GROUND_PRIOR: hedef yerde (z = 0, kalkış noktası seviyesi) varsayılır."""
        if len(self.direction_lines) < 2:
            return None
        import numpy as np
        M = np.zeros((3, 3))
        b = np.zeros(3)
        for dl in self.direction_lines:
            c = np.array(dl['center'], float)
            v = np.array(dl['direction'], float)
            v = v / np.linalg.norm(v)
            P = np.eye(3) - np.outer(v, v)
            M += P
            b += P @ c
        try:
            if GROUND_PRIOR:
                xy = np.linalg.lstsq(M[:2, :2], b[:2], rcond=None)[0]
                return np.array([xy[0], xy[1], 0.0])
            return np.linalg.lstsq(M, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None

    def report(self):
        print("\\n" + "=" * 55)
        print("[RAPOR] Hesaplanan hedef doğruları:")
        for i, dl in enumerate(self.direction_lines, 1):
            c, d = dl['center'], dl['direction']
            print(f"  #{i}: C=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) Yön=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})")
        target = self.compute_target_least_squares()
        if target is not None:
            print(f"[HEDEF] Doğu(X)={target[0]:.2f} m  Kuzey(Y)={target[1]:.2f} m  Z={target[2]:.2f} m")
            if self.simulation:
                err = math.hypot(target[0] - SIM_TARGET_ENU[0], target[1] - SIM_TARGET_ENU[1])
                print(f"[SİM] Gerçek hedef ENU={SIM_TARGET_ENU} -> yatay hata = {err:.2f} m")
        print("=" * 55)''')

# shutdown: old report block replaced by report()
rep('''        # Son rapor
        if self.direction_lines:''', '''        # Son rapor
        if self.direction_lines and self.state != getattr(self, "STATE_DONE", "DONE"):
            self.report()
        if False:''')

open(DST, "w", encoding="utf-8").write(s)
print("written", DST)
