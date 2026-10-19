"""
sim_core.py - TUBITAK 123E294, Scenario II (sphere formation) no-flight simulator
==================================================================================

Everything needed to run the sphere-based localisation of Guzey (Drones 2022),
Guzey (IEEE Access 2024, ISMF) and Guzey (IJSS 2026, SISMF) without flying:

  * ChannelModel  - calibrated log-distance RSSI model of OUR hardware:
                    P = P_ref - 10 n log10(d) + shadowing(x) + AR(1) noise,
                    1 dB quantisation, 1 sample / 2 s per drone (field logs),
                    residual per-drone calibration bias.
  * RSSI filters  - none, scalar Kalman (Report 2, Eq. 24-28), SISMF used as an
                    RSSI smoother (as in ground_station_core_kalman.py), moving average.
  * Angle laws    - PID (Report 2, Eq. 32), ISMF-2024 (IEEE Access, Eq. 8-11),
                    SISMF-2026 (IJSS, Eq. 3-6).
  * Geometry      - corrected D2/D4 positions (D2/D4 = c -/+ R(cos b, 0, sin b)),
                    optional reproduction of the typo found in the reports.
  * Localisation  - line through the sphere centre along v1 x v2 (IEEE Access
                    Eq. 16-17 / IJSS Eq. 7), least-squares intersection (IJSS Eq. 8),
                    optional ground-plane prior.
  * Improvements  - lateral baseline between sphere centres, angle averaging over
                    the converged window, calibration-bias cancelling "flip"
                    (each pair also converged with its two drones swapped).

No hardware, no MAVLink. Pure numpy. Deterministic for a given seed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np

LN10_10 = 10.0 / math.log(10.0)  # 4.3429 dB per neper

# Accepted field calibration (kalibrasyon_sonucu_saha.json, RMSE 2.29 dB)
CAL_P_REF = -78.71961498201888
CAL_N = 1.2638323630534862
CAL_OFFSETS = (0.0, -1.3205128205128256, 2.3846153846153806, 1.1025641025640998)


# ----------------------------------------------------------------------------
# Channel
# ----------------------------------------------------------------------------
@dataclass
class ChannelCfg:
    p_ref: float = CAL_P_REF          # RSSI at 1 m (dBm)
    n: float = CAL_N                  # path-loss exponent
    sigma_t: float = 2.5              # temporal (fast-fading/receiver) noise std, dB
    rho: float = 0.65                 # AR(1) correlation of consecutive samples (logs: 0.6-0.85)
    sigma_s: float = 2.0              # spatial shadowing std, dB (calibration RMSE 2.29 dB)
    corr_len: float = 10.0            # shadowing correlation length, m
    bias_std: float = 0.5             # residual per-drone calibration error after offsets, dB
    quant: float = 1.0                # RSSI quantisation step (logs are integer dB); 0 = off
    floor: float = -120.0             # receiver noise floor, dBm
    n_waves: int = 64                 # plane waves used to synthesise the shadowing field


class ChannelModel:
    """Received power of the 4 drones for one fixed transmitter."""

    def __init__(self, cfg: ChannelCfg, target: np.ndarray, rng: np.random.Generator):
        self.cfg = cfg
        self.target = np.asarray(target, float)
        self.rng = rng
        m = cfg.n_waves
        # Gaussian random field with covariance sigma_s^2 exp(-r^2 / 2 Lc^2)
        self.k = rng.normal(0.0, 1.0 / max(cfg.corr_len, 1e-6), size=(m, 3))
        self.phi = rng.uniform(0, 2 * np.pi, size=m)
        self.bias = rng.normal(0.0, cfg.bias_std, size=4)       # residual after calibration
        self.ar = rng.normal(0.0, cfg.sigma_t, size=4)          # AR(1) state, stationary start

    def shadowing(self, pos: np.ndarray) -> np.ndarray:
        if self.cfg.sigma_s <= 0:
            return np.zeros(len(pos))
        m = self.cfg.n_waves
        return self.cfg.sigma_s * math.sqrt(2.0 / m) * np.cos(pos @ self.k.T + self.phi).sum(axis=1)

    def mean_power(self, pos: np.ndarray) -> np.ndarray:
        """Noise-free model power (no shadowing, no bias) - used for ground truth checks."""
        d = np.maximum(np.linalg.norm(pos - self.target, axis=1), 0.5)
        return self.cfg.p_ref - 10.0 * self.cfg.n * np.log10(d)

    def sample(self, pos: np.ndarray) -> np.ndarray:
        """One calibrated RSSI sample per drone (offsets already applied)."""
        c = self.cfg
        rho = c.rho
        self.ar = rho * self.ar + math.sqrt(max(1.0 - rho * rho, 0.0)) * c.sigma_t * self.rng.normal(size=4)
        p = self.mean_power(pos) + self.shadowing(pos) + self.bias + self.ar
        if c.quant > 0:
            # the SDR reports integer dB on the RAW value; calibration offset is added afterwards
            raw = p - np.array(CAL_OFFSETS)
            raw = np.round(raw / c.quant) * c.quant
            p = raw + np.array(CAL_OFFSETS)
        return np.maximum(p, c.floor)


# ----------------------------------------------------------------------------
# RSSI (signal-level) filters - one instance per drone
# ----------------------------------------------------------------------------
class NoFilter:
    name = "Raw"

    def __init__(self, x0):
        self.x = float(x0)

    def update(self, z, dt):
        self.x = float(z)
        return self.x


class ScalarKalman:
    """Report 2, Eq. 24-28 with F = H = 1."""
    name = "Kalman"

    def __init__(self, x0, q=1.0, r=20.0, p0=10.0):
        self.x, self.p, self.q, self.r = float(x0), p0, q, r

    def update(self, z, dt):
        p_pred = self.p + self.q
        w = p_pred / (p_pred + self.r)
        self.x = self.x + w * (z - self.x)
        self.p = (1.0 - w) * p_pred
        return self.x


class SISMFSignal:
    """SISMF of IJSS 2026 used as an RSSI smoother, exactly as implemented in
    ground_station_core_kalman.py (SmootherIntegralSlidingModeFilter2026)."""
    name = "SISMF-signal"

    def __init__(self, x0, lam=0.02, k=0.3, kappa=0.008, phi=2.5, imax=5.0):
        self.x = float(x0)
        self.lam, self.k, self.kappa, self.phi, self.imax = lam, k, kappa, phi, imax
        self.i = 0.0

    def update(self, z, dt):
        e = z - self.x
        self.i = float(np.clip(self.i + e * dt, -self.imax, self.imax))
        s = e + self.lam * self.i
        xdot = self.k * math.tanh(s / self.phi) + self.kappa * self.i
        self.x += xdot * dt
        return self.x


class MovingAverage:
    name = "MovAvg"

    def __init__(self, x0, n=5):
        self.buf = [float(x0)]
        self.n = n

    def update(self, z, dt):
        self.buf.append(float(z))
        self.buf = self.buf[-self.n:]
        return sum(self.buf) / len(self.buf)


def make_signal_filter(kind: str, x0: float, **kw):
    kind = kind.lower()
    if kind in ("raw", "none"):
        return NoFilter(x0)
    if kind == "kalman":
        return ScalarKalman(x0, **kw)
    if kind in ("sismf", "sismf-signal"):
        return SISMFSignal(x0, **kw)
    if kind in ("movavg", "ma"):
        return MovingAverage(x0, **kw)
    raise ValueError(kind)


# ----------------------------------------------------------------------------
# Angle update laws (error e in dB -> angle rate in rad/s)
# ----------------------------------------------------------------------------
class PIDLaw:
    """Report 2, Eq. 32: a_dot = kp e + ki int(e) + kd e_dot (repo gains)."""
    name = "PID"

    def __init__(self, kp=0.02, ki=0.005, kd=0.01, imax=50.0):
        self.kp, self.ki, self.kd, self.imax = kp, ki, kd, imax
        self.reset()

    def reset(self):
        self.i, self.prev = 0.0, None

    def rate(self, e, dt):
        if self.prev is None:
            self.prev = e
        self.i = float(np.clip(self.i + e * dt, -self.imax, self.imax))
        de = (e - self.prev) / dt
        self.prev = e
        return self.kp * e + self.ki * self.i + self.kd * de


class ISMF2024Law:
    """IEEE Access 2024, Eq. 8 + 10: s = e_dot + lam e ; a_dot = ks sgn(s) + ki int(e).
    sgn replaced by tanh(s/phi) exactly as in ground_station_core_sphere.py."""
    name = "ISMF-2024"

    def __init__(self, ks=0.08, ki=0.008, lam=1.0, phi=2.0, imax=20.0):
        self.ks, self.ki, self.lam, self.phi, self.imax = ks, ki, lam, phi, imax
        self.reset()

    def reset(self):
        self.i, self.prev = 0.0, None

    def rate(self, e, dt):
        if self.prev is None:
            self.prev = e
        de = (e - self.prev) / dt
        self.prev = e
        s = de + self.lam * e
        self.i = float(np.clip(self.i + e * dt, -self.imax, self.imax))
        return self.ks * math.tanh(s / self.phi) + self.ki * self.i


class SISMF2026Law:
    """IJSS 2026, Eq. 3-6: s = e + lam int(e) ; a_dot = k sgn(s) + kappa int(e).
    tanh boundary layer as in ground_station_core_sphere.py."""
    name = "SISMF-2026"

    def __init__(self, k=0.2, kappa=0.05, lam=1.0, phi=2.0, imax=30.0):
        self.k, self.kappa, self.lam, self.phi, self.imax = k, kappa, lam, phi, imax
        self.reset()

    def reset(self):
        self.i = 0.0

    def rate(self, e, dt):
        self.i = float(np.clip(self.i + e * dt, -self.imax, self.imax))
        s = e + self.lam * self.i
        return self.k * math.tanh(s / self.phi) + self.kappa * self.i


LAW_DEFAULTS = {
    # gains currently in ground_station_core_sphere.py
    "pid": dict(kp=0.02, ki=0.005, kd=0.01, imax=50.0),
    "ismf2024": dict(ks=0.08, ki=0.008, lam=1.0, phi=2.0, imax=20.0),
    "sismf2026": dict(k=0.2, kappa=0.05, lam=1.0, phi=2.0, imax=30.0),
}


def make_law(kind: str, **kw):
    kind = kind.lower()
    params = dict(LAW_DEFAULTS[kind])
    params.update(kw)
    return {"pid": PIDLaw, "ismf2024": ISMF2024Law, "sismf2026": SISMF2026Law}[kind](**params)


# ----------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------
def axis_h(alpha):
    return np.array([math.cos(alpha), math.sin(alpha), 0.0])


def axis_v(beta, alpha=None, geometry="corrected"):
    """Axis of the vertical pair (D2-D4).
    corrected / paper_typo : fixed x-z plane, b = (cos b, 0, sin b)  (papers, Eq. 15)
    aligned                : vertical plane yawed to be perpendicular to the D1-D3 axis,
                             b = cos b (-sin a, cos a, 0) + sin b (0, 0, 1)   (proposed)
    """
    if geometry == "aligned":
        return np.array([-math.cos(beta) * math.sin(alpha), math.cos(beta) * math.cos(alpha), math.sin(beta)])
    return np.array([math.cos(beta), 0.0, math.sin(beta)])


def drone_slots(c, R, alpha, beta, geometry="corrected"):
    """Commanded positions, rows = D1..D4.  Pair (D1,D3) in x-y, pair (D2,D4) in x-z."""
    a = axis_h(alpha)
    b = axis_v(beta, alpha, geometry)
    d1 = c - R * a
    d3 = c + R * a
    if geometry in ("corrected", "aligned"):
        d2 = c - R * b
        d4 = c + R * b
    elif geometry == "paper_typo":
        # Eq. 12/18/29/34 as printed: D2 = c - R(sin b, 0, cos b), D4 = c + R(cos b, 0, sin b)
        d2 = c - R * np.array([math.sin(beta), 0.0, math.cos(beta)])
        d4 = c + R * b
    else:
        raise ValueError(geometry)
    return np.vstack([d1, d2, d3, d4])


def line_direction(alpha, beta, geometry="corrected"):
    """v3 = v1 x v2 (IEEE Access Eq. 16), unit length."""
    v3 = np.cross(axis_h(alpha), axis_v(beta, alpha, geometry))
    n = np.linalg.norm(v3)
    return v3 / n if n > 1e-9 else v3


def ls_intersection(centers, dirs, weights=None, ground_z=None):
    """Point closest to all lines (IJSS Eq. 8 style least squares).
    ground_z: if given, the point is constrained to z = ground_z (target on the ground)."""
    centers = np.asarray(centers, float)
    dirs = np.asarray(dirs, float)
    if weights is None:
        weights = np.ones(len(centers))
    M = np.zeros((3, 3))
    b = np.zeros(3)
    for c, v, w in zip(centers, dirs, weights):
        v = v / np.linalg.norm(v)
        P = np.eye(3) - np.outer(v, v)
        M += w * P
        b += w * P @ c
    if ground_z is None:
        return np.linalg.lstsq(M, b, rcond=None)[0]
    # minimise over (x, y) with z fixed
    A = M[:2, :2]
    rhs = b[:2] - M[:2, 2] * ground_z
    xy = np.linalg.lstsq(A, rhs, rcond=None)[0]
    return np.array([xy[0], xy[1], ground_z])


def angle_between_lines(v, w):
    """Unsigned angle between two undirected lines, radians."""
    v = v / np.linalg.norm(v)
    w = w / np.linalg.norm(w)
    return math.acos(min(1.0, abs(float(v @ w))))


def axial_mean(vectors):
    """Mean direction of undirected lines (principal eigenvector of sum v v^T)."""
    M = np.zeros((3, 3))
    for v in vectors:
        n = np.linalg.norm(v)
        if n > 1e-9:
            M += np.outer(v / n, v / n)
    w, V = np.linalg.eigh(M)
    return V[:, -1]


def circ_mean_mod_pi(angles):
    """Mean of axial angles (defined modulo pi)."""
    a = np.asarray(angles) * 2.0
    return 0.5 * math.atan2(np.sin(a).mean(), np.cos(a).mean())


# ----------------------------------------------------------------------------
# Localisation run
# ----------------------------------------------------------------------------
@dataclass
class SphereCfg:
    R: float = 5.0                    # circle radius (FORMATION_RADIUS_M)
    alt: float = 15.0                 # formation centre altitude
    geometry: str = "corrected"       # or "paper_typo"
    signal_filter: str = "kalman"     # raw | kalman | sismf | movavg
    filter_kw: dict = field(default_factory=dict)
    law: str = "sismf2026"            # pid | ismf2024 | sismf2026
    law_kw: dict = field(default_factory=dict)
    sample_dt: float = 2.0            # RSSI sample period per drone (logs: ~2 s)
    ctrl_every: int = 2               # control update every N samples (4 s; code uses 5 s)
    max_step_deg: float = 8.0         # MAX_ANGLE_STEP_RAD in ground_station_core_sphere.py
    v_max: float = 3.0                # drone speed limit, m/s
    gps_std: float = 0.5              # position hold error of each drone, m
    # convergence
    criterion: str = "code"           # "code": |e|<tol for n_conv cycles (as in repo)
                                      # "window": mean |e| over window < tol and dwell >= t_min
    tol_db: float = 1.0               # CONVERGENCE_THRESHOLD_DBM
    n_conv: int = 3
    window: int = 15                  # control cycles used for window criterion / angle averaging
    t_min: float = 120.0              # minimum dwell per centre for "window" criterion, s
    t_max: float = 600.0              # timeout per centre, s
    average_angles: bool = False      # use window-mean angle instead of last angle
    flip: bool = False                # bias-cancelling swap of each pair
    # sphere centres
    plan: str = "paper"               # "paper": fixed list ; "baseline": c0, c0+B p, c0-B p
    centers: tuple = ((0.0, 0.0, 15.0), (30.0, 10.0, 15.0), (0.0, 30.0, 15.0))
    baseline: float = 30.0
    ground_prior: bool = False        # constrain estimate to the ground plane z = 0
    bearing_2d: bool = False          # use only the horizontal bearing of each line (alpha);
                                      # 2-D triangulation, z set to the ground plane
    anneal_tau: float = 0.0           # >0: law output scaled by 1/(1 + dwell/tau) (stochastic-
                                      # approximation gain decay -> angle noise shrinks with dwell)
    normalize: bool = False           # divide e by its predicted full-scale value E_max(d_hat)
                                      # (d_hat from mean RSSI + calibration) -> distance-free loop gain
    alpha0: float = math.radians(45.0)
    beta0: float = math.radians(45.0)
    record: bool = False              # keep full time histories (for plots)


@dataclass
class LineResult:
    center: np.ndarray
    direction: np.ndarray
    alpha: float
    beta: float
    converged: bool
    t_used: float
    ang_err_deg: float                # angle between estimated line and true centre->target line
    chatter: float                    # total |delta alpha| + |delta beta| over the dwell, rad


@dataclass
class RunResult:
    estimate: np.ndarray
    target: np.ndarray
    err3d: float
    err2d: float
    lines: list
    total_time: float
    travel_m: float
    min_sep_m: float = float("nan")
    history: Optional[dict] = None


class SphereLocaliser:
    def __init__(self, cfg: SphereCfg, chan: ChannelModel, rng: np.random.Generator):
        self.cfg = cfg
        self.chan = chan
        self.rng = rng
        self.t = 0.0
        self.pos = None          # actual drone positions (4x3)
        self.travel = 0.0
        self.filters = None
        self.min_sep = float("inf")      # smallest commanded distance between two drones
        self.hist = {k: [] for k in ("t", "alpha", "beta", "e1", "e2", "raw", "filt", "center")} if cfg.record else None

    # -- low level -----------------------------------------------------------
    def _move_towards(self, slots, dt):
        step = slots - self.pos
        dist = np.linalg.norm(step, axis=1, keepdims=True)
        lim = self.cfg.v_max * dt
        scale = np.where(dist > lim, lim / np.maximum(dist, 1e-9), 1.0)
        mv = step * scale
        self.travel += float(np.linalg.norm(mv, axis=1).sum())
        self.pos = self.pos + mv

    def _measure(self):
        jitter = self.rng.normal(0.0, self.cfg.gps_std, size=(4, 3))
        return self.chan.sample(self.pos + jitter)

    def _fly_to(self, slots, tol=1.0, t_lim=200.0):
        """Transit between centres. No measurements are used."""
        t0 = self.t
        while np.max(np.linalg.norm(slots - self.pos, axis=1)) > tol and self.t - t0 < t_lim:
            self._move_towards(slots, self.cfg.sample_dt)
            self.t += self.cfg.sample_dt

    def predicted_emax(self, mean_rssi):
        """Full-scale pair difference |P_near - P_far| when the pair axis points at the
        source, predicted from the calibrated path-loss model. Used only for gain scaling."""
        ch = self.chan.cfg
        R = self.cfg.R
        d_hat = 10.0 ** ((CAL_P_REF - mean_rssi) / (10.0 * CAL_N))
        d_hat = max(d_hat, 1.5 * R)
        return max(10.0 * CAL_N * math.log10((d_hat + R) / (d_hat - R)), 0.05)

    # -- one sphere centre ---------------------------------------------------
    def converge_at(self, c, alpha, beta, sign=-1.0, sign_b=None):
        """Run the angle loop at centre c. sign=-1 is the stable branch for e = P3-P1
        with D3 = c + R a; after a pair flip the stable branch is sign=+1."""
        cfg = self.cfg
        R = cfg.R
        sign_b = sign if sign_b is None else sign_b
        law_a = make_law(cfg.law, **cfg.law_kw)
        law_b = make_law(cfg.law, **cfg.law_kw)
        ctrl_dt = cfg.sample_dt * cfg.ctrl_every
        max_step = math.radians(cfg.max_step_deg)

        slots = drone_slots(c, R, alpha, beta, cfg.geometry)
        self._fly_to(slots)
        z0 = self._measure()
        if self.filters is None:
            self.filters = [make_signal_filter(cfg.signal_filter, z0[i], **cfg.filter_kw) for i in range(4)]

        t_start = self.t
        acc = np.zeros(4)
        nacc = 0
        e_hist, a_hist, b_hist = [], [], []
        conv_count = 0
        chatter = 0.0
        converged = False
        k = 0
        while True:
            slots = drone_slots(c, R, alpha, beta, cfg.geometry)
            dd = np.linalg.norm(slots[:, None, :] - slots[None, :, :], axis=2)
            self.min_sep = min(self.min_sep, float(dd[np.triu_indices(4, 1)].min()))
            self._move_towards(slots, cfg.sample_dt)
            self.t += cfg.sample_dt
            z = self._measure()
            f = np.array([self.filters[i].update(z[i], cfg.sample_dt) for i in range(4)])
            acc += f
            nacc += 1
            k += 1
            if self.hist is not None:
                self.hist["t"].append(self.t)
                self.hist["raw"].append(z.copy())
                self.hist["filt"].append(f.copy())
            if k % cfg.ctrl_every:
                continue
            p = acc / nacc            # window average of filtered samples (get_windowed_rssi)
            acc[:] = 0.0
            nacc = 0
            e1 = p[2] - p[0]          # P3 - P1
            e2 = p[3] - p[1]          # P4 - P2
            if cfg.normalize:
                emax = self.predicted_emax(p.mean())
                u1, u2 = e1 / emax, e2 / emax
            else:
                u1, u2 = e1, e2
            g = 1.0 / (1.0 + (self.t - t_start) / cfg.anneal_tau) if cfg.anneal_tau > 0 else 1.0
            da = float(np.clip(sign * g * law_a.rate(u1, ctrl_dt) * ctrl_dt, -max_step, max_step))
            db = float(np.clip(sign_b * g * law_b.rate(u2, ctrl_dt) * ctrl_dt, -max_step, max_step))
            alpha += da
            beta += db
            chatter += abs(da) + abs(db)
            e_hist.append((e1, e2))
            a_hist.append(alpha)
            b_hist.append(beta)
            if self.hist is not None:
                self.hist["alpha"].append((self.t, alpha))
                self.hist["beta"].append((self.t, beta))
                self.hist["e1"].append((self.t, e1))
                self.hist["e2"].append((self.t, e2))
            dwell = self.t - t_start
            if cfg.criterion == "code":
                conv_count = conv_count + 1 if (abs(e1) < cfg.tol_db and abs(e2) < cfg.tol_db) else 0
                if conv_count >= cfg.n_conv:
                    converged = True
            else:
                if dwell >= cfg.t_min and len(e_hist) >= cfg.window:
                    w = np.array(e_hist[-cfg.window:])
                    if abs(w[:, 0].mean()) < cfg.tol_db and abs(w[:, 1].mean()) < cfg.tol_db:
                        converged = True
            if converged or dwell >= cfg.t_max:
                break

        if cfg.average_angles and len(a_hist) >= 2:
            w = min(cfg.window, len(a_hist))
            v_out = axial_mean([line_direction(a, b, cfg.geometry) for a, b in zip(a_hist[-w:], b_hist[-w:])])
        else:
            v_out = line_direction(alpha, beta, cfg.geometry)
        return alpha, beta, v_out, converged, self.t - t_start, chatter

    def localise_line(self, c, alpha, beta):
        cfg = self.cfg
        a_end, b_end, v, conv, t_used, chat = self.converge_at(c, alpha, beta, sign=-1.0)
        if cfg.flip:
            # Swap the two drones of each pair. A static per-drone bias (calibration
            # residual, unequal antennas) now pushes the equilibrium to the other side;
            # the axial mean of the two lines cancels it to first order.
            if cfg.geometry == "aligned":
                # b(a+pi, -b) = -b(a, b): swaps D2/D4; the beta loop keeps its sign
                a2, b2, v2, conv2, t2, chat2 = self.converge_at(c, a_end + math.pi, -b_end, sign=+1.0, sign_b=-1.0)
                a_end, b_end = a2 - math.pi, -b2
            else:
                a2, b2, v2, conv2, t2, chat2 = self.converge_at(c, a_end + math.pi, b_end + math.pi, sign=+1.0)
                a_end, b_end = a2 - math.pi, b2 - math.pi
            v = axial_mean([v, v2])
            conv = conv and conv2
            t_used += t2
            chat += chat2
        true_dir = self.chan.target - c
        err = math.degrees(angle_between_lines(v, true_dir)) if np.linalg.norm(v) > 0 else 90.0
        return LineResult(np.array(c, float), v, a_end, b_end, conv, t_used, err, chat), a_end, b_end

    def run(self) -> RunResult:
        cfg = self.cfg
        if cfg.plan == "paper":
            plan = [np.array(c, float) for c in cfg.centers]
        else:
            plan = [np.array(cfg.centers[0], float)]
        self.pos = drone_slots(plan[0], cfg.R, cfg.alpha0, cfg.beta0, cfg.geometry)
        alpha, beta = cfg.alpha0, cfg.beta0
        lines = []
        i = 0
        while i < len(plan):
            c = plan[i]
            lr, alpha, beta = self.localise_line(c, alpha, beta)
            lines.append(lr)
            if self.hist is not None:
                self.hist["center"].append((self.t, c.copy()))
            if cfg.plan == "baseline" and i == 0:
                vh = lr.direction.copy()
                vh[2] = 0.0
                if np.linalg.norm(vh) < 1e-6:
                    vh = np.array([1.0, 0.0, 0.0])
                vh /= np.linalg.norm(vh)
                perp = np.array([-vh[1], vh[0], 0.0])
                plan += [c + cfg.baseline * perp, c - cfg.baseline * perp]
            i += 1
        centers = [l.center for l in lines]
        dirs = [l.direction for l in lines]
        if cfg.bearing_2d:
            d2 = []
            for v in dirs:
                h = np.array([v[0], v[1], 0.0])
                d2.append(h / max(np.linalg.norm(h), 1e-9))
            c2 = [np.array([c[0], c[1], 0.0]) for c in centers]
            est = ls_intersection(c2, d2, ground_z=self.chan.target[2])
        else:
            est = ls_intersection(centers, dirs, ground_z=(self.chan.target[2] if cfg.ground_prior else None))
        tgt = self.chan.target
        return RunResult(est, tgt, float(np.linalg.norm(est - tgt)),
                         float(np.linalg.norm(est[:2] - tgt[:2])),
                         lines, self.t, self.travel, self.min_sep, self.hist)


def simulate(cfg: SphereCfg, ch: ChannelCfg, target, seed: int) -> RunResult:
    rng = np.random.default_rng(seed)
    chan = ChannelModel(ch, np.asarray(target, float), rng)
    return SphereLocaliser(cfg, chan, rng).run()


def random_target(rng, center=(0.0, 0.0), r_min=40.0, r_max=100.0):
    """Ground target (z = 0) uniformly over an annulus around the first sphere centre."""
    r = math.sqrt(rng.uniform(r_min ** 2, r_max ** 2))
    th = rng.uniform(0, 2 * math.pi)
    return np.array([center[0] + r * math.cos(th), center[1] + r * math.sin(th), 0.0])


def monte_carlo(cfg: SphereCfg, ch: ChannelCfg, n_runs=100, seed=0, r_min=40.0, r_max=100.0,
                targets=None):
    rng = np.random.default_rng(10_000 + seed)
    out = []
    for k in range(n_runs):
        tgt = targets[k] if targets is not None else random_target(rng, cfg.centers[0][:2], r_min, r_max)
        out.append(simulate(cfg, ch, tgt, seed * 100_003 + k))
    return out


def summarise(results):
    e3 = np.array([r.err3d for r in results])
    e2 = np.array([r.err2d for r in results])
    ang = np.array([l.ang_err_deg for r in results for l in r.lines])
    conv = np.array([l.converged for r in results for l in r.lines])
    chat = np.array([l.chatter / max(l.t_used, 1e-9) for r in results for l in r.lines])
    t = np.array([r.total_time for r in results])
    sep = np.array([r.min_sep_m for r in results])
    return dict(
        mean3d=float(e3.mean()), median3d=float(np.median(e3)), p90_3d=float(np.percentile(e3, 90)),
        mean2d=float(e2.mean()), median2d=float(np.median(e2)), p90_2d=float(np.percentile(e2, 90)),
        line_err_med_deg=float(np.median(ang)), line_err_mean_deg=float(ang.mean()),
        conv_rate=float(conv.mean()), chatter_rad_per_min=float(chat.mean() * 60.0),
        time_min=float(t.mean() / 60.0), n=len(results),
        min_sep_median_m=float(np.median(sep)), frac_sep_below_3m=float(np.mean(sep < 3.0)),
    )
