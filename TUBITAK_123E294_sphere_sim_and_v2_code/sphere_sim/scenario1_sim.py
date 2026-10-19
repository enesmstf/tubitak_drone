"""
scenario1_sim.py - Scenario I ("+" formation, direct tracking) no-flight simulator
==================================================================================
Reproduces the decision engine of ground_station_core_kalman.py:

  D1 = C + (0, +L), D2 = C + (0, -L), D3 = C + (+L, 0), D4 = C + (-L, 0)   (L = 5 m, alt 15 m)
  every 5 s:  P_i = mean of the filtered samples received in the window
              dN = P1 - P2,  dE = P3 - P4
              step = P:     clip(K * d, +-5 m)                (K = 0.5 m/dB)
                     SISMF: AxisSISMFController.update_shift   (repo gains)
              virtual centre += step  (integrator), geofence +-50 m

and runs it against the calibrated channel model of sim_core.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from sim_core import ChannelCfg, ChannelModel, make_signal_filter


class AxisSISMF:
    """AxisSISMFController of ground_station_core_kalman.py (identical maths)."""

    def __init__(self, lam=0.02, k=0.5, kappa=0.02, phi=2.5, imax=5.0, step_max=5.0):
        self.lam, self.k, self.kappa, self.phi, self.imax, self.step_max = lam, k, kappa, phi, imax, step_max
        self.i = 0.0

    def step(self, e, dt):
        self.i = float(np.clip(self.i + e * dt, -self.imax, self.imax))
        s = e + self.lam * self.i
        v = self.k * math.tanh(s / self.phi) + self.kappa * self.i
        return float(np.clip(v * dt, -self.step_max, self.step_max))


class AxisP:
    def __init__(self, k=0.5, step_max=5.0, deadband=0.0):
        self.k, self.step_max, self.deadband = k, step_max, deadband

    def step(self, e, dt):
        if abs(e) < self.deadband:
            return 0.0
        return float(np.clip(self.k * e, -self.step_max, self.step_max))


@dataclass
class S1Cfg:
    L: float = 5.0                    # FORMATION_DISTANCE_M
    alt: float = 15.0
    signal_filter: str = "kalman"     # raw | kalman | sismf | kalman+sismf | movavg
    controller: str = "p"             # p | sismf
    ctrl_kw: dict = field(default_factory=dict)
    loop_dt: float = 5.0
    sample_dt: float = 2.0
    v_max: float = 3.0
    gps_std: float = 0.5
    t_end: float = 600.0
    geofence: float = 50.0


class ChainFilter:
    def __init__(self, x0):
        self.a = make_signal_filter("kalman", x0)
        self.b = make_signal_filter("sismf", x0)

    def update(self, z, dt):
        return self.b.update(self.a.update(z, dt), dt)


def run_s1(cfg: S1Cfg, ch: ChannelCfg, target, seed, record=False):
    rng = np.random.default_rng(seed)
    chan = ChannelModel(ch, np.asarray(target, float), rng)
    C = np.array([0.0, 0.0])
    offs = np.array([[0, cfg.L], [0, -cfg.L], [cfg.L, 0], [-cfg.L, 0]])
    pos = np.column_stack([C + offs, np.full(4, cfg.alt)])
    z0 = chan.sample(pos)
    if cfg.signal_filter == "kalman+sismf":
        filt = [ChainFilter(z0[i]) for i in range(4)]
    else:
        filt = [make_signal_filter(cfg.signal_filter, z0[i], **({"n": 5} if cfg.signal_filter == "movavg" else {}))
                for i in range(4)]
    ctrl = [AxisSISMF(**cfg.ctrl_kw) if cfg.controller == "sismf" else AxisP(**cfg.ctrl_kw) for _ in range(2)]
    t = 0.0
    next_ctrl = cfg.loop_dt
    acc = np.zeros(4)
    nacc = 0
    hist_t, hist_c = [], []
    path = 0.0
    while t < cfg.t_end:
        slots = np.column_stack([C + offs, np.full(4, cfg.alt)])
        step = slots - pos
        d = np.linalg.norm(step, axis=1, keepdims=True)
        lim = cfg.v_max * cfg.sample_dt
        pos = pos + step * np.where(d > lim, lim / np.maximum(d, 1e-9), 1.0)
        t += cfg.sample_dt
        z = chan.sample(pos + rng.normal(0, cfg.gps_std, size=(4, 3)))
        f = np.array([filt[i].update(z[i], cfg.sample_dt) for i in range(4)])
        acc += f
        nacc += 1
        if t + 1e-9 >= next_ctrl:
            p = acc / nacc
            acc[:] = 0
            nacc = 0
            dN = p[0] - p[1]
            dE = p[2] - p[3]
            sN = ctrl[0].step(dN, cfg.loop_dt)
            sE = ctrl[1].step(dE, cfg.loop_dt)
            newC = np.clip(C + np.array([sE, sN]), -cfg.geofence, cfg.geofence)
            path += float(np.linalg.norm(newC - C))
            C = newC
            next_ctrl += cfg.loop_dt
            hist_t.append(t)
            hist_c.append(C.copy())
    hist_c = np.array(hist_c)
    hist_t = np.array(hist_t)
    err = np.linalg.norm(hist_c - np.asarray(target)[:2], axis=1)
    last = hist_t >= cfg.t_end - 180.0
    within = np.where(err < 5.0)[0]
    t_reach = float(hist_t[within[0]]) if len(within) else float("nan")
    out = dict(final_err=float(err[last].mean()), jitter=float(np.linalg.norm(hist_c[last] - hist_c[last].mean(0), axis=1).mean()),
               t_reach=t_reach, path=path, straight=float(np.linalg.norm(np.asarray(target)[:2])))
    if record:
        out["t"] = hist_t
        out["c"] = hist_c
        out["err"] = err
    return out


def mc_s1(cfg, ch, n=40, seed=0, r=(20.0, 40.0)):
    rng = np.random.default_rng(777 + seed)
    res = []
    for k in range(n):
        rr = math.sqrt(rng.uniform(r[0] ** 2, r[1] ** 2))
        th = rng.uniform(0, 2 * math.pi)
        tgt = np.array([rr * math.cos(th), rr * math.sin(th), 0.0])
        res.append(run_s1(cfg, ch, tgt, seed * 1000 + k))
    fe = np.array([x["final_err"] for x in res])
    tr = np.array([x["t_reach"] for x in res])
    return dict(final_err_median=float(np.median(fe)), final_err_mean=float(fe.mean()),
                final_err_p90=float(np.percentile(fe, 90)),
                reached_frac=float(np.mean(~np.isnan(tr))),
                t_reach_median_s=float(np.nanmedian(tr)) if np.any(~np.isnan(tr)) else float("nan"),
                jitter_m=float(np.mean([x["jitter"] for x in res])),
                path_over_straight=float(np.mean([x["path"] / x["straight"] for x in res])))
