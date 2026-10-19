"""
run_experiments.py - all no-flight experiments for the TUBITAK 123E294 report
============================================================================
Usage:
    pip install numpy matplotlib scipy
    python run_experiments.py            # full run (~10-20 min on 2 cores)
    python run_experiments.py --quick    # fewer Monte Carlo runs, for a smoke test
    python run_experiments.py --logs ../tubitak_drone   # also replay real field logs (E5b)

Outputs go to ./results/: PNG figures (300 dpi), CSV tables and summary.md.

E1  Paper reproduction       paper-like noise, R = 5 m, fixed centres, laws as in repo vs tuned
E2  Filters x laws           field noise, full localisation, R = 5 m and R = 20 m
E3  Feasibility map          line error vs radius R and target distance; vs noise level
E4  Ablation                 repo version -> each fix/improvement, R = 5 m and R = 20 m
E5  RSSI filter benchmark    synthetic truth + field-like noise: RMSE, noise and lag
E5b Real logs                same filters on the recorded field logs (variance, jitter, lag)
E6  Analytic sensitivity     full-scale pair difference E_max(d) vs tolerance and noise
E7  Example runs             time histories and 3-D line plots
E8  Scenario I               "+" formation: RSSI filter x controller (ground_station_core_kalman.py)
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import time
from dataclasses import replace
from multiprocessing import Pool

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from sim_core import (  # noqa: E402
    CAL_N, LN10_10, ChannelCfg, SphereCfg, monte_carlo, simulate, summarise, random_target,
    make_signal_filter, line_direction,
)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
CH = {
    # IEEE Access 2024: Gaussian noise of "4 % variance in signal power" ~ 0.17 dB
    "paper-like": ChannelCfg(sigma_t=0.17, rho=0.0, sigma_s=0.0, bias_std=0.0, quant=0.0),
    # quietest field log (1791809751): ~1.2 dB around a 30 s median, rho ~0.3
    "field-quiet": ChannelCfg(sigma_t=1.2, rho=0.3, sigma_s=1.0, corr_len=20.0, bias_std=0.5),
    # filters only act on temporal noise: same as field-typical but WITHOUT spatial shadowing
    "field-temporal": ChannelCfg(sigma_t=2.5, rho=0.5, sigma_s=0.0, bias_std=0.5),
    # typical field logs (1792067603 / 1792069619): 2.5-4 dB, rho ~0.5
    "field-typical": ChannelCfg(sigma_t=2.5, rho=0.5, sigma_s=1.5, corr_len=20.0, bias_std=0.5),
}

# Gains selected by tune.py (grid search; normalised error, window criterion)
TUNED = {
    "pid": dict(kp=0.1, ki=0.0016, kd=0.0),
    "ismf2024": dict(ks=0.2, ki=0.005, lam=1.0, phi=1.0),
    "sismf2026": dict(k=0.2, kappa=0.0016, lam=0.01, phi=1.0),
}
LAW_LABEL = {"pid": "PID (Rapor 2)", "ismf2024": "ISMF-2024", "sismf2026": "SISMF-2026"}
FILT_LABEL = {"raw": "Raw", "kalman": "Kalman", "sismf": "SISMF-signal", "movavg": "MovAvg-5"}


def centres(alt):
    return ((0.0, 0.0, alt), (30.0, 10.0, alt), (0.0, 30.0, alt))


def repo_cfg(R=5.0, alt=15.0):
    """ground_station_core_sphere.py as found: typo geometry, repo gains, |e|<1 dB x3."""
    return SphereCfg(R=R, alt=alt, geometry="paper_typo", law="sismf2026", signal_filter="kalman",
                     criterion="code", tol_db=1.0, n_conv=3, t_max=600.0, plan="paper",
                     centers=centres(alt))


def best_cfg(R=20.0, alt=25.0, law="sismf2026", filt="kalman", **kw):
    cfg = SphereCfg(R=R, alt=alt, geometry="aligned", law=law, law_kw=TUNED[law], signal_filter=filt,
                    normalize=True, criterion="window", average_angles=True, t_min=300.0, t_max=600.0,
                    window=30, tol_db=0.15, flip=True, plan="baseline", baseline=40.0,
                    centers=centres(alt), ground_prior=True)
    return replace(cfg, **kw)


# ---------------------------------------------------------------------------
# Plot style (reference palette of the dataviz skill, light mode, print)
# ---------------------------------------------------------------------------
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def style():
    plt.rcParams.update({
        "figure.dpi": 110, "savefig.dpi": 300, "font.size": 9.5,
        "axes.edgecolor": INK2, "axes.labelcolor": INK, "axes.titlesize": 10.5,
        "axes.titleweight": "bold", "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "xtick.color": INK2, "ytick.color": INK2, "legend.frameon": False,
        "lines.linewidth": 2.0, "figure.facecolor": "white", "axes.facecolor": "white",
    })


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def write_csv(name, rows):
    if not rows:
        return
    path = os.path.join(OUT, name)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


# ---------------------------------------------------------------------------
# Parallel Monte Carlo
# ---------------------------------------------------------------------------
def _mc(args):
    tag, cfg, ch, n, seed, rmin, rmax = args
    res = monte_carlo(cfg, ch, n_runs=n, seed=seed, r_min=rmin, r_max=rmax)
    s = summarise(res)
    s["tag"] = tag
    s["errs2d"] = [r.err2d for r in res]
    return s


def run_many(jobs, pool):
    return pool.map(_mc, jobs)


# ---------------------------------------------------------------------------
# E1  paper reproduction
# ---------------------------------------------------------------------------
def e1(pool, N):
    ch = CH["paper-like"]
    jobs = []
    for law in ("pid", "ismf2024", "sismf2026"):
        base = SphereCfg(R=5.0, alt=15.0, law=law, signal_filter="kalman", centers=centres(15.0))
        jobs.append((("typo", law, "repo gains, |e|<1 dB"), replace(base, geometry="paper_typo"), ch, N, 1, 40, 100))
        jobs.append((("fixed", law, "repo gains, |e|<1 dB"), base, ch, N, 1, 40, 100))
        jobs.append((("fixed", law, "repo gains, |e|<0.02 dB"), replace(base, tol_db=0.02, t_max=1200.0), ch, N, 1, 40, 100))
        jobs.append((("fixed", law, "tuned, window"), replace(base, law_kw=TUNED[law], normalize=True,
                     criterion="window", average_angles=True, t_min=300.0, t_max=600.0, window=30, tol_db=0.15),
                     ch, N, 1, 40, 100))
    res = run_many(jobs, pool)
    rows = []
    for s in res:
        geom, law, mode = s["tag"]
        rows.append(dict(geometry=geom, law=LAW_LABEL[law], setting=mode, mean_err_3d_m=s["mean3d"],
                         median_err_3d_m=s["median3d"], mean_err_2d_m=s["mean2d"],
                         line_err_median_deg=s["line_err_med_deg"], converged_frac=s["conv_rate"],
                         angle_activity_rad_per_min=s["chatter_rad_per_min"], time_min=s["time_min"],
                         min_drone_sep_median_m=s["min_sep_median_m"], runs_with_sep_below_3m=s["frac_sep_below_3m"]))
    write_csv("E1_paper_reproduction.csv", rows)
    return rows


# ---------------------------------------------------------------------------
# E2  filters x laws under field noise
# ---------------------------------------------------------------------------
def e2(pool, N):
    jobs = []
    for chname in ("field-temporal", "field-quiet"):
        for R, alt in ((5.0, 15.0), (20.0, 25.0)):
            for filt in ("raw", "kalman", "sismf", "movavg"):
                for law in ("pid", "ismf2024", "sismf2026"):
                    fkw = dict(n=5) if filt == "movavg" else {}
                    cfg = best_cfg(R=R, alt=alt, law=law, filt=filt, filter_kw=fkw)
                    jobs.append(((chname, R, filt, law), cfg, CH[chname], N, 2, 40, 100))
    res = run_many(jobs, pool)
    rows = []
    for s in res:
        chname, R, filt, law = s["tag"]
        rows.append(dict(channel=chname, R_m=R, rssi_filter=FILT_LABEL[filt], angle_law=LAW_LABEL[law],
                         mean_err_2d_m=s["mean2d"], median_err_2d_m=s["median2d"], p90_err_2d_m=s["p90_2d"],
                         line_err_median_deg=s["line_err_med_deg"], time_min=s["time_min"]))
    write_csv("E2_filters_x_laws.csv", rows)

    style()
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.4), sharey=False)
    filts = ["raw", "kalman", "sismf", "movavg"]
    laws = ["pid", "ismf2024", "sismf2026"]
    for ax, (chname, R) in zip(axes.ravel(), [(c, r) for c in ("field-temporal", "field-quiet") for r in (5.0, 20.0)]):
        x = np.arange(len(filts))
        w = 0.26
        for j, law in enumerate(laws):
            vals = [next(r["median_err_2d_m"] for r in rows if r["channel"] == chname and r["R_m"] == R
                         and r["rssi_filter"] == FILT_LABEL[f] and r["angle_law"] == LAW_LABEL[law]) for f in filts]
            ax.bar(x + (j - 1) * w, vals, w * 0.92, color=SERIES[j], label=LAW_LABEL[law])
        ax.set_xticks(x, [FILT_LABEL[f] for f in filts])
        ax.set_title(f"{chname}, R = {R:.0f} m")
        ax.set_ylabel("median horizontal error (m)")
    axes[0, 0].legend(loc="upper right", fontsize=8)
    fig.suptitle("E2 - RSSI filter x angle law (full localisation, 3 centres + flip + ground prior)", y=1.0)
    fig.tight_layout()
    save(fig, "E2_filters_x_laws.png")
    return rows


# ---------------------------------------------------------------------------
# E3  feasibility map
# ---------------------------------------------------------------------------
def _line_job(args):
    tag, cfg, ch, n, seed, rmin, rmax = args
    cfg = replace(cfg, plan="paper", centers=(cfg.centers[0],), ground_prior=False)
    res = monte_carlo(cfg, ch, n_runs=n, seed=seed, r_min=rmin, r_max=rmax)
    hb = []
    for r in res:
        l = r.lines[0]
        u = r.target - l.center
        th = math.atan2(u[1], u[0])
        bh = math.atan2(l.direction[1], l.direction[0])
        hb.append(math.degrees(abs((bh - th + math.pi / 2) % math.pi - math.pi / 2)))
    return dict(tag=tag, bearing_med=float(np.median(hb)),
                line_med=float(np.median([r.lines[0].ang_err_deg for r in res])))


def e3(pool, N):
    radii = [5.0, 10.0, 15.0, 20.0, 30.0]
    dists = [(20, 30), (40, 50), (60, 70), (90, 100)]
    jobs = []
    for R in radii:
        alt = max(15.0, R + 5.0)
        for d in dists:
            jobs.append(((R, d), best_cfg(R=R, alt=alt, centers=((0.0, 0.0, alt),)), CH["field-quiet"], N, 3, d[0], d[1]))
    res = pool.map(_line_job, jobs)
    grid = np.zeros((len(radii), len(dists)))
    rows = []
    for r in res:
        R, d = r["tag"]
        grid[radii.index(R), dists.index(d)] = r["bearing_med"]
        rows.append(dict(R_m=R, dist_m=f"{d[0]}-{d[1]}", bearing_err_median_deg=r["bearing_med"],
                         line_err_median_deg=r["line_med"]))
    write_csv("E3_feasibility_R_vs_distance.csv", rows)

    # noise sweep at R = 20 m, d = 60-70 m
    sig_t = [0.17, 0.5, 1.0, 2.0, 3.0, 4.0]
    sig_s = [0.0, 0.5, 1.0, 2.0]
    jobs = []
    for st in sig_t:
        for ss in sig_s:
            ch = ChannelCfg(sigma_t=st, rho=0.3, sigma_s=ss, corr_len=20.0, bias_std=0.5)
            jobs.append(((st, ss), best_cfg(R=20.0, alt=25.0, centers=((0.0, 0.0, 25.0),)), ch, N, 4, 60, 70))
    res2 = pool.map(_line_job, jobs)
    rows2 = [dict(sigma_t_db=r["tag"][0], sigma_s_db=r["tag"][1], bearing_err_median_deg=r["bearing_med"],
                  line_err_median_deg=r["line_med"]) for r in res2]
    write_csv("E3_noise_sweep_R20_d60-70.csv", rows2)

    style()
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    im = a1.imshow(grid, cmap="Blues", origin="lower", aspect="auto", vmin=0, vmax=60)
    a1.set_xticks(range(len(dists)), [f"{a}-{b} m" for a, b in dists])
    a1.set_yticks(range(len(radii)), [f"{int(R)} m" for R in radii])
    a1.set_xlabel("horizontal distance centre -> target")
    a1.set_ylabel("circle radius R")
    a1.grid(False)
    for i in range(len(radii)):
        for j in range(len(dists)):
            v = grid[i, j]
            a1.text(j, i, f"{v:.0f}°", ha="center", va="center", fontsize=9,
                    color="white" if v > 35 else INK)
    fig.colorbar(im, ax=a1, label="median bearing error (deg)")
    a1.set_title("E3a - single-line bearing error, field-quiet noise")
    for k, ss in enumerate(sig_s):
        y = [next(r["bearing_err_median_deg"] for r in rows2 if r["sigma_t_db"] == st and r["sigma_s_db"] == ss)
             for st in sig_t]
        a2.plot(sig_t, y, marker="o", ms=5, color=SERIES[k], label=f"shadowing σs = {ss} dB")
    a2.set_xlabel("temporal noise σt (dB)")
    a2.set_ylabel("median bearing error (deg)")
    a2.set_title("E3b - noise sweep, R = 20 m, d = 60-70 m")
    a2.legend(fontsize=8)
    fig.tight_layout()
    save(fig, "E3_feasibility.png")
    return rows, rows2


# ---------------------------------------------------------------------------
# E4  ablation
# ---------------------------------------------------------------------------
def e4(pool, N):
    steps = []
    for R, alt in ((5.0, 15.0), (20.0, 25.0)):
        a = repo_cfg(R, alt)
        b = replace(a, geometry="corrected")
        b2 = replace(b, geometry="aligned")
        c = replace(b2, law_kw=TUNED["sismf2026"], normalize=True, criterion="window", average_angles=True,
                    t_min=300.0, t_max=600.0, window=30, tol_db=0.15)
        d = replace(c, flip=True)
        e = replace(d, plan="baseline", baseline=40.0)
        f = replace(e, ground_prior=True)
        for name, cfg in (("A repo as found", a), ("B + D2/D4 geometry fix", b),
                          ("B2 + yaw-aligned vertical circle", b2),
                          ("C + normalised tuned SISMF, window, angle avg", c), ("D + bias-cancelling flip", d),
                          ("E + 40 m lateral baseline", e), ("F + ground-plane prior", f)):
            steps.append(((R, name), cfg))
    chans = ("paper-like", "field-temporal", "field-quiet")
    jobs = [((chn,) + tag, cfg, CH[chn], N, 5, 40, 100) for chn in chans for tag, cfg in steps]
    res = run_many(jobs, pool)
    rows = []
    for s in res:
        chn, R, name = s["tag"]
        rows.append(dict(channel=chn, R_m=R, step=name, mean_err_2d_m=s["mean2d"], median_err_2d_m=s["median2d"],
                         p90_err_2d_m=s["p90_2d"], line_err_median_deg=s["line_err_med_deg"],
                         time_min=s["time_min"], min_drone_sep_median_m=s["min_sep_median_m"],
                         runs_with_sep_below_3m=s["frac_sep_below_3m"]))
    write_csv("E4_ablation.csv", rows)

    style()
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), sharey=True)
    names = [n for (_, n), _ in steps[:7]]
    for ax, chn in zip(axes, chans):
        y = np.arange(len(names))
        for j, R in enumerate((5.0, 20.0)):
            vals = [next(r["median_err_2d_m"] for r in rows if r["channel"] == chn and r["R_m"] == R and r["step"] == n)
                    for n in names]
            ax.barh(y + (j - 0.5) * 0.38, vals, 0.35, color=SERIES[j], label=f"R = {R:.0f} m")
            for yy, v in zip(y, vals):
                ax.text(v + 0.5, yy + (j - 0.5) * 0.38, f"{v:.1f}", va="center", fontsize=7.5, color=INK2)
        ax.set_yticks(y, names)
        ax.invert_yaxis()
        ax.set_xlabel("median horizontal error (m)")
        ax.set_title(f"E4 - ablation, {chn}")
    axes[0].legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    save(fig, "E4_ablation.png")
    return rows


# ---------------------------------------------------------------------------
# E5  RSSI filter benchmark (synthetic truth)
# ---------------------------------------------------------------------------
FILTERS5 = [("raw", {}), ("kalman", {}), ("sismf", {}), ("kalman+sismf", {}), ("movavg", dict(n=5))]


def run_filter(kind, z, dt):
    if kind == "kalman+sismf":
        f1 = make_signal_filter("kalman", z[0])
        f2 = make_signal_filter("sismf", z[0])
        return np.array([f2.update(f1.update(v, dt), dt) for v in z])
    kw = dict(FILTERS5)[kind]
    f = make_signal_filter(kind, z[0], **kw)
    return np.array([f.update(v, dt) for v in z])


def ar_noise(rng, n, sigma, rho):
    e = np.empty(n)
    e[0] = rng.normal(0, sigma)
    for i in range(1, n):
        e[i] = rho * e[i - 1] + math.sqrt(1 - rho * rho) * sigma * rng.normal()
    return e


def profiles(dt=2.0):
    t = np.arange(0, 400, dt)
    steady = np.full_like(t, -88.0)
    ramp = np.interp(t, [0, 60, 180, 300, 400], [-100, -100, -76, -100, -100])   # radio carried in and out
    step = np.where(t < 200, -95.0, -83.0)
    return t, {"steady": steady, "ramp (moving radio)": ramp, "step +12 dB": step}


def lag_estimate(truth, y, dt):
    """Delay (s) that best aligns the filter output with the truth (least squares)."""
    best, bl = None, 0
    for k in range(0, 40):
        a = y[k:]
        b = truth[:len(truth) - k]
        m = np.mean((a - b) ** 2)
        if best is None or m < best:
            best, bl = m, k
    return bl * dt


def e5(N):
    rng = np.random.default_rng(55)
    dt = 2.0
    t, prof = profiles(dt)
    rows = []
    for sigma, rho, lab in ((1.2, 0.3, "field-quiet"), (2.5, 0.5, "field-typical")):
        for pname, truth in prof.items():
            acc = {k: dict(rmse=[], outvar=[], jitter=[], lag=[]) for k, _ in FILTERS5}
            for _ in range(N):
                z = np.round(truth + ar_noise(rng, len(t), sigma, rho))     # integer-dB RSSI
                for k, _ in FILTERS5:
                    y = run_filter(k, z, dt)
                    sl = slice(10, None)                                        # skip start-up
                    acc[k]["rmse"].append(np.sqrt(np.mean((y[sl] - truth[sl]) ** 2)))
                    acc[k]["outvar"].append(np.var(y[sl]))
                    acc[k]["jitter"].append(np.mean(np.abs(np.diff(y[sl]))))
                    if pname != "steady":
                        acc[k]["lag"].append(lag_estimate(truth, y, dt))
            raw = {m: np.mean(v) for m, v in acc["raw"].items() if v}
            for k, _ in FILTERS5:
                a = {m: (np.mean(v) if v else float("nan")) for m, v in acc[k].items()}
                rows.append(dict(noise=lab, profile=pname, filter=k, rmse_vs_truth_db=a["rmse"],
                                 rmse_reduction_pct=100 * (1 - a["rmse"] / raw["rmse"]),
                                 output_variance_db2=a["outvar"],
                                 output_var_reduction_pct=100 * (1 - a["outvar"] / raw["outvar"]),
                                 jitter_db=a["jitter"], lag_s=a["lag"]))
    write_csv("E5_rssi_filter_benchmark.csv", rows)

    style()
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), sharey=False)
    z = np.round(prof["ramp (moving radio)"] + ar_noise(np.random.default_rng(7), len(t), 2.5, 0.5))
    truth = prof["ramp (moving radio)"]
    axes[0].plot(t, z, color="#b9b8b2", lw=1.0, label="raw (1 dB steps)")
    axes[0].plot(t, truth, color=INK, lw=1.4, ls="--", label="true RSSI")
    for j, k in enumerate(["kalman", "sismf", "kalman+sismf"]):
        axes[0].plot(t, run_filter(k, z, dt), color=SERIES[j], lw=1.6, label=k)
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("RSSI (dBm)")
    axes[0].set_title("E5 - moving radio, field-typical noise")
    axes[0].legend(fontsize=7.5, loc="lower center")
    for ax, metric, ttl in ((axes[1], "rmse_vs_truth_db", "error vs true RSSI (lower is better)"),
                            (axes[2], "lag_s", "lag behind true RSSI (lower is better)")):
        ks = [k for k, _ in FILTERS5]
        x = np.arange(len(ks))
        for j, pname in enumerate(["steady", "ramp (moving radio)", "step +12 dB"]):
            if metric == "lag_s" and pname == "steady":
                continue
            vals = [next(r[metric] for r in rows if r["noise"] == "field-typical" and r["profile"] == pname
                         and r["filter"] == k) for k in ks]
            ax.bar(x + (j - 1) * 0.27, vals, 0.25, color=SERIES[j], label=pname)
        ax.set_xticks(x, ks, rotation=20)
        ax.set_ylabel("dB" if metric != "lag_s" else "seconds")
        ax.set_title(ttl)
        ax.legend(fontsize=7.5)
    fig.tight_layout()
    save(fig, "E5_rssi_filter_benchmark.png")
    return rows


# ---------------------------------------------------------------------------
# E5b real field logs
# ---------------------------------------------------------------------------
def e5b(log_dir):
    files = sorted(glob.glob(os.path.join(log_dir, "kalman_deney_log_*.csv")))
    rows = []
    example = None
    for fpath in files:
        with open(fpath, encoding="utf-8") as f:
            data = list(csv.DictReader(f))
        if len(data) < 200:
            continue
        for d in "1234":
            r = [(float(x["Gecen_Sure_Sn"]), float(x["Ham_RSSI"])) for x in data if x["Drone_ID"] == d]
            if len(r) < 60:
                continue
            tt, z = np.array(r).T
            keep = z > -115            # drop "radio off" floor samples
            tt, z = tt[keep], z[keep]
            if len(z) < 60:
                continue
            dt = float(np.median(np.diff(tt)))
            ys = {k: (z if k == "raw" else run_filter(k, z, dt)) for k, _ in FILTERS5}
            # lag of each filter w.r.t. a centred 31-sample median of the raw data (zero-phase reference)
            from scipy.ndimage import median_filter
            ref = median_filter(z, size=15, mode="nearest")
            for k, y in ys.items():
                rows.append(dict(log=os.path.basename(fpath), drone=int(d), filter=k, n=len(z),
                                 output_variance_db2=float(np.var(y)),
                                 jitter_db=float(np.mean(np.abs(np.diff(y)))),
                                 rms_dev_from_centred_median_db=float(np.sqrt(np.mean((y - ref) ** 2))),
                                 lag_vs_centred_median_s=lag_estimate(ref, y, dt)))
            if example is None and "1792067603" in fpath:
                example = (tt - tt[0], z, ys, os.path.basename(fpath), d)
    write_csv("E5b_real_logs_filters.csv", rows)
    if example:
        style()
        tt, z, ys, name, d = example
        fig, ax = plt.subplots(figsize=(10, 3.6))
        ax.plot(tt, z, color="#b9b8b2", lw=1.0, label="raw")
        for j, k in enumerate(["kalman", "sismf", "kalman+sismf"]):
            ax.plot(tt, ys[k], color=SERIES[j], lw=1.5, label=k)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("RSSI (dBm)")
        ax.set_title(f"E5b - field log {name}, drone {d}")
        ax.legend(fontsize=8, ncol=4)
        fig.tight_layout()
        save(fig, "E5b_real_log_example.png")
    # aggregate
    agg = []
    for k, _ in FILTERS5:
        rr = [r for r in rows if r["filter"] == k]
        if rr:
            agg.append(dict(filter=k, mean_output_variance_db2=float(np.mean([r["output_variance_db2"] for r in rr])),
                            mean_jitter_db=float(np.mean([r["jitter_db"] for r in rr])),
                            mean_lag_s=float(np.mean([r["lag_vs_centred_median_s"] for r in rr])),
                            n_series=len(rr)))
    write_csv("E5b_real_logs_summary.csv", agg)
    return agg


# ---------------------------------------------------------------------------
# E6  analytic sensitivity
# ---------------------------------------------------------------------------
def e6():
    style()
    d = np.linspace(15, 150, 300)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.2))
    for j, R in enumerate([5, 10, 20, 30]):
        dd = d[d > 1.5 * R]
        emax = 10 * CAL_N * np.log10((dd + R) / (dd - R))
        ax.plot(dd, emax, color=SERIES[j], label=f"R = {R} m")
    ax.axhline(1.0, color=INK2, lw=1.2, ls="--")
    ax.text(150, 1.05, "CONVERGENCE_THRESHOLD_DBM = 1.0", ha="right", va="bottom", fontsize=8, color=INK2)
    ax.axhline(2.5 * math.sqrt(2), color=SERIES[7], lw=1.2, ls=":")
    ax.text(150, 2.5 * math.sqrt(2) + 0.1, "noise std of one pair difference (σt = 2.5 dB)", ha="right",
            va="bottom", fontsize=8, color=INK2)
    ax.set_yscale("log")
    ax.set_xlabel("distance sphere centre -> target (m)")
    ax.set_ylabel("full-scale |P3 - P1| (dB)")
    ax.set_title(f"E6a - largest possible pair difference, n = {CAL_N:.3f}")
    ax.legend(fontsize=8, loc="lower left")
    # bearing bias from a spatial shadowing gradient (independent of R and of the filter)
    g_pl = LN10_10 * CAL_N / d                     # path-loss gradient, dB/m
    for j, (ss, lc) in enumerate([(0.5, 20), (1.0, 20), (1.0, 10), (2.0, 10)]):
        ax2.plot(d, np.degrees(np.arctan((ss / lc) / g_pl)), color=SERIES[j], label=f"σs = {ss} dB, Lc = {lc} m")
    ax2.set_xlabel("distance to target (m)")
    ax2.set_ylabel("typical bearing bias (deg)")
    ax2.set_title("E6b - multipath limit: atan(shadowing gradient / path-loss gradient)")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    save(fig, "E6_sensitivity.png")


# ---------------------------------------------------------------------------
# E7  example runs
# ---------------------------------------------------------------------------
def e7():
    style()
    tgt = np.array([55.0, 45.0, 0.0])
    fig, axes = plt.subplots(2, 3, figsize=(15, 6.2), sharex="col")
    cases = [("repo as found, R = 5 m, paper-like noise", repo_cfg(5.0, 15.0), CH["paper-like"]),
             ("best version, R = 20 m, paper-like noise", best_cfg(), CH["paper-like"]),
             ("best version, R = 20 m, field-quiet noise", best_cfg(), CH["field-quiet"])]
    out = []
    for col, (lab, cfg, ch) in enumerate(cases):
        r = simulate(replace(cfg, record=True), ch, tgt, 7)
        out.append((lab, r))
        h = r.history
        a = np.array(h["alpha"])
        b = np.array(h["beta"])
        e1 = np.array(h["e1"])
        e2 = np.array(h["e2"])
        ax = axes[0, col]
        ax.plot(a[:, 0] / 60, np.degrees(a[:, 1]), color=SERIES[0], lw=1.4, label="α (D1-D3, horizontal)")
        ax.plot(b[:, 0] / 60, np.degrees(b[:, 1]), color=SERIES[1], lw=1.4, label="β (D2-D4, vertical)")
        for tc, _ in h["center"]:
            ax.axvline(tc / 60, color=GRID, lw=1.0)
        ax.set_ylabel("angle (deg)")
        ax.set_title(lab)
        ax.legend(fontsize=7.5)
        ax = axes[1, col]
        ax.plot(e1[:, 0] / 60, e1[:, 1], color=SERIES[0], lw=1.0, label="e1 = P3 - P1")
        ax.plot(e2[:, 0] / 60, e2[:, 1], color=SERIES[1], lw=1.0, label="e2 = P4 - P2")
        ax.set_xlabel("time (min)")
        ax.set_ylabel("dB")
        ax.legend(fontsize=7.5)
    fig.tight_layout()
    save(fig, "E7_time_histories.png")

    fig = plt.figure(figsize=(15, 4.8))
    for k, (lab, r) in enumerate(out):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        for j, l in enumerate(r.lines):
            v = l.direction
            if v[2] > 0:
                v = -v
            ts = np.linspace(0, 120, 2)
            pts = l.center[None, :] + ts[:, None] * v[None, :]
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=SERIES[j % 8], lw=1.6)
            ax.scatter(*l.center, color=SERIES[j % 8], s=18)
        ax.scatter(*r.target, color=INK, marker="*", s=120, label="true target")
        ax.scatter(*r.estimate, color=SERIES[7], marker="x", s=70, label=f"estimate ({r.err2d:.1f} m)")
        ax.set_xlabel("x east (m)")
        ax.set_ylabel("y north (m)")
        ax.set_zlabel("z (m)")
        ax.set_zlim(-20, 40)
        ax.set_title(lab, fontsize=9.5)
        ax.legend(fontsize=7.5, loc="upper left")
    fig.tight_layout()
    save(fig, "E7_lines_3d.png")
    return [(lab, r.err2d, r.err3d) for lab, r in out]



# ---------------------------------------------------------------------------
# E8  Scenario I ("+" formation, ground_station_core_kalman.py)
# ---------------------------------------------------------------------------
def _s1job(a):
    from scenario1_sim import S1Cfg, mc_s1
    chn, filt, ctl, kw, n = a
    return (chn, filt, ctl), mc_s1(S1Cfg(signal_filter=filt, controller=ctl, ctrl_kw=kw), CH[chn], n=n, seed=8)


def e8(pool, N):
    from scenario1_sim import S1Cfg, run_s1
    ctls = [("p", {}, "P, K=0.5 (repo)"), ("sismf", {}, "Axis-SISMF (repo gains)")]
    filts = ["raw", "kalman", "sismf", "kalman+sismf"]
    chans = ["paper-like", "field-quiet", "field-typical"]
    jobs = [(c, f, k, kw, N) for c in chans for f in filts for k, kw, _ in ctls]
    res = pool.map(_s1job, jobs)
    lab = {k: l for k, _, l in ctls}
    rows = []
    for (chn, f, k), s in res:
        rows.append(dict(channel=chn, rssi_filter=f, controller=lab[k], final_err_median_m=s["final_err_median"],
                         final_err_p90_m=s["final_err_p90"], reached_5m_frac=s["reached_frac"],
                         t_reach_median_s=s["t_reach_median_s"], hover_jitter_m=s["jitter_m"],
                         path_over_straight=s["path_over_straight"]))
    write_csv("E8_scenario1_filters_x_controllers.csv", rows)
    style()
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    x = np.arange(len(filts))
    for ax, metric, ttl in ((axes[0], "final_err_median_m", "median final error (m)"),
                            (axes[1], "path_over_straight", "flown path / straight distance"),):
        for j, (k, _, l) in enumerate(ctls):
            for i2, chn in enumerate(["field-quiet", "field-typical"]):
                vals = [next(r[metric] for r in rows if r["channel"] == chn and r["rssi_filter"] == f
                             and r["controller"] == l) for f in filts]
                ax.bar(x + (j * 2 + i2 - 1.5) * 0.2, vals, 0.18, color=SERIES[j * 2 + i2],
                       label=f"{l}, {chn}")
        ax.set_xticks(x, filts)
        ax.set_title(f"E8 - Scenario I, {ttl}")
    axes[0].legend(fontsize=7)
    tgt = np.array([28.0, -18.0, 0.0])
    for j, (k, kw, l) in enumerate(ctls):
        r = run_s1(S1Cfg(signal_filter="kalman", controller=k, ctrl_kw=kw), CH["field-quiet"], tgt, 3, record=True)
        axes[2].plot(r["c"][:, 0], r["c"][:, 1], color=SERIES[j * 2], lw=1.4, marker="o", ms=2.5, label=l)
    axes[2].scatter([tgt[0]], [tgt[1]], marker="*", s=160, color=INK, label="target", zorder=5)
    axes[2].scatter([0], [0], marker="s", s=40, color=INK2, label="start", zorder=5)
    axes[2].set_aspect("equal")
    axes[2].set_xlabel("east (m)")
    axes[2].set_ylabel("north (m)")
    axes[2].set_title("E8 - virtual-centre track, Kalman, field-quiet")
    axes[2].legend(fontsize=7.5)
    fig.tight_layout()
    save(fig, "E8_scenario1.png")
    return rows

# ---------------------------------------------------------------------------
def md_table(rows, cols, fmt=None):
    fmt = fmt or {}
    head = "| " + " | ".join(cols) + " |\n|" + "|".join("---" for _ in cols) + "|\n"
    body = ""
    for r in rows:
        body += "| " + " | ".join((fmt.get(c, "{:.1f}").format(r[c]) if isinstance(r[c], float) else str(r[c]))
                                  for c in cols) + " |\n"
    return head + body


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--logs", default=None, help="folder with kalman_deney_log_*.csv")
    ap.add_argument("--only", default=None, help="comma list, e.g. E1,E5")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    n_big = 12 if args.quick else 60
    n_mid = 8 if args.quick else 40
    n_cell = 8 if args.quick else 60
    n_f = 30 if args.quick else 300
    only = set(args.only.split(",")) if args.only else None
    want = (lambda k: only is None or k in only)
    t0 = time.time()
    summary = {}
    with Pool() as pool:
        if want("E1"):
            summary["E1"] = e1(pool, n_big); print("E1 done", round(time.time() - t0), "s", flush=True)
        if want("E2"):
            summary["E2"] = e2(pool, n_mid); print("E2 done", round(time.time() - t0), "s", flush=True)
        if want("E3"):
            summary["E3"] = e3(pool, n_cell); print("E3 done", round(time.time() - t0), "s", flush=True)
        if want("E4"):
            summary["E4"] = e4(pool, n_big); print("E4 done", round(time.time() - t0), "s", flush=True)
        if want("E8"):
            summary["E8"] = e8(pool, n_mid); print("E8 done", round(time.time() - t0), "s", flush=True)
    if want("E5"):
        summary["E5"] = e5(n_f); print("E5 done", round(time.time() - t0), "s", flush=True)
    if want("E5b") and args.logs:
        summary["E5b"] = e5b(args.logs); print("E5b done", flush=True)
    if want("E6"):
        e6()
    if want("E7"):
        summary["E7"] = e7(); print("E7 done", round(time.time() - t0), "s", flush=True)
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=str)
    print("all done in", round(time.time() - t0), "s ->", OUT)


if __name__ == "__main__":
    main()
