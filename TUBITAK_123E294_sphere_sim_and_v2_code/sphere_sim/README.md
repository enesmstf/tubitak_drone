# sphere_sim – no-flight simulator for TÜBİTAK 123E294

Simulates Scenario II (sphere formation, Güzey 2022 / IEEE Access 2024 / IJSS 2026) and
Scenario I ("+" formation) with an RSSI channel calibrated from our own field data, and
regenerates every table and figure used in the results document.

```
pip install numpy matplotlib scipy
python run_experiments.py --logs <path to tubitak_drone repo>     # full run, ~10 min on 2 cores
python run_experiments.py --quick                                  # smoke test, ~2 min
python run_experiments.py --only E4,E8                             # selected experiments
python tune.py                                                     # gain grid search (~30 s)
```

Outputs: `results/` (PNG at 300 dpi, CSV tables, summary.json).

| File | Content |
| --- | --- |
| `sim_core.py` | Channel model, RSSI filters (raw / Kalman / SISMF-signal / moving average), angle laws (PID, ISMF-2024, SISMF-2026), sphere geometry (paper typo, corrected, yaw-aligned), localisation run, Monte Carlo |
| `scenario1_sim.py` | Scenario I decision engine of `ground_station_core_kalman.py` (P and axis-SISMF controllers) |
| `run_experiments.py` | Experiments E1–E8 (see its docstring) |
| `tune.py` | Gain grid search behind `TUNED` in `run_experiments.py` |

## Channel model

P_i = P_ref − 10 n log10(d_i) + S(x_i) + b_i + w_i(k), P_ref = −78.72 dBm, n = 1.264
(kalibrasyon_sonucu_saha.json). S = spatial shadowing field (σs, Lc), b_i = residual
calibration error (0.5 dB), w_i = AR(1) temporal noise (σt, ρ), raw values rounded to 1 dB,
one sample per drone every 2 s. Noise scenarios are defined in `CH` in `run_experiments.py`.

## Notes on reproducibility

* All runs use fixed seeds; numbers in the document come from `results/*.csv`.
* The gain optimum is shallow: re-running `tune.py` after code changes picks neighbouring
  gains (e.g. k = 0.1 instead of 0.2) with differences inside the Monte Carlo spread.
* σs and Lc (airborne multipath) are assumptions until a hover test measures them; they
  dominate the field-noise results of Scenario II.
