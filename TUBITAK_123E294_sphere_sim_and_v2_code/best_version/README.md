# best_version – v2 ground-station code (TÜBİTAK 123E294)

Drop these files into `guncel/Guncel_Proje_Dosyalari/Yer_Istasyonu_Kodlari/` next to the
v1 files. Nothing in v1 is modified.

| File | What it is |
| --- | --- |
| `ground_station_core_sphere_v2.py` | Scenario II decision engine, corrected (see header docstring for the 9 changes) |
| `ground_station_core_kalman_v2.py` | Scenario I decision engine: field calibration offsets, separate filter/controller switches, Kalman + P default, log-line fix |
| `ground_station_{windows,linux}_{sphere,kalman}_v2.py` | Launchers (same ports as v1) |
| `run_sphere_simulation_v2.py` | Runs the real Scenario II v2 code with no hardware: virtual drones + calibrated RSSI |
| `make_sphere_v2.py`, `make_kalman_v2.py` | Scripts that generate the v2 files from the v1 files by explicit text replacements (review these to see every change) |

## Run Scenario II without flying

```
pip install pyserial pymavlink numpy
python run_sphere_simulation_v2.py                         # target (55, 45) m, field-quiet noise, R = 5 m
python run_sphere_simulation_v2.py 30 -60 --radius 20 --noise 1.2 --scale 40 --quiet
```

`--scale` speeds up mission time (40 → a 50-minute mission in about 75 s). The run ends with
the least-squares estimate and the horizontal error against the virtual target.

## Before any real flight

* Scenario II v2 has run only in its simulation mode. Run it in ArduPilot SITL first.
* `BENCH_TEST_MODE` is False in sphere v2 (simulation mode no longer needs it); set True
  for propeller-off bench tests.
* `FORMATION_ALT_M − FORMATION_RADIUS_M` must be ≥ `MIN_SAFE_ALT_M` (5 m); the code exits otherwise.
* Set the ArduPilot GCS failsafe manually, as with v1.
* With `VERTICAL_PLANE = "fixed_xz"` (paper geometry) two drones can be commanded to the same
  point when the target is north/south of the centre; keep `"aligned"`.

## Regenerate

```
python make_sphere_v2.py <repo>/.../ground_station_core_sphere.py ground_station_core_sphere_v2.py
python make_kalman_v2.py <repo>/.../ground_station_core_kalman.py  ground_station_core_kalman_v2.py
```
