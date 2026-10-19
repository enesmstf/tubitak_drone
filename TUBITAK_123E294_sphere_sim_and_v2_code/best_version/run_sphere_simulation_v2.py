"""
run_sphere_simulation_v2.py - Küre (Senaryo II) yer istasyonu kodunu UÇMADAN çalıştırır.

Gerçek ground_station_core_sphere_v2.py kodu (aynı karar motoru, aynı LoRa paket
ayrıştırma, aynı Kalman, aynı açı yasası) sanal bir dünyaya bağlanır: sanal dronlar
komut edilen noktalara uçar, RSSI saha kalibrasyonundan + gürültüden üretilir.

KULLANIM:
    pip install pyserial pymavlink numpy
    python run_sphere_simulation_v2.py                  # varsayılan hedef (55, 45, 0)
    python run_sphere_simulation_v2.py 30 -60 --noise 0.5 --scale 60
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ground_station_core_sphere_v2 as gs  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("east", nargs="?", type=float, default=55.0, help="hedef Doğu (m)")
ap.add_argument("north", nargs="?", type=float, default=45.0, help="hedef Kuzey (m)")
ap.add_argument("--noise", type=float, default=gs.SIM_NOISE_STD_DB, help="zamansal gürültü std (dB)")
ap.add_argument("--radius", type=float, default=gs.FORMATION_RADIUS_M, help="çember yarıçapı R (m)")
ap.add_argument("--scale", type=float, default=gs.SIM_TIME_SCALE, help="zaman hızlandırma")
ap.add_argument("--quiet", action="store_true", help="döngü çıktısını gizle, sadece özet")
args = ap.parse_args()

gs.SIM_TARGET_ENU = (args.east, args.north, 0.0)
gs.SIM_NOISE_STD_DB = args.noise
gs.SIM_TIME_SCALE = args.scale
gs.FORMATION_RADIUS_M = args.radius
gs.FORMATION_ALT_M = max(gs.FORMATION_ALT_M, args.radius + 5.0)

if args.quiet:
    import builtins
    _print = builtins.print
    KEEP = ("[DOĞRU", "[HEDEF", "[SİM]", "[RAPOR]", "  #", "[GÖREV]", "[PLAN]", "[FLIP]", "[KRİTİK", "=====")
    builtins.print = lambda *a, **k: _print(*a, **k) if a and str(a[0]).lstrip("\n").startswith(KEEP) else None

station = gs.GroundStation(lora_port=None, pixhawk_ports={1: None, 2: None, 3: None, 4: None},
                           simulation=True, platform_label="Fiziksel simülasyon (v2)")
try:
    station.run()
except KeyboardInterrupt:
    pass
finally:
    station.shutdown()
