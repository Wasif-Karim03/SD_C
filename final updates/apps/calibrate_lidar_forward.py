#!/usr/bin/env python3
"""
apps/calibrate_lidar_forward.py — find the LiDAR angle that points to car-FORWARD.

Differential method (robust to any clutter): capture a BASELINE scan with the
front clear, then capture a scan with a box placed DEAD AHEAD. Whatever newly
appears close = the box. Its bearing is the car's forward direction.

Steps (it prompts you):
  1. Clear ~1.5 m in front of the car. Press ENTER  -> baseline.
  2. Put a box ~0.5 m directly in front. Press ENTER -> compare.
  3. It prints the forward angle to put in config.LIDAR_FORWARD_DEG.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 calibrate_lidar_forward.py
"""
import os
import sys
import time
import math

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.lidar import ThreadedLidar   # noqa: E402

MIN_M = 0.15
APPEAR_DROP = 0.20    # a bin must get >=20 cm closer to count as "object appeared"
NEAR_M = 1.2          # ...and end up within this range


def capture(lid, secs=2.0):
    """Min distance (m) per 1° bearing bin over `secs`. inf = no return."""
    bins = [float("inf")] * 360
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        scan, age = lid.latest()
        if scan and age < 0.5:
            for _q, a, dmm in scan:
                d = dmm / 1000.0
                if d < MIN_M:
                    continue
                k = int(a) % 360
                if d < bins[k]:
                    bins[k] = d
        time.sleep(0.02)
    return bins


def main():
    print("Starting LiDAR ...")
    try:
        lid = ThreadedLidar().start()
    except Exception as exc:  # noqa: BLE001
        sys.exit(f"could not start LiDAR: {exc}")

    try:
        input("\n[1] Clear ~1.5 m in front of the car, then press ENTER for BASELINE...")
        base = capture(lid)
        print("    baseline captured.")
        input("\n[2] Now place a box ~0.5 m DEAD AHEAD, then press ENTER...")
        boxed = capture(lid)
        print("    captured. comparing ...")

        appeared = []
        for k in range(360):
            if boxed[k] < NEAR_M and (base[k] - boxed[k]) > APPEAR_DROP:
                appeared.append((k, boxed[k]))
        if not appeared:
            print("\nNo new object detected ahead. Was the box placed close enough "
                  "(~0.5 m) and the baseline actually clear? Try again.")
            return 1

        # circular mean of the bearings where the box appeared
        mx = sum(math.cos(math.radians(k)) for k, _ in appeared)
        my = sum(math.sin(math.radians(k)) for k, _ in appeared)
        fwd = math.degrees(math.atan2(my, mx)) % 360.0
        dmean = sum(d for _, d in appeared) / len(appeared)
        ks = sorted(k for k, _ in appeared)
        print("\n" + "=" * 56)
        print(f"  object appeared across {len(appeared)} bearings "
              f"({ks[0]}°..{ks[-1]}°), ~{dmean:.2f} m away")
        print(f"  >>> FORWARD = {fwd:.0f}°")
        print(f"  Set in config.py:   LIDAR_FORWARD_DEG = {fwd:.0f}")
        print("=" * 56)
    finally:
        lid.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
