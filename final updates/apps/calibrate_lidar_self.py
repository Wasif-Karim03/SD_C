#!/usr/bin/env python3
"""
apps/calibrate_lidar_self.py — record the car's OWN LiDAR returns, per bearing.

The chassis/mast/wiring around the LiDAR produce returns at fixed bearings (a band
directly behind, plus side bits). This captures them with the area clear and writes
lidar_self_profile.json: per-bearing mask radius (metres). ThreadedLidar then drops
anything closer than that at each bearing, so the car never sees itself — while real
obstacles beyond the body are kept.

Steps:
  1. Clear ~2 m all around the car (nothing within 2 m). Step back.
  2. Run it. It captures for a few seconds and saves the profile.
  3. Re-run any app (selfdrive, dashboard, control_center) — the self-returns are gone.

Run on the Jetson (LiDAR free):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 calibrate_lidar_self.py
"""
import os
import sys
import json
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                             # noqa: E402
from drivers.lidar import RPLidarC1       # noqa: E402

SELF_MAX = 0.5      # the car's OWN body is within ~0.4 m; anything past this is the
                    # environment (walls) and must NOT be masked. Keeps the mask tight.
MARGIN = 0.06       # mask a little beyond the farthest self return per bearing
CAP = 0.55          # never mask beyond this (safety — don't hide real obstacles)


def main():
    print("=" * 60)
    print("LiDAR self-profile calibration")
    print("=" * 60)
    lidar = RPLidarC1()
    if not os.path.exists(lidar.port):
        print("  !! LiDAR port missing."); return 1
    lidar.connect()
    try:
        input("\nClear ~0.7 m around the car (nothing within arm's reach; walls "
              "further out are fine). Press ENTER ...")
        print("capturing self-returns (~4 s, keep the area clear) ...")
        maxself = [0.0] * 360
        n = 0
        t0 = time.monotonic()
        for scan in lidar.iter_scans(min_points=90):
            for _q, a, dmm in scan:
                d = dmm / 1000.0
                if 0 < d < SELF_MAX:
                    b = int(a) % 360
                    if d > maxself[b]:
                        maxself[b] = d
            n += 1
            if time.monotonic() - t0 > 4.0:
                break
        mask = [min(m + MARGIN, CAP) if m > 0 else 0.0 for m in maxself]
        with open(config.LIDAR_SELF_PROFILE, "w") as fh:
            json.dump(mask, fh)
        masked_bins = sum(1 for m in mask if m > 0)
        print(f"\ncaptured {n} scans.")
        print(f"  bearings with a self-return masked : {masked_bins}/360")
        print(f"  largest mask radius                : {max(mask):.2f} m")
        if masked_bins > 200:
            print("  !! that's a LOT of bearings — the area wasn't clear enough "
                  "(objects/walls within 0.5 m got counted as the car). Clear a bit "
                  "more within arm's reach and re-run for a tighter profile.")
        print(f"  saved -> {config.LIDAR_SELF_PROFILE}")
        print("\nDone. Re-run selfdrive/dashboard/control_center — the car's own")
        print("body will no longer show as an obstacle.")
    finally:
        lidar.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
