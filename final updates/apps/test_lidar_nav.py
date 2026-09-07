#!/usr/bin/env python3
"""
apps/test_lidar_nav.py — prove LiDAR path-finding + speed, and CALIBRATE forward.

Uses the threaded LiDAR to (1) confirm the new scan rate, (2) turn each scan into
nearest-ahead / blocked / steer, and (3) tell us which raw scan angle points to the
car's FRONT so we can set config.LIDAR_FORWARD_DEG.

>>> CALIBRATION: place a box/wall ~0.5 m DIRECTLY IN FRONT of the car, then run
    this. The reported "nearest bearing" is the raw LiDAR angle of straight-ahead
    — put that number into config.LIDAR_FORWARD_DEG. (Also check: after setting it,
    blocking the FRONT should raise 'blocked', and an opening on one side should
    steer toward it; if it steers the wrong way, flip config.LIDAR_STEER_SIGN.)

Saves lidar_nav_scan.png (polar: points + forward arrow + front cone + nearest).

Run on the Jetson (nothing else using the LiDAR):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_lidar_nav.py
"""
import os
import sys
import time
import math
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                     # noqa: E402
from drivers.lidar import ThreadedLidar           # noqa: E402
from perception.lidar_nav import LidarNavigator   # noqa: E402

REPORT = os.path.join(HERE, "lidar_nav_report.txt")
PLOT = os.path.join(HERE, "lidar_nav_scan.png")
SECONDS = 8.0
lines = []


def log(m=""):
    print(m)
    lines.append(m)


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def plot(scan, nav, plan):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as exc:  # noqa: BLE001
        log(f"  (matplotlib unavailable, skipping plot: {exc})")
        return
    ang = np.radians([a for (_q, a, _d) in scan])
    dist = np.array([d for (_q, _a, d) in scan]) / 1000.0
    fig = plt.figure(figsize=(6.2, 6.2))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.scatter(ang, dist, s=4, color="#3b7")
    fwd = math.radians(nav.forward_deg)
    rmax = min(max(dist.max(), 1.0) * 1.1, 12)
    ax.plot([fwd, fwd], [0, rmax], color="#e39b4a", lw=2)          # forward
    ax.fill_between(np.linspace(fwd - math.radians(nav.front_arc / 2),
                                fwd + math.radians(nav.front_arc / 2), 30),
                    0, rmax, color="#e39b4a", alpha=0.12)          # front cone
    if plan["nearest_bearing_deg"] is not None:
        ax.scatter([math.radians(plan["nearest_bearing_deg"])],
                   [plan["nearest_dist_m"]], s=90, color="red", marker="x")
    ax.set_rmax(rmax)
    ax.set_title(f"LiDAR nav — forward={nav.forward_deg:.0f}°, "
                 f"nearest-ahead={plan['nearest_ahead_m']:.2f} m")
    fig.savefig(PLOT, dpi=110, bbox_inches="tight")
    log(f"  plot -> {PLOT}")


def main():
    log("=" * 64)
    log(f"LiDAR nav test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)
    log(f"forward_deg={config.LIDAR_FORWARD_DEG}  front_arc={config.LIDAR_FRONT_ARC_DEG}"
        f"  stop={config.LIDAR_STOP_M}m  clear={config.LIDAR_CLEAR_M}m")

    try:
        lid = ThreadedLidar().start()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not start LiDAR: {exc}")
        _save()
        return 1
    if lid.info:
        log(f"model {lid.info.get('model_hex')} fw{lid.info.get('firmware')}  "
            f"health={ (lid.health or {}).get('status_str') }")
    nav = LidarNavigator()

    nearest_bearings = []
    scans_seen = 0
    last_scan = None
    t0 = time.monotonic()
    last_log = 0.0
    try:
        while time.monotonic() - t0 < SECONDS:
            scan, age = lid.latest()
            if scan is None or age > 0.5:
                time.sleep(0.02)
                continue
            plan = nav.plan(scan)
            last_scan = scan
            scans_seen += 1
            if plan["nearest_bearing_deg"] is not None:
                nearest_bearings.append(plan["nearest_bearing_deg"])
            now = time.monotonic()
            if now - last_log >= 0.5:
                last_log = now
                st = "BLOCKED" if plan["blocked"] else "clear  "
                na = plan["nearest_ahead_m"]
                na_s = f"{na:5.2f}" if na != float("inf") else "  inf"
                nb = plan["nearest_bearing_deg"]
                nb_s = f"{nb:6.1f}" if nb is not None else "   ---"
                nd = plan["nearest_dist_m"]
                nd_s = f"{nd:.2f}" if nd not in (None, float("inf")) else "inf"
                log(f"  {st} nearest-ahead={na_s}m steer={plan['steer']:+.2f} "
                    f"nearest@{nb_s}deg ({nd_s}m) pts={plan['n_points']}")
            time.sleep(0.02)
    finally:
        rate = scans_seen / max(time.monotonic() - t0, 1e-6)
        lid.stop()

    log("\n[SUMMARY]")
    log(f"   scans processed : {scans_seen}  (~{rate:.1f} Hz consumed)")
    if last_scan is not None:
        # Forward candidate = circular-mean bearing of the CALIBRATION OBJECT,
        # isolated to the 0.3-1.2 m band: ignores the car's own <0.3 m self-return
        # AND far walls (>1.2 m). Place the box ~0.5 m dead ahead, nothing else that
        # close, and this bearing IS the car's forward direction.
        band = [a for (_q, a, d) in last_scan if 0.3 <= d / 1000.0 <= 1.5]
        if band:
            # densest 10° bin = the box (a tight cluster), robust to stray points
            bins = {}
            for a in band:
                k = int(a // 10) * 10
                bins[k] = bins.get(k, 0) + 1
            best_bin = max(bins, key=bins.get)
            # circular mean of points within that bin (+/-10°) for a precise angle
            near = [a for a in band
                    if abs(((a - (best_bin + 5) + 180) % 360) - 180) <= 12]
            mx = sum(math.cos(math.radians(a)) for a in near)
            my = sum(math.sin(math.radians(a)) for a in near)
            fwd = math.degrees(math.atan2(my, mx)) % 360.0
            log(f"   calibration obj : densest near cluster = {len(near)} pts around "
                f"{fwd:.1f}°  (of {len(band)} in 0.3-1.5 m)")
            log(f"   --> if that is your box DEAD AHEAD, set "
                f"config.LIDAR_FORWARD_DEG = {fwd:.0f}")
        else:
            log("   no object in the 0.3-1.5 m band — place a box ~0.5 m dead ahead.")
        plan = nav.plan(last_scan)
        plot(last_scan, nav, plan)
    log("\n[RESULT] LiDAR path-finding is running. Use the nearest-bearing above to")
    log("         calibrate forward, then we fuse it with the camera.")
    _save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
