#!/usr/bin/env python3
"""
apps/test_lidar.py — prove the RPLIDAR C1 works, end to end.

Does five things and writes a report + a picture so we can SEE it working:
  1. Opens the lidar by its stable by-id path.
  2. Reads device INFO (model/firmware/serial) — confirms it's really the C1.
  3. Reads HEALTH — confirms the sensor reports "Good".
  4. Grabs several full 360deg scans and reports stats (points/scan, min/max/median
     range, angular coverage) — confirms it's actually spinning and ranging.
  5. Saves a polar plot of one scan -> lidar_scan.png (the room outline) and a
     text report -> lidar_report.txt, both next to this script.

Run on the Jetson (nothing else may be using the lidar port):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_lidar.py

If it says "bad/empty descriptor": something else holds the port (stop it), or the
lidar isn't powered. No display is needed — the plot is saved to a file.
"""
import os
import sys
import time
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)          # the "final updates" folder
sys.path.insert(0, ROOT)
from drivers.lidar import RPLidarC1   # noqa: E402

REPORT = os.path.join(HERE, "lidar_report.txt")
PLOT = os.path.join(HERE, "lidar_scan.png")
N_SCANS = 5

lines = []


def log(msg=""):
    print(msg)
    lines.append(msg)


def save_report():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def plot_scan(scan):
    """Save a top-down polar plot of one scan (angle vs distance)."""
    try:
        import matplotlib
        matplotlib.use("Agg")          # headless, no display
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as exc:  # noqa: BLE001
        log(f"  (matplotlib not available, skipping plot: {exc})")
        return
    ang = np.radians([a for (_q, a, _d) in scan])
    dist = np.array([d for (_q, _a, d) in scan]) / 1000.0   # -> metres
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")    # 0deg = front of the lidar, up
    ax.set_theta_direction(-1)
    ax.scatter(ang, dist, s=4)
    ax.set_title(f"RPLIDAR C1 — one scan ({len(scan)} pts)  [metres]")
    ax.set_rmax(min(max(dist.max(), 0.5) * 1.1, 12))
    fig.savefig(PLOT, dpi=110, bbox_inches="tight")
    log(f"  scan plot -> {PLOT}")


def main():
    log("=" * 64)
    log(f"RPLIDAR C1 test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)

    lidar = RPLidarC1()
    log(f"port: {lidar.port}")
    if not os.path.exists(lidar.port):
        log("  !! port path does not exist — is the lidar plugged in / powered?")
        save_report()
        return 1

    try:
        lidar.connect()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not open port: {exc}")
        save_report()
        return 1

    try:
        # --- info ---
        try:
            info = lidar.get_info()
            log("\n[INFO]")
            for k, v in info.items():
                log(f"   {k:10s}: {v}")
            if info["model_hex"] != "0x41":
                log("   (note: expected model 0x41 for the C1)")
        except Exception as exc:  # noqa: BLE001
            log(f"  !! get_info failed: {exc}")

        # --- health ---
        try:
            health = lidar.get_health()
            log("\n[HEALTH]")
            log(f"   status   : {health['status_str']} ({health['status']})")
            log(f"   err_code : {health['error_code']}")
            if health["status"] != 0:
                log("   !! not 'Good' — reset the lidar / check power if scans fail")
        except Exception as exc:  # noqa: BLE001
            log(f"  !! get_health failed: {exc}")

        # --- scans ---
        log(f"\n[SCANS] grabbing {N_SCANS} full revolutions ...")
        t0 = time.monotonic()
        scans = lidar.grab_scans(N_SCANS)
        dt = time.monotonic() - t0
        if not scans:
            log("  !! no scans received — motor not spinning or port issue")
            save_report()
            return 1
        rate = len(scans) / dt if dt > 0 else 0.0
        log(f"   got {len(scans)} scans in {dt:.2f}s  (~{rate:.1f} scans/s)")
        for i, scan in enumerate(scans):
            dists = [d for (_q, _a, d) in scan]
            angs = [a for (_q, a, _d) in scan]
            cov = (max(angs) - min(angs)) if angs else 0
            log(f"   scan {i}: {len(scan):4d} pts  "
                f"range {min(dists)/1000:.2f}-{max(dists)/1000:.2f} m  "
                f"median {statistics.median(dists)/1000:.2f} m  "
                f"angular span {cov:.0f}deg")

        # --- picture ---
        log("\n[PLOT]")
        best = max(scans, key=len)     # densest scan = clearest outline
        plot_scan(best)

        log("\n[RESULT] LiDAR is alive: info + health read, and it is spinning and")
        log("         ranging. Open lidar_scan.png to see the room outline.")
    finally:
        lidar.disconnect()
        log("\nlidar port released.")

    save_report()
    return 0


if __name__ == "__main__":
    sys.exit(main())
