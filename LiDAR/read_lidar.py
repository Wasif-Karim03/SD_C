"""
Quick test: read RPLIDAR C1 data and prove we get a clean 360deg scan.
Captures a few revolutions, prints stats, and saves a top-down scatter plot.

Usage:
    python3 read_lidar.py            # capture 5 revolutions, save scan_plot.png
    python3 read_lidar.py 10         # capture 10 revolutions
"""
import sys
import math
import os
from rplidar_c1 import RPLidarC1

N_REVS = int(sys.argv[1]) if len(sys.argv) > 1 else 5
OUT_PNG = os.path.join(os.path.dirname(__file__), "scan_plot.png")


def main():
    lidar = RPLidarC1().connect()
    try:
        lidar.stop()
        info = lidar.get_info()
        health = lidar.get_health()
        print(f"Device : model=0x{info['model']:02X} fw={info['firmware']} "
              f"hw={info['hardware']} serial={info['serial']}")
        print(f"Health : {health['status_str']} (err={health['error_code']})")
        if health["status"] == 2:
            print("!! Device reports ERROR state, try reset/power-cycle")
            return

        print(f"\nCapturing {N_REVS} revolutions...")
        last_scan = None
        for i, scan in enumerate(lidar.iter_scans()):
            dists = [d for _, _, d in scan]
            angs = [a for _, a, _ in scan]
            print(f"  rev {i+1:2d}: {len(scan):4d} pts | "
                  f"dist {min(dists):6.0f}-{max(dists):6.0f} mm | "
                  f"angle span {min(angs):5.1f}-{max(angs):5.1f} deg")
            last_scan = scan
            if i + 1 >= N_REVS:
                break

        if last_scan:
            save_plot(last_scan)
    finally:
        lidar.disconnect()
        print("\nDisconnected.")


def save_plot(scan):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"(matplotlib unavailable, skipping plot: {e})")
        return
    xs, ys = [], []
    for _, ang, dist in scan:
        r = ang * math.pi / 180.0
        xs.append(dist * math.cos(r) / 1000.0)   # meters
        ys.append(dist * math.sin(r) / 1000.0)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(xs, ys, s=4, c="tab:blue")
    ax.scatter([0], [0], s=60, c="red", marker="^", label="LiDAR")
    ax.set_aspect("equal")
    ax.set_title(f"RPLIDAR C1 scan ({len(scan)} points)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3); ax.legend()
    fig.savefig(OUT_PNG, dpi=110, bbox_inches="tight")
    print(f"\nSaved top-down scan plot -> {OUT_PNG}")


if __name__ == "__main__":
    main()
