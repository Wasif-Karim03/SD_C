#!/usr/bin/env python3
"""
apps/test_fusion.py — prove LiDAR + camera fusion (NO motors).

Runs the front camera and the LiDAR together, and for each frame prints THREE
decisions side by side:
    CAM  (depth free-space)   |   LIDAR (geometry)   |   FUSED
so you can see how they combine. Nothing moves.

Great things to try while it runs:
  * Put a box only the CAMERA sees low to the ground (below the LiDAR plane) ->
    camera should block while LiDAR stays clear -> FUSED blocks.
  * Put a wall/box in the LiDAR plane -> LiDAR blocks -> FUSED blocks.
  * Open one side -> steer should lean toward the open side.

Saves fusion_view.jpg (camera annotated) + fusion_lidar.png (lidar polar).

Run on the Jetson (front camera + LiDAR both free):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_fusion.py            # ~60 frames
"""
import os
import sys
import time
import math
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                    # noqa: E402
from drivers.camera import Camera                # noqa: E402
from drivers.lidar import ThreadedLidar          # noqa: E402
from perception.fusion import FusedNavigator     # noqa: E402

VIEW = os.path.join(HERE, "fusion_view.jpg")
LIDAR_PLOT = os.path.join(HERE, "fusion_lidar.png")
REPORT = os.path.join(HERE, "fusion_report.txt")
lines = []


def log(m=""):
    print(m)
    lines.append(m)


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def plot_lidar(scan, nav):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return
    ang = np.radians([a for (_q, a, _d) in scan])
    dist = np.array([d for (_q, _a, d) in scan]) / 1000.0
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N"); ax.set_theta_direction(-1)
    ax.scatter(ang, dist, s=4, color="#3b7")
    fwd = math.radians(nav.lidar_nav.forward_deg)
    rmax = min(max(dist.max(), 1.0) * 1.1, 12)
    ax.plot([fwd, fwd], [0, rmax], color="#e39b4a", lw=2)
    ax.set_rmax(rmax); ax.set_title("FUSED — LiDAR (orange = forward)")
    fig.savefig(LIDAR_PLOT, dpi=110, bbox_inches="tight")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()

    log("=" * 72)
    log(f"fusion test (camera + LiDAR, no motors) @ {time.strftime('%H:%M:%S')}")
    log("=" * 72)
    try:
        cam = Camera("front").start()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! front camera: {exc}"); _save(); return 1
    try:
        lid = ThreadedLidar().start()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! lidar: {exc}"); cam.release(); _save(); return 1

    log("loading depth model ...")
    nav = FusedNavigator(device=0)
    import numpy as np
    nav.estimate_camera(np.zeros((cam.height, cam.width, 3), np.uint8))
    log("running.\n")

    seq, n = 0, 0
    last_frame = last_scan = None
    try:
        while n < args.frames:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            scan, age = lid.latest()
            scan = scan if (scan and age < 0.5) else None
            fused = nav.plan(frame, scan)
            cam_d, lid_d = fused["camera"], fused["lidar"]
            last_frame, last_scan = frame, scan
            n += 1
            if n % 4 == 0:
                cam_s = f"CAM[{'BLK' if cam_d['blocked'] else 'clr'} " \
                        f"st{cam_d['steer']:+.2f} {cam_d['center_reach']:.1f}m]"
                if lid_d:
                    lid_s = f"LIDAR[{'BLK' if lid_d['blocked'] else 'clr'} " \
                            f"st{lid_d['steer']:+.2f} {lid_d['nearest_ahead_m']:.2f}m]"
                else:
                    lid_s = "LIDAR[--]"
                fus_s = f"FUSED[{'BLOCKED' if fused['blocked'] else 'clear'} " \
                        f"st{fused['steer']:+.2f} ({fused['source']})]"
                log(f"  {cam_s}  {lid_s}  ->  {fus_s}")
    except KeyboardInterrupt:
        log("\ninterrupted.")
    finally:
        # save visuals
        if last_frame is not None:
            import cv2
            fused = nav.plan(last_frame, last_scan)
            annotated = nav.cam_nav.draw(last_frame, fused["cam_depth"], fused["camera"])
            cv2.imwrite(VIEW, annotated)
            log(f"\ncamera view -> {VIEW}")
        if last_scan is not None:
            plot_lidar(last_scan, nav)
            log(f"lidar plot  -> {LIDAR_PLOT}")
        lid.stop()
        cam.release()

    log("\n[RESULT] Fusion ran: camera + LiDAR -> one decision. Check that FUSED")
    log("         blocks when EITHER sensor sees an obstacle, and steers to open space.")
    _save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
