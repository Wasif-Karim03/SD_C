"""
LIVE lidar-only SLAM viewer for RPLIDAR C1 — watch the map build in real time.

Opens a matplotlib window (on DISPLAY :0) that updates every few scans while
you move the car around by hand. Reuses the mapping/ICP helpers from slam_test.

Usage:
    DISPLAY=:0 python3 slam_live.py
Close the window (or Ctrl+C in terminal) to stop and save slam_map.png.
Move the car SLOWLY so consecutive scans overlap.
"""
import os
import math
import time
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from rplidar_c1 import RPLidarC1
import slam_test as S   # reuse RES, SIZE, ORIGIN, scan_to_xy, transform, icp, integrate

OUT_PNG = os.path.join(os.path.dirname(__file__), "slam_map.png")
UPDATE_EVERY = 3        # redraw every N scans


def main():
    lidar = RPLidarC1().connect()
    logodds = np.zeros((S.SIZE, S.SIZE), dtype=np.float32)
    pose = (0.0, 0.0, 0.0)
    traj = [(0.0, 0.0)]
    prev = None
    frames = 0

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    extent = [-S.ORIGIN * S.RES, (S.SIZE - S.ORIGIN) * S.RES,
              -S.ORIGIN * S.RES, (S.SIZE - S.ORIGIN) * S.RES]
    img = ax.imshow(np.zeros((S.SIZE, S.SIZE)), cmap="gray_r", origin="lower",
                    vmin=0, vmax=1, extent=extent)
    (path_ln,) = ax.plot([], [], "-", color="tab:red", lw=1.2)
    (robot_pt,) = ax.plot([], [], "^", color="lime", ms=12)
    ax.set_xlim(-8, 8); ax.set_ylim(-8, 8)
    ax.set_title("RPLIDAR C1 live SLAM — move the car slowly")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.canvas.draw(); plt.pause(0.01)

    running = {"go": True}
    fig.canvas.mpl_connect("close_event", lambda e: running.__setitem__("go", False))

    try:
        lidar.stop()
        h = lidar.get_health()
        print(f"Health: {h['status_str']}. Live window open on DISPLAY :0. "
              f"Move the car slowly. Close window to stop.")
        t0 = time.time()
        for scan in lidar.iter_scans(min_points=120):
            if not running["go"]:
                break
            pts = S.scan_to_xy(scan)
            if len(pts) < 20:
                continue
            if prev is not None:
                pose, ok = S.icp(pts, prev, pose)
            world = S.transform(pts, pose)
            S.integrate(logodds, world, pose)
            prev = world
            traj.append((pose[0], pose[1]))
            frames += 1

            if frames % UPDATE_EVERY == 0:
                prob = 1.0 - 1.0 / (1.0 + np.exp(logodds))
                img.set_data(prob)
                tr = np.array(traj)
                path_ln.set_data(tr[:, 0], tr[:, 1])
                robot_pt.set_data([pose[0]], [pose[1]])
                ax.set_title(f"LIVE SLAM  |  frame {frames}  "
                             f"pose=({pose[0]:+.2f},{pose[1]:+.2f}) "
                             f"{math.degrees(pose[2]):+.0f}deg")
                fig.canvas.draw_idle()
                plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        lidar.disconnect()
        # final full-quality save
        matplotlib.use("Agg")
        S.save_map(logodds, traj)
        print(f"Frames: {frames} | Saved map -> {OUT_PNG}")


if __name__ == "__main__":
    main()
