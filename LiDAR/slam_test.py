"""
Self-contained lidar-only SLAM test for RPLIDAR C1 (no external SLAM libs).

Pipeline:
  scan -> ICP scan-matching vs previous scan -> incremental pose ->
  accumulate into a global log-odds occupancy grid -> save map PNG.

This is a minimal demonstrator (point-to-point ICP, no loop closure), enough
to prove we can build a consistent 2D map by driving the car slowly around.

Usage:
    python3 slam_test.py             # run 60 s, save slam_map.png
    python3 slam_test.py 120         # run 120 s
Move the car SLOWLY (scans must overlap frame-to-frame) for a clean map.
Ctrl+C to stop early and save.
"""
import sys
import math
import os
import numpy as np
from scipy.spatial import cKDTree
from rplidar_c1 import RPLidarC1

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT_PNG = os.path.join(os.path.dirname(__file__), "slam_map.png")

# --- map config ---
RES = 0.05                 # meters per cell
SIZE = 500                 # cells per side  -> 25m x 25m map
ORIGIN = SIZE // 2         # sensor start at grid center
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX = -5.0, 6.0
MAX_RANGE = 12.0           # m, ignore returns beyond this for mapping


def scan_to_xy(scan):
    """(quality,angle_deg,dist_mm) list -> Nx2 array in meters, sensor frame."""
    pts = []
    for _, ang, dist in scan:
        r = dist / 1000.0
        if 0.10 < r < MAX_RANGE:
            a = math.radians(ang)
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))


def transform(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + np.array([x, y])


def icp(src, dst, pose, iters=25, tol=1e-4, max_corr=0.6):
    """Return refined pose that aligns src (sensor frame) onto dst (world)."""
    if len(src) < 15 or len(dst) < 15:
        return pose, False
    tree = cKDTree(dst)
    x, y, th = pose
    for _ in range(iters):
        cur = transform(src, (x, y, th))
        d, idx = tree.query(cur)
        m = d < max_corr
        if m.sum() < 15:
            return pose, False
        A = cur[m]
        B = dst[idx[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Rr = Vt.T @ U.T
        if np.linalg.det(Rr) < 0:
            Vt[1] *= -1
            Rr = Vt.T @ U.T
        dth = math.atan2(Rr[1, 0], Rr[0, 0])
        t = cb - Rr @ ca
        # compose incremental (Rr,t) onto current pose
        x, y = (Rr @ np.array([x, y])) + t
        th += dth
        if abs(dth) < tol and np.linalg.norm(t) < tol:
            break
    return (x, y, th), True


def world_to_grid(p):
    gx = np.round(p[:, 0] / RES).astype(int) + ORIGIN
    gy = np.round(p[:, 1] / RES).astype(int) + ORIGIN
    return gx, gy


def integrate(logodds, pts_world, pose):
    """Insert one scan: mark endpoints occupied, ray free-space."""
    sx = int(round(pose[0] / RES)) + ORIGIN
    sy = int(round(pose[1] / RES)) + ORIGIN
    gx, gy = world_to_grid(pts_world)
    for ex, ey in zip(gx, gy):
        # free space along ray (integer Bresenham, coarse but fine)
        for cx, cy in bresenham(sx, sy, ex, ey):
            if 0 <= cx < SIZE and 0 <= cy < SIZE:
                logodds[cy, cx] = max(L_MIN, logodds[cy, cx] + L_FREE)
        if 0 <= ex < SIZE and 0 <= ey < SIZE:
            logodds[ey, ex] = min(L_MAX, logodds[ey, ex] + L_OCC)


def bresenham(x0, y0, x1, y1):
    """Yield cells from (x0,y0) up to but NOT including endpoint."""
    dx = abs(x1 - x0); dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while (x0, y0) != (x1, y1):
        yield x0, y0
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 < dx:
            err += dx; y0 += sy


def save_map(logodds, traj):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    prob = 1.0 - 1.0 / (1.0 + np.exp(logodds))   # occupancy prob
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(prob, cmap="gray_r", origin="lower", vmin=0, vmax=1,
              extent=[-ORIGIN * RES, (SIZE - ORIGIN) * RES,
                      -ORIGIN * RES, (SIZE - ORIGIN) * RES])
    if traj:
        tr = np.array(traj)
        ax.plot(tr[:, 0], tr[:, 1], "-", color="tab:red", lw=1.2, label="path")
        ax.plot(tr[-1, 0], tr[-1, 1], "^", color="lime", ms=10, label="robot")
    ax.set_title("RPLIDAR C1 lidar-only SLAM map")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(loc="upper right")
    fig.savefig(OUT_PNG, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main():
    import time
    lidar = RPLidarC1().connect()
    logodds = np.zeros((SIZE, SIZE), dtype=np.float32)
    pose = (0.0, 0.0, 0.0)
    traj = [(0.0, 0.0)]
    prev = None
    frames = 0
    try:
        lidar.stop()
        h = lidar.get_health()
        print(f"Health: {h['status_str']}. Building map for {DURATION:.0f}s "
              f"(Ctrl+C to stop). Drive the car SLOWLY.\n")
        t0 = time.time()
        for scan in lidar.iter_scans(min_points=120):
            pts = scan_to_xy(scan)
            if len(pts) < 20:
                continue
            if prev is not None:
                pose, ok = icp(pts, prev, pose)
            world = transform(pts, pose)
            integrate(logodds, world, pose)
            prev = world               # match next scan against the world map cloud
            traj.append((pose[0], pose[1]))
            frames += 1
            if frames % 10 == 0:
                el = time.time() - t0
                print(f"  {el:5.1f}s | frame {frames:4d} | "
                      f"pose x={pose[0]:+.2f} y={pose[1]:+.2f} "
                      f"th={math.degrees(pose[2]):+6.1f}deg | {len(pts)} pts")
                save_map(logodds, traj)     # live-ish updates
            if time.time() - t0 > DURATION:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        lidar.disconnect()
        save_map(logodds, traj)
        print(f"\nFrames: {frames} | Saved map -> {OUT_PNG}")


if __name__ == "__main__":
    main()
