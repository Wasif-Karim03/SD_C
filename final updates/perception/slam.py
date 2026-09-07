#!/usr/bin/env python3
"""
perception/slam.py — 2D LiDAR SLAM (ICP scan-matching + log-odds occupancy grid).

Ported from the proven LiDAR/slam_test.py. Each scan is matched against the
accumulated map cloud with point-to-point ICP to get the car's pose (so mapping
does NOT need wheel-odometry calibration), then folded into an occupancy grid.

  slam = LidarSLAM()
  pose = slam.add_scan(scan)     # scan = [(quality, angle_deg, dist_mm), ...]
  png  = slam.render_png()       # occupancy map as PNG bytes (walls dark)
  slam.save("maps/room")         # -> room.npy (grid) + room.png + room.meta.json

Drive SLOWLY so consecutive scans overlap — that's what ICP needs for a clean map.
"""
import os
import math
import json

import numpy as np
import cv2
from scipy.spatial import cKDTree

# --- map config (metres / cells) ---
RES = 0.05                  # m per cell
SIZE = 500                  # 500 -> 25 m x 25 m
ORIGIN = SIZE // 2          # car starts at grid center
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX = -5.0, 6.0
MAX_RANGE = 12.0
MIN_RANGE = 0.15


def scan_to_xy(scan):
    pts = []
    for _q, ang, dist in scan:
        r = dist / 1000.0
        if MIN_RANGE < r < MAX_RANGE:
            a = math.radians(ang)
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))


def transform(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + np.array([x, y])


def icp(src, dst, pose, iters=25, tol=1e-4, max_corr=0.6):
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
        x, y = (Rr @ np.array([x, y])) + t
        th += dth
        if abs(dth) < tol and np.linalg.norm(t) < tol:
            break
    return (x, y, th), True


def _bresenham(x0, y0, x1, y1):
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


class LidarSLAM:
    def __init__(self):
        self.grid = np.zeros((SIZE, SIZE), dtype=np.float32)
        self.pose = (0.0, 0.0, 0.0)
        self.traj = [(0.0, 0.0)]
        self._prev = None
        self.frames = 0

    def add_scan(self, scan):
        pts = scan_to_xy(scan)
        if len(pts) < 20:
            return self.pose
        if self._prev is not None:
            self.pose, _ = icp(pts, self._prev, self.pose)
        world = transform(pts, self.pose)
        self._integrate(world)
        self._prev = world
        self.traj.append((self.pose[0], self.pose[1]))
        self.frames += 1
        return self.pose

    def _integrate(self, pts_world):
        sx = int(round(self.pose[0] / RES)) + ORIGIN
        sy = int(round(self.pose[1] / RES)) + ORIGIN
        gx = np.round(pts_world[:, 0] / RES).astype(int) + ORIGIN
        gy = np.round(pts_world[:, 1] / RES).astype(int) + ORIGIN
        g = self.grid
        for ex, ey in zip(gx, gy):
            for cx, cy in _bresenham(sx, sy, ex, ey):
                if 0 <= cx < SIZE and 0 <= cy < SIZE:
                    g[cy, cx] = max(L_MIN, g[cy, cx] + L_FREE)
            if 0 <= ex < SIZE and 0 <= ey < SIZE:
                g[ey, ex] = min(L_MAX, g[ey, ex] + L_OCC)

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.grid))

    # ---- world <-> grid-pixel helpers (for the web UI clicks later) ---- #
    @staticmethod
    def world_to_px(x, y, out_size):
        col = (x / RES + ORIGIN) * out_size / SIZE
        row = (SIZE - 1 - (y / RES + ORIGIN)) * out_size / SIZE   # flipud
        return col, row

    @staticmethod
    def px_to_world(col, row, out_size):
        gx = col * SIZE / out_size
        gy = SIZE - 1 - row * SIZE / out_size
        return (gx - ORIGIN) * RES, (gy - ORIGIN) * RES

    def render_png(self, out_size=500):
        prob = self.prob()
        img = ((1.0 - prob) * 255).astype(np.uint8)   # occupied dark, free light, unknown ~127
        img = np.flipud(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if out_size != SIZE:
            img = cv2.resize(img, (out_size, out_size), interpolation=cv2.INTER_NEAREST)
        # trajectory
        if len(self.traj) > 1:
            poly = np.array([self.world_to_px(x, y, out_size) for x, y in self.traj],
                            dtype=np.int32)
            cv2.polylines(img, [poly], False, (0, 90, 230), 1)
        # robot marker
        rc, rr = self.world_to_px(self.pose[0], self.pose[1], out_size)
        cv2.circle(img, (int(rc), int(rr)), 5, (90, 220, 90), -1)
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else None

    def save(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        np.save(path + ".npy", self.grid)
        png = self.render_png()
        if png:
            with open(path + ".png", "wb") as fh:
                fh.write(png)
        with open(path + ".meta.json", "w") as fh:
            json.dump({"res": RES, "size": SIZE, "origin": ORIGIN,
                       "frames": self.frames, "pose": self.pose}, fh)
        return path
