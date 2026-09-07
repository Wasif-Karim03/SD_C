#!/usr/bin/env python3
"""
mapper.py — Step 3: fuse many depth frames into ONE world map.

Two subcommands:
  record  — capture a timestamped frame sequence to a folder (for later replay
            with real VESC/GPS odometry).
  build   — turn a recorded sequence + a pose track into a single fused 3D map
            (voxel-downsampled colored point cloud) + previews.

Key ideas that make single-camera mapping work on the car:

1. SCALE-LOCK by per-frame normalization. The metric model's absolute meters drift
   frame-to-frame (measured: 0.2 m one frame, 1.3 m another). We do NOT trust them.
   Instead each frame's depth is normalized to a common reference (median -> 1.0),
   giving mutually-consistent relative geometry; a single global scale (later from
   VESC distance / GPS) converts the whole map to real meters.

2. POSE placement. Each frame's camera-frame cloud is rotated+translated into the
   world using the robot pose (x, z, yaw) at capture time, then merged. Pose comes
   from odometry; here we can also synthesize a motion to validate the math.

3. VOXEL fusion. A numpy dict-of-voxels grid (no Open3D) accumulates color per
   3 cm cell, so overlapping frames dedup instead of exploding the point count.

Camera frame: +x right, +y down, +z forward. Robot drives on the x-z ground plane,
yaw about the +y (down) axis; yaw=0 means camera +z is world +z.
"""

import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Camera Control"))
from threaded_camera import ThreadedCamera  # noqa: E402

from depth_engine import DepthEngine, INDOOR, OUTDOOR  # noqa: E402
from pointcloud import (backproject, default_intrinsics, render_bev,  # noqa: E402
                        render_view, write_ply)


# ----------------------------------------------------------------------------- #
#  Pose helpers (ground-plane SE(2): x, z, yaw)                                  #
# ----------------------------------------------------------------------------- #
def pose_matrix(x, z, yaw):
    """4x4 that maps camera-frame points into the world for robot pose (x,z,yaw)."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], np.float32)
    T[0, 3], T[2, 3] = x, z
    return T


def transform(xyz, T):
    return xyz @ T[:3, :3].T + T[:3, 3]


# ----------------------------------------------------------------------------- #
#  Voxel map                                                                    #
# ----------------------------------------------------------------------------- #
class VoxelMap:
    """Accumulate colored points into a voxel grid (dedups overlaps).

    Voxels are keyed by a single packed int64 so per-frame reduction is vectorized
    (np.unique + bincount) and only the small set of unique cells touches Python —
    fast enough for a live mapping loop.
    """

    OFF = 1 << 20          # coordinate offset to keep packed keys non-negative
    S = 1 << 21            # per-axis span (supports +/-1M voxels/axis)

    def __init__(self, voxel=0.03):
        self.voxel = voxel
        self.sum = {}      # packed key -> np.float64[3] color sum
        self.cnt = {}      # packed key -> int count

    def _pack(self, xyz):
        ijk = np.floor(xyz / self.voxel).astype(np.int64) + self.OFF
        return (ijk[:, 0] * self.S + ijk[:, 1]) * self.S + ijk[:, 2]

    def add(self, xyz, rgb):
        if len(xyz) == 0:
            return
        key = self._pack(xyz)
        uk, inv = np.unique(key, return_inverse=True)
        csum = np.zeros((len(uk), 3), np.float64)
        np.add.at(csum, inv, rgb.astype(np.float64))
        ccnt = np.bincount(inv, minlength=len(uk))
        for k, s, c in zip(uk.tolist(), csum, ccnt.tolist()):
            if k in self.cnt:
                self.sum[k] += s
                self.cnt[k] += c
            else:
                self.sum[k] = s
                self.cnt[k] = c

    def cloud(self):
        if not self.cnt:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
        keys = np.fromiter(self.cnt.keys(), np.int64, len(self.cnt))
        kz = keys % self.S - self.OFF
        ky = (keys // self.S) % self.S - self.OFF
        kx = keys // (self.S * self.S) - self.OFF
        xyz = (np.stack([kx, ky, kz], 1).astype(np.float32) + 0.5) * self.voxel
        sums = np.array([self.sum[int(k)] for k in keys], np.float64)
        cnts = np.array([self.cnt[int(k)] for k in keys], np.float64)[:, None]
        rgb = (sums / cnts).astype(np.uint8)
        return xyz, rgb


# ----------------------------------------------------------------------------- #
#  record                                                                       #
# ----------------------------------------------------------------------------- #
def cmd_record(args):
    os.makedirs(args.out, exist_ok=True)
    cam = ThreadedCamera().start()
    time.sleep(1.5)
    manifest, seq, saved = [], None, 0
    t0 = time.time()
    print(f"recording up to {args.n} frames to {args.out}/ ...")
    try:
        while saved < args.n:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            ts = time.time() - t0
            name = f"frame_{saved:04d}.jpg"
            cv2.imwrite(os.path.join(args.out, name), frame)
            manifest.append({"file": name, "t": round(ts, 4)})
            saved += 1
            if args.interval:
                time.sleep(args.interval)
    finally:
        cam.release()
    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"saved {saved} frames + manifest.json")
    sys.stdout.flush()
    os._exit(0)


# ----------------------------------------------------------------------------- #
#  build                                                                        #
# ----------------------------------------------------------------------------- #
def load_poses(args, n):
    """Return a list of (x, z, yaw) poses of length n."""
    if args.poses and os.path.exists(args.poses):
        arr = np.loadtxt(args.poses).reshape(-1, 3)
        return [tuple(r) for r in arr[:n]]
    if args.synthetic:  # straight-ish drive: forward speed + yaw rate per frame
        poses, x, z, yaw = [], 0.0, 0.0, 0.0
        for _ in range(n):
            poses.append((x, z, yaw))
            yaw += args.yaw_rate
            x += args.speed * np.sin(yaw)
            z += args.speed * np.cos(yaw)
        return poses
    return [(0.0, 0.0, 0.0)] * n  # stationary (overlap test)


def cmd_build(args):
    files = sorted(glob.glob(os.path.join(args.seq, "frame_*.jpg")))
    if not files:
        print("no frames in", args.seq); return
    files = files[:: args.every]
    frame0 = cv2.imread(files[0])
    h, w = frame0.shape[:2]
    fx, fy, cx, cy = default_intrinsics(w, h, args.hfov)

    eng = DepthEngine(model_id=OUTDOOR if args.outdoor else INDOOR, half=True)
    poses = load_poses(args, len(files))
    vmap = VoxelMap(voxel=args.voxel)

    for i, (fp, pose) in enumerate(zip(files, poses)):
        frame = cv2.imread(fp)
        depth = eng.infer(frame)
        if args.normalize == "median":
            med = np.median(depth[np.isfinite(depth) & (depth > 0)])
            depth = depth / max(med, 1e-6) * args.scale   # median -> args.scale meters
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        xyz, cols = backproject(depth, rgb, fx, fy, cx, cy,
                                stride=args.stride, zmin=0.05, zmax=args.zmax)
        xyz = transform(xyz, pose_matrix(*pose))
        vmap.add(xyz, cols)
        print(f"  [{i+1}/{len(files)}] pose(x={pose[0]:.2f},z={pose[1]:.2f},"
              f"yaw={np.degrees(pose[2]):.0f})  +{len(xyz):,}pts  map={len(vmap.cnt):,} voxels")

    xyz, cols = vmap.cloud()
    write_ply(f"{args.out}.ply", xyz, cols)
    cv2.imwrite(f"{args.out}_bev.png", render_bev(xyz, cols, size=700))
    cv2.imwrite(f"{args.out}_view.png", render_view(xyz, cols, size=700))
    print(f"MAP: {len(xyz):,} voxels  extent "
          f"x[{xyz[:,0].min():.1f},{xyz[:,0].max():.1f}] "
          f"z[{xyz[:,2].min():.1f},{xyz[:,2].max():.1f}] m")
    print(f"wrote {args.out}.ply / _bev.png / _view.png")


def main():
    ap = argparse.ArgumentParser(description="Step 3: fuse frames into one 3D map")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("record", help="capture a frame sequence")
    r.add_argument("--out", default="scratch_seq")
    r.add_argument("--n", type=int, default=30)
    r.add_argument("--interval", type=float, default=0.0, help="sec between frames")
    r.set_defaults(func=cmd_record)

    b = sub.add_parser("build", help="fuse a sequence into a map")
    b.add_argument("--seq", default="scratch_seq")
    b.add_argument("--out", default="scratch_map")
    b.add_argument("--every", type=int, default=1, help="use every Nth frame")
    b.add_argument("--stride", type=int, default=3, help="pixel subsample per frame")
    b.add_argument("--voxel", type=float, default=0.03, help="voxel size (m)")
    b.add_argument("--zmax", type=float, default=8.0, help="max depth kept (m)")
    b.add_argument("--hfov", type=float, default=60.0)
    b.add_argument("--outdoor", action="store_true")
    b.add_argument("--normalize", choices=["median", "none"], default="median",
                   help="per-frame scale-lock (kills metric drift)")
    b.add_argument("--scale", type=float, default=1.0,
                   help="global scale: median depth -> this many meters")
    b.add_argument("--poses", help="Nx3 text file of (x z yaw) poses")
    b.add_argument("--synthetic", action="store_true", help="synthesize a drive")
    b.add_argument("--speed", type=float, default=0.15, help="m forward per frame (synthetic)")
    b.add_argument("--yaw-rate", dest="yaw_rate", type=float, default=0.0,
                   help="rad yaw per frame (synthetic)")
    b.set_defaults(func=cmd_build)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
