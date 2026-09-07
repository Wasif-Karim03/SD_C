#!/usr/bin/env python3
"""
pointcloud.py — Step 2: metric depth frame -> colored 3D point cloud.

Back-projects every pixel through the pinhole camera model using the metric depth
from depth_engine, producing a colored 3D point cloud in CAMERA coordinates:

    x = (u - cx) * Z / fx     (right)
    y = (v - cy) * Z / fy     (down)
    z = Z                     (forward, metric depth in meters)

This is "one frame of the map" — Step 3 will place many of these into a single
world map using VESC/GPS pose. Outputs:
  - a standard binary .ply (open in MeshLab / CloudCompare on a desktop)
  - dependency-free preview PNGs (bird's-eye "floor plan" + an angled 3D render)
    so the geometry is viewable headless, without Open3D.

Camera intrinsics: the icspring USB cam is 640x480 with no calibration on file yet.
We default to an estimate from an assumed ~60 deg horizontal FOV. Run
../Camera\ Control/calibrate_camera.py with the checkerboard for accurate values;
pass them via --fx/--fy/--cx/--cy.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Camera Control"))
from threaded_camera import ThreadedCamera  # noqa: E402

from depth_engine import DepthEngine, INDOOR, OUTDOOR  # noqa: E402


def default_intrinsics(w, h, hfov_deg=60.0):
    """Estimate fx,fy,cx,cy from image size + assumed horizontal FOV."""
    fx = (w / 2) / np.tan(np.radians(hfov_deg) / 2)
    fy = fx  # assume square pixels
    return fx, fy, w / 2.0, h / 2.0


def backproject(depth, rgb, fx, fy, cx, cy, stride=2, zmin=0.05, zmax=15.0, scale=1.0):
    """Return (xyz Nx3, rgb Nx3 uint8) point cloud from a metric depth map."""
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    z = depth[vs, us].astype(np.float32) * scale
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    xyz = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    cols = rgb[vs, us].reshape(-1, 3)  # rgb already RGB order
    m = (xyz[:, 2] > zmin) & (xyz[:, 2] < zmax) & np.isfinite(xyz).all(1)
    return xyz[m], cols[m]


def write_ply(path, xyz, rgb):
    """Write a binary little-endian PLY (x y z r g b)."""
    n = len(xyz)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    ).encode()
    verts = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                               ("r", "u1"), ("g", "u1"), ("b", "u1")])
    verts["x"], verts["y"], verts["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    verts["r"], verts["g"], verts["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(header)
        f.write(verts.tobytes())


def _boost(img, alpha=2.2, beta=25, k=3):
    """Make a sparse point render VISIBLE: fatten points + brighten colors."""
    img = cv2.dilate(img, np.ones((k, k), np.uint8))     # 1px points -> kxk blobs
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)  # brighten dark colors


def render_bev(xyz, rgb, size=600, span=None):
    """Top-down bird's-eye view (X right, Z forward) — the room 'floor plan'."""
    img = np.zeros((size, size, 3), np.uint8)
    x, z = xyz[:, 0], xyz[:, 2]
    span = span or float(np.percentile(np.abs(np.concatenate([x, z])), 98)) * 2
    span = max(span, 0.5)
    px = ((x / span + 0.5) * size).astype(int)
    pz = ((1 - z / span) * size).astype(int)  # near (small z) at bottom
    ok = (px >= 0) & (px < size) & (pz >= 0) & (pz < size)
    img[pz[ok], px[ok]] = rgb[ok][:, ::-1]  # RGB->BGR for cv2
    img = _boost(img)
    cv2.putText(img, f"BEV top-down  span~{span:.1f}m", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    cv2.drawMarker(img, (size // 2, size - 4), (0, 255, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)
    return img


def render_view(xyz, rgb, size=600, yaw=0.35, pitch=0.25):
    """Splat the cloud onto a virtual pinhole camera rotated by yaw/pitch."""
    c = xyz.mean(0)
    p = xyz - c
    cy_, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cy_, 0, sy], [0, 1, 0], [-sy, 0, cy_]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    q = p @ Ry.T @ Rx.T
    extent = float(np.percentile(np.linalg.norm(p, axis=1), 95)) or 1.0
    q[:, 2] += extent * 3.0  # push scene in front of virtual cam
    f = size * 0.9
    valid = q[:, 2] > 1e-3
    q, col = q[valid], rgb[valid]
    u = (f * q[:, 0] / q[:, 2] + size / 2).astype(int)
    v = (f * q[:, 1] / q[:, 2] + size / 2).astype(int)
    ok = (u >= 0) & (u < size) & (v >= 0) & (v < size)
    order = np.argsort(-q[ok, 2])  # far first (painter's algorithm)
    img = np.zeros((size, size, 3), np.uint8)
    uu, vv, cc = u[ok][order], v[ok][order], col[ok][order][:, ::-1]
    img[vv, uu] = cc
    img = _boost(img)
    cv2.putText(img, "angled 3D render", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    return img


def capture_frame():
    cam = ThreadedCamera().start()
    time.sleep(1.5)
    frame, seq = None, None
    for _ in range(30):
        f, seq = cam.read(wait=True, last_seq=seq)
        if f is not None:
            frame = f
    cam.release()
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="use this image instead of live capture")
    ap.add_argument("--outdoor", action="store_true")
    ap.add_argument("--stride", type=int, default=2, help="pixel subsample (bigger=fewer points)")
    ap.add_argument("--scale", type=float, default=1.0, help="metric scale correction")
    ap.add_argument("--hfov", type=float, default=60.0, help="assumed horizontal FOV (deg)")
    ap.add_argument("--fx", type=float); ap.add_argument("--fy", type=float)
    ap.add_argument("--cx", type=float); ap.add_argument("--cy", type=float)
    ap.add_argument("--out", default="cloud", help="output basename")
    args = ap.parse_args()

    frame = cv2.imread(args.image) if args.image else capture_frame()
    if frame is None:
        print("no frame"); return
    h, w = frame.shape[:2]

    eng = DepthEngine(model_id=OUTDOOR if args.outdoor else INDOOR, half=True)
    depth = eng.infer(frame)

    fx, fy, cx, cy = default_intrinsics(w, h, args.hfov)
    if args.fx: fx = args.fx
    if args.fy: fy = args.fy
    if args.cx: cx = args.cx
    if args.cy: cy = args.cy

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    xyz, cols = backproject(depth, rgb, fx, fy, cx, cy, stride=args.stride, scale=args.scale)
    print(f"cloud: {len(xyz):,} points  intrinsics fx={fx:.0f} cx={cx:.0f}  "
          f"z {xyz[:,2].min():.2f}..{xyz[:,2].max():.2f} m")

    write_ply(f"{args.out}.ply", xyz, cols)
    cv2.imwrite(f"{args.out}_bev.png", render_bev(xyz, cols))
    cv2.imwrite(f"{args.out}_view.png", render_view(xyz, cols))
    cv2.imwrite(f"{args.out}_frame.png", frame)
    print(f"wrote {args.out}.ply / _bev.png / _view.png / _frame.png")


if __name__ == "__main__":
    main()
