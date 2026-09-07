#!/usr/bin/env python3
"""
live_map.py — build a 3D map LIVE by moving the camera (no external sensors).

This is poor-man's monocular SLAM: camera motion is estimated from the images
themselves (visual odometry), so you can walk the camera around and watch a map
grow — before any VESC/GPS wiring.

Per frame:
  1. Depth-Anything V2 -> metric depth, scale-locked (median -> 1 m) so scale is
     consistent frame to frame.
  2. ORB features matched against the previous frame. The previous frame's depth
     gives each matched point a 3D position, so solvePnPRansac recovers the
     camera's motion IN METERS (not just direction) -> global 6-DoF pose.
  3. The current frame's colored cloud is transformed by the global pose and fused
     into the voxel map.

If VO is too weak on a frame (few matches / PnP fails) that frame is skipped rather
than smearing the map. A rolling preview (map BEV + angled render + current depth)
is written so you can watch it build headless; with a display you get live windows.

  python3 live_map.py                    # live from camera
  python3 live_map.py --replay scratch_seq   # offline test on a recorded sequence
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Camera Control"))
from threaded_camera import ThreadedCamera  # noqa: E402

from depth_engine import DepthEngine, INDOOR, OUTDOOR  # noqa: E402
from mapper import VoxelMap, transform  # noqa: E402
from pointcloud import (backproject, default_intrinsics, render_bev,  # noqa: E402
                        render_view, write_ply)


def inv_rigid(T):
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4, dtype=np.float32)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


class VisualOdometry:
    """Metric frame-to-frame VO using ORB matches + depth-backed PnP."""

    def __init__(self, K, min_matches=20, min_inliers=12,
                 max_step=0.5, max_rot_deg=25.0, min_inlier_ratio=0.35):
        self.K = K
        self.orb = cv2.ORB_create(1200)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.min_matches = min_matches
        self.min_inliers = min_inliers
        self.max_step = max_step            # max plausible translation per frame (m)
        self.max_rot = np.radians(max_rot_deg)  # max plausible rotation per frame
        self.min_inlier_ratio = min_inlier_ratio
        self.prev = None  # (kp, des, depth)
        self.reject = None  # last rejection reason, for logging

    def _match(self, desA, desB):
        pairs = self.bf.knnMatch(desA, desB, k=2)
        good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.75 * n.distance]
        return good

    def track(self, gray, depth):
        """Return relative motion M (x_prev-cam -> x_cur-cam) as 4x4, or None."""
        kp, des = self.orb.detectAndCompute(gray, None)
        result = None
        if self.prev is not None and des is not None and self.prev[1] is not None:
            kpA, desA, depthA = self.prev
            good = self._match(desA, des)
            if len(good) >= self.min_matches:
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                obj, img = [], []
                for m in good:
                    ua, va = kpA[m.queryIdx].pt
                    z = float(depthA[int(va), int(ua)])
                    if 0.05 < z < 30:
                        obj.append([(ua - cx) * z / fx, (va - cy) * z / fy, z])
                        img.append(kp[m.trainIdx].pt)
                if len(obj) >= self.min_matches:
                    obj = np.array(obj, np.float32)
                    img = np.array(img, np.float32)
                    ok, rvec, tvec, inl = cv2.solvePnPRansac(
                        obj, img, self.K, None, reprojectionError=2.0,
                        iterationsCount=150, flags=cv2.SOLVEPNP_ITERATIVE)
                    n_inl = 0 if inl is None else len(inl)
                    if ok and n_inl >= self.min_inliers and \
                            n_inl >= self.min_inlier_ratio * len(obj):
                        t_norm = float(np.linalg.norm(tvec))
                        rot = float(np.linalg.norm(rvec))
                        # Motion sanity gate: reject physically-impossible jumps
                        # (bad matches -> huge/degenerate PnP). Skip -> self-recover.
                        if t_norm <= self.max_step and rot <= self.max_rot:
                            R, _ = cv2.Rodrigues(rvec)
                            M = np.eye(4, dtype=np.float32)
                            M[:3, :3] = R
                            M[:3, 3] = tvec.ravel()
                            result = (M, n_inl)
                        else:
                            self.reject = f"jump t={t_norm:.1f}m r={np.degrees(rot):.0f}deg"
                    else:
                        self.reject = f"weak inl={n_inl}/{len(obj)}"
        self.prev = (kp, des, depth)
        return result


def save_previews(vmap, depth_eng, depth, frame, out, live_window):
    xyz, cols = vmap.cloud()
    if len(xyz) > 20:
        bev = render_bev(xyz, cols, size=640)
        view = render_view(xyz, cols, size=640)
    else:
        bev = view = np.zeros((640, 640, 3), np.uint8)

    # A single, easy-to-read dashboard: camera | depth on top, 3D map | floor plan
    # below — so you can actually SEE the scene and the map together.
    cam = cv2.resize(frame, (640, 480))
    dep = cv2.resize(depth_eng.colorize(depth), (640, 480))
    for im, lbl in ((cam, "camera"), (dep, "depth")):
        cv2.putText(im, lbl, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    top = np.hstack([cam, dep])
    bot = np.hstack([cv2.resize(view, (640, 480)), cv2.resize(bev, (640, 480))])
    dash = np.vstack([top, bot])
    cv2.putText(dash, f"map voxels: {len(xyz):,}", (10, 470),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imwrite(f"{out}_dashboard.png", dash)
    cv2.imwrite(f"{out}_bev.png", bev)
    cv2.imwrite(f"{out}_view.png", view)
    if live_window:
        cv2.imshow("live map (camera | depth / 3D | floorplan)", dash)
        return cv2.waitKey(1) & 0xFF in (27, ord("q"))
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", help="folder of frame_*.jpg to replay instead of camera")
    ap.add_argument("--outdoor", action="store_true")
    ap.add_argument("--out", default="scratch_livemap")
    ap.add_argument("--stride", type=int, default=4, help="pixel subsample for mapping")
    ap.add_argument("--voxel", type=float, default=0.04)
    ap.add_argument("--scale", type=float, default=1.0, help="median depth -> meters")
    ap.add_argument("--zmax", type=float, default=6.0)
    ap.add_argument("--hfov", type=float, default=60.0)
    ap.add_argument("--every", type=int, default=2, help="preview every N integrated frames")
    ap.add_argument("--max", type=int, default=100000, help="stop after N frames")
    ap.add_argument("--blur-frac", dest="blur_frac", type=float, default=0.5,
                    help="skip frames sharper-than this frac of recent median (0=off)")
    ap.add_argument("--kf-dist", dest="kf_dist", type=float, default=0.04,
                    help="min move (m) since last keyframe before fusing")
    ap.add_argument("--kf-rot", dest="kf_rot", type=float, default=4.0,
                    help="min rotation (deg) since last keyframe before fusing")
    ap.add_argument("--exposure", type=int, default=None,
                    help="set exposure_time_absolute (100us units); short=less blur")
    ap.add_argument("--gain", type=float, default=1.0,
                    help="digital brightness gain (recovers features at short exposure)")
    ap.add_argument("--gain-beta", dest="gain_beta", type=int, default=10,
                    help="digital gain offset added after scaling")
    args = ap.parse_args()

    replay_files = sorted(glob.glob(os.path.join(args.replay, "frame_*.jpg"))) if args.replay else None
    src0 = cv2.imread(replay_files[0]) if replay_files else None
    if replay_files is None:
        cam = ThreadedCamera().start(); time.sleep(1.5)
        if args.exposure:
            import subprocess
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c",
                            f"auto_exposure=1,exposure_time_absolute={args.exposure}"])
            time.sleep(0.3)
        src0, _ = cam.read(wait=True)
    h, w = src0.shape[:2]
    fx, fy, cx, cy = default_intrinsics(w, h, args.hfov)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float32)

    print("loading depth model ...")
    eng = DepthEngine(model_id=OUTDOOR if args.outdoor else INDOOR, half=True)
    vo = VisualOdometry(K)
    vmap = VoxelMap(voxel=args.voxel)
    T_wc = np.eye(4, dtype=np.float32)    # camera-to-world (global pose)
    has_display = bool(os.environ.get("DISPLAY"))

    print(f"mapping ({'replay' if replay_files else 'LIVE camera'}). "
          f"preview -> {args.out}_bev/view/depth.png")
    seq, i, integrated, skipped, blurred = None, 0, 0, 0, 0
    traj = []
    sharp_hist = []
    last_kf = np.eye(4, dtype=np.float32)   # pose at last mapped keyframe
    have_kf = False
    err_count = 0
    try:
        while i < args.max:
          try:
            if replay_files is not None:
                if i >= len(replay_files):
                    break
                frame = cv2.imread(replay_files[i])
            else:
                frame, seq = cam.read(wait=True, last_seq=seq)
                if frame is None:
                    continue
            i += 1

            # Digital gain: with a short exposure (less motion blur) the raw image
            # is dark; boost it so depth + ORB still see structure.
            if args.gain != 1.0:
                frame = cv2.convertScaleAbs(frame, alpha=args.gain, beta=args.gain_beta)

            # --- Blur gate (cheap, runs before the expensive depth step) --------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
            sharp_hist.append(sharp)
            if len(sharp_hist) > 40:
                sharp_hist.pop(0)
            if len(sharp_hist) >= 8:
                thr = args.blur_frac * float(np.median(sharp_hist))
                if sharp < thr:
                    blurred += 1
                    if blurred % 15 == 1:
                        print(f"  f{i} BLUR-skip (sharp {sharp:.0f}<{thr:.0f})  blur={blurred}")
                    continue   # keep vo.prev as the last SHARP frame

            depth = eng.infer(frame)
            med = np.median(depth[np.isfinite(depth) & (depth > 0)])
            depth = depth / max(med, 1e-6) * args.scale     # scale-lock

            mv = vo.track(gray, depth)
            if mv is None:
                if have_kf:                # not the first frame -> tracking lost
                    skipped += 1
                    if skipped % 10 == 1:
                        print(f"  f{i} SKIP ({vo.reject})  int={integrated} skip={skipped}")
                    if not has_display and skipped % args.every == 0:
                        save_previews(vmap, eng, depth, frame, args.out, False)
                    continue
            else:
                M, ninl = mv
                T_wc = T_wc @ inv_rigid(M)   # advance global pose

            # --- Keyframe gate: only fuse after real movement (less redundant noise)
            rel = inv_rigid(last_kf) @ T_wc
            moved = float(np.linalg.norm(rel[:3, 3]))
            rot = np.degrees(np.arccos(np.clip((np.trace(rel[:3, :3]) - 1) / 2, -1, 1)))
            if have_kf and moved < args.kf_dist and rot < args.kf_rot:
                continue   # tracked but too close to last keyframe -> skip mapping

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            xyz, cols = backproject(depth, rgb, fx, fy, cx, cy,
                                    stride=args.stride, zmin=0.05, zmax=args.zmax)
            vmap.add(transform(xyz, T_wc), cols)
            integrated += 1
            last_kf = T_wc.copy()
            have_kf = True
            traj.append(T_wc[:3, 3].copy())

            pos = T_wc[:3, 3]
            print(f"  f{i} kf={integrated} skip={skipped} blur={blurred} "
                  f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})m voxels={len(vmap.cnt):,}")

            if integrated % args.every == 0:
                if save_previews(vmap, eng, depth, frame, args.out, has_display):
                    break
          except Exception as e:
              # One bad frame (v4l2 glitch, cv2 hiccup) must not kill the whole run.
              err_count += 1
              if err_count % 20 == 1:
                  print(f"  f{i} frame-error ({type(e).__name__}: {e}) — continuing")
              continue
    except KeyboardInterrupt:
        pass
    finally:
        if replay_files is None:
            try: cam.release()
            except Exception: pass
        xyz, cols = vmap.cloud()
        if len(xyz):
            write_ply(f"{args.out}.ply", xyz, cols)
            cv2.imwrite(f"{args.out}_bev.png", render_bev(xyz, cols, size=700))
            cv2.imwrite(f"{args.out}_view.png", render_view(xyz, cols, size=700))
        if len(traj) > 1:
            t = np.array(traj)
            print(f"trajectory length ~{np.linalg.norm(np.diff(t, axis=0), axis=1).sum():.2f} m "
                  f"over {integrated} frames")
        print(f"DONE: {len(xyz):,} voxels | {integrated} keyframes, "
              f"{skipped} track-lost, {blurred} blur-skipped -> {args.out}.ply")
        if has_display:
            cv2.destroyAllWindows()
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
