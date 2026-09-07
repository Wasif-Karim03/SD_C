#!/usr/bin/env python3
"""
calibrate_camera.py — intrinsic calibration for the USB camera.

Live-capture a checkerboard from many angles, then compute the camera matrix
and distortion coefficients and save them for undistortion / metric geometry.

Workflow:
  1. Print checkerboard_9x6_25mm.pdf at 100%, tape it flat onto something rigid.
  2. Measure one square with a ruler; pass it as --square-mm if not 25.0.
  3. Run this script. Hold the board in view; when corners are detected they are
     drawn in colour. Press SPACE (or 'c') to capture that view.
  4. Capture ~15 views: near/far, tilted left/right/up/down, and with the board
     in each corner of the frame (not just centred). Variety matters more than
     count — flat-on, centred-only captures give a poor calibration.
  5. Press 'g' to compute. Results print and save to calibration.npz / .yaml.
     An undistorted preview opens so you can eyeball the result.

Keys:  SPACE / c = capture    g = compute    u = toggle undistort preview
       d = delete last capture    q = quit

Uses the system OpenCV (JetPack). No pip installs.
"""

import argparse
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("ERROR: OpenCV (cv2) not importable — install the JetPack build.")

# Reuse camera detection + exposure handling from the preview foundation.
import live_preview as lp


def build_object_points(cols, rows, square_mm):
    """3D coordinates of the inner corners on the (flat, Z=0) board, in mm."""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm
    return objp


def find_corners(gray, pattern_size):
    """Return (found, corners) using the robust SB detector with a fallback."""
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
    if found:
        return True, corners
    # Fallback to the classic detector + sub-pixel refine.
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return found, corners


def save_results(path_stem, K, dist, image_size, rms, square_mm,
                 pattern_size, n_views, per_view_err):
    """Save calibration to .npz (authoritative) and .yaml (human-readable)."""
    npz = path_stem + ".npz"
    np.savez(npz, camera_matrix=K, dist_coeffs=dist,
             image_width=image_size[0], image_height=image_size[1],
             rms_reproj_error=rms, square_mm=square_mm,
             pattern_cols=pattern_size[0], pattern_rows=pattern_size[1],
             num_views=n_views)

    def mat(rows_):
        return "\n".join("    - [" + ", ".join(f"{v:.8g}" for v in r) + "]"
                         for r in rows_)

    yaml = path_stem + ".yaml"
    with open(yaml, "w") as f:
        f.write("# Camera intrinsic calibration (OpenCV pinhole model)\n")
        f.write(f"image_width: {image_size[0]}\n")
        f.write(f"image_height: {image_size[1]}\n")
        f.write(f"square_size_mm: {square_mm}\n")
        f.write(f"pattern_inner_cols: {pattern_size[0]}\n")
        f.write(f"pattern_inner_rows: {pattern_size[1]}\n")
        f.write(f"num_views: {n_views}\n")
        f.write(f"rms_reproj_error_px: {rms:.6f}\n")
        f.write("camera_matrix:  # 3x3 [[fx,0,cx],[0,fy,cy],[0,0,1]]\n")
        f.write(mat(K) + "\n")
        f.write("dist_coeffs:  # [k1, k2, p1, p2, k3]\n")
        f.write("    - [" + ", ".join(f"{v:.8g}" for v in dist.ravel()) + "]\n")
    return npz, yaml


def main():
    ap = argparse.ArgumentParser(description="USB camera intrinsic calibration.")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="measured printed square size in mm")
    ap.add_argument("--min-views", type=int, default=12,
                    help="minimum captures before computing")
    ap.add_argument("--out", default="calibration", help="output file stem")
    args = ap.parse_args()

    pattern_size = (args.cols, args.rows)
    objp = build_object_points(args.cols, args.rows, args.square_mm)

    dev, info = lp.detect_usb_camera()
    index = int("".join(ch for ch in dev if ch.isdigit()) or "0")
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"ERROR: could not open {dev}. Is the live preview still "
                 "holding the camera? Stop it first (only one client allowed).")
    if info and info["resolutions"]:
        target = max((r for r in info["resolutions"] if r[0] <= 1280 and r[1] <= 720),
                     default=info["resolutions"][-1])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, target[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target[1])
    if lp.USE_MANUAL_EXPOSURE:
        lp.apply_manual_exposure(dev, lp.EXPOSURE)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    image_size = (w, h)
    save_dir = os.path.dirname(os.path.abspath(__file__))
    win = "Calibration capture"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    print(f"Streaming {dev} at {w}x{h}. Pattern {args.cols}x{args.rows} inner "
          f"corners, {args.square_mm} mm squares.")
    print("SPACE/c capture | g compute | u undistort | d delete last | q quit")

    objpoints, imgpoints = [], []
    K = dist = None
    show_undistort = False
    last_center = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                if cv2.waitKey(50) & 0xFF == ord("q"):
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, pattern_size)

            view = frame
            if show_undistort and K is not None:
                view = cv2.undistort(frame, K, dist)
            else:
                view = frame.copy()
                if found:
                    cv2.drawChessboardCorners(view, pattern_size, corners, found)

            status = "DETECTED" if found else "no board"
            color = (0, 255, 0) if found else (0, 0, 255)
            cv2.putText(view, f"{status}  captured: {len(imgpoints)}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3,
                        cv2.LINE_AA)
            cv2.putText(view, f"{status}  captured: {len(imgpoints)}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1,
                        cv2.LINE_AA)
            mode = "UNDISTORTED" if (show_undistort and K is not None) else ""
            if mode:
                cv2.putText(view, mode, (12, h - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
                            cv2.LINE_AA)
            cv2.imshow(win, view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key in (ord("c"), ord(" ")):
                if not found:
                    print("  (no board detected — not captured)")
                    continue
                center = corners.reshape(-1, 2).mean(axis=0)
                if last_center is not None and \
                        np.linalg.norm(center - last_center) < 15:
                    print("  (too similar to last view — move the board more)")
                    continue
                objpoints.append(objp.copy())
                imgpoints.append(corners)
                last_center = center
                print(f"  captured view {len(imgpoints)} "
                      f"(board center {center.astype(int)})")
            elif key == ord("d"):
                if imgpoints:
                    objpoints.pop(); imgpoints.pop()
                    last_center = None
                    print(f"  deleted; {len(imgpoints)} remain")
            elif key == ord("u"):
                if K is None:
                    print("  (compute first with 'g')")
                else:
                    show_undistort = not show_undistort
            elif key == ord("g"):
                if len(imgpoints) < args.min_views:
                    print(f"  need >= {args.min_views} views, have "
                          f"{len(imgpoints)}. Capture more.")
                    continue
                print(f"\nCalibrating from {len(imgpoints)} views...")
                rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, image_size, None, None)

                # Per-view reprojection error to flag bad captures.
                per_view = []
                for i in range(len(objpoints)):
                    proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                                K, dist)
                    err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
                    per_view.append(err)

                print(f"  RMS reprojection error: {rms:.4f} px "
                      f"({'good' if rms < 1.0 else 'high — recapture'})")
                print(f"  fx={K[0,0]:.2f} fy={K[1,1]:.2f} "
                      f"cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
                print(f"  dist (k1 k2 p1 p2 k3): "
                      f"{', '.join(f'{v:.4f}' for v in dist.ravel())}")
                worst = int(np.argmax(per_view))
                print(f"  worst view: #{worst+1} err={per_view[worst]:.3f} px "
                      f"(press 'd' near that pose to drop & recapture if high)")

                stem = os.path.join(save_dir, args.out)
                npz, yaml = save_results(stem, K, dist, image_size, rms,
                                         args.square_mm, pattern_size,
                                         len(imgpoints), per_view)
                print(f"  saved: {npz}\n         {yaml}")
                print("  press 'u' to toggle the undistorted preview.")
                show_undistort = True
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Calibration session ended.")


if __name__ == "__main__":
    main()
