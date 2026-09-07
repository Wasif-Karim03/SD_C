#!/usr/bin/env python3
"""
live_depth.py — live camera + metric depth preview (a testable milestone).

Streams the USB camera through Depth-Anything V2 (metric) and shows, side by side,
the raw frame and a colorized depth heatmap. A crosshair reports the depth in
METERS at the center pixel — aim at a wall/object you can measure to see how close
the model's absolute scale is (that gap is what VESC/GPS odometry will calibrate
away in Step 3).

Works headless: if there's no X display it writes a rolling JPEG you can open over
NoMachine / scp instead of using a live window.

  python3 live_depth.py                 # live window if DISPLAY, else rolling file
  python3 live_depth.py --outdoor       # outdoor metric model (0-80 m)
  python3 live_depth.py --save out.jpg  # force rolling-file mode to this path
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# Reuse the project's low-latency camera (in ../Camera Control).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Camera Control"))
from threaded_camera import ThreadedCamera  # noqa: E402

from depth_engine import DepthEngine, INDOOR, OUTDOOR  # noqa: E402


def draw_overlay(frame, depth, colored, fps):
    h, w = depth.shape
    cy, cx = h // 2, w // 2
    center_m = float(np.median(depth[cy - 3:cy + 4, cx - 3:cx + 4]))

    # Crosshair + center distance on the depth panel.
    cv2.drawMarker(colored, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(colored, f"{center_m:.2f} m", (cx + 12, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    panel = np.hstack([frame, colored])
    cv2.putText(panel, f"{fps:4.1f} FPS   near..far {depth.min():.1f}-{depth.max():.1f} m",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return panel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdoor", action="store_true", help="outdoor metric model")
    ap.add_argument("--fp32", action="store_true", help="disable half precision")
    ap.add_argument("--save", default=None, help="rolling-JPEG path (forces headless)")
    args = ap.parse_args()

    has_display = bool(os.environ.get("DISPLAY")) and args.save is None
    save_path = args.save or "scratch_live_depth.jpg"

    print("Loading depth model ...")
    eng = DepthEngine(model_id=OUTDOOR if args.outdoor else INDOOR, half=not args.fp32)
    cam = ThreadedCamera().start()
    time.sleep(1.5)  # let auto/manual exposure settle

    mode = "live window" if has_display else f"rolling file -> {save_path}"
    print(f"Running ({mode}).  Ctrl-C to stop.")

    seq = None
    ema_fps = 0.0
    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            t0 = time.time()
            depth = eng.infer(frame)
            colored = eng.colorize(depth)
            inst = 1.0 / max(time.time() - t0, 1e-3)
            ema_fps = inst if ema_fps == 0 else 0.9 * ema_fps + 0.1 * inst

            panel = draw_overlay(frame, depth, colored, ema_fps)
            if has_display:
                cv2.imshow("frame | metric depth", panel)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            else:
                cv2.imwrite(save_path, panel)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cam.release()
            if has_display:
                cv2.destroyAllWindows()
        except Exception:
            pass
        print("\nstopped.")
        # UVC cap.release()/CUDA teardown can block on this device; the daemon
        # camera thread is already stopped, so hard-exit for an instant clean quit.
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
