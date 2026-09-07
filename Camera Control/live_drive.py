#!/usr/bin/env python3
"""
live_drive.py — indoor obstacle-avoidance perception (the main driving view).

For indoor driving with no lanes: estimate free space with monocular depth and
output a reactive STEERING signal toward open space (+ a BLOCKED warning when the
path ahead is too close), regardless of what the obstacles are. YOLO11n runs on
top to LABEL the known objects ("understand objects"), but avoidance does not
depend on it — so the car won't drive into a wall just because YOLO has no class
for it.

Pipeline (all on the newest frame, low latency):
  threaded capture -> MiDaS depth (GPU) -> free-space plan -> [optional YOLO] ->
  overlay (clearance bars, heading arrow, steer/blocked, depth inset) + FPS.

Keys:  q quit   s save   o toggle object labels (YOLO)   i toggle depth inset
       b toggle blocked-stop overlay text

Default: depth avoidance + YOLO both ON (~10-13 FPS). Turn YOLO off ('o') for
faster pure avoidance (~18 FPS, camera-bound).
"""

import os
import sys
import time
from collections import deque

import cv2

import live_preview as lp
from threaded_camera import ThreadedCamera
from depth_nav import DepthNavigator
from live_detect import draw_detections, MODEL, CONF, IMGSZ, DEVICE

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("ERROR: ultralytics not importable. Run the Stage 1 install first.")


def main():
    try:
        cam = ThreadedCamera().start()
    except RuntimeError as e:
        sys.exit(f"ERROR: {e}")
    dev, w, h = cam.dev, cam.width, cam.height

    print("Loading MiDaS depth navigator (GPU) ...")
    nav = DepthNavigator(device=DEVICE)
    print(f"Loading {MODEL} for object labels ...")
    model = YOLO(MODEL, task="detect")
    names = model.names
    # Warm up both networks so the first frames aren't a stall.
    import numpy as np
    blank = np.zeros((h, w, 3), dtype=np.uint8)
    nav.estimate(blank)
    model.predict(blank, device=DEVICE, imgsz=IMGSZ, conf=CONF, verbose=False)

    print(f"Streaming {dev} at {w}x{h}. q=quit s=save o=objects i=inset b=stoptext")
    save_dir = os.path.dirname(os.path.abspath(__file__))
    win = "Indoor Drive - Depth Avoidance"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    times = deque(maxlen=30)
    saved = 0
    yolo_on = True
    inset_on = True
    seq = 0

    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                if cv2.waitKey(50) & 0xFF == ord("q"):
                    break
                continue
            frame = frame.copy()

            # --- analyze the CLEAN frame: depth (always) + YOLO (optional) ---
            depth = nav.estimate(frame)
            plan = nav.plan(depth)
            result = None
            if yolo_on:
                result = model.predict(frame, device=DEVICE, imgsz=IMGSZ,
                                       conf=CONF, verbose=False)[0]

            # --- draw: avoidance overlay, then object boxes, then guides/FPS ---
            nav.draw(frame, depth, plan, show_depth_inset=inset_on)
            n = draw_detections(frame, result, names) if result is not None else 0

            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            lp.draw_overlay(frame, fps)
            tag = (f"depth-avoid  det:{n if yolo_on else '--'}  "
                   f"steer:{plan['steer']:+.2f}")
            cv2.putText(frame, tag, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, tag, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow(win, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("o"):
                yolo_on = not yolo_on
            elif key == ord("i"):
                inset_on = not inset_on
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(save_dir, f"drive_{ts}_{saved:03d}.jpg")
                if cv2.imwrite(path, frame):
                    saved += 1
                    print(f"Saved {path}  (steer {plan['steer']:+.2f}, "
                          f"{'BLOCKED' if plan['blocked'] else 'clear'})")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        print(f"Done. Saved {saved} frame(s).")


if __name__ == "__main__":
    main()
