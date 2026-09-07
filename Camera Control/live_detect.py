#!/usr/bin/env python3
"""
live_detect.py — STAGE 3: live YOLO11n object detection on the USB camera.

Built on top of live_preview.py (imported, not modified): same USB-camera
detection, manual-exposure handling, alignment overlay (crosshair / horizon /
3x3 grid) and FPS counter — with YOLO11n object detection added on top, drawing
boxes + class labels + confidence.

Model runs at DEFAULT PyTorch settings on the GPU (device 0). TensorRT
optimization comes later. The FPS shown is end-to-end (capture + inference +
draw), so it reflects the real detection frame rate, not just the camera.

Keys:  q = quit    s = save annotated frame    d = toggle detection    l = toggle lane

Uses the system OpenCV (JetPack) + the user-site torch/ultralytics. No installs.
"""

import os
import re
import sys
import time
from collections import deque

try:
    import cv2
except ImportError:
    sys.exit("ERROR: OpenCV (cv2) not importable — install the JetPack build.")

# Reuse the preview foundation (camera detect, exposure, alignment overlay).
import live_preview as lp
# Low-latency, always-newest-frame capture (decouples inference from camera I/O).
from threaded_camera import ThreadedCamera
# Classic-CV lane/path detection + steering signal (CPU, runs alongside YOLO).
from lane_detect import detect_lane

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("ERROR: ultralytics not importable. Run Stage 1 install first.")

# --- Detection settings ----------------------------------------------------- #
# Prefer the TensorRT FP16 engine when it exists (faster inference, lower power);
# fall back to the PyTorch weights. Rebuild the engine with:
#   python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='engine', half=True, imgsz=640, device=0)"
MODEL = "yolo11n.engine" if os.path.exists(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolo11n.engine")
) else "yolo11n.pt"
CONF = 0.25          # confidence threshold
IMGSZ = 640          # inference size
DEVICE = 0           # CUDA device


def draw_detections(frame, result, names):
    """Draw YOLO boxes + 'label conf' on frame in place."""
    boxes = result.boxes
    if boxes is None:
        return 0
    for b in boxes:
        x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
        conf = float(b.conf)
        label = f"{names[int(b.cls)]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 128, 0), 2, cv2.LINE_AA)
        # Label chip above the box (or inside if near the top edge).
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly = y1 - 4 if y1 - th - 6 > 0 else y1 + th + 6
        cv2.rectangle(frame, (x1, ly - th - bl - 2), (x1 + tw + 4, ly + 1),
                      (255, 128, 0), -1, cv2.LINE_AA)
        cv2.putText(frame, label, (x1 + 2, ly - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return len(boxes)


def main():
    try:
        cam = ThreadedCamera().start()
    except RuntimeError as e:
        sys.exit(f"ERROR: {e}")
    dev, w, h = cam.dev, cam.width, cam.height

    print(f"Loading {MODEL} on GPU (device {DEVICE}) ...")
    model = YOLO(MODEL)
    names = model.names
    # Warm up so the first displayed frame isn't a multi-second stall.
    import numpy as np
    print("Warming up CUDA kernels ...")
    model.predict(np.zeros((h, w, 3), dtype=np.uint8),
                  device=DEVICE, imgsz=IMGSZ, conf=CONF, verbose=False)

    print(f"Streaming {dev} at {w}x{h} with YOLO11n + lane detection. "
          "q=quit  s=save  d=toggle detection  l=toggle lane")

    save_dir = os.path.dirname(os.path.abspath(__file__))
    win = "Live Detection - YOLO11n"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    times = deque(maxlen=30)
    saved = 0
    detect_on = True
    lane_on = True
    lane_state = {}
    seq = 0

    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                if cv2.waitKey(50) & 0xFF == ord("q"):
                    break
                continue
            frame = frame.copy()  # capture thread reuses its buffer; draw on a copy

            # Analyze the CLEAN frame first (YOLO + lane edges), then draw.
            n = 0
            result = None
            if detect_on:
                result = model.predict(frame, device=DEVICE, imgsz=IMGSZ,
                                       conf=CONF, verbose=False)[0]
            lane = None
            if lane_on:
                lane = detect_lane(frame, lane_state)   # draws path overlay
            if result is not None:
                n = draw_detections(frame, result, names)

            # FPS = end-to-end (capture + inference + draw).
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

            # Keep the alignment overlay + FPS from the preview foundation.
            lp.draw_overlay(frame, fps)
            steer = lane["offset"] if lane else None
            tag = (f"YOLO11n  det:{n}" + ("" if detect_on else "  [OFF]")
                   + ("  steer:--" if steer is None else f"  steer:{steer:+.2f}"))
            cv2.putText(frame, tag, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, tag, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow(win, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("d"):
                detect_on = not detect_on
            elif key == ord("l"):
                lane_on = not lane_on
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(save_dir, f"detect_{ts}_{saved:03d}.jpg")
                if cv2.imwrite(path, frame):
                    saved += 1
                    print(f"Saved {path}")
                else:
                    print(f"ERROR: failed to write {path}")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        print(f"Done. Saved {saved} frame(s).")


if __name__ == "__main__":
    main()
