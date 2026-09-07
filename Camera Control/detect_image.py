#!/usr/bin/env python3
"""
detect_image.py — STAGE 2: confirm YOLO11n inference works on a single image.

Loads YOLO11n (downloads yolo11n.pt ~5MB on first run), runs detection on one
image on the Jetson GPU, prints what it found + timing, and saves an annotated
copy next to the input as <name>_detected.jpg.

Usage:  python3 detect_image.py <image.jpg>   (defaults to stage2_test.jpg)
"""

import sys
import os
import time

import cv2
from ultralytics import YOLO

MODEL = "yolo11n.pt"


def main():
    img_path = sys.argv[1] if len(sys.argv) > 1 else "stage2_test.jpg"
    if not os.path.exists(img_path):
        sys.exit(f"ERROR: image not found: {img_path}")

    print(f"Loading {MODEL} ...")
    model = YOLO(MODEL)                    # auto-downloads weights on first use

    print(f"Running inference on {img_path} (GPU) ...")
    t0 = time.monotonic()
    results = model(img_path, device=0, verbose=False)
    dt = (time.monotonic() - t0) * 1000
    r = results[0]

    n = len(r.boxes)
    print(f"\nInference time: {dt:.1f} ms   detections: {n}")
    if n:
        print("  conf  class")
        for b in r.boxes:
            cls = model.names[int(b.cls)]
            conf = float(b.conf)
            xyxy = [int(v) for v in b.xyxy[0].tolist()]
            print(f"  {conf:4.2f}  {cls:<15} box={xyxy}")
    else:
        print("  (no objects detected — try an image with people/chairs/etc.)")

    out = os.path.splitext(img_path)[0] + "_detected.jpg"
    cv2.imwrite(out, r.plot())            # r.plot() returns the annotated BGR frame
    print(f"\nSaved annotated image: {out}")


if __name__ == "__main__":
    main()
