#!/usr/bin/env python3
"""
apps/test_detect.py — prove object detection works on BOTH cameras (no motors).

Runs YOLO on a front frame and a rear frame, prints what each sees, and saves
annotated images (detect_front.jpg / detect_rear.jpg) so you can eyeball it.

Stand in front of / behind the car (or put objects there) and run it.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_detect.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.camera import Camera             # noqa: E402
from perception.detect import Detector        # noqa: E402

lines = []


def log(m=""):
    print(m)
    lines.append(m)


def run_cam(det, which):
    try:
        cam = Camera(which).start()
    except Exception as exc:  # noqa: BLE001
        log(f"  {which}: camera error: {exc}")
        return
    import cv2
    frame, _ = cam.read(wait=True)
    t0 = time.monotonic()
    dets = det.detect(frame)
    dt = (time.monotonic() - t0) * 1000
    out = os.path.join(HERE, f"detect_{which}.jpg")
    cv2.imwrite(out, det.draw(frame.copy(), dets))
    cam.release()
    log(f"\n[{which.upper()}]  {len(dets)} objects in {dt:.0f} ms  -> {out}")
    for d in sorted(dets, key=lambda x: -x["conf"]):
        tag = " (VRU!)" if d["vru"] else ""
        log(f"    {d['name']:14s} {d['conf']*100:5.1f}%{tag}")


def main():
    log("=" * 56)
    log(f"object detection test — both cameras  {time.strftime('%H:%M:%S')}")
    log("=" * 56)
    log("loading YOLO ...")
    det = Detector()
    log(f"model: {det.kind}")
    run_cam(det, "front")
    run_cam(det, "rear")
    log("\n[RESULT] If both saved images have boxes on the objects/people, "
        "detection works on both cameras.")
    with open(os.path.join(HERE, "detect_report.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
