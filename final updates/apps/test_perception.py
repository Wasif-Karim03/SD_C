#!/usr/bin/env python3
"""
apps/test_perception.py — prove the perception layer works (NO motors).

Front camera -> metric depth -> free-space decision. Prints steer / blocked /
distance-ahead per frame, and saves annotated images so we can SEE what the car
"thinks": green bars = open direction, red = capped, orange arrow = chosen
heading, top-right inset = the depth map (warm = near).

Saves:  perception_view.jpg (annotated), perception_depth.jpg (depth heatmap),
        perception_report.txt

Nothing moves. This is the dry check before wiring perception to throttle/steer.

Run on the Jetson (front camera free — stop live_cameras first):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_perception.py            # ~40 frames
    python3 test_perception.py --frames 100
"""
import os
import sys
import time
import argparse
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.camera import Camera            # noqa: E402
from perception.freespace import FreeSpacePlanner   # noqa: E402

VIEW = os.path.join(HERE, "perception_view.jpg")
DEPTH_IMG = os.path.join(HERE, "perception_depth.jpg")
REPORT = os.path.join(HERE, "perception_report.txt")
lines = []


def log(m=""):
    print(m)
    lines.append(m)


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    args = ap.parse_args()

    log("=" * 64)
    log(f"perception test @ {time.strftime('%Y-%m-%d %H:%M:%S')}  (no motors)")
    log("=" * 64)

    try:
        cam = Camera("front").start()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! front camera: {exc}")
        _save()
        return 1
    log(f"front camera: {cam.dev} @ {cam.width}x{cam.height}")

    log("loading Depth-Anything V2 metric model (first run may download / be slow)...")
    try:
        import cv2
        nav = FreeSpacePlanner(device=0)
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not load depth model: {exc}")
        cam.release()
        _save()
        return 1
    # warm up
    nav.estimate(np.zeros((cam.height, cam.width, 3), np.uint8))
    log("model loaded. running ...\n")

    times = deque(maxlen=30)
    seq, n = 0, 0
    last_depth = None
    try:
        while n < args.frames:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            depth = nav.estimate(frame)
            plan = nav.plan(depth)
            last_depth = depth
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            n += 1
            if n % 5 == 0:
                state = "BLOCKED" if plan["blocked"] else "clear  "
                log(f"  [{n:3d}] {state} steer={plan['steer']:+.2f} "
                    f"ahead={plan['center_reach']:4.1f}m best_col={plan['best']:2d} "
                    f"fps={fps:4.1f}")
            # keep saving the latest annotated view
            annotated = nav.draw(frame, depth, plan)
            cv2.imwrite(VIEW, annotated)
    except KeyboardInterrupt:
        log("\ninterrupted.")
    finally:
        if last_depth is not None:
            import cv2
            cv2.imwrite(DEPTH_IMG, nav.eng.colorize(last_depth))
        cam.release()

    log("\n[SUMMARY]")
    log(f"   processed {n} frames")
    log(f"   annotated view -> {VIEW}")
    log(f"   depth heatmap  -> {DEPTH_IMG}")
    log("\n[RESULT] Perception ran: camera -> metric depth -> free-space decision.")
    log("         Open the two images to see the depth map + the steer/blocked call.")
    _save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
