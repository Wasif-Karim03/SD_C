#!/usr/bin/env python3
"""
apps/selfdrive.py — situation-aware autonomy (front + rear + LiDAR + detection).

Two modes:

  DRY (default) — runs the full brain and PRINTS the decision each cycle, NO motors:
      python3 selfdrive.py
      python3 selfdrive.py --frames 0        # until Ctrl-C

  DRIVE — the brain actually drives the VESC + steering (lean; no web/3D overhead):
      python3 selfdrive.py --drive           # WHEELS-UP first!
      python3 selfdrive.py --drive --duty 0.08
    Safety: 3-2-1 countdown then it drives; DRIVE=forward, RECOVER=reverse toward
    open, HOLD=stop; blocked/person -> throttle 0; Ctrl-C = instant stop + center.
    DO THE FIRST RUN ON A STAND (wheels off the ground) to confirm the steering
    turns the right way and RECOVER reverses correctly. Then gentle floor.
"""
import os
import sys
import time
import argparse
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                 # noqa: E402
from drivers.camera import Camera             # noqa: E402
from drivers.lidar import ThreadedLidar       # noqa: E402
from control.brain import SmartBrain          # noqa: E402

RAMP = 0.01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=60, help="dry: 0 = until Ctrl-C")
    ap.add_argument("--drive", action="store_true", help="ACTUALLY drive the motors")
    ap.add_argument("--duty", type=float, default=0.08, help="drive duty (default 0.08)")
    ap.add_argument("--max-duty", type=float, default=config.MAX_DUTY)
    ap.add_argument("--arm-after", type=float, default=3.0, help="countdown before driving")
    args = ap.parse_args()
    import cv2
    import numpy as np

    drive = args.drive
    setp = max(0.0, min(args.duty, args.max_duty, config.MAX_DUTY))

    print("opening front camera ...")
    front = Camera("front").start()
    rear = None
    try:
        rear = Camera("rear").start()
    except Exception as e:  # noqa: BLE001
        print("  rear cam n/a:", e)
    lid = None
    try:
        lid = ThreadedLidar().start()
    except Exception as e:  # noqa: BLE001
        print("  lidar n/a:", e)
    print("loading brain (depth + YOLO) ...")
    brain = SmartBrain(device=0)
    brain.front.estimate_camera(np.zeros((front.height, front.width, 3), np.uint8))

    vesc = steer = None
    if drive:
        from drivers.vesc import VESC, resolve_port
        from drivers.steering import ServoController
        if not os.path.exists(resolve_port()):
            print("  !! VESC port missing — motor battery on? Aborting drive.")
            front.release()
            return 1
        vesc = VESC()
        steer = ServoController()
        steer.center(read_reply=False)
        print("\n*** DRIVE MODE — WHEELS SHOULD BE OFF THE GROUND ***")
        for n in (3, 2, 1):
            print(f"   self-driving in {n}...  (Ctrl-C aborts)")
            time.sleep(1.0)
        print("   GO. Ctrl-C to stop.\n")
    else:
        print("running (DRY — no motors).\n")

    seq, n, last = 0, 0, 0.0
    duty = 0.0
    times = deque(maxlen=20)
    try:
        while args.frames <= 0 or drive or n < args.frames:
            frame, seq = front.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            rframe = rear.read(wait=False)[0] if rear else None
            scan = None
            if lid:
                s, age = lid.latest()
                scan = s if (s and age < 0.5) else None
            d = brain.decide(frame, scan, rframe)
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            n += 1

            if drive:
                target = d["throttle_sign"] * setp        # +fwd / -rev / 0 hold
                target = max(-args.max_duty, min(args.max_duty, target))
                duty = min(target, duty + RAMP) if duty < target else max(target, duty - RAMP)
                steer.steer(d["steer"], read_reply=False)
                vesc.set_duty(duty)

            if now - last >= 0.5:
                last = now
                extra = f" duty={duty*100:+5.1f}%" if drive else ""
                pf = "PERSON!" if d["person_front"] else ""
                print(f"{d['mode']:8s} thr{d['throttle_sign']:+.0f} st{d['steer']:+.2f}{extra} "
                      f"| front {'BLK' if d['front_blocked'] else 'clr'} {d['front_near']}m "
                      f"| rear {'BLK' if d['rear_blocked'] else 'clr'} {d['rear_near']}m "
                      f"| open {d['best_bearing']:+.0f}deg {pf} | {fps:.1f}fps", flush=True)
                print(f"         -> {d['reason']}")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        if vesc:
            try:
                vesc.set_duty(0.0); vesc.stop(); vesc.close()
            except Exception:
                pass
        if steer:
            try:
                steer.center(); steer.close()
            except Exception:
                pass
        if lid:
            lid.stop()
        if rear:
            rear.release()
        front.release()
        print("stopped, centered, released." if drive else "done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
