#!/usr/bin/env python3
"""
autodrive.py — closed-loop indoor obstacle-avoidance driving.

Fuses perception with actuation: monocular-depth free-space navigation produces a
steering target and a "blocked" flag; the controller steers toward open space via
the Arduino and drives the throttle via the VESC, avoiding obstacles class-
agnostically. YOLO labels are optional (toggle) and NOT used for safety.

SAFETY MODEL (read before driving on the floor):
  - Throttle starts DISARMED. Steering is always live (safe to watch it react).
  - 'a' arms throttle; SPACE is an instant e-stop (disarm + zero throttle now).
  - Hard duty cap (MAX_DUTY). Throttle ramps smoothly; eases off in hard turns.
  - blocked => throttle forced to zero.
  - Failsafes: VESC stops the motor if commands stop (~1s); the Arduino re-centers
    steering if commands stop (~1s); on exit/exception we stop + center.
  - TEST ON A STAND FIRST (wheels up). Watch steering track open space and the
    throttle stop on 'blocked' before ever putting it on the floor.

Keys:  a = arm throttle   SPACE = e-stop/disarm   o = toggle YOLO labels   q = quit
"""

import sys
import time
from collections import deque

import cv2
import numpy as np

import live_preview as lp
from threaded_camera import ThreadedCamera
from depth_nav import DepthNavigator
from vesc_driver import VESC
from steering_driver import ArduinoSteering
from devices import VESC_PORT, ARDUINO_PORT
from live_detect import draw_detections, MODEL, CONF, IMGSZ, DEVICE

# --- Control tuning (conservative for indoor bring-up) ---------------------- #
DRIVE_DUTY = 0.05      # forward duty when the path is clear (~ the tested gentle spin)
MAX_DUTY = 0.07        # hard ceiling — never exceed
RAMP_STEP = 0.004      # per-loop duty change (smooth accel/decel)
STEER_GAIN = 1.0       # multiply perception steer before sending
TURN_EASE = 0.4        # fraction of throttle shed at full steering lock


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def draw_hud(frame, fps, armed, blocked, steer, duty, yolo_on):
    h, w = frame.shape[:2]
    # Top status banner.
    if blocked:
        state, col = "BLOCKED - STOP", (0, 0, 255)
    elif armed:
        state, col = "ARMED - DRIVING", (0, 0, 255)
    else:
        state, col = "SAFE (throttle disarmed)", (0, 200, 0)
    cv2.rectangle(frame, (0, 0), (w, 26), (0, 0, 0), -1)
    cv2.putText(frame, state, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2,
                cv2.LINE_AA)
    hud = f"steer {steer:+.2f}  duty {duty*100:4.1f}%  fps {fps:4.1f}" + \
          ("  YOLO" if yolo_on else "")
    cv2.putText(frame, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "a=arm SPACE=stop o=yolo q=quit", (10, h - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)


def main():
    print("Opening camera, VESC, Arduino ...")
    cam = ThreadedCamera().start()
    vesc = VESC(VESC_PORT)
    steering = ArduinoSteering(ARDUINO_PORT)
    print("Loading depth navigator ...")
    nav = DepthNavigator(device=DEVICE)
    model = None
    nav.estimate(np.zeros((cam.height, cam.width, 3), np.uint8))  # warm up

    win = "AutoDrive - Obstacle Avoidance"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    times = deque(maxlen=30)
    state = {}
    seq = 0
    armed = False
    yolo_on = False
    duty = 0.0

    print("Ready. Throttle DISARMED. 'a' to arm, SPACE to e-stop, 'q' to quit.")
    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                vesc.set_duty(0.0)
                if cv2.waitKey(50) & 0xFF == ord("q"):
                    break
                continue
            frame = frame.copy()

            # --- perceive ---
            depth = nav.estimate(frame)
            plan = nav.plan(depth)
            steer = clamp(plan["steer"] * STEER_GAIN, -1.0, 1.0)
            blocked = plan["blocked"]

            # --- decide throttle ---
            if not armed or blocked:
                target = 0.0
            else:
                target = DRIVE_DUTY * (1.0 - TURN_EASE * abs(steer))
            # smooth ramp toward target, hard-capped
            if duty < target:
                duty = min(target, duty + RAMP_STEP)
            else:
                duty = max(target, duty - RAMP_STEP)
            duty = clamp(duty, 0.0, MAX_DUTY)

            # --- actuate ---
            steering.steer(steer)
            vesc.set_duty(duty)

            # --- visualize ---
            nav.draw(frame, depth, plan)
            if yolo_on:
                if model is None:
                    from ultralytics import YOLO
                    model = YOLO(MODEL, task="detect")
                res = model.predict(frame, device=DEVICE, imgsz=IMGSZ, conf=CONF,
                                    verbose=False)[0]
                draw_detections(frame, res, model.names)
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            draw_hud(frame, fps, armed, blocked, steer, duty, yolo_on)
            cv2.imshow(win, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("a"):
                armed = True
                print("ARMED")
            elif key == ord(" "):
                armed = False
                duty = 0.0
                vesc.set_duty(0.0)          # instant stop
                print("E-STOP / disarmed")
            elif key == ord("o"):
                yolo_on = not yolo_on
    finally:
        # Always leave the car stopped and centered.
        try:
            vesc.set_duty(0.0); vesc.stop()
        finally:
            steering.center()
            steering.close()
            vesc.close()
            cam.release()
            cv2.destroyAllWindows()
            print("Stopped, centered, released.")


if __name__ == "__main__":
    main()
