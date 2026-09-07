#!/usr/bin/env python3
"""
control/loop.py — closed-loop free-space driver (front camera -> throttle + steer).

Reproduces the proven autonav behavior on the clean `final updates/` stack:

  perceive : Camera("front") -> FreeSpacePlanner (metric depth) -> steer + blocked
  decide   : EMA-smooth steer + center deadband; distance-proportional speed;
             ease off throttle in turns; BLOCKED -> throttle 0 (the reflex)
  actuate  : ServoController.steer(-1..+1) + VESC.set_duty (hard-capped)

SAFETY MODEL:
  * Throttle starts DISARMED. Steering is live so you can watch it react safely.
  * 'a' arms throttle; SPACE = instant e-stop (disarm + zero now); 'q' quits.
  * Hard duty cap (config.MAX_DUTY). Duty ramps smoothly; eases off in hard turns.
  * blocked => throttle forced to 0 (reflex). Perception watchdog: a stale/lost
    frame forces throttle to 0.
  * On exit/crash: motor stopped, wheels centered, ports released.
  * dry=True: run the FULL decision chain but send NOTHING to the motors — prints
    the throttle it WOULD send. Use this to validate logic with wheels on the desk.

Keys need an OpenCV window (a display). Headless full runs stay disarmed (safe).
"""
import os
import sys
import time
from collections import deque

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config                                        # noqa: E402
from drivers.camera import Camera                    # noqa: E402
from perception.freespace import FreeSpacePlanner    # noqa: E402
from control.safety import Watchdog, clamp, ramp     # noqa: E402

# --- tuning (conservative indoor bring-up) --------------------------------- #
STEER_SMOOTH = 0.6      # EMA factor on steer
STEER_DEADBAND = 0.12   # |steer| below this -> go straight
TURN_EASE = 0.4         # fraction of throttle shed at full steering lock
RAMP_STEP = 0.010       # per-loop duty change (smooth accel/decel)
SLOW_REACH = config.FREESPACE_SLOW_REACH  # forward view for full speed; below -> scale down


class DriveLoop:
    def __init__(self, drive_duty=0.06, max_duty=None, headless=False, dry=False):
        self.dry = dry
        self.headless = headless
        self.max_duty = clamp(max_duty if max_duty is not None else config.MAX_DUTY,
                              0.0, config.MAX_DUTY)
        self.drive_duty = min(drive_duty, self.max_duty)
        print("Opening front camera ...")
        self.cam = Camera("front").start()
        print(f"Loading depth model ({'DRY' if dry else 'LIVE'}) ...")
        self.nav = FreeSpacePlanner()
        self.nav.estimate(np.zeros((self.cam.height, self.cam.width, 3), np.uint8))
        self.vesc = None
        self.steer = None
        if not dry:
            from drivers.vesc import VESC
            from drivers.steering import ServoController
            self.vesc = VESC()
            self.steer = ServoController()

    def _hud(self, frame, fps, armed, blocked, steer, duty):
        h, w = frame.shape[:2]
        if blocked:
            state, col = "BLOCKED - STOP", (0, 0, 255)
        elif armed:
            state, col = "ARMED - DRIVING", (0, 0, 255)
        else:
            state, col = "SAFE (disarmed)", (0, 200, 0)
        cv2.rectangle(frame, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(frame, state, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2,
                    cv2.LINE_AA)
        hud = f"steer {steer:+.2f}  duty {duty*100:4.1f}%  fps {fps:4.1f}"
        cv2.putText(frame, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "a=arm SPACE=stop q=quit", (10, h - 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    def run(self, frames=0, arm_after=0.0):
        show = not self.headless
        win = "RoboCar - free-space drive"
        if show:
            try:
                cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            except Exception:
                show = False
        wd = Watchdog(timeout=0.5)
        times = deque(maxlen=30)
        seq, n = 0, 0
        armed = False
        duty = 0.0
        steer_f = 0.0
        last_log = 0.0
        start = time.monotonic()
        auto_armed = False
        print("Ready. Throttle DISARMED." +
              ("  (dry run — no motors)" if self.dry else
               "  'a' arm, SPACE e-stop, 'q' quit."))
        if arm_after and not self.dry:
            print(f"AUTO-ARM in {arm_after:.0f}s (STAND TEST — wheels off ground!). "
                  "Ctrl-C to stop at any time.")
        try:
            while frames <= 0 or n < frames:
                frame, seq = self.cam.read(wait=True, last_seq=seq)
                if frame is None:
                    if not self.dry and self.vesc:
                        self.vesc.set_duty(0.0)     # stale frame -> stop
                    if show and (cv2.waitKey(30) & 0xFF) == ord("q"):
                        break
                    continue
                wd.pet()
                frame = frame.copy()

                depth = self.nav.estimate(frame)
                plan = self.nav.plan(depth)
                raw = clamp(plan["steer"], -1.0, 1.0)
                steer_f += STEER_SMOOTH * (raw - steer_f)
                steer = 0.0 if abs(steer_f) < STEER_DEADBAND else steer_f
                blocked = plan["blocked"]
                now = time.monotonic()

                # --- auto-arm after a delay (stand testing without key focus) ---
                if (arm_after and not self.dry and not armed
                        and (now - start) >= arm_after):
                    armed = True
                    auto_armed = True
                    print("AUTO-ARMED (stand test). Ctrl-C to stop.", flush=True)

                # --- decide throttle target ---
                # In dry mode there are no motors, so compute the throttle we
                # WOULD send (as if armed) purely to display intent. Live mode
                # still requires a real 'a' arm before anything moves.
                drive_ok = (armed or self.dry) and not blocked and not wd.stale()
                if not drive_ok:
                    target = 0.0
                else:
                    reach = plan.get("center_reach")
                    stop_m = getattr(self.nav, "stop_m", 2.5)
                    scale = clamp((reach - stop_m) / max(SLOW_REACH - stop_m, 1e-6),
                                  0.25, 1.0)
                    target = self.drive_duty * scale * (1.0 - TURN_EASE * abs(steer))
                    # Floor it so we command enough to actually start the motor
                    # (below ~5-6% a sensorless BLDC just cogs and never rolls).
                    target = clamp(max(target, config.MIN_MOVE_DUTY),
                                   0.0, self.max_duty)
                duty = ramp(duty, target, RAMP_STEP)
                duty = clamp(duty, -self.max_duty, self.max_duty)

                # --- actuate (or, in dry mode, just report) ---
                if not self.dry:
                    self.steer.steer(steer, read_reply=False)
                    self.vesc.set_duty(duty)

                times.append(now)
                fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
                n += 1

                if now - last_log >= 0.5:
                    last_log = now
                    st = "BLOCKED" if blocked else "clear  "
                    tag = "would-duty" if self.dry else "duty"
                    prefix = "DRY  " if self.dry else ("ARMED" if armed else "safe ")
                    print(f"{prefix:5} {st} "
                          f"steer={steer:+.2f} ahead={plan['center_reach']:4.1f}m "
                          f"{tag}={duty*100:+5.1f}% fps={fps:4.1f}", flush=True)

                if show:
                    self.nav.draw(frame, depth, plan)
                    self._hud(frame, fps, armed, blocked, steer, duty)
                    cv2.imshow(win, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                    elif key == ord("a") and not self.dry:
                        armed = True
                        print("ARMED")
                    elif key == ord(" "):
                        armed = False
                        duty = 0.0
                        if self.vesc:
                            self.vesc.set_duty(0.0)
                        print("E-STOP / disarmed")
        finally:
            self.shutdown(win, show)

    def shutdown(self, win=None, show=False):
        try:
            if self.vesc:
                self.vesc.set_duty(0.0)
                self.vesc.stop()
        finally:
            if self.steer:
                self.steer.center()
                self.steer.close()
            if self.vesc:
                self.vesc.close()
            self.cam.release()
            if show:
                cv2.destroyAllWindows()
            print("Stopped, centered, released.")
