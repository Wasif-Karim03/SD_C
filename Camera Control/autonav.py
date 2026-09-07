#!/usr/bin/env python3
"""
autonav.py — camera-based free-space autonomous driving (current hardware).

The car looks ahead with the USB camera, decides whether the path in front is
FREE, steers toward the most open space, and creeps forward at a gentle, hard-
capped throttle. It avoids obstacles class-agnostically (it doesn't need to know
*what* is ahead, only how near) using monocular-depth free-space planning.

  perception:  ThreadedCamera  ->  DepthNavigator (MiDaS-small)  ->  steer + blocked
  actuation :  steering = ServoController (Arduino Nano, /dev/ttyUSB0, 60..115deg)
               throttle = VESC set_duty   (/dev/ttyACM0, hard-capped at MAX_DUTY)

This is wired to the hardware we verified: the Nano steering (servo_control.py,
with the bench-tested 60/115 limits) and the VESC throttle (vesc_driver.py).

SAFETY MODEL — read before putting it on the floor:
  * Throttle starts DISARMED. Steering is live so you can watch it react safely.
  * 'a' arms throttle; SPACE is an instant e-stop (disarm + zero throttle now).
  * Hard duty cap MAX_DUTY (10%). Throttle ramps smoothly and eases off in turns.
  * blocked  => throttle forced to zero.
  * Failsafes: the VESC stops the motor if commands stop (~1 s); on exit/crash we
    stop the motor and re-center the wheels.
  * TEST ON A STAND FIRST (wheels up): confirm steering tracks the open space and
    throttle drops to 0 on 'blocked' before ever driving on the floor.

Usage:
  python3 autonav.py            # full run: window + arm/e-stop keys (needs display)
  python3 autonav.py --dry      # perception only: NO motors, prints steer/blocked
  python3 autonav.py --dry --frames 120
  python3 autonav.py --max-duty 0.05    # even gentler cap

Keys (full run):  a = arm   SPACE = e-stop/disarm   q = quit
"""
import os
import sys
import time
import argparse
from collections import deque

import numpy as np
import cv2

# Steering driver lives in the sibling ../servo package.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                 "..", "servo")))

from threaded_camera import ThreadedCamera


def make_navigator(backend):
    """Build the perception navigator. 'metric' = Depth-Anything V2 (real metres,
    detects walls); 'midas' = MiDaS relative depth (legacy, cannot see flat walls)."""
    if backend == "midas":
        from depth_nav import DepthNavigator
        return DepthNavigator(device=0)
    from metric_nav import MetricDepthNavigator
    return MetricDepthNavigator(device=0)


def plan_metric(plan):
    """Compact per-frame telemetry field, whichever backend is running."""
    if "center_reach" in plan:
        return f"ahead={plan['center_reach']:4.1f}m"
    return f"contrast={plan.get('contrast', 0.0):.3f}"

# Stable device path for the VESC (number can swap on replug; identity can't).
VESC_BY_ID = ("/dev/serial/by-id/"
              "usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00")

# --- conservative control tuning for indoor bring-up ----------------------- #
DRIVE_DUTY = 0.10      # forward duty when the path is clear
MAX_DUTY = 0.10        # hard ceiling — never exceed (the requested 10%)
RAMP_STEP = 0.004      # per-loop duty change (smooth accel/decel)
STEER_GAIN = 1.0       # multiply perception steer before sending
TURN_EASE = 0.4        # fraction of throttle shed at full steering lock
STEER_SMOOTH = 0.6     # EMA factor on steer (nav already denoises; keep responsive)
STEER_DEADBAND = 0.12  # |steer| below this -> go straight (nav gives proportional steer)
SLOW_REACH = 3.5       # metres of forward view at which we allow full speed; below
                       # this the throttle scales down toward the stop distance

# --- stuck-recovery / dead-end escape ---------------------------------------- #
# On a real block the car can't spin in place (Ackermann steering), so it escapes
# with repeated 3-point-turn "scans": reverse a little (BLIND — no rear sensor, so
# keep it short/slow), then arc forward toward the openest side to reorient, then
# re-check. Each cycle sweeps the heading further around until the front is clear.
RECOVER_BACK_DUTY = 0.09   # reverse duty while backing out (needs to break stiction)
RECOVER_BACK_TIME = 0.9    # s to reverse per attempt
RECOVER_TURN_DUTY = 0.10   # forward duty while arcing to a new heading
RECOVER_TURN_TIME = 1.8    # s to arc toward the open side per attempt
RECOVER_MAX_CYCLES = 10    # back+turn attempts (~a full sweep) before it just holds
# Sensorless BLDC makes almost no torque at ~10% duty from a dead stop, so each
# maneuver starts with a brief higher-duty KICK to break stiction and get the
# motor spinning, then settles to the gentle duty above. The kick briefly exceeds
# the cruise cap but only for a moment at ~zero speed.
RECOVER_KICK_DUTY = 0.18   # reverse startup pulse magnitude
RECOVER_TURN_KICK = 0.22   # forward-turn startup pulse (harder: must overcome tire scrub)
RECOVER_KICK_TIME = 0.40   # s of kick at the start of each back/turn phase
RECOVER_STEER_RAMP = 0.6   # s to ease steering 0 -> full lock so it can START rolling
                           # (full lock from a dead stop scrubs the tires and stalls
                           #  the sensorless motor; start straight, then curve)
RECOVER_MAX_DUTY = 0.25    # duty cap during recovery only (allows the kick)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def vesc_port():
    return VESC_BY_ID if os.path.exists(VESC_BY_ID) else "/dev/ttyACM0"


def draw_hud(frame, fps, armed, blocked, steer, duty, rec_phase=None):
    h, w = frame.shape[:2]
    if rec_phase:
        state, col = f"RECOVERING ({rec_phase})", (0, 165, 255)
    elif blocked:
        state, col = "BLOCKED - STOP", (0, 0, 255)
    elif armed:
        state, col = "ARMED - DRIVING", (0, 0, 255)
    else:
        state, col = "SAFE (throttle disarmed)", (0, 200, 0)
    cv2.rectangle(frame, (0, 0), (w, 26), (0, 0, 0), -1)
    cv2.putText(frame, state, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2,
                cv2.LINE_AA)
    hud = f"steer {steer:+.2f}  duty {duty*100:4.1f}%  fps {fps:4.1f}"
    cv2.putText(frame, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "a=arm  SPACE=stop  q=quit", (10, h - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)


def run_dry(frames, backend):
    """Perception only — open camera + depth model, print decisions, NO motors."""
    print(f"DRY RUN: perception only ({backend}), no motors will move.")
    cam = ThreadedCamera().start()
    print(f"Camera {cam.dev} @ {cam.width}x{cam.height}")
    nav = make_navigator(backend)
    nav.estimate(np.zeros((cam.height, cam.width, 3), np.uint8))  # warm up
    times = deque(maxlen=30)
    seq = 0
    n = 0
    try:
        while frames <= 0 or n < frames:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            plan = nav.plan(nav.estimate(frame))
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            n += 1
            if n % 5 == 0 or frames > 0:
                state = "BLOCKED" if plan["blocked"] else "clear  "
                print(f"  [{n:4d}] {state} steer={plan['steer']:+.2f} "
                      f"{plan_metric(plan)} best_col={plan['best']} fps={fps:4.1f}")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        cam.release()
    print("Dry run done.")
    return 0


def run_drive(max_duty, headless, backend):
    from servo_control import ServoController
    from vesc_driver import VESC

    print("Opening camera, VESC, steering ...")
    cam = ThreadedCamera().start()
    vesc = VESC(vesc_port())
    steering = ServoController()                 # auto-detects the Nano on ttyUSB0
    print(f"Loading depth navigator ({backend}) ...")
    nav = make_navigator(backend)
    nav.estimate(np.zeros((cam.height, cam.width, 3), np.uint8))  # warm up

    if not headless:
        win = "AutoNav - free-space driving"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    times = deque(maxlen=30)
    seq = 0
    armed = False
    duty = 0.0
    steer_f = 0.0          # EMA-smoothed steering state
    mode = "drive"         # "drive" or "recover" (dead-end escape)
    rec_phase = "back"     # within recover: "back" -> "turn" -> ... -> "hold"
    rec_start = 0.0        # monotonic time the current recover phase began
    rec_dir = 1.0          # escape steer direction (+right / -left)
    rec_cycles = 0         # back+turn attempts made this recovery

    print("Ready. Throttle DISARMED. 'a' to arm, SPACE to e-stop, 'q' to quit.")
    last_log = 0.0
    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                vesc.set_duty(0.0)               # keep the motor stopped on dropout
                if not headless and (cv2.waitKey(50) & 0xFF) == ord("q"):
                    break
                continue
            frame = frame.copy()

            # --- perceive: is the front free, and where is open space? ---
            depth = nav.estimate(frame)
            plan = nav.plan(depth)
            raw = clamp(plan["steer"] * STEER_GAIN, -1.0, 1.0)
            # Smooth (EMA) then apply a center deadband so a broadly-open path
            # drives straight instead of chasing the single "most open" column.
            steer_f += STEER_SMOOTH * (raw - steer_f)
            steer = 0.0 if abs(steer_f) < STEER_DEADBAND else steer_f
            blocked = plan["blocked"]
            now = time.monotonic()

            # --- recovery state transitions (dead-end escape) ---
            if not armed:
                mode, rec_cycles = "drive", 0          # disarm cancels recovery
            elif mode == "drive" and blocked:
                # Enter recovery: escape toward the openest side the camera sees.
                mode, rec_phase, rec_start, rec_cycles = "recover", "back", now, 0
                rec_dir = 1.0 if plan["steer"] >= 0 else -1.0
            elif mode == "recover" and not blocked and rec_phase != "back":
                mode, rec_cycles = "drive", 0          # reoriented into open space

            # --- decide steering command + throttle target ---
            if not armed:
                steer_cmd, target = 0.0, 0.0
            elif mode == "recover":
                if rec_cycles >= RECOVER_MAX_CYCLES:
                    rec_phase = "hold"                 # scanned all round, still boxed
                kicking = (now - rec_start) < RECOVER_KICK_TIME    # startup pulse
                if rec_phase == "back":
                    steer_cmd = 0.0                     # reverse straight (blind)
                    target = -(RECOVER_KICK_DUTY if kicking else RECOVER_BACK_DUTY)
                    if now - rec_start >= RECOVER_BACK_TIME:
                        rec_phase, rec_start = "turn", now
                elif rec_phase == "turn":
                    # Ease the wheels from straight -> full lock so the car can
                    # actually START rolling (full lock from a stop scrubs tires
                    # and stalls the motor), then curve toward the open side.
                    steer_frac = min(1.0, (now - rec_start) / RECOVER_STEER_RAMP)
                    steer_cmd = rec_dir * steer_frac
                    target = RECOVER_TURN_KICK if kicking else RECOVER_TURN_DUTY
                    if now - rec_start >= RECOVER_TURN_TIME:
                        rec_cycles += 1
                        rec_phase, rec_start = "back", now
                else:                                  # "hold": stopped, wait it out
                    steer_cmd, target = rec_dir, 0.0
            else:
                # Normal drive. Distance-proportional speed: crawl when the forward
                # view is closing in, full (capped) speed only when it can see far.
                steer_cmd = steer
                reach = plan.get("center_reach")
                stop_m = getattr(nav, "stop_m", None)
                if reach is not None and stop_m is not None:
                    speed_scale = clamp((reach - stop_m) / max(SLOW_REACH - stop_m, 1e-6),
                                        0.25, 1.0)
                else:
                    speed_scale = 1.0
                target = DRIVE_DUTY * speed_scale * (1.0 - TURN_EASE * abs(steer))

            # Ramp toward target during normal drive (smooth accel); snap during
            # recovery so the short back/turn phases actually reach their duty.
            if mode == "recover":
                duty = target                          # snap; short kick/back/turn phases
                duty = clamp(duty, -RECOVER_MAX_DUTY, RECOVER_MAX_DUTY)
            else:
                if duty < target:
                    duty = min(target, duty + RAMP_STEP)
                else:
                    duty = max(target, duty - RAMP_STEP)
                duty = clamp(duty, -max_duty, max_duty)

            # --- actuate ---
            steering.steer(steer_cmd)
            vesc.set_duty(duty)

            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

            # Periodic telemetry to stdout so behaviour is visible without the HUD.
            if now - last_log >= 0.5:
                last_log = now
                state = f"RECOVER:{rec_phase}" if mode == "recover" else \
                        ("BLOCKED" if blocked else "clear  ")
                print(f"{'ARMED' if armed else 'safe ':5} {state:14} "
                      f"{plan_metric(plan)} "
                      f"steer={steer_cmd:+.2f} duty={duty*100:+5.1f}% "
                      f"target={target*100:+5.1f}% fps={fps:4.1f}", flush=True)

            if not headless:
                nav.draw(frame, depth, plan)
                draw_hud(frame, fps, armed, blocked, steer_cmd, duty,
                         rec_phase=rec_phase if mode == "recover" else None)
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
                    vesc.set_duty(0.0)
                    print("E-STOP / disarmed")
    finally:
        try:
            vesc.set_duty(0.0)
            vesc.stop()
        finally:
            steering.center()
            steering.close()
            vesc.close()
            cam.release()
            if not headless:
                cv2.destroyAllWindows()
            print("Stopped, centered, released.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="perception only: no motors, print steer/blocked")
    ap.add_argument("--frames", type=int, default=0,
                    help="dry-run frame count (0 = until Ctrl-C)")
    ap.add_argument("--max-duty", type=float, default=MAX_DUTY,
                    help="hard throttle cap (default 0.10 = 10%%)")
    ap.add_argument("--headless", action="store_true",
                    help="no window (no arm/e-stop keys — auto-disarmed; for tests)")
    ap.add_argument("--depth", choices=["metric", "midas"], default="metric",
                    help="perception backend: metric=Depth-Anything V2 (real "
                         "metres, sees walls; default), midas=relative (legacy)")
    args = ap.parse_args()

    if args.dry:
        return run_dry(args.frames, args.depth)
    return run_drive(clamp(args.max_duty, 0.0, 0.10), args.headless, args.depth)


if __name__ == "__main__":
    sys.exit(main())
