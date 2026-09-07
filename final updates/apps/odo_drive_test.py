#!/usr/bin/env python3
"""
apps/odo_drive_test.py — validate/calibrate odometry with a short DRIVEN run.

Sensorless VESC only counts tach while the motor is driven, so we measure the
distance scale by driving straight a little on the FLOOR and comparing the
tachometer ticks to a tape-measured distance.

  * Drives FORWARD, wheels centered, at a gentle duty for a short time, then stops.
  * Prints the tach delta and the odometry distance.
  * You measure how far it ACTUALLY travelled with a tape. Then:
        meters_per_tach = actual_metres / tach_delta
    Tell me the numbers and I'll update config.

SAFETY: this MOVES THE CAR on the floor. Clear ~3-4 m of straight space ahead, keep
a finger on Ctrl-C (stops the motor immediately). Start gentle.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 odo_drive_test.py                 # duty 0.09 for 1.5 s
    python3 odo_drive_test.py --duty 0.08 --secs 1.2
"""
import os
import sys
import time
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                       # noqa: E402
from drivers.vesc import VESC, resolve_port         # noqa: E402
from drivers.steering import ServoController         # noqa: E402
from control.odometry import Odometry               # noqa: E402

RAMP = 0.01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duty", type=float, default=0.09, help="forward duty (default 0.09)")
    ap.add_argument("--secs", type=float, default=1.5, help="hold time at duty (default 1.5)")
    args = ap.parse_args()
    duty_target = max(0.0, min(args.duty, 0.15))

    print("=" * 60)
    print(f"Driven odometry calibration — {duty_target*100:.0f}% for {args.secs:.1f}s")
    print("=" * 60)
    if not os.path.exists(resolve_port()):
        print("  !! VESC port missing — motor battery on?")
        return 1

    print("CLEAR ~3-4 m straight ahead. Car will DRIVE FORWARD. Ctrl-C = stop.")
    for n in (3, 2, 1):
        print(f"  starting in {n}...")
        time.sleep(1.0)

    steer = None
    vesc = None
    try:
        steer = ServoController()      # resets Nano -> wheels centered
        steer.center(read_reply=False)
        vesc = VESC()
        odo = Odometry()
        v = vesc.get_values()
        tach0 = v["tach"] if v else 0
        odo.update(tach0)

        duty = 0.0
        t0 = time.monotonic()
        # ramp up
        while duty < duty_target:
            duty = min(duty_target, duty + RAMP)
            vesc.set_duty(duty)
            time.sleep(0.02)
        # hold
        t_hold = time.monotonic()
        while time.monotonic() - t_hold < args.secs:
            vesc.set_duty(duty_target)
            v = vesc.get_values()
            if v:
                p = odo.update(v["tach"])
                print(f"  driving... odo x={p['x']:.2f} m  tach={v['tach']} "
                      f"erpm={v['erpm']}", flush=True)
            time.sleep(0.05)
        # ramp down + stop
        while duty > 0:
            duty = max(0.0, duty - RAMP)
            vesc.set_duty(duty)
            time.sleep(0.02)
        vesc.set_duty(0.0)
        vesc.stop()
        time.sleep(0.3)
        v = vesc.get_values()
        tach1 = v["tach"] if v else tach0
        dtach = tach1 - tach0
        odo_dist = dtach * config.METERS_PER_TACH
        print("\n" + "=" * 60)
        print(f"  tach delta      : {dtach}")
        print(f"  odometry says   : {odo_dist:.2f} m (using current "
              f"meters_per_tach={config.METERS_PER_TACH})")
        print(f"  >>> MEASURE the actual distance with a tape.")
        print(f"      corrected scale = actual_metres / {dtach}")
        print("=" * 60)
    except KeyboardInterrupt:
        print("\nSTOP.")
    finally:
        try:
            if vesc:
                vesc.set_duty(0.0); vesc.stop(); vesc.close()
        finally:
            if steer:
                steer.center(read_reply=False); steer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
