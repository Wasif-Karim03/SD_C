#!/usr/bin/env python3
"""
apps/probe_duty.py — watch the raw signals while duty is stepped up.

Written because the breakaway detector guessed wrong twice: first on ERPM
(a stalled sensorless motor twitches and reports ERPM without turning), then
on a small tachometer displacement (6 counts is a few commutation steps, which
cogging also produces). Both were thresholds picked before anyone had looked
at what the signals actually do on this car.

So: look first. This holds each duty for a second and prints what the VESC
says, with no interpretation and no threshold. You watch the car and say when
it moves; then the detector gets calibrated against that instead of a guess.

  python3 probe_duty.py                 # floor, 0.02 -> 0.20
  python3 probe_duty.py --max 0.30      # if it still will not move
  python3 probe_duty.py --hold 2.0      # longer at each step

Ctrl-C stops the motor. Nothing here drives for more than a second at a time.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                              # noqa: E402
from drivers.vesc import VESC                              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=float, default=0.02)
    ap.add_argument("--max", type=float, default=0.20)
    ap.add_argument("--step", type=float, default=0.01)
    ap.add_argument("--hold", type=float, default=1.0)
    a = ap.parse_args()

    v = VESC()
    print(f"  VESC {v.port}\n")
    print(f"  {'duty':>6} {'erpm':>8} {'tach':>9} {'d.tach':>7} {'cm':>7} "
          f"{'amps':>6}   watch the car")
    print("  " + "-" * 62)
    try:
        d = a.start
        while d <= a.max + 1e-9:
            v.set_duty(0.0)
            time.sleep(0.35)
            t0 = None
            v.set_duty(d)
            t_end = time.monotonic() + a.hold
            last = None
            while time.monotonic() < t_end:
                r = v.get_values(settle=0.01)
                if r:
                    if t0 is None:
                        t0 = r["tach"]
                    last = r
                time.sleep(0.05)
            v.set_duty(0.0)
            if last and t0 is not None:
                dt = last["tach"] - t0
                cm = dt * config.METERS_PER_TACH * 100.0
                print(f"  {d:6.3f} {last['erpm']:8.0f} {last['tach']:9d} "
                      f"{dt:7d} {cm:7.1f} {last['motor_current']:6.1f}")
            else:
                print(f"  {d:6.3f}       -- no telemetry reply --")
            d = round(d + a.step, 4)
    except KeyboardInterrupt:
        print("\n  stopped by operator")
    finally:
        try:
            v.set_duty(0.0); v.stop(); v.close()
        except Exception:                                   # noqa: BLE001
            pass
        print("\n  motor safed.")
        print("  Tell me the duty at which the car ACTUALLY started rolling.")


if __name__ == "__main__":
    sys.exit(main())
