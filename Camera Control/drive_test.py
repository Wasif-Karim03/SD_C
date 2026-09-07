#!/usr/bin/env python3
"""
drive_test.py — gentle open-loop drive test for the VESC over /dev/ttyACM0.

Sequence (the "move the wheels slowly" test):
    forward at 5% throttle for 5 s  ->  stop 2 s  ->  reverse 5% for 5 s  ->  stop

Throttle here is DUTY CYCLE (% of battery voltage), which works even before the
motor has been through VESC Tool detection — it just applies a small PWM. The
VESC times out and cuts the motor if it gets no command within ~1 s, so we
stream the duty command continuously at ~25 Hz and ramp in/out instead of
stepping, so nothing jerks.

Usage:
  python3 drive_test.py check          # read-only: link + telemetry, NO motion
  python3 drive_test.py                # run the forward/stop/back sequence
  python3 drive_test.py --duty 0.07 --secs 4

SAFETY: this spins the drive motor. Put the car up on a stand with the wheels
OFF THE GROUND (or somewhere it can roll a couple of metres safely) before
running the motion. Ctrl-C stops the motor at any time.
"""
import sys
import time
import argparse

from vesc_driver import VESC

PORT = "/dev/ttyACM0"


def show_telemetry(v, tag=""):
    t = v.get_values()
    if not t:
        print("  telemetry: (no response)")
        return None
    print("  %-7s duty=%+.2f erpm=%+6d motor_I=%+.2fA in_I=%+.2fA "
          "Vin=%.1fV Tmos=%.1f°C fault=%d"
          % (tag, t["duty"], t["erpm"], t["motor_current"],
             t["input_current"], t["v_in"], t["temp_mos"], t["fault"]))
    return t


def link_check(v):
    fw = v.firmware()
    if fw:
        print("VESC firmware %d.%d  hw=%s" % (fw["major"], fw["minor"], fw["hw"]))
    else:
        print("No firmware reply — is the VESC powered and on %s?" % PORT)
    t = show_telemetry(v, "now")
    if t and t["v_in"] < 6.0:
        print("  ** Vin looks low (%.1f V) — battery off or not connected?"
              % t["v_in"])
    return t is not None


def drive(v, target, seconds, ramp=0.6, hz=25):
    """Stream `target` duty for `seconds`, ramping from 0 over `ramp` seconds."""
    dt = 1.0 / hz
    t0 = time.time()
    next_log = 0.0
    while True:
        el = time.time() - t0
        if el >= seconds:
            break
        duty = target * min(1.0, el / ramp)
        v.set_duty(duty)
        if el >= next_log:
            show_telemetry(v, "run")
            next_log = el + 1.0
        time.sleep(dt)


def hold_stop(v, seconds, hz=25):
    """Keep commanding zero so the VESC doesn't fault, motor coasts to rest."""
    dt = 1.0 / hz
    t0 = time.time()
    while time.time() - t0 < seconds:
        v.set_current(0.0)
        time.sleep(dt)


def run_sequence(duty, secs):
    print("Opening VESC on %s ..." % PORT)
    with VESC(PORT) as v:
        if not link_check(v):
            print("Link check failed — aborting (no motion).")
            return 1
        print("\nWheels clear? Starting in 3 s — Ctrl-C to abort.")
        for n in (3, 2, 1):
            print("  %d..." % n)
            time.sleep(1.0)

        print("\nFORWARD  %.0f%% for %.0f s" % (duty * 100, secs))
        drive(v, +duty, secs)

        print("STOP 2 s")
        hold_stop(v, 2.0)

        print("\nREVERSE  %.0f%% for %.0f s" % (duty * 100, secs))
        drive(v, -duty, secs)

        print("STOP")
        hold_stop(v, 1.0)
        v.stop()
        print("Done — motor at zero.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", nargs="?", default="run",
                    help="'check' = telemetry only (no motion), else run sequence")
    ap.add_argument("--duty", type=float, default=0.05, help="duty 0..0.95 (def 0.05)")
    ap.add_argument("--secs", type=float, default=5.0, help="seconds each direction")
    args = ap.parse_args()

    if args.action == "check":
        with VESC(PORT) as v:
            link_check(v)
        return 0

    try:
        return run_sequence(args.duty, args.secs)
    except KeyboardInterrupt:
        print("\nAborted — sending stop.")
        try:
            with VESC(PORT) as v:
                v.stop()
        except Exception:
            pass
        return 130


if __name__ == "__main__":
    sys.exit(main())
