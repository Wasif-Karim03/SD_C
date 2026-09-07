#!/usr/bin/env python3
"""
apps/drive.py — run the closed-loop free-space driver.

Stages (do them in order the first time):
  1. DRY  — full decision chain, NO motors (wheels can stay on the desk):
        python3 drive.py --dry
     Watch the printed steer / ahead / would-duty react as you move things in
     front of the camera. Nothing moves.

  2. LIVE, on a STAND (wheels off the ground), on the Jetson desktop (need a
     window for the keys):
        python3 drive.py                 # throttle DISARMED; press 'a' to arm
        python3 drive.py --max-duty 0.05 # even gentler cap
     Keys:  a = arm throttle   SPACE = e-stop/disarm   q = quit
     Confirm: steering tracks open space; throttle stays 0 until armed; throttle
     drops to 0 the instant it reads BLOCKED. THEN try it on the floor.

Only ONE program may own the VESC/steering at a time — stop the web console / other
autonomy first.  The motor battery must be on (or the VESC won't be found).
"""
import os
import sys
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from control.loop import DriveLoop   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="full decision chain, NO motors (safe validation)")
    ap.add_argument("--frames", type=int, default=0,
                    help="stop after N frames (0 = until q/Ctrl-C)")
    ap.add_argument("--duty", type=float, default=0.10,
                    help="forward duty when clear (default 0.10 = 10%%; try 0.15)")
    ap.add_argument("--max-duty", type=float, default=None,
                    help=f"hard cap (default config.MAX_DUTY)")
    ap.add_argument("--headless", action="store_true",
                    help="no window (no keys -> stays disarmed unless --arm-after)")
    ap.add_argument("--arm-after", type=float, default=0.0,
                    help="STAND TEST ONLY: auto-arm throttle after N seconds "
                         "(no key focus needed). Ctrl-C stops. Wheels OFF the ground!")
    args = ap.parse_args()

    headless = args.headless or args.dry
    loop = DriveLoop(drive_duty=args.duty, max_duty=args.max_duty,
                     headless=headless, dry=args.dry)
    try:
        loop.run(frames=args.frames, arm_after=args.arm_after)
    except KeyboardInterrupt:
        loop.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
