#!/usr/bin/env python3
"""
apps/test_odometry.py — prove wheel odometry (distance) works. NO motor drive.

Reads the VESC tachometer live and integrates position. To validate the distance
scale, PUSH THE CAR BY HAND in a straight line a known distance and check the
reported forward distance matches.

  1. Motor battery ON (so the VESC enumerates). Wheels on the ground.
  2. Run it, note it says "push the car now".
  3. Push the car STRAIGHT FORWARD a measured distance (e.g. 2.0 m), then stop.
  4. Read 'x' / 'dist' — should be close to how far you pushed. Push back -> x
     decreases toward 0.

(The steering/turning part of the model is exercised later during real driving;
this bench check validates the distance scale, which is the critical bit.)

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_odometry.py [seconds]     # default 25 s, Ctrl-C to stop early
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                 # noqa: E402
from drivers.vesc import VESC, resolve_port   # noqa: E402
from control.odometry import Odometry         # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0


def main():
    print("=" * 60)
    print("Odometry test (hand-push, no motor drive)")
    print("=" * 60)
    if not os.path.exists(resolve_port()):
        print("  !! VESC port missing — turn the MOTOR BATTERY on.")
        return 1
    vesc = VESC()
    odo = Odometry()
    print(f"wheelbase={odo.L} m  meters_per_tach={odo.mpt}  "
          f"max_steer={odo.max_steer} rad")

    # seed tach
    v = vesc.get_values()
    if not v:
        print("  !! no VESC telemetry."); vesc.close(); return 1
    odo.update(v["tach"])
    print(f"\n>>> PUSH THE CAR STRAIGHT now (measured distance). Reading {DURATION:.0f}s ...\n")

    t0 = time.monotonic()
    last = 0.0
    try:
        while time.monotonic() - t0 < DURATION:
            v = vesc.get_values()
            if v:
                p = odo.update(v["tach"], steer_norm=0.0)
                now = time.monotonic()
                if now - last >= 0.4:
                    last = now
                    print(f"  x={p['x']:+.2f} m  y={p['y']:+.2f} m  "
                          f"yaw={p['yaw_deg']:5.1f}°  path={p['dist']:.2f} m  "
                          f"(tach={v['tach']})", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        vesc.close()

    p = odo.pose()
    print("\n[RESULT] final: forward x = %.2f m, total path = %.2f m." % (p["x"], p["dist"]))
    print("         If that matches how far you pushed, the distance scale is good.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
