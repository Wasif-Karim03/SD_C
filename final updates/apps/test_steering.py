#!/usr/bin/env python3
"""
apps/test_steering.py — prove the steering (Nano -> servo) works.

Moves the front wheels through: center -> full LEFT (60) -> center -> full RIGHT
(115) -> center, then a smooth sweep. Prints the Nano's "ANGLE nn" echoes so the
serial link + firmware are confirmed programmatically; YOU confirm the wheels
physically move (I can't see them).

SAFETY / SETUP:
  * The servo needs its OWN 5-6 V BEC powered (common ground) — on USB power alone
    it may just twitch/shake. Make sure the servo battery/BEC is on.
  * Wheels can move — have the car where the front wheels can turn freely (on a
    stand or lifted front), and keep fingers clear of the linkage.
  * Resolves the Nano by stable by-id, so it won't grab the LiDAR on ttyUSB0.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_steering.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.steering import (ServoController, resolve_port,   # noqa: E402
                              LEFT_LIMIT, RIGHT_LIMIT, CENTER)

REPORT = os.path.join(HERE, "steering_report.txt")
lines = []


def log(m=""):
    print(m)
    lines.append(m)


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def main():
    log("=" * 64)
    log(f"Steering test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)

    port = resolve_port()
    log(f"port: {port}")
    if not port:
        log("  !! steering Nano not found. Plugged in? CH340 driver loaded?")
        log("     check: ls /dev/serial/by-id/ | grep 1a86")
        _save()
        return 1

    try:
        srv = ServoController(port)
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not open steering Nano: {exc}")
        _save()
        return 1

    if srv.banner:
        log("Nano banner:")
        for b in srv.banner:
            log(f"   {b}")
    ready = any(b.startswith("READY") for b in srv.banner)
    log(f"READY seen: {'yes' if ready else 'no (older sketch? still trying)'}")

    acks = []
    try:
        seq = [("center", lambda: srv.center()),
               (f"LEFT ({LEFT_LIMIT})", lambda: srv.steer(-1.0)),
               ("center", lambda: srv.center()),
               (f"RIGHT ({RIGHT_LIMIT})", lambda: srv.steer(+1.0)),
               ("center", lambda: srv.center())]
        log("\nstepping through positions (watch the wheels):")
        for label, act in seq:
            reply = act()
            got = ", ".join(reply) if reply else "(no echo)"
            log(f"   -> {label:12s}  nano: {got}")
            if reply:
                acks.extend(reply)
            time.sleep(1.0)

        log("\nsmooth sweep left<->right x2 (watch for smooth motion):")
        for _ in range(2):
            for d in range(LEFT_LIMIT, RIGHT_LIMIT + 1, 3):
                srv.set_angle(d, read_reply=False)
                time.sleep(0.03)
            for d in range(RIGHT_LIMIT, LEFT_LIMIT - 1, -3):
                srv.set_angle(d, read_reply=False)
                time.sleep(0.03)
        srv.center()
        time.sleep(0.5)
    finally:
        srv.close()
        log("\ncentered + detached, port closed.")

    link_ok = ready or any(a.upper().startswith("ANGLE") for a in acks)
    log("\n[SUMMARY]")
    log(f"   serial link + firmware: {'OK' if link_ok else 'NOT confirmed'}")
    if link_ok:
        log("\n[RESULT] Steering link WORKS (Nano acked commands). Confirm with your")
        log("         eyes: did the wheels go LEFT, center, RIGHT, then sweep? If the")
        log("         servo only buzzed/twitched, its BEC/battery isn't powered.")
    else:
        log("\n[RESULT] No acks from the Nano — check the CH340 link / sketch / baud.")
        _save()
        return 1
    _save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
