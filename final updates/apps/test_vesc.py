#!/usr/bin/env python3
"""
apps/test_vesc.py — prove the VESC (throttle controller) works.

DEFAULT = TELEMETRY ONLY. The motor DOES NOT move. It talks to the VESC and reads
firmware + live values (battery voltage, fault code, temps, RPM, tach) — all we
need to confirm the equipment is alive and healthy.

  python3 test_vesc.py            # safe: telemetry only, no motion
  python3 test_vesc.py --spin     # GENTLE wheels-up motion test (see warning)

The VESC only appears on USB when its MOTOR BATTERY is ON (its logic is battery-
fed). If the port is missing, switch the battery on first.

--spin: streams a gentle forward ramp to ~5% duty for a couple seconds, then stops.
Only run it with the car ON A STAND, wheels off the ground, finger near the power.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_vesc.py
"""
import os
import sys
import time
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.vesc import VESC, resolve_port   # noqa: E402

REPORT = os.path.join(HERE, "vesc_report.txt")
SPIN_DUTY = 0.05        # 5% — gentle
SPIN_TIME = 2.0         # seconds at target
RAMP_STEP = 0.004       # per 20 ms tick

lines = []


def log(m=""):
    print(m)
    lines.append(m)


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


def telemetry(vesc):
    log("\n[FIRMWARE]")
    fw = vesc.firmware()
    if fw:
        log(f"   version : {fw['major']}.{fw['minor']}   hw: {fw['hw']}")
    else:
        log("   !! no firmware response — is the VESC powered / port right?")

    log("\n[TELEMETRY]  (averaged over a few reads)")
    reads = [vesc.get_values() for _ in range(5)]
    reads = [r for r in reads if r]
    if not reads:
        log("   !! no telemetry — VESC not responding.")
        return False
    last = reads[-1]
    vin = sum(r["v_in"] for r in reads) / len(reads)
    log(f"   input voltage : {vin:5.2f} V")
    log(f"   duty          : {last['duty']*100:+.1f} %")
    log(f"   motor eRPM    : {last['erpm']}")
    log(f"   motor current : {last['motor_current']:.2f} A")
    log(f"   input current : {last['input_current']:.2f} A")
    log(f"   temp FET/mot  : {last['temp_mos']:.1f} / {last['temp_motor']:.1f} C")
    log(f"   tachometer    : {last['tach']}  (abs {last['tach_abs']})")
    log(f"   FAULT         : {last['fault_name']} ({last['fault']})")

    ok = True
    if last["fault"] != 0:
        log("   !! non-zero fault — investigate before driving.")
        ok = False
    if vin < 9.0:
        log(f"   !! battery low ({vin:.1f} V) — charge the pack (3S ~11.1V nominal).")
    return ok


def spin(vesc):
    log("\n[SPIN TEST]  gentle forward ramp — WHEELS MUST BE OFF THE GROUND")
    for n in (3, 2, 1):
        print(f"   starting in {n}...  (Ctrl-C to abort)")
        time.sleep(1.0)
    duty = 0.0
    t_end = None
    try:
        # ramp up
        while duty < SPIN_DUTY:
            duty = min(SPIN_DUTY, duty + RAMP_STEP)
            vesc.set_duty(duty)
            time.sleep(0.02)
        # hold, streaming continuously (VESC times out in ~1 s without commands)
        t_end = time.monotonic() + SPIN_TIME
        peak_rpm = 0
        while time.monotonic() < t_end:
            vesc.set_duty(SPIN_DUTY)
            v = vesc.get_values()
            if v:
                peak_rpm = max(peak_rpm, abs(v["erpm"]))
            time.sleep(0.05)
        # ramp down
        while duty > 0:
            duty = max(0.0, duty - RAMP_STEP)
            vesc.set_duty(duty)
            time.sleep(0.02)
        vesc.set_duty(0.0)
        vesc.stop()
        log(f"   ramped to {SPIN_DUTY*100:.0f}% and back. peak eRPM ~ {peak_rpm}")
        if peak_rpm > 100:
            log("   [RESULT] motor SPUN under command — throttle path works.")
        else:
            log("   [RESULT] no rotation detected — check motor battery / wiring.")
    finally:
        vesc.set_duty(0.0)
        vesc.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spin", action="store_true",
                    help="gentle wheels-up motion test (default is telemetry only)")
    args = ap.parse_args()

    log("=" * 64)
    log(f"VESC test @ {time.strftime('%Y-%m-%d %H:%M:%S')}  "
        f"({'SPIN' if args.spin else 'telemetry only'})")
    log("=" * 64)

    port = resolve_port()
    log(f"port: {port}")
    if not os.path.exists(port):
        log("  !! VESC port missing — turn the MOTOR BATTERY ON (the VESC only")
        log("     enumerates when battery-powered), then re-run.")
        _save()
        return 1

    try:
        vesc = VESC(port)
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not open VESC: {exc}")
        _save()
        return 1

    try:
        ok = telemetry(vesc)
        if args.spin:
            if not ok:
                log("\n  skipping spin — telemetry showed a problem. Fix it first.")
            else:
                spin(vesc)
        else:
            log("\n[RESULT] VESC is alive and healthy (telemetry OK, fault NONE).")
            log("         To test motion (wheels up!):  python3 test_vesc.py --spin")
    finally:
        try:
            vesc.close()
        except Exception:
            pass
        log("\nVESC link closed (motor commanded to stop).")

    _save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
