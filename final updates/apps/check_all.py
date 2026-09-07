#!/usr/bin/env python3
"""
apps/check_all.py — verify the WHOLE car in one command (safe, no driving).

Runs a quick non-destructive check of every piece of equipment and prints a
PASS / WARN / FAIL table. Nothing drives: the VESC is telemetry-only and the
motor never spins. (Opening the steering Nano resets it, which re-centers the
servo — a harmless small move. The LiDAR spins whenever powered, as normal.)

  PASS = working.   WARN = not fatal / needs attention (e.g. GPS no fix indoors,
  VESC battery off).   FAIL = broken or not responding.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 check_all.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config   # noqa: E402

REPORT = os.path.join(HERE, "check_all_report.txt")
results = []   # (name, status, detail)
lines = []


def log(m=""):
    print(m)
    lines.append(m)


def record(name, status, detail=""):
    results.append((name, status, detail))
    icon = {"PASS": "✅", "WARN": "⚠️ ", "FAIL": "❌"}.get(status, "  ")
    log(f"   {icon} {name:16s} {status:4s}  {detail}")


def check_ports():
    log("\n[1/8] device paths (stable identities)")
    for name, path in [("VESC", config.vesc_port()),
                       ("Steering", config.steering_port()),
                       ("LiDAR", config.lidar_port()),
                       ("GPS", config.GPS_PORT),
                       ("I2C bus", f"/dev/i2c-{config.I2C_BUS}"),
                       ("Cam front", config.CAM_FRONT_BYPATH),
                       ("Cam rear", config.CAM_REAR_BYPATH)]:
        log(f"      {name:10s}: {path}  {'(present)' if os.path.exists(path) else '(MISSING)'}")


def check_cameras():
    log("\n[2/8] cameras")
    try:
        from drivers.camera import Camera
    except Exception as exc:  # noqa: BLE001
        record("Cameras", "FAIL", f"import error: {exc}")
        return
    for which in ("front", "rear"):
        try:
            cam = Camera(which).start()
            frame, _ = cam.read(wait=True)
            ok = frame is not None
            cam.release()
            record(f"Camera {which}", "PASS" if ok else "FAIL",
                   f"{cam.width}x{cam.height} rot={cam.rotate}" if ok else "no frame")
        except Exception as exc:  # noqa: BLE001
            record(f"Camera {which}", "FAIL", str(exc))


def check_lidar():
    log("\n[3/8] LiDAR")
    try:
        from drivers.lidar import RPLidarC1
        lidar = RPLidarC1()
        if not os.path.exists(lidar.port):
            record("LiDAR", "FAIL", "port missing / unpowered")
            return
        lidar.connect()
        try:
            info = lidar.get_info()
            health = lidar.get_health()
            scans = lidar.grab_scans(1)
            pts = len(scans[0]) if scans else 0
            good = health["status"] == 0 and pts > 0
            record("LiDAR", "PASS" if good else "WARN",
                   f"model {info['model_hex']} fw{info['firmware']} "
                   f"health={health['status_str']} pts={pts}")
        finally:
            lidar.disconnect()
    except Exception as exc:  # noqa: BLE001
        record("LiDAR", "FAIL", str(exc))


def check_compass():
    log("\n[4/8] compass")
    try:
        from drivers.compass import Compass
        c = Compass().open()
        r = c.read()
        c.close()
        ok = c.whoami == 0x10
        record("Compass", "PASS" if ok else "WARN",
               f"WHO_AM_I=0x{c.whoami:02x} heading={r['heading_deg']:.0f}° "
               f"({'cal' if r['calibrated'] else 'raw'})")
    except Exception as exc:  # noqa: BLE001
        record("Compass", "FAIL", str(exc))


def check_oled():
    log("\n[5/8] OLED")
    try:
        from drivers.oled import OLED
        oled = OLED().open()
        oled.text(["check_all", "OLED OK"])
        time.sleep(0.6)
        oled.close()
        record("OLED", "PASS", "drew test text (glance at the panel)")
    except Exception as exc:  # noqa: BLE001
        record("OLED", "WARN", f"{exc}  (oled_stats.service holding it?)")


def check_gps():
    log("\n[6/8] GPS (reading ~4s)")
    try:
        from drivers.gps import GPS
        with GPS() as gps:
            st = gps.poll(4.0)
        if st["nmea_lines"] == 0:
            record("GPS", "WARN", "no NMEA — port busy? (fuser -v /dev/ttyTHS1)")
        elif st["has_fix"]:
            record("GPS", "PASS", f"fix: {st['lat']:.5f},{st['lon']:.5f} "
                   f"sats_used={st['sats_used']}")
        else:
            record("GPS", "PASS", f"NMEA flowing ({st['nmea_lines']} lines), "
                   "no fix yet (normal indoors)")
    except Exception as exc:  # noqa: BLE001
        record("GPS", "WARN", f"{exc}  (free /dev/ttyTHS1 and retry)")


def check_vesc():
    log("\n[7/8] VESC (telemetry only — no motion)")
    try:
        from drivers.vesc import VESC, resolve_port
        if not os.path.exists(resolve_port()):
            record("VESC", "WARN", "port missing — MOTOR BATTERY off?")
            return
        v = VESC()
        try:
            fw = v.firmware()
            vals = v.get_values()
        finally:
            v.close()
        if vals:
            st = "PASS" if vals["fault"] == 0 else "WARN"
            record("VESC", st, f"fw{fw['major']}.{fw['minor']} {vals['v_in']:.1f}V "
                   f"fault={vals['fault_name']}")
        else:
            record("VESC", "FAIL", "no telemetry response")
    except Exception as exc:  # noqa: BLE001
        record("VESC", "FAIL", str(exc))


def check_steering():
    log("\n[8/8] steering (link check — re-centers servo)")
    try:
        from drivers.steering import ServoController
        srv = ServoController()
        acks = srv.status() or []
        ready = any(b.startswith("READY") for b in srv.banner)
        srv.close()
        ok = ready or any("STATUS" in a or "ANGLE" in a for a in acks)
        record("Steering", "PASS" if ok else "WARN",
               "Nano link OK" if ok else "no ack from Nano")
    except Exception as exc:  # noqa: BLE001
        record("Steering", "FAIL", str(exc))


def main():
    log("=" * 64)
    log(f"robocar — full equipment check @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)
    check_ports()
    check_cameras()
    check_lidar()
    check_compass()
    check_oled()
    check_gps()
    check_vesc()
    check_steering()

    n_pass = sum(1 for _, s, _ in results if s == "PASS")
    n_warn = sum(1 for _, s, _ in results if s == "WARN")
    n_fail = sum(1 for _, s, _ in results if s == "FAIL")
    log("\n" + "=" * 64)
    log(f"SUMMARY: {n_pass} PASS · {n_warn} WARN · {n_fail} FAIL")
    if n_fail == 0 and n_warn == 0:
        log("All systems go. 🚗")
    elif n_fail == 0:
        log("No failures — check the WARNs above (usually battery/port/indoors).")
    else:
        log("Some equipment FAILED — see above.")
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
