#!/usr/bin/env python3
"""
apps/sysid.py — system identification: measure what this car actually does.

WHY
---
A simulator tuned to guessed numbers produces a policy that transfers to a car
that does not exist. Four measurements make a simulated car behave like THIS car:

  1. duty -> steady-state speed      (there is no such mapping anywhere yet)
  2. throttle rise time              (first-order lag from a duty step)
  3. steer_norm -> turn radius       (gives the REAL max steer angle;
                                      config.MAX_STEER_ANGLE_RAD = 0.45 is a guess)
  4. actuation delay                 (named as a primary sim-to-real failure cause
                                      in three separate papers; nobody measures it)

This script runs scripted manoeuvres and records each one with the recording
layer. `recording/sysid_fit.py` then reads the sessions and prints the numbers.

SAFETY — read this before running on the floor
----------------------------------------------
This is the car driving itself under script. Every run is:
  * capped at --max-duty (default 0.12),
  * bounded by a hard per-run timeout,
  * preceded by an explicit y/N confirmation and a 3-2-1 countdown,
  * guarded by a LiDAR watchdog that cuts throttle if anything comes within
    --guard-m ahead (default 1.2 m) -- unless --no-guard,
  * followed by an unconditional stop + centre, including on Ctrl-C or a crash.

Run the manoeuvres in this order. The first needs no floor space at all:

    python3 sysid.py latency   --stand      # WHEELS UP. Safe. Do this first.
    python3 sysid.py throttle                # needs a clear straight ~5 m
    python3 sysid.py steering                # needs ~3 m x 3 m of open floor

    python3 sysid.py all                     # all three, in order
    python3 sysid.py all --dry               # no hardware; exercises the logic

Nothing else may hold the ports:
    pkill -f cockpit.py ; pkill -f navigate_web.py ; pkill -f mapper_web.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import config                                                    # noqa: E402
from recording.recorder import Recorder                          # noqa: E402

# ----------------------------------------------------------------- defaults --
TICK_HZ = 50.0          # default control/log rate for the driving manoeuvres
# The latency manoeuvre is the one measurement whose resolution IS the sample
# period: at 50 Hz the answer comes out quantised to 20 ms, which happens to be
# the same size as the delay we are trying to measure. You cannot resolve a
# thing by sampling it at its own scale. Wheels are up and nothing is moving,
# so there is no reason not to sample fast here.
LAT_TICK_HZ = 200.0
SETTLE = 0.005          # VESC reply wait; see drivers/vesc.py get_values()

THROTTLE_DUTIES = [0.06, 0.07, 0.08, 0.09, 0.10, 0.12]
THROTTLE_DRIVE_S = 1.5
THROTTLE_COAST_S = 2.0   # coast-down after the step -> drag / rolling resistance

STEER_VALUES = [-1.0, -0.6, -0.3, 0.3, 0.6, 1.0]
STEER_DUTY = 0.08
STEER_SETTLE_S = 0.8     # let the servo reach the angle BEFORE moving
STEER_DRIVE_S = 4.0

LAT_DUTY = 0.09          # wheels-up square wave
LAT_CYCLES = 6
LAT_ON_S = 0.6
LAT_OFF_S = 0.9
LAT_STEER_CYCLES = 6
LAT_STEER_ON_S = 0.7


def banner(msg):
    print(f"\n{'=' * 68}\n  {msg}\n{'=' * 68}")


def confirm(prompt, auto=False):
    if auto:
        print(f"  [auto] {prompt} -> yes")
        return True
    try:
        return input(f"  {prompt} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def countdown(n=3):
    for i in range(n, 0, -1):
        print(f"  {i}...", flush=True)
        time.sleep(1.0)
    print("  GO", flush=True)


# ------------------------------------------------------------------- the rig --
class Rig:
    """Owns the hardware for the duration of a sysid session.

    Fails safe on every exit path: duty 0, motor stopped, wheels centred.
    """

    def __init__(self, max_duty, guard_m, dry=False, use_lidar=True):
        self.dry = dry
        self.max_duty = min(abs(max_duty), config.MAX_DUTY)
        self.guard_m = guard_m
        self.vesc = None
        self.steer = None
        self.lidar = None
        self.use_lidar = use_lidar
        self._sim = {"v": 0.0, "tach": 0.0, "yaw": 0.0, "steer": 0.0,
                     "x": 0.0, "y": 0.0, "t_scan": 0.0, "seq": 0}

    def open(self):
        if self.dry:
            print("  [dry] no hardware opened; simulating a plausible car")
            return self
        from drivers.vesc import VESC
        from drivers.steering import ServoController
        self.vesc = VESC()
        fw = self.vesc.firmware()
        print(f"  VESC  : {self.vesc.port}  fw {fw['major']}.{fw['minor'] if fw else '?'}"
              if fw else f"  VESC  : {self.vesc.port}  (no firmware reply)")
        self.steer = ServoController()
        print(f"  STEER : {self.steer.port}")
        self.steer.center(read_reply=False)
        if self.use_lidar:
            try:
                from drivers.lidar import ThreadedLidar
                self.lidar = ThreadedLidar().start()
                print("  LIDAR : up (safety guard + yaw-rate source)")
            except Exception as e:                               # noqa: BLE001
                print(f"  LIDAR : unavailable ({e}) -- guard DISABLED")
                self.lidar = None
        return self

    def close(self):
        try:
            if self.vesc:
                self.vesc.set_duty(0.0); self.vesc.stop(); self.vesc.close()
        except Exception:                                        # noqa: BLE001
            pass
        try:
            if self.steer:
                self.steer.center(read_reply=False); time.sleep(0.2); self.steer.close()
        except Exception:                                        # noqa: BLE001
            pass
        try:
            if self.lidar:
                self.lidar.stop()
        except Exception:                                        # noqa: BLE001
            pass

    # ------------------------------------------------------------ actuation --
    def set_duty(self, d):
        d = max(-self.max_duty, min(self.max_duty, d))
        if self.dry:
            return d
        try:
            self.vesc.set_duty(d)
        except Exception:                                        # noqa: BLE001
            pass
        return d

    def set_steer(self, s):
        s = max(-1.0, min(1.0, s))
        self._sim["steer"] = s
        if not self.dry and self.steer:
            try:
                self.steer.steer(s, read_reply=False)
            except Exception:                                    # noqa: BLE001
                pass
        return s

    # ----------------------------------------------------------- telemetry --
    def telemetry(self, dt, duty):
        if self.dry:
            # first-order speed response + a plausible duty->speed gain, so the
            # whole pipeline (script -> log -> fit) can be exercised offline.
            tau, gain, dead = 0.35, 14.0, 0.045
            target = max(0.0, (abs(duty) - dead)) * gain * (1 if duty >= 0 else -1)
            s = self._sim
            s["v"] += (target - s["v"]) * (dt / tau)
            s["tach"] += s["v"] * dt / config.METERS_PER_TACH
            if abs(s["v"]) > 1e-3:
                delta = s["steer"] * 0.38          # "true" max steer for the sim car
                s["yaw"] += (s["v"] / 0.32) * math.tan(delta) * dt
                s["x"] += s["v"] * math.cos(s["yaw"]) * dt
                s["y"] += s["v"] * math.sin(s["yaw"]) * dt
            return {"erpm": s["v"] * 900.0, "tach": int(s["tach"]), "v_in": 11.8,
                    "motor_current": 4.0, "temp_mos": 31.0, "temp_motor": 0.0,
                    "fault": "NONE", "duty": duty}
        try:
            v = self.vesc.get_values(settle=SETTLE)
        except Exception:                                        # noqa: BLE001
            v = None
        if not v:
            return {}
        return {"erpm": v["erpm"], "tach": v["tach"], "v_in": v["v_in"],
                "motor_current": v["motor_current"], "temp_mos": v["temp_mos"],
                "temp_motor": v["temp_motor"], "fault": v["fault_name"],
                "duty": v["duty"]}

    # ---------------------------------------------------------- lidar guard --
    def nearest_ahead(self):
        """Metres to the closest return inside the front arc, or inf."""
        if self.lidar is None:
            return float("inf")
        scan, age = self.lidar.latest()
        if not scan or age > 0.5:
            return float("inf")
        fwd = config.LIDAR_FORWARD_DEG
        half = config.LIDAR_FRONT_ARC_DEG / 2.0
        best = float("inf")
        for _q, ang, dist in scan:
            d = dist / 1000.0
            if d < config.LIDAR_MIN_M:
                continue
            rel = ((ang - fwd + 180.0) % 360.0) - 180.0
            if abs(rel) <= half and d < best:
                best = d
        return best

    def latest_scan(self):
        if self.dry:
            return self._sim_scan()
        if self.lidar is None:
            return None, float("inf")
        return self.lidar.latest()

    # A synthetic 360-degree scan of a rectangular room, ray-cast from the
    # simulated pose. Only used in --dry, but it is what lets the OFFLINE test
    # exercise the real yaw-rate path (scan -> ICP -> omega) instead of only the
    # bookkeeping around it.
    _ROOM = (-2.5, 2.5, -2.5, 2.5)      # xmin, xmax, ymin, ymax (metres)

    def _sim_scan(self):
        now = time.monotonic()
        if now - self._sim["t_scan"] < 0.10:        # C1 runs at ~10 Hz
            return None, float("inf")
        self._sim["t_scan"] = now
        x0, y0, th = self._sim["x"], self._sim["y"], self._sim["yaw"]
        xmin, xmax, ymin, ymax = self._ROOM
        scan = []
        for i in range(500):                        # ~500 pts/rev, like the C1
            a_scan = i * 360.0 / 500.0
            a = math.radians(a_scan) + th + math.radians(config.LIDAR_FORWARD_DEG)
            cx, sy = math.cos(a), math.sin(a)
            best = float("inf")
            for bound, comp, val in ((cx, x0, xmax), (cx, x0, xmin),
                                     (sy, y0, ymax), (sy, y0, ymin)):
                if abs(bound) > 1e-9:
                    t = (val - comp) / bound
                    if 0 < t < best:
                        px = x0 + cx * t
                        py = y0 + sy * t
                        if xmin - 1e-6 <= px <= xmax + 1e-6 and \
                           ymin - 1e-6 <= py <= ymax + 1e-6:
                            best = t
            if best < float("inf"):
                scan.append((15, a_scan, best * 1000.0))
        self._sim["seq"] += 1
        return scan, 0.0


# ------------------------------------------------------------------- runner --
def run_segment(rig, rec, segments, label, guard=True, on_tick=None, tick_hz=None):
    """Execute a list of (duration_s, duty, steer, phase) at tick_hz, logging.

    Returns (completed, reason). Fails safe: duty 0 on every exit path.
    """
    dt_nom = 1.0 / (tick_hz or TICK_HZ)
    total = sum(s[0] for s in segments)
    deadline = time.monotonic() + total + 2.0
    last_steer = None
    last_t = time.monotonic()
    scan_seq = None
    last_arrival = 0.0
    reason = "ok"
    try:
        for dur, duty, steer, phase in segments:
            t_end = time.monotonic() + dur
            while time.monotonic() < t_end:
                now = time.monotonic()
                if now > deadline:
                    return False, "run timeout"
                if guard and duty != 0.0:
                    near = rig.nearest_ahead()
                    if near < rig.guard_m:
                        rig.set_duty(0.0)
                        return False, f"LIDAR GUARD: obstacle at {near:.2f} m"
                if steer != last_steer:
                    rig.set_steer(steer)
                    last_steer = steer
                applied = rig.set_duty(duty)
                dt = now - last_t
                last_t = now
                tel = rig.telemetry(dt if dt > 0 else dt_nom, applied)

                # Log a NEW lidar revolution if one arrived (yaw-rate source).
                scan, age = rig.latest_scan()
                if scan and age < 1.0:
                    arrival = now - age
                    if arrival > last_arrival + 1e-6:
                        last_arrival = arrival
                        scan_seq = rec.log_scan(scan, t=rec.now() - age)

                rec.log({"cmd_duty": applied, "cmd_steer": steer,
                         "mode": f"{label}:{phase}", "armed": True, "estop": False,
                         "duty_actual": tel.get("duty"), "erpm": tel.get("erpm"),
                         "tach": tel.get("tach"), "v_in": tel.get("v_in"),
                         "motor_current": tel.get("motor_current"),
                         "temp_mos": tel.get("temp_mos"), "fault": tel.get("fault"),
                         "scan_seq": scan_seq})
                if on_tick:
                    on_tick(tel)
                slp = dt_nom - (time.monotonic() - now)
                if slp > 0:
                    time.sleep(slp)
    except KeyboardInterrupt:
        reason = "interrupted by operator"
        return False, reason
    finally:
        rig.set_duty(0.0)
    return True, reason


# --------------------------------------------------------------- manoeuvres --
def man_latency(rig, args):
    banner("LATENCY  —  WHEELS UP, ON A STAND")
    print("  Square-waves throttle and steering to measure actuation delay:")
    print("  how long after a command the motor / servo actually responds.")
    print("  This is the number every failed sim-to-real transfer skipped.\n")
    print("  REQUIREMENT: car on a stand, wheels free to spin. No floor space.")
    if not confirm("Wheels are OFF the ground and clear?", args.auto):
        return None

    segs = [(0.5, 0.0, 0.0, "rest")]
    for _ in range(LAT_CYCLES):
        segs.append((LAT_ON_S, LAT_DUTY, 0.0, "on"))
        segs.append((LAT_OFF_S, 0.0, 0.0, "off"))
    segs.append((0.5, 0.0, 0.0, "rest"))
    for i in range(LAT_STEER_CYCLES):
        segs.append((LAT_STEER_ON_S, 0.0, 1.0 if i % 2 == 0 else -1.0, "steerstep"))
    segs.append((0.6, 0.0, 0.0, "rest"))

    rec = Recorder(note="sysid latency (wheels up)", source="sysid").start()
    countdown(3)
    ok, why = run_segment(rig, rec, segs, "latency", guard=False,
                          tick_hz=args.tick_hz or LAT_TICK_HZ)
    st = rec.stop()
    print(f"  {'done' if ok else 'ABORTED: ' + why}   -> {os.path.basename(st['dir'])}")
    return st["dir"]


def man_throttle(rig, args):
    banner("THROTTLE  —  duty to speed, and rise time")
    print("  For each duty: step from rest, hold, then coast to a stop.")
    print("  The hold gives steady-state speed and rise time; the coast gives drag.\n")
    print(f"  REQUIREMENT: a clear straight run. At {max(THROTTLE_DUTIES):.2f} duty for "
          f"{THROTTLE_DRIVE_S:.1f}s plus coast, allow ~5 m.")
    print("  Keep a hand ready. Ctrl-C stops everything immediately.")
    dirs = []
    for duty in THROTTLE_DUTIES:
        print(f"\n  --- duty {duty:.2f} ---")
        if not confirm(f"Car at the start, path clear? (duty {duty:.2f})", args.auto):
            print("  skipped.")
            continue
        segs = [(0.4, 0.0, 0.0, "rest"),
                (THROTTLE_DRIVE_S, duty, 0.0, "step"),
                (THROTTLE_COAST_S, 0.0, 0.0, "coast")]
        rec = Recorder(note=f"sysid throttle duty={duty:.3f}", source="sysid").start()
        countdown(3)
        ok, why = run_segment(rig, rec, segs, f"throttle_{duty:.3f}",
                              guard=not args.no_guard)
        st = rec.stop()
        dirs.append(st["dir"])
        print(f"  {'done' if ok else 'ABORTED: ' + why}   -> {os.path.basename(st['dir'])}")
        if not ok and "GUARD" in why:
            print("  (guard tripped -- more space, or lower --guard-m)")
    return dirs


def man_steering(rig, args):
    banner("STEERING  —  steer command to turn radius")
    print("  Drives a steady arc at each steering value. Turn radius comes from")
    print("  yaw rate (LiDAR scan matching) divided by speed (wheel odometry),")
    print("  which yields the REAL max steer angle instead of the guess in config.\n")
    print("  REQUIREMENT: ~3 m x 3 m of open floor. The car will drive in circles.")
    if rig.lidar is None and not rig.dry:
        print("\n  !! No LiDAR. Yaw rate cannot be measured automatically.")
        print("     Fallback: run it anyway, then tape-measure the circle the car")
        print("     drove and pass --radius-m to sysid_fit.py.")
    dirs = []
    for s in STEER_VALUES:
        print(f"\n  --- steer {s:+.2f} ---")
        if not confirm(f"Open floor around the car? (steer {s:+.2f})", args.auto):
            print("  skipped.")
            continue
        segs = [(0.3, 0.0, 0.0, "rest"),
                (STEER_SETTLE_S, 0.0, s, "settle"),
                (STEER_DRIVE_S, STEER_DUTY, s, "arc"),
                (1.2, 0.0, s, "coast"),
                (0.3, 0.0, 0.0, "center")]
        rec = Recorder(note=f"sysid steering steer={s:+.3f}", source="sysid").start()
        countdown(3)
        ok, why = run_segment(rig, rec, segs, f"steering_{s:+.3f}",
                              guard=not args.no_guard)
        st = rec.stop()
        dirs.append(st["dir"])
        print(f"  {'done' if ok else 'ABORTED: ' + why}   -> {os.path.basename(st['dir'])}")
    return dirs


# --------------------------------------------------------------------- main --
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="System identification runs for the car.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Then:  python3 recording/sysid_fit.py --all")
    ap.add_argument("maneuver", choices=["latency", "throttle", "steering", "all"])
    ap.add_argument("--max-duty", type=float, default=0.12,
                    help="hard cap on commanded duty (default 0.12)")
    ap.add_argument("--guard-m", type=float, default=1.2,
                    help="LiDAR guard distance in metres (default 1.2)")
    ap.add_argument("--no-guard", action="store_true",
                    help="disable the LiDAR obstacle guard (not recommended)")
    ap.add_argument("--stand", action="store_true",
                    help="wheels-up: skips the floor-space warnings")
    ap.add_argument("--auto", action="store_true",
                    help="do not prompt between runs (only with --dry or on a stand)")
    ap.add_argument("--tick-hz", type=float, default=None,
                    help="override the sample/control rate (latency defaults to "
                         f"{LAT_TICK_HZ:.0f} Hz, driving runs to {TICK_HZ:.0f} Hz)")
    ap.add_argument("--dry", action="store_true",
                    help="no hardware; simulate a plausible car to exercise the pipeline")
    args = ap.parse_args(argv)

    if args.auto and not (args.dry or args.stand):
        print("refusing --auto on the floor: each run needs a human to confirm the "
              "path is clear. Add --stand (wheels up) or --dry.")
        return 2

    banner("SYSTEM IDENTIFICATION")
    print(f"  max duty : {min(args.max_duty, config.MAX_DUTY):.3f}"
          f"   (config.MAX_DUTY = {config.MAX_DUTY})")
    print(f"  guard    : {'OFF' if args.no_guard else f'{args.guard_m:.2f} m ahead'}")
    print(f"  mode     : {'DRY (no hardware)' if args.dry else 'LIVE'}")
    print(f"  sampling : {args.tick_hz or TICK_HZ:.0f} Hz driving, "
          f"{args.tick_hz or LAT_TICK_HZ:.0f} Hz latency "
          f"(-> {1000.0 / (args.tick_hz or LAT_TICK_HZ):.0f} ms resolution)")
    print("\n  opening hardware ...")

    rig = Rig(args.max_duty, args.guard_m, dry=args.dry,
              use_lidar=not args.dry)
    dirs = []
    try:
        rig.open()
        if args.maneuver in ("latency", "all"):
            d = man_latency(rig, args)
            if d:
                dirs.append(d)
        if args.maneuver in ("throttle", "all"):
            dirs += man_throttle(rig, args)
        if args.maneuver in ("steering", "all"):
            dirs += man_steering(rig, args)
    except KeyboardInterrupt:
        print("\n  interrupted -- stopping.")
    finally:
        rig.close()
        print("\n  hardware safe: duty 0, motor stopped, wheels centred.")

    banner("DONE")
    if dirs:
        print(f"  {len(dirs)} session(s) recorded. Now fit them:\n")
        print("      cd \"final updates\"")
        print("      python3 recording/sysid_fit.py --all\n")
    else:
        print("  no sessions recorded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
