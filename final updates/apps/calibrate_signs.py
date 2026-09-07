#!/usr/bin/env python3
"""
apps/calibrate_signs.py — WHEELS-UP calibration of the three unknowns that decide
whether this car drives toward its goal or away from it.

  1. STEERING POLARITY   does steer(+1) actually turn the wheels RIGHT?
  2. LIDAR_FORWARD_DEG   which raw scan angle points out the car's nose?
  3. STEER_SIGN          does the scan angle sweep clockwise or counter-clockwise?

HOW IT MEASURES (and why the obvious way fails):
  The first version of this script took "the nearest return" as the box. That is
  wrong on this car. There is a fixed return at roughly 82-93 deg, 0.22-0.27 m —
  visible in lidar_nav_report.txt from 2026-08-08 and again today — which
  lidar_self_profile.json does NOT mask (it covers only 10 of 360 bearings). It is
  always closer than the box, so "nearest" reported it every time, and moving the
  box from the nose to the right flank changed the answer by 9 degrees.

  So we measure DIFFERENTIALLY instead. First a baseline profile of the world with
  nothing placed; then, with the box in position, we look for the bearings whose
  range DROPPED. Whatever is permanently near the car cancels out, because it is in
  both profiles. This is immune to the mask being wrong.

    *** WHEELS OFF THE GROUND. The motor is never commanded by this script. ***

    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 calibrate_signs.py
    python3 calibrate_signs.py --skip-steering    # if step 1 already passed

At the end it prints the config.py lines to change and offers to write them.
"""
import os
import sys
import time
import math
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                    # noqa: E402
from drivers.steering import ServoController     # noqa: E402
from drivers.lidar import ThreadedLidar          # noqa: E402

CONFIG_PATH = os.path.join(ROOT, "config.py")
SETTLE_S = 1.2

PROFILE_MAX_M = 3.0      # ignore returns past this when profiling
MIN_DROP_M = 0.15        # a bearing "changed" if its range fell by at least this
NEW_OBJ_MAX_M = 1.5      # ...or if a return appeared inside this where none was
MIN_CLUSTER_BINS = 3     # a real object subtends several bearings; noise does not
BASELINE_S = 4.0
OBJECT_S = 4.0


# --------------------------------------------------------------------------- #
#  console helpers
# --------------------------------------------------------------------------- #
def rule(title=""):
    print("\n" + "=" * 68)
    if title:
        print(title)
        print("=" * 68)


def ask(prompt, choices):
    opts = "/".join(choices)
    while True:
        a = input(f"{prompt} [{opts}]: ").strip().lower()
        if a in choices:
            return a
        print(f"   please answer one of: {opts}")


def wait(msg="press ENTER when ready"):
    input(f"   ({msg}) ")


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _circular_mean(angles_deg):
    if not angles_deg:
        return None
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(s, c)) % 360.0


# --------------------------------------------------------------------------- #
#  LiDAR profiling + differential object finding
# --------------------------------------------------------------------------- #
def profile(lid, seconds, label):
    """Per-bearing MINIMUM range over `seconds`. Returns a 360-list (inf = nothing).

    Minimum-over-time fills the gaps a single revolution leaves, and for a static
    scene it converges on the true nearest surface at each bearing.
    """
    prof = [float("inf")] * 360
    revs = 0
    last_t = -1.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        scan, age = lid.latest()
        if not scan or age > 0.5:
            time.sleep(0.02)
            continue
        stamp = time.monotonic() - age
        if stamp <= last_t:
            time.sleep(0.02)
            continue
        last_t = stamp
        revs += 1
        for _q, ang, dmm in scan:
            d = dmm / 1000.0
            if config.LIDAR_MIN_M < d < PROFILE_MAX_M:
                b = int(round(ang)) % 360
                if d < prof[b]:
                    prof[b] = d
        time.sleep(0.02)
    hits = sum(1 for v in prof if v != float("inf"))
    print(f"   {label}: {revs} revolutions, {hits}/360 bearings with a return")
    return prof, revs


def find_object(base, cur):
    """Bearings where `cur` is closer than `base` -> the thing we just placed.

    Returns (bearing_deg, distance_m, n_bins, detail) or (None, None, 0, reason).
    """
    changed = []
    for b in range(360):
        bv, cv = base[b], cur[b]
        if cv == float("inf"):
            continue
        if bv == float("inf"):
            if cv <= NEW_OBJ_MAX_M:          # appeared where there was nothing
                changed.append(b)
        elif bv - cv >= MIN_DROP_M:          # got closer by a real margin
            changed.append(b)
    if not changed:
        return None, None, 0, ("nothing moved closer — was the box actually placed, "
                               "and is it within 1.5 m?")

    # group contiguous bearings, wrapping across 0/360
    flag = [False] * 360
    for b in changed:
        flag[b] = True
    clusters, cur_run = [], []
    for b in list(range(360)) + [0]:
        if flag[b]:
            cur_run.append(b)
        elif cur_run:
            clusters.append(cur_run)
            cur_run = []
    if cur_run:
        clusters.append(cur_run)
    # merge a run ending at 359 with one starting at 0
    if len(clusters) > 1 and 359 in clusters[-1] and 0 in clusters[0]:
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()
    clusters.sort(key=len, reverse=True)
    best = clusters[0]
    if len(best) < MIN_CLUSTER_BINS:
        return None, None, len(best), (
            f"only {len(best)} bearing(s) changed — too few to be a box "
            f"(need {MIN_CLUSTER_BINS}). Probably noise.")

    bearing = _circular_mean([b % 360 for b in best])
    dist = sum(cur[b % 360] for b in best) / len(best)
    detail = ""
    if len(clusters) > 1 and len(clusters[1]) >= MIN_CLUSTER_BINS:
        second = _circular_mean([b % 360 for b in clusters[1]])
        detail = (f"note: a second changed region at ~{second:.0f} deg "
                  f"({len(clusters[1])} bearings) — something else moved too")
    return bearing, dist, len(best), detail


def measure(lid, base, what):
    """Prompt, capture, and differentially locate the object. -> bearing or None."""
    print(f"   measuring for {OBJECT_S:.0f} s ...")
    cur, revs = profile(lid, OBJECT_S, "with object")
    if revs < 3:
        print("   [FAIL] barely any scans — is another process holding the LiDAR?")
        return None, None
    bearing, dist, nbins, detail = find_object(base, cur)
    if bearing is None:
        print(f"   [FAIL] {detail}")
        return None, None
    print(f"\n   {what}: bearing {bearing:.1f} deg at {dist:.2f} m "
          f"({nbins} bearings changed)")
    if detail:
        print(f"   {detail}")
    return bearing, dist


# --------------------------------------------------------------------------- #
#  step 1 — steering polarity
# --------------------------------------------------------------------------- #
def step_steering(steer):
    rule("STEP 1 of 3 — steering polarity")
    print("""
Watch the FRONT WHEELS. "Left" means the car's own left (as if you were driving
it), not your left standing in front of it.
""")
    wait("wheels up? ENTER to centre")
    steer.center()
    time.sleep(SETTLE_S)

    print("\n   commanding norm = -1.0  (should be FULL LEFT, servo 60 deg) ...")
    steer.steer(-1.0)
    time.sleep(SETTLE_S)
    a = ask("   which way did the wheels turn?", {"left": 1, "right": 1, "neither": 1})

    steer.center()
    time.sleep(SETTLE_S)
    print("\n   commanding norm = +1.0  (should be FULL RIGHT, servo 115 deg) ...")
    steer.steer(+1.0)
    time.sleep(SETTLE_S)
    b = ask("   which way did the wheels turn?", {"left": 1, "right": 1, "neither": 1})
    steer.center()
    time.sleep(SETTLE_S)

    if "neither" in (a, b):
        print("""
   [STOP] The servo didn't move for at least one command. That is a POWER problem:
   the servo needs its own 5-6 V BEC with a common ground to the Nano.""")
        return None
    if a == "left" and b == "right":
        print("\n   [OK] Steering polarity is correct. -1 = left, +1 = right.")
        return True
    if a == "right" and b == "left":
        print("""
   [INVERTED] The wheels move opposite to the command. Fix this MECHANICALLY or in
   the driver, not with STEER_SIGN — that constant is about the LiDAR's handedness,
   and using it to paper over a mirrored linkage moves the bug rather than fixing it.

   Cleanest fix: swap these in config.py, then re-run:
       STEER_LEFT  = 115
       STEER_RIGHT = 60""")
        return False
    print("\n   [?] Inconsistent answers — re-run step 1.")
    return None


# --------------------------------------------------------------------------- #
#  step 0 — baseline
# --------------------------------------------------------------------------- #
def step_baseline(lid):
    rule("STEP 0 — baseline (what the LiDAR sees with nothing placed)")
    print("""
Everything below is measured as a CHANGE from this baseline, so whatever sits
permanently near the car — chassis, mast, wiring, a table leg — cancels out
instead of being mistaken for the box.

Take the box AWAY. Leave the car exactly where it will stay for the next two
steps; don't move or rotate it after this point. Walls and furniture can stay.
""")
    wait("area clear of anything you'll place later? ENTER to capture baseline")
    print(f"   capturing baseline for {BASELINE_S:.0f} s ...")
    base, revs = profile(lid, BASELINE_S, "baseline")
    if revs < 3:
        print("   [FAIL] barely any scans — is another process holding the LiDAR?")
        return None

    near = [(b, base[b]) for b in range(360) if base[b] < 0.5]
    if near:
        bearings = sorted(b for b, _ in near)
        print(f"\n   FYI — {len(near)} bearings have something within 0.5 m:")
        print(f"        {bearings}")
        masked = sum(1 for v in (config.load_self_mask() or [0.0] * 360) if v > 0)
        print(f"        lidar_self_profile.json masks {masked}/360 bearings.")
        if len(near) > masked:
            print("""        Some of that near stuff is UNMASKED. If it's the car's own body it will
        also be corrupting the SLAM map (those points get integrated as a wall
        that follows the car). Worth re-running calibrate_lidar_self.py after
        this. It does not affect the measurements below — they're differential.""")
    return base


# --------------------------------------------------------------------------- #
#  steps 2 & 3
# --------------------------------------------------------------------------- #
def step_forward(lid, base):
    rule("STEP 2 of 3 — LIDAR_FORWARD_DEG (where is the car's nose?)")
    print(f"""
config.py currently says LIDAR_FORWARD_DEG = {config.LIDAR_FORWARD_DEG}. This is the number
the pure-pursuit follower uses to turn map headings into "which way am I
pointing", so an error here bends every route.

Put the BOX DIRECTLY IN FRONT of the car, centred on its nose, 40-80 cm away.
Don't move the car.
""")
    wait("box dead ahead? ENTER to measure")
    bearing, _d = measure(lid, base, "object ahead")
    if bearing is None:
        return None
    delta = abs(_wrap180(bearing - config.LIDAR_FORWARD_DEG))
    print(f"   current config value: {config.LIDAR_FORWARD_DEG} deg  (off by {delta:.0f} deg)")
    if delta > 25:
        print("   --> big disagreement; the measurement is the one to trust.")
    return round(bearing, 1)


def step_sign(lid, base, forward_deg):
    rule("STEP 3 of 3 — STEER_SIGN (does the scan sweep CW or CCW?)")
    print(f"""
Every left/right decision on this car comes from the sign of a bearing measured
relative to forward ({forward_deg:.0f} deg). We settle it by putting the box where we
already know the answer.

Move the box to the car's RIGHT-HAND SIDE — right as if you were sitting in the
car driving it — level with the front wheels, 40-80 cm out. Take it off the nose.
""")
    wait("box on the car's RIGHT? ENTER to measure")
    bearing, _d = measure(lid, base, "object to the right")
    if bearing is None:
        return None
    rel = _wrap180(bearing - forward_deg)
    print(f"   relative to forward: {rel:+.1f} deg")

    if abs(abs(rel) - 90) > 40:
        print(f"""
   [WARN] That reads {abs(rel):.0f} deg off the nose, not ~90. Either the box isn't
   square to the side, or step 2's forward angle is off.""")
        if ask("   re-place the box and measure again?", {"y": 1, "n": 1}) == "y":
            return step_sign(lid, base, forward_deg)

    sign = +1.0 if rel > 0 else -1.0
    hand = ("CLOCKWISE (rel > 0 is the car's RIGHT)" if sign > 0 else
            "COUNTER-CLOCKWISE (rel > 0 is the car's LEFT)")
    print(f"\n   scan angle increases {hand}")
    print(f"   --> STEER_SIGN = {sign:+.1f}")

    # independent confirmation on the other side — cheap, and it catches a
    # mis-placed box before the number reaches the follower.
    print("\n   CONFIRMATION: now move the box to the car's LEFT side, same distance.")
    if ask("   ready to confirm?", {"y": 1, "skip": 1}) == "y":
        wait("box on the car's LEFT? ENTER to measure")
        lb, _d = measure(lid, base, "object to the left")
        if lb is not None:
            lrel = _wrap180(lb - forward_deg)
            print(f"   relative to forward: {lrel:+.1f} deg")
            if (lrel < 0) == (rel > 0):
                print("   [OK] left and right land on opposite signs — consistent.")
            else:
                print(f"""
   [FAIL] Left ({lrel:+.0f}) and right ({rel:+.0f}) came out on the SAME side. One of the
   placements was wrong, or the forward angle is off. Do not trust STEER_SIGN from
   this run — re-run the whole script.""")
                return None
    return sign


# --------------------------------------------------------------------------- #
def write_config(forward_deg, sign):
    import re
    src = open(CONFIG_PATH).read()
    original = src
    if forward_deg is not None:
        src = re.sub(r"^LIDAR_FORWARD_DEG = [-\d.]+.*$",
                     f"LIDAR_FORWARD_DEG = {forward_deg}    "
                     f"# MEASURED by apps/calibrate_signs.py",
                     src, count=1, flags=re.M)
    if sign is not None:
        src = re.sub(r"^STEER_SIGN = [-\d.]+.*$",
                     f"STEER_SIGN = {sign}    # MEASURED by apps/calibrate_signs.py",
                     src, count=1, flags=re.M)
        src = re.sub(r"^STEER_SIGN_VERIFIED = .*$",
                     "STEER_SIGN_VERIFIED = True   # measured, not guessed",
                     src, count=1, flags=re.M)
    if src == original:
        print("   [!] nothing matched in config.py — edit it by hand (lines above).")
        return False
    with open(CONFIG_PATH + ".bak", "w") as fh:
        fh.write(original)
    with open(CONFIG_PATH, "w") as fh:
        fh.write(src)
    print("   config.py updated (previous version saved as config.py.bak)")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-steering", action="store_true",
                    help="skip step 1 if polarity already checked out")
    args = ap.parse_args()

    rule("RoboCar — wheels-up sign & forward calibration")
    print("""
The MOTOR IS NEVER COMMANDED by this script — only the steering servo moves. Keep
the wheels off the ground anyway.

You'll need: the car on a stand it won't shift on, and one box / thick book.
Measurements are differential, so DON'T MOVE THE CAR once step 0 starts.
""")
    if ask("ready?", {"y": 1, "n": 1}) == "n":
        return 0

    steer = None
    lid = None
    forward_deg = sign = None
    try:
        if args.skip_steering:
            print("\nskipping step 1 (polarity already confirmed).")
        else:
            print("\nopening steering ...")
            steer = ServoController()
            if step_steering(steer) is not True:
                print("\nStopping — fix step 1 before the LiDAR steps mean anything.")
                return 1
            steer.center(); steer.close(); steer = None

        print("\nstarting LiDAR (a few seconds to spin up) ...")
        lid = ThreadedLidar().start()
        time.sleep(1.5)
        s, _age = lid.latest()
        if not s:
            print("   [FAIL] No scans. Another process probably holds the port:")
            print("          pkill -f cockpit.py ; pkill -f navigate_web.py")
            return 1

        base = step_baseline(lid)
        if base is None:
            return 1
        forward_deg = step_forward(lid, base)
        if forward_deg is None:
            return 1
        sign = step_sign(lid, base, forward_deg)
        if sign is None:
            return 1

    except KeyboardInterrupt:
        print("\naborted.")
        return 1
    finally:
        if steer:
            try:
                steer.center(); steer.close()
            except Exception:
                pass
        if lid:
            lid.stop()

    rule("RESULTS")
    print(f"""
   config.py should read:

       LIDAR_FORWARD_DEG = {forward_deg}
       STEER_SIGN        = {sign:+.1f}

   Was:
       LIDAR_FORWARD_DEG = {config.LIDAR_FORWARD_DEG}
       STEER_SIGN        = {config.STEER_SIGN:+.1f}
""")
    if ask("write these into config.py now?", {"y": 1, "n": 1}) == "y":
        write_config(forward_deg, sign)
    else:
        print("   left unchanged — copy the two lines above in by hand.")

    print("""
NEXT (still wheels-up):
   1. If step 0 flagged unmasked near returns, re-run calibrate_lidar_self.py —
      an unmasked chassis return is a wall that follows the car through SLAM.
   2. python3 cockpit.py -> NAVIGATE, load the map, click a goal, GO. Watch which
      way the wheels point. They won't go anywhere on a stand; direction is the test.
   3. Unplug the LiDAR mid-run: the note should become "stopped: no LiDAR".
   4. Only then, wheels down, short goal, finger on SPACE.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
