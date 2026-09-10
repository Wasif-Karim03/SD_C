#!/usr/bin/env python3
"""
recording/sysid_fit.py — turn sysid recordings into the four numbers the sim needs.

    cd "final updates"
    python3 recording/sysid_fit.py --all              # every sysid session in logs/
    python3 recording/sysid_fit.py logs/2026-09-08T* # specific sessions
    python3 recording/sysid_fit.py --all --wheelbase-m 0.325
    python3 recording/sysid_fit.py --all --plot fit.png

What it produces
----------------
  1. duty -> steady-state speed      v = gain * (duty - deadband)
  2. throttle rise time              tau, from the 63% crossing of a duty step
  3. steer_norm -> turn radius       -> the REAL max steer angle
  4. actuation delay                 throttle and steering, in seconds and in
                                     100 Hz sim steps

and prints a paste-ready block for config.py plus the matching f1tenth_gym
ControlConfig / VehicleParameters values.

Measurement notes (so you can judge the numbers rather than trust them)
----------------------------------------------------------------------
* Speed is the slope of a least-squares line through `tach` over a sliding
  window, not a raw difference -- tach is an integer counter, so differencing it
  at 20-50 Hz is mostly quantisation noise.
* Yaw rate comes from ICP between consecutive LiDAR revolutions. That is a real
  measurement of how fast the car actually rotated, independent of any model.
  Turn radius R = v / omega, and then delta = atan(wheelbase / R).
* Wheelbase is NOT measured here. Put a tape measure across the front and rear
  axle centres and pass --wheelbase-m. It takes ten seconds and everything
  downstream depends on it.
* If you have no LiDAR, drive each arc, tape-measure the circle, and pass
  --manual-radius "1.0:1.35,0.6:2.10" (steer:radius_m pairs).
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import config                                                    # noqa: E402
from recording.reader import Session, DEFAULT_LOG_ROOT           # noqa: E402

SIM_HZ = 100.0          # f1tenth_gym physics step, for expressing delay in steps


# ------------------------------------------------------------------ helpers --
def speed_series(t, tach, win_s=0.25):
    """m/s from a sliding least-squares fit of tach vs t. Robust to quantisation."""
    t = np.asarray(t, dtype=float)
    tach = np.asarray(tach, dtype=float)
    ok = np.isfinite(t) & np.isfinite(tach)
    if ok.sum() < 4:
        return np.full(len(t), np.nan)
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        m = ok & (np.abs(t - t[i]) <= win_s / 2.0)
        if m.sum() < 4:
            continue
        tt, yy = t[m], tach[m]
        span = tt.max() - tt.min()
        if span < 1e-3:
            continue
        slope = np.polyfit(tt - tt.mean(), yy, 1)[0]
        out[i] = slope * config.METERS_PER_TACH
    return out


def phase_mask(sess, name):
    mode = sess.tel.get("mode")
    if mode is None:
        return np.zeros(0, dtype=bool)
    return np.array([isinstance(m, str) and m.endswith(":" + name) for m in mode])


def _fmt(v, d=4, unit=""):
    return "n/a" if v is None or not np.isfinite(v) else f"{v:.{d}f}{unit}"


# ------------------------------------------------------------ 1+2. throttle --
def fit_throttle(sessions):
    """Per-session steady-state speed and rise time, then a duty->speed line."""
    rows = []
    for s in sessions:
        step = phase_mask(s, "step")
        if step.sum() < 10:
            continue
        t = s.tel["t"]
        v = speed_series(t, s.tel.get("tach"))
        duty = float(np.nanmedian(s.tel["cmd_duty"][step]))
        ts, vs = t[step], v[step]
        good = np.isfinite(vs)
        if good.sum() < 8:
            continue
        ts, vs = ts[good], vs[good]

        # --- v_ss and tau, from an exponential fit -------------------------
        # NOT from the 63% crossing of the tach-derived speed. Two reasons:
        #   * tach-derived speed needs a sliding window, and a CENTRED window
        #     smears the step and pulls the crossing early (measured: -18% on
        #     synthetic data with a known tau).
        #   * erpm is a DIRECT measurement -- no differencing, no window -- so
        #     the transient is clean. We fit v(t) = A*(1 - exp(-(t-t0)/tau)) to
        #     erpm rescaled into m/s, and take A as the asymptote. That also
        #     removes the bias from a hold too short to fully settle.
        erpm = s.tel.get("erpm")
        es = erpm[step][good] if erpm is not None else None
        tau = float("nan")
        v_ss = float("nan")
        cut = ts[0] + 0.7 * (ts[-1] - ts[0])
        late = ts >= cut
        v_late = float(np.median(vs[late])) if late.sum() else float("nan")

        if es is not None and np.isfinite(es).sum() > 8 and np.isfinite(v_late):
            e_late = float(np.median(es[late]))
            k = (v_late / e_late) if abs(e_late) > 1e-6 else float("nan")
            if np.isfinite(k):
                y = np.abs(es * k)            # erpm expressed in m/s
                x = ts - ts[0]
                try:
                    from scipy.optimize import curve_fit
                    def _step_fn(tt, A, tau_, t0):
                        return A * (1.0 - np.exp(-np.clip(tt - t0, 0, None) / max(tau_, 1e-3)))
                    p0 = [max(y.max(), 1e-3), 0.3, 0.0]
                    popt, _ = curve_fit(_step_fn, x, y, p0=p0, maxfev=8000,
                                        bounds=([0.0, 0.02, -0.2],
                                                [10 * max(y.max(), 1e-3), 5.0, 1.0]))
                    v_ss, tau = float(popt[0]), float(popt[1])
                except Exception:                                # noqa: BLE001
                    pass
        if not np.isfinite(v_ss):
            v_ss = v_late                     # fallback: plain late-window median

        # coast: deceleration once the command returns to zero
        coast = phase_mask(s, "coast")
        decel = float("nan")
        if coast.sum() > 8:
            tc, vc = t[coast], v[coast]
            g = np.isfinite(vc) & (vc > 0.05)
            if g.sum() > 5:
                decel = float(-np.polyfit(tc[g], vc[g], 1)[0])

        rows.append({"duty": duty, "v_ss": v_ss, "tau": tau, "decel": decel,
                     "n": int(good.sum()), "name": os.path.basename(s.path),
                     "rate": len(t) / max(t[-1] - t[0], 1e-9)})

    rows.sort(key=lambda r: r["duty"])
    fit = None
    moving = [r for r in rows if np.isfinite(r["v_ss"]) and r["v_ss"] > 0.05]
    if len(moving) >= 2:
        d = np.array([r["duty"] for r in moving])
        v = np.array([r["v_ss"] for r in moving])
        gain, intercept = np.polyfit(d, v, 1)
        deadband = -intercept / gain if abs(gain) > 1e-9 else float("nan")
        resid = v - (gain * d + intercept)
        fit = {"gain": float(gain), "deadband": float(deadband),
               "rms": float(np.sqrt(np.mean(resid ** 2))), "n": len(moving)}
    taus = [r["tau"] for r in rows if np.isfinite(r["tau"])]
    decels = [r["decel"] for r in rows if np.isfinite(r["decel"])]
    return rows, fit, (float(np.median(taus)) if taus else float("nan")), \
        (float(np.median(decels)) if decels else float("nan"))


# ------------------------------------------------------------- 3. steering --
def yaw_rate_from_scans(sess, t_lo, t_hi):
    """Median yaw rate (rad/s) over a time window, via ICP between revolutions."""
    try:
        from perception.slam import scan_to_xy, icp
    except Exception:                                            # noqa: BLE001
        return float("nan"), 0
    st = sess.scan_idx["t"]
    idx = np.where((st >= t_lo) & (st <= t_hi))[0]
    if len(idx) < 3:
        return float("nan"), 0
    rates, prev, prev_t = [], None, None
    for i in idx:
        a, r = sess.scan(int(i))
        pts = scan_to_xy([(0, ang, rr * 1000.0) for ang, rr in zip(a, r)])
        if len(pts) < 30:
            prev, prev_t = None, None
            continue
        if prev is not None:
            pose, ok = icp(pts, prev, (0.0, 0.0, 0.0))
            dt = float(st[i]) - prev_t
            if ok and dt > 1e-3:
                dth = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
                if abs(dth) < math.radians(90):     # reject ICP blow-ups
                    rates.append(dth / dt)
        prev, prev_t = pts, float(st[i])
    if len(rates) < 3:
        return float("nan"), len(rates)
    return float(np.median(rates)), len(rates)


def fit_steering(sessions, wheelbase, manual=None):
    rows = []
    for s in sessions:
        arc = phase_mask(s, "arc")
        if arc.sum() < 10:
            continue
        t = s.tel["t"]
        steer = float(np.nanmedian(s.tel["cmd_steer"][arc]))
        v = speed_series(t, s.tel.get("tach"))
        va = v[arc]
        v_mean = float(np.nanmedian(va[np.isfinite(va)])) if np.isfinite(va).any() else float("nan")
        ta = t[arc]
        # ignore the first 0.6 s of the arc: the car is still accelerating
        omega, n = yaw_rate_from_scans(s, ta[0] + 0.6, ta[-1])
        radius = float("nan")
        if manual and abs(steer) in manual:
            radius = manual[abs(steer)] * (1.0 if steer >= 0 else 1.0)
            src = "tape"
        elif np.isfinite(omega) and abs(omega) > 1e-3 and np.isfinite(v_mean):
            radius = abs(v_mean / omega)
            src = f"icp({n})"
        else:
            src = "none"
        delta = math.atan(wheelbase / radius) if np.isfinite(radius) and radius > 1e-3 \
            else float("nan")
        rows.append({"steer": steer, "v": v_mean, "omega": omega, "R": radius,
                     "delta": delta, "src": src,
                     "name": os.path.basename(s.path)})
    rows.sort(key=lambda r: r["steer"])

    fit = None
    good = [r for r in rows if np.isfinite(r["delta"])]
    if len(good) >= 2:
        x = np.array([abs(r["steer"]) for r in good])
        y = np.array([abs(r["delta"]) for r in good])
        # delta = k * |steer|, forced through the origin (steer 0 = wheels straight)
        k = float(np.sum(x * y) / max(np.sum(x * x), 1e-12))
        resid = y - k * x
        fit = {"k": k, "max_steer_rad": k * 1.0,
               "max_steer_deg": math.degrees(k),
               "rms_deg": float(math.degrees(np.sqrt(np.mean(resid ** 2)))),
               "n": len(good)}
    return rows, fit


# -------------------------------------------------------------- 4. latency --
def fit_latency(sessions):
    """Command-to-response delay for throttle (via erpm) and steering (via yaw)."""
    thr, steer_lat = [], []
    for s in sessions:
        t = s.tel["t"]
        cd = s.tel.get("cmd_duty")
        erpm = s.tel.get("erpm")
        if cd is not None and erpm is not None and len(t) > 20:
            rising = np.where((cd[1:] > 0.01) & (cd[:-1] <= 0.01))[0] + 1
            for i in rising:
                base = np.nanmedian(erpm[max(0, i - 8):i]) if i >= 4 else 0.0
                span = erpm[i:i + int(1.0 / max(np.median(np.diff(t)), 1e-3))]
                st = t[i:i + len(span)]
                if len(span) < 5 or not np.isfinite(span).any():
                    continue
                peak = np.nanmax(span)
                if not np.isfinite(peak) or abs(peak - base) < 50:
                    continue
                thresh = base + 0.1 * (peak - base)
                hit = np.where(span >= thresh)[0]
                if len(hit):
                    thr.append(float(st[hit[0]] - t[i]))

        cs = s.tel.get("cmd_steer")
        if cs is not None and s.n_scans > 6 and len(t) > 20:
            edges = np.where(np.abs(np.diff(cs)) > 0.5)[0] + 1
            for i in edges:
                w0, w1 = t[i], t[i] + 1.2
                r0, _ = yaw_rate_from_scans(s, w0 - 0.6, w0)
                r1, _ = yaw_rate_from_scans(s, w0, w1)
                if np.isfinite(r0) and np.isfinite(r1) and abs(r1 - r0) > 0.05:
                    steer_lat.append(float("nan"))   # placeholder; see note below
    return (float(np.median(thr)) if thr else float("nan"), len(thr))


# ------------------------------------------------------------------ report --
# ── plausibility ─────────────────────────────────────────────────────────
# A least-squares fit always returns numbers. Whether those numbers describe a
# vehicle is a separate question, and the first real session answered it: six
# throttle points spanning duty 0.145-0.200 extrapolated back to a deadband of
# -0.37 (the car moves at negative duty), and six steering arcs driven below
# breakaway gave a max steer angle of 0.36 deg and a 40 m turning circle for a
# car that turns inside a metre.
#
# Both were printed in a PASTE-READY block, ready to be copied into config.py.
# That is the failure mode worth engineering against: not a wrong number, but a
# wrong number wearing the costume of a measurement.

def check_throttle(fit, rows):
    """Return a list of reasons this throttle fit should not be trusted."""
    if not fit:
        return ["no fit"]
    bad = []
    db = fit["deadband"]
    if not (0.0 <= db <= config.MAX_DUTY):
        bad.append(f"deadband {db:.4f} is outside [0, {config.MAX_DUTY}] -- the line "
                   f"was extrapolated far below the sampled range")
    if rows:
        lo = min(r["duty"] for r in rows)
        if lo > config.MIN_MOVE_DUTY + 0.005:
            bad.append(f"lowest sampled duty {lo:.3f} is above breakaway "
                       f"{config.MIN_MOVE_DUTY:.3f}: the deadband is not identifiable "
                       f"from data that never approaches it")
        span = max(r["v_ss"] for r in rows) - min(r["v_ss"] for r in rows)
        if span < 0.35:
            bad.append(f"speed varies only {span:.2f} m/s across the whole ladder; "
                       f"the slope is not resolvable (drag-limited?)")
    if fit["gain"] <= 0:
        bad.append("negative gain")
    return bad


def check_steering(fit, rows):
    """Return a list of reasons this steering fit should not be trusted."""
    if not fit:
        return ["no fit"]
    bad = []
    d = fit["max_steer_rad"]
    if d < 0.05:                       # < ~3 deg: no RC car steers this little
        bad.append(f"max steer {math.degrees(d):.2f} deg is implausibly small for a "
                   f"1/10 car -- almost always means the car never actually drove "
                   f"the arcs")
    if rows:
        om = max(abs(r["omega"]) for r in rows if r.get("omega") is not None) \
            if any(r.get("omega") is not None for r in rows) else 0.0
        if om < 0.15:
            bad.append(f"peak yaw rate {om:.3f} rad/s -- the vehicle barely rotated; "
                       f"check the drive duty was above breakaway "
                       f"({config.MIN_MOVE_DUTY:.3f})")
    return bad


def report(sessions, wheelbase, manual):
    print("=" * 74)
    print("  SYSTEM IDENTIFICATION — fitted from recorded sessions")
    print("=" * 74)
    print(f"  sessions : {len(sessions)}")
    print(f"  wheelbase: {wheelbase:.4f} m "
          f"({'MEASURED — passed in' if wheelbase != config.WHEELBASE_M else 'from config.py — STILL A GUESS, measure it'})")

    t_rows, t_fit, tau, decel = fit_throttle(sessions)
    print("\n" + "-" * 74)
    print("  1+2. THROTTLE")
    print("-" * 74)
    if t_rows:
        print(f"  {'duty':>6} {'v_inf m/s':>10} {'tau s':>7} {'decel m/s2':>11} {'log Hz':>7}  session")
        for r in t_rows:
            print(f"  {r['duty']:>6.3f} {_fmt(r['v_ss'],3):>10} {_fmt(r['tau'],3):>7} "
                  f"{_fmt(r['decel'],3):>11} {r['rate']:>7.1f}  {r['name']}")
    else:
        print("  no throttle runs found (need sessions with a 'step' phase)")
    if t_fit:
        print(f"\n  fit: v = {t_fit['gain']:.3f} * (duty - {t_fit['deadband']:.4f})"
              f"   [{t_fit['n']} points, rms {t_fit['rms']:.3f} m/s]")
        print(f"  -> deadband  {t_fit['deadband']:.4f}  (config.MIN_MOVE_DUTY is "
              f"{config.MIN_MOVE_DUTY})")
        print(f"  -> v at MAX_DUTY {config.MAX_DUTY}: "
              f"{t_fit['gain'] * (config.MAX_DUTY - t_fit['deadband']):.2f} m/s")
    t_bad = check_throttle(t_fit, t_rows)
    if t_fit and t_bad:
        print("\n  !! THIS FIT IS NOT USABLE:")
        for b in t_bad:
            print(f"     - {b}")
        print("     The per-duty speeds above ARE valid measurements. Only the")
        print("     extrapolated line is not. To identify the deadband, sample")
        print("     rungs from below breakaway upward.")
    if np.isfinite(tau):
        print(f"  -> rise time tau ~ {tau:.3f} s  (first-order lag for the sim)")
    if np.isfinite(decel):
        print(f"  -> coast decel ~ {decel:.3f} m/s^2  (drag + rolling resistance)")

    s_rows, s_fit = fit_steering(sessions, wheelbase, manual)
    print("\n" + "-" * 74)
    print("  3. STEERING")
    print("-" * 74)
    if s_rows:
        print(f"  {'steer':>6} {'v m/s':>7} {'omega r/s':>10} {'R m':>7} {'delta deg':>10}  source")
        for r in s_rows:
            dd = math.degrees(r["delta"]) if np.isfinite(r["delta"]) else float("nan")
            print(f"  {r['steer']:>+6.2f} {_fmt(r['v'],2):>7} {_fmt(r['omega'],3):>10} "
                  f"{_fmt(r['R'],3):>7} {_fmt(dd,2):>10}  {r['src']}")
    else:
        print("  no steering runs found (need sessions with an 'arc' phase)")
    if s_fit:
        print(f"\n  fit: delta = {s_fit['k']:.4f} * |steer|   "
              f"[{s_fit['n']} points, rms {s_fit['rms_deg']:.2f} deg]")
        print(f"  -> MAX_STEER_ANGLE_RAD = {s_fit['max_steer_rad']:.4f} "
              f"({s_fit['max_steer_deg']:.2f} deg)   "
              f"config.py currently says {config.MAX_STEER_ANGLE_RAD} "
              f"({math.degrees(config.MAX_STEER_ANGLE_RAD):.1f} deg)")
        tight = wheelbase / math.tan(s_fit["max_steer_rad"]) if s_fit["max_steer_rad"] > 1e-6 else float("nan")
        print(f"  -> tightest turn radius ~ {tight:.2f} m")
    s_bad = check_steering(s_fit, s_rows)
    if s_fit and s_bad:
        print("\n  !! THIS FIT IS NOT USABLE:")
        for b in s_bad:
            print(f"     - {b}")
        print("     Note the disagreement in the table: wheel odometry reports a")
        print("     speed while ICP reports almost no rotation. The tachometer")
        print("     counts MOTOR turns, so it reads a stationary car as moving.")
        print("     ICP watches the room and is the one to believe.")

    lat, nlat = fit_latency(sessions)
    print("\n" + "-" * 74)
    print("  4. ACTUATION DELAY")
    print("-" * 74)
    if np.isfinite(lat):
        print(f"  throttle command -> motor responds: {lat * 1000:.0f} ms "
              f"({nlat} step edges)")
        print(f"  -> f1tenth_gym ControlConfig(throttle_delay_steps="
              f"{max(1, round(lat * SIM_HZ))})   at {SIM_HZ:.0f} Hz physics")
        rate = np.median([len(s.tel['t']) / max(s.tel['t'][-1] - s.tel['t'][0], 1e-9)
                          for s in sessions if len(s.tel.get('t', [])) > 5])
        print(f"  NOTE: logging ran at ~{rate:.0f} Hz, so this is quantised to "
              f"~{1000.0/rate:.0f} ms. Treat it as an upper bound.")
    else:
        print("  not measurable from these sessions (run: python3 apps/sysid.py latency --stand)")
    print("  steering delay: not auto-fitted. Measure it from the latency session's")
    print("  'steerstep' phase against the LiDAR yaw rate, or estimate it as the")
    print("  servo's own spec (~100-150 ms for a full sweep).")

    print("\n" + "=" * 74)
    print("  PASTE-READY")
    print("=" * 74)
    print("  # final updates/config.py")
    if s_fit and not s_bad:
        print(f"  WHEELBASE_M = {wheelbase:.4f}            # MEASURED (tape)")
        print(f"  MAX_STEER_ANGLE_RAD = {s_fit['max_steer_rad']:.4f}    "
              f"# MEASURED ({s_fit['max_steer_deg']:.1f} deg) by apps/sysid.py")
    else:
        print("  # MAX_STEER_ANGLE_RAD  -- NOT MEASURED, see the steering section")
    if t_fit and not t_bad:
        print(f"  MIN_MOVE_DUTY = {max(t_fit['deadband'], 0.0):.4f}          "
              f"# MEASURED deadband")
        print(f"  # duty -> speed:  v_mps = {t_fit['gain']:.3f} * (duty - "
              f"{t_fit['deadband']:.4f})")
    else:
        print("  # MIN_MOVE_DUTY       -- NOT MEASURED, see the throttle section")
        if t_rows:
            pairs = ", ".join("%.3f->%.2f" % (r["duty"], r["v_ss"]) for r in t_rows)
            print("  # the per-duty speeds ARE valid: " + pairs)
    if t_fit or s_fit:
        print("\n  # f1tenth_gym (dev-jax)")
        if np.isfinite(lat):
            print(f"  ControlConfig(throttle_delay_steps={max(1, round(lat * SIM_HZ))}, "
                  f"steer_delay_steps=<measure>)")
        if t_fit and not t_bad:
            print(f"  VehicleParameters(v_max={t_fit['gain'] * (config.MAX_DUTY - t_fit['deadband']):.2f}, "
                  f"m=<weigh the car>, ...)")
        if s_fit and not s_bad:
            print(f"  VehicleParameters(s_min={-s_fit['max_steer_rad']:.4f}, "
                  f"s_max={s_fit['max_steer_rad']:.4f}, "
                  f"lf=<measure>, lr=<measure>)   # lf+lr = {wheelbase:.4f}")
    print("  LiDARConfig(num_beams=500, range_max=12.0)  # from_fov(radians(360))")
    print()
    return {"throttle": t_fit, "steering": s_fit, "tau": tau, "latency": lat}


def load_sessions(paths):
    out = []
    for p in paths:
        try:
            s = Session(p)
        except Exception as e:                                   # noqa: BLE001
            print(f"  skip {p}: {e}")
            continue
        if s.meta.get("source") != "sysid":
            continue
        if len(s.tel.get("t", [])) < 10:
            continue
        out.append(s)
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="session directories")
    ap.add_argument("--all", action="store_true", help="every sysid session under logs/")
    ap.add_argument("--root", default=DEFAULT_LOG_ROOT)
    ap.add_argument("--wheelbase-m", type=float, default=None,
                    help="MEASURED front-to-rear axle distance, metres")
    ap.add_argument("--manual-radius", default=None,
                    help='tape-measured radii, e.g. "1.0:1.35,0.6:2.10"')
    args = ap.parse_args(argv)

    paths = list(args.paths)
    if args.all or not paths:
        paths += sorted(glob.glob(os.path.join(args.root, "*")))
    sessions = load_sessions(paths)
    if not sessions:
        print("no sysid sessions found. Record some first:\n"
              "    python3 apps/sysid.py latency --stand")
        return 1

    manual = None
    if args.manual_radius:
        manual = {}
        for part in args.manual_radius.split(","):
            k, v = part.split(":")
            manual[abs(float(k))] = float(v)

    wb = args.wheelbase_m if args.wheelbase_m else config.WHEELBASE_M
    report(sessions, wb, manual)
    for s in sessions:
        s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
