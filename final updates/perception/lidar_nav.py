#!/usr/bin/env python3
"""
perception/lidar_nav.py — turn a 2D LiDAR scan into forward free-space + steer.

The LiDAR is the geometry/path sensor: metric, reliable, 360°. From one scan
(list of (quality, angle_deg, dist_mm)) this computes, relative to the car's
FRONT (config.LIDAR_FORWARD_DEG):

  nearest_ahead_m : closest obstacle inside the front cone (blocking distance)
  blocked         : nearest_ahead < stop (with hysteresis)
  steer           : -1..+1 toward the most-open direction in the forward arc
  nearest_bearing : raw scan angle of the overall closest point (for CALIBRATION —
                    put an object dead ahead and this tells you LIDAR_FORWARD_DEG)

Angle convention after re-referencing to forward: rel = 0 straight ahead; we treat
rel>0 as one side and rel<0 the other, and config.LIDAR_STEER_SIGN maps that onto
the car's steering (+1 = right). If the car turns the wrong way, flip that sign.
"""
import os
import sys
import math

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # noqa: E402


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class LidarNavigator:
    def __init__(self, forward_deg=None, front_arc=None, steer_arc=None,
                 stop_m=None, clear_m=None, min_m=None, steer_sign=None,
                 sector_deg=10.0, react_m=1.3):
        self.forward_deg = config.LIDAR_FORWARD_DEG if forward_deg is None else forward_deg
        self.front_arc = config.LIDAR_FRONT_ARC_DEG if front_arc is None else front_arc
        self.steer_arc = config.LIDAR_STEER_ARC_DEG if steer_arc is None else steer_arc
        self.stop_m = config.LIDAR_STOP_M if stop_m is None else stop_m
        self.clear_m = config.LIDAR_CLEAR_M if clear_m is None else clear_m
        self.min_m = config.LIDAR_MIN_M if min_m is None else min_m
        self.steer_sign = config.LIDAR_STEER_SIGN if steer_sign is None else steer_sign
        self.sector_deg = sector_deg
        self.react_m = react_m     # only steer to avoid when something's within this
        self._blocked = False

    def plan(self, scan):
        """scan: list of (quality, angle_deg, dist_mm). Returns a decision dict."""
        # to (rel_bearing_deg, dist_m), forward-referenced, valid returns only
        pts = []
        nearest_global = (None, float("inf"))     # (raw_angle, dist_m) overall
        for _q, ang, dmm in scan:
            d = dmm / 1000.0
            if d < self.min_m:
                continue
            if d < nearest_global[1]:
                nearest_global = (ang, d)
            rel = _wrap180(ang - self.forward_deg)
            pts.append((rel, d))

        # nearest obstacle in the front cone (this is the blocking distance)
        half_front = self.front_arc / 2.0
        ahead = [d for rel, d in pts if abs(rel) <= half_front]
        nearest_ahead = min(ahead) if ahead else float("inf")

        # blocked with hysteresis
        if self._blocked:
            self._blocked = nearest_ahead < self.clear_m
        else:
            self._blocked = nearest_ahead < self.stop_m
        blocked = bool(self._blocked)

        # Steering: go STRAIGHT when the way ahead is clear; only turn to AVOID
        # something within react_m, toward the more-open side. (Aiming at the
        # single most-open far sector even when clear made the car weave.)
        half_steer = self.steer_arc / 2.0
        nsec = max(1, int(self.steer_arc / self.sector_deg))
        sec_min = [float("inf")] * nsec
        left, right = [], []
        for rel, d in pts:
            if abs(rel) <= half_steer:
                k = min(nsec - 1, max(0, int((rel + half_steer) / self.steer_arc * nsec)))
                if d < sec_min[k]:
                    sec_min[k] = d
                if rel < 0:
                    left.append(d)
                elif rel > 0:
                    right.append(d)
        openness = [1e3 if m == float("inf") else m for m in sec_min]
        best = max(range(nsec), key=lambda k: openness[k])
        best_center = -half_steer + (best + 0.5) * (self.steer_arc / nsec)
        left_clear = min(left) if left else 1e3
        right_clear = min(right) if right else 1e3
        if nearest_ahead > self.react_m:
            steer = 0.0                          # clear ahead -> straight
        else:
            diff = right_clear - left_clear      # + => more room on the right
            steer = self.steer_sign * max(-1.0, min(1.0, diff / max(self.react_m, 0.5)))
        steer = float(max(-1.0, min(1.0, steer)))

        return {
            "nearest_ahead_m": nearest_ahead,
            "blocked": blocked,
            "steer": float(steer),
            "best_bearing_deg": float(best_center),
            "nearest_bearing_deg": nearest_global[0],   # raw angle (calibration)
            "nearest_dist_m": nearest_global[1],
            "n_points": len(pts),
            "sector_open_m": openness,
        }
