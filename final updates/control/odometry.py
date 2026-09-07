#!/usr/bin/env python3
"""
control/odometry.py — dead-reckoning pose from wheel distance + steering (bicycle model).

The prerequisite for mapping, localization and A->B: the car must know how far and
which way it has moved. We integrate an Ackermann/bicycle model:

    d       = (tach - last_tach) * meters_per_tach          # distance this step
    steer   = steer_norm * MAX_STEER_ANGLE_RAD              # front-wheel angle
    yaw    += d / wheelbase * tan(steer)                    # heading change
    x      += d * cos(yaw);   y += d * sin(yaw)             # position

Distance comes from the VESC tachometer (meters_per_tach is CALIBRATED). Heading is
dead-reckoned from the commanded steering angle — good short-term, but it drifts
over long paths; the compass/GPS will correct that drift later (EKF). wheelbase and
MAX_STEER_ANGLE_RAD are still provisional in config — measure them on the chassis to
sharpen turning accuracy.

Convention: +x = forward at yaw 0, yaw grows CCW. If turning sign is backwards in
practice, flip the sign on the yaw update (we'll confirm on the floor).
"""
import os
import sys
import math

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # noqa: E402


class Odometry:
    def __init__(self, wheelbase=None, meters_per_tach=None, max_steer=None):
        self.L = wheelbase if wheelbase is not None else config.WHEELBASE_M
        self.mpt = meters_per_tach if meters_per_tach is not None else config.METERS_PER_TACH
        self.max_steer = max_steer if max_steer is not None else config.MAX_STEER_ANGLE_RAD
        self.reset()

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.dist = 0.0            # total path length travelled
        self._last_tach = None

    def update(self, tach, steer_norm=0.0):
        """Feed the latest VESC tach + current steering (-1..+1). Returns pose dict."""
        if self._last_tach is None:
            self._last_tach = tach
            return self.pose()
        d = (tach - self._last_tach) * self.mpt
        self._last_tach = tach
        if d == 0.0:
            return self.pose()
        self.dist += abs(d)
        steer_angle = max(-1.0, min(1.0, steer_norm)) * self.max_steer
        if abs(steer_angle) > 1e-4:
            self.yaw += d / self.L * math.tan(steer_angle)
        self.x += d * math.cos(self.yaw)
        self.y += d * math.sin(self.yaw)
        return self.pose()

    def pose(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw,
                "yaw_deg": math.degrees(self.yaw) % 360.0, "dist": self.dist}
