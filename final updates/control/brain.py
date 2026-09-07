#!/usr/bin/env python3
"""
control/brain.py — situation-aware decision maker (front + rear + LiDAR + detections).

Combines every sensor into ONE reasoned driving decision, including a safe-exit
recovery when stuck:

  inputs : front camera (depth free-space + fusion with LiDAR front), LiDAR 360°
           (front/rear clearance + the most-open direction), object detection on
           BOTH cameras (people/VRUs -> caution).
  output : a decision dict {mode, throttle_sign, steer, reason, ...} where
             mode = DRIVE  -> path ahead is clear, go
                    RECOVER -> front blocked but behind is clear: back out toward
                               the most-open direction (safe 3-point-turn escape)
                    HOLD    -> boxed in (front AND rear blocked) or a person is
                               close in the direction of travel -> stop

Design notes:
  * LiDAR is the trusted geometry (front/rear clearance + openest bearing).
  * The camera adds semantics (what's there) + off-plane obstacles via depth.
  * The REAR camera mostly sees the car's own body, so rear detection uses only the
    far field (top crop) and ignores chassis-sized boxes; rear CLEARANCE comes from
    the LiDAR, not the rear camera.
"""
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config                                       # noqa: E402
from perception.fusion import FusedNavigator        # noqa: E402

REAR_STOP_M = 0.6          # blocked-behind if LiDAR rear sector closer than this
PERSON_NEAR_FRAC = 0.30    # a VRU box taller than this frac of the frame = "close"
REAR_BODY_FRAC = 0.55      # ignore rear boxes bigger than this (the car's own frame)


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class SmartBrain:
    def __init__(self, device=0, use_detect=True):
        self.front = FusedNavigator(device=device)   # front cam depth + LiDAR fusion
        self.det = None
        if use_detect:
            from perception.detect import Detector
            self.det = Detector()
        self.fwd = config.LIDAR_FORWARD_DEG
        self.half_front = config.LIDAR_FRONT_ARC_DEG / 2.0

    # ---- LiDAR 360 analysis -------------------------------------------- #
    def _lidar(self, scan):
        if not scan:
            return {"front_near": float("inf"), "rear_near": float("inf"),
                    "best_bearing": 0.0}
        rel = []
        for _q, a, dmm in scan:
            d = dmm / 1000.0
            if d > config.LIDAR_MIN_M:
                rel.append((_wrap180(a - self.fwd), d))
        front = [d for r, d in rel if abs(r) <= self.half_front]
        rear = [d for r, d in rel if abs(r) >= 180 - self.half_front]
        # openness in 24 sectors of 15°; sector clearance = closest return in it
        nsec = 24
        secmin = [float("inf")] * nsec
        for r, d in rel:
            k = int((r + 180) / 360 * nsec) % nsec
            if d < secmin[k]:
                secmin[k] = d
        openness = [1e3 if m == float("inf") else m for m in secmin]
        best = max(range(nsec), key=lambda k: openness[k])
        best_bearing = _wrap180(-180 + (best + 0.5) * (360 / nsec))
        return {"front_near": min(front) if front else float("inf"),
                "rear_near": min(rear) if rear else float("inf"),
                "best_bearing": best_bearing}

    # ---- detections ---------------------------------------------------- #
    def _person_near(self, frame, dets):
        if not dets:
            return False
        h = frame.shape[0]
        for d in dets:
            if d["vru"] and (d["box"][3] - d["box"][1]) / h >= PERSON_NEAR_FRAC:
                return True
        return False

    def _rear_dets(self, rear_frame):
        """Detect only in the far field (top crop) and drop chassis-sized boxes."""
        if self.det is None or rear_frame is None:
            return []
        h, w = rear_frame.shape[:2]
        crop = rear_frame[0:int(h * 0.45), :]        # far field only
        out = []
        for d in self.det.detect(crop):
            bw = (d["box"][2] - d["box"][0]) / w
            bh = (d["box"][3] - d["box"][1]) / max(1, crop.shape[0])
            if bw * bh < REAR_BODY_FRAC:             # skip car-body-sized blobs
                out.append(d)
        return out

    # ---- the decision -------------------------------------------------- #
    def decide(self, front_frame, scan, rear_frame=None):
        fused = self.front.plan(front_frame, scan)
        front_blocked = fused["blocked"]
        steer = fused["steer"]
        lg = self._lidar(scan)

        front_dets = self.det.detect(front_frame) if self.det else []
        rear_dets = self._rear_dets(rear_frame)
        person_front = self._person_near(front_frame, front_dets)
        person_rear = self._person_near(rear_frame, rear_dets) if rear_frame is not None else False

        rear_blocked = lg["rear_near"] < REAR_STOP_M
        best = lg["best_bearing"]              # deg rel forward; + = one side, - = other
        esc = 1.0 if best >= 0 else -1.0       # escape steer side (sign verified on floor)

        # --- policy ---
        if person_front and not front_blocked:
            mode, thr, scmd, why = "HOLD", 0.0, 0.0, "person close ahead"
        elif not front_blocked:
            mode, thr, scmd, why = "DRIVE", 1.0, steer, "path ahead clear"
        else:
            if not rear_blocked and not person_rear:
                mode, thr = "RECOVER", -1.0
                scmd = esc * config.LIDAR_STEER_SIGN
                why = (f"front blocked ({fused['nearest_ahead_m']}m) -> backing toward "
                       f"open side ({best:+.0f}deg)")
            else:
                mode, thr, scmd, why = "HOLD", 0.0, 0.0, (
                    "boxed in: front blocked and " +
                    ("person behind" if person_rear else "rear blocked"))

        return {
            "mode": mode, "throttle_sign": thr, "steer": float(np.clip(scmd, -1, 1)),
            "reason": why,
            "front_blocked": bool(front_blocked),
            "front_near": fused["nearest_ahead_m"],
            "rear_blocked": bool(rear_blocked),
            "rear_near": (round(lg["rear_near"], 2) if lg["rear_near"] != float("inf") else None),
            "best_bearing": round(best, 0),
            "person_front": person_front, "person_rear": person_rear,
            "front_dets": front_dets, "rear_dets": rear_dets,
            "fused": fused,
        }
