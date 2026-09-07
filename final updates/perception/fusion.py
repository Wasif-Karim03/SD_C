#!/usr/bin/env python3
"""
perception/fusion.py — fuse LiDAR geometry + camera depth into one drive decision.

The division of labour (per the perception report):
  * LiDAR  = the PATH sensor. Metric, reliable, 360°. It owns "is the way ahead
             clear, and how far?" and the primary steering-toward-open-space.
  * Camera = the SEMANTICS + OFF-PLANE sensor. Its monocular depth catches low
             obstacles / curbs / things off the LiDAR's single scan plane that the
             LiDAR simply can't see.

Fusion rule (safety-first):
  blocked = LiDAR_blocked OR camera_blocked      # either sensor can stop the car
  steer:
    - both clear      -> blend, LiDAR-weighted (geometry leads)
    - one blocked     -> take the steer from whichever sensor sees the obstacle
    - both blocked    -> take the stronger turn-away suggestion
  nearest_ahead = LiDAR metric distance (the trustworthy number for speed control)

Returns a dict with the fused decision AND both sub-decisions, so callers/HUDs can
show who said what.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config                                       # noqa: E402
from perception.freespace import FreeSpacePlanner   # noqa: E402
from perception.lidar_nav import LidarNavigator     # noqa: E402

# how much LiDAR vs camera steer is trusted when BOTH are clear
LIDAR_STEER_WEIGHT = 0.6
CAM_STEER_WEIGHT = 0.4


def _clip(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


class FusedNavigator:
    def __init__(self, device=0):
        self.cam_nav = FreeSpacePlanner(device=device)   # loads the depth model
        self.lidar_nav = LidarNavigator()

    def estimate_camera(self, frame_bgr):
        return self.cam_nav.estimate(frame_bgr)

    def plan(self, frame_bgr, scan, cam_depth=None):
        """frame_bgr: front camera frame. scan: lidar scan (or None). -> fused dict."""
        if cam_depth is None:
            cam_depth = self.cam_nav.estimate(frame_bgr)
        cam = self.cam_nav.plan(cam_depth)
        lid = self.lidar_nav.plan(scan) if scan else None

        if lid is None:
            fused_blocked = cam["blocked"]
            fused_steer = cam["steer"]
            nearest = None
            source = "camera-only"
        else:
            fused_blocked = bool(cam["blocked"] or lid["blocked"])
            nearest = lid["nearest_ahead_m"]
            if lid["blocked"] and not cam["blocked"]:
                fused_steer = lid["steer"]; source = "lidar-block"
            elif cam["blocked"] and not lid["blocked"]:
                fused_steer = cam["steer"]; source = "camera-block"
            elif lid["blocked"] and cam["blocked"]:
                fused_steer = (lid["steer"] if abs(lid["steer"]) >= abs(cam["steer"])
                               else cam["steer"])
                source = "both-block"
            else:
                fused_steer = (LIDAR_STEER_WEIGHT * lid["steer"]
                               + CAM_STEER_WEIGHT * cam["steer"])
                source = "both-clear"

        return {
            "blocked": fused_blocked,
            "steer": _clip(fused_steer),
            "nearest_ahead_m": nearest,          # metric, from LiDAR
            "camera": cam,                       # sub-decision (has center_reach)
            "lidar": lid,                        # sub-decision (has nearest_ahead_m)
            "cam_depth": cam_depth,
            "source": source,
        }
