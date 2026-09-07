#!/usr/bin/env python3
"""
metric_nav.py — metric-depth free-space navigation (Depth-Anything V2).

Drop-in alternative to depth_nav.DepthNavigator that reasons in real METERS
instead of MiDaS *relative* inverse depth. Metric depth is stable frame-to-frame
(no scale drift), which is what lets us use a simple, absolute rule:

    free space = "how far can I see in this direction?"

Per vertical column of the drive zone we take a high percentile of the metric
depth = how far the view extends before something caps it. Open floor sees
several metres; a wall or box caps the view close. We steer toward the column
that sees farthest, and raise `blocked` when even straight-ahead is capped short
(a wall spanning the path) — the case MiDaS relative depth could not detect.

  estimate(frame) -> HxW float32 metric depth (metres)
  plan(depth)     -> {steer, blocked, best, clearance, reach, center_reach, ...}

Interface matches DepthNavigator (estimate/plan/draw) so autonav.py can swap it
in with --depth metric.
"""
import os
import sys

import numpy as np
import cv2

# DepthEngine lives in the sibling "Camera Lab" project.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                 "..", "Camera Lab")))
from depth_engine import DepthEngine


class MetricDepthNavigator:
    def __init__(self, device=0, n_cols=15, drive_top=0.45,
                 reach_pct=90, stop_m=2.5, clear_m=2.9,
                 reach_smooth=0.4, steer_power=3.0, steer_deadband=0.10):
        # NOTE: stop_m/clear_m are conservative because this model's ABSOLUTE
        # metres drift a lot in plain/dim spaces (open hallway has read anywhere
        # from 1.5 m to 7 m). Stopping early is the safe failure mode. A dedicated
        # forward distance sensor (ultrasonic/ToF) is the reliable hard-stop.
        self.eng = DepthEngine(device=device)      # Depth-Anything V2 metric (indoor)
        self.n_cols = n_cols               # number of steering columns (finer = smoother)
        self.drive_top = drive_top         # drive zone = below this frac of height
        self.reach_pct = reach_pct         # "how far can I see" = this depth pctile
        self.stop_m = stop_m               # block if center view capped below this (m)
        self.clear_m = clear_m             # ...and only unblock once it opens past this (hysteresis)
        self.reach_smooth = reach_smooth   # EMA on per-column reach (temporal denoise), 0..1
        self.steer_power = steer_power      # sharpen openness weighting for the steering centroid
        self.steer_deadband = steer_deadband
        self._reach_ema = None             # smoothed per-column reach (state)
        self._blocked = False              # latched block state for hysteresis

    def estimate(self, frame_bgr):
        """BGR frame -> metric depth map (HxW float32, metres)."""
        return self.eng.infer(frame_bgr)

    def plan(self, depth):
        """Return a planning dict from a metric depth map (metres)."""
        h, w = depth.shape
        top = int(h * self.drive_top)
        zone = depth[top:, :]                       # the area the car drives into
        cols = np.array_split(zone, self.n_cols, axis=1)
        # Reach = how far this direction sees before something caps it (metres).
        reach = np.array([np.percentile(c, self.reach_pct) for c in cols])

        # Temporal denoise: metric depth is noisy frame-to-frame, so EMA the
        # per-column reach. Steering and stop decisions then move smoothly instead
        # of reacting to single-frame flicker.
        if self._reach_ema is None or self._reach_ema.shape != reach.shape:
            self._reach_ema = reach.copy()
        else:
            self._reach_ema += self.reach_smooth * (reach - self._reach_ema)
        reach = self._reach_ema

        center = (self.n_cols - 1) / 2.0
        cmid = self.n_cols // 2
        half = max(1, self.n_cols // 6)             # center cone = middle third
        center_reach = float(reach[cmid - half: cmid + half + 1].mean())

        # Proportional steering: an openness-weighted centroid of the columns.
        # Uniformly open -> centroid at center -> straight. An obstacle on one
        # side lowers those weights, shifting the centroid smoothly toward the
        # open side (no quantized column-to-column jumps). steer_power sharpens
        # the pull toward the most-open direction.
        clearance = np.clip(reach / max(reach.max(), 1e-6), 0.0, 1.0)
        weights = clearance ** self.steer_power
        idx = np.arange(self.n_cols)
        centroid = float((idx * weights).sum() / weights.sum()) \
            if weights.sum() > 1e-6 else center
        steer = (centroid - center) / center        # -1..1, continuous
        if abs(steer) < self.steer_deadband:
            steer = 0.0
        steer = float(np.clip(steer, -1.0, 1.0))
        best = int(np.argmax(reach))                # for the HUD heading arrow

        # Blocked WITH HYSTERESIS: stop when the forward view caps below stop_m,
        # and only resume once it clearly opens past clear_m. Prevents stop/go
        # chatter when hovering right at the threshold.
        if self._blocked:
            self._blocked = center_reach < self.clear_m
        else:
            self._blocked = center_reach < self.stop_m
        blocked = bool(self._blocked)
        return {"steer": float(steer), "blocked": blocked, "best": best,
                "clearance": clearance, "nearness": reach, "reach": reach,
                "center_reach": center_reach, "drive_top": top}

    def draw(self, frame, depth, plan, show_depth_inset=True):
        """Overlay per-column reach bars + heading + status + metric depth inset."""
        h, w = frame.shape[:2]
        n = self.n_cols

        # Per-column reach bars across the bottom (green=far/open, red=capped).
        bar_h = 60
        y0 = h - bar_h - 4
        cw = w / n
        for i in range(n):
            c = float(plan["clearance"][i])
            col = (0, int(255 * c), int(255 * (1 - c)))   # BGR red->green
            x1, x2 = int(i * cw) + 2, int((i + 1) * cw) - 2
            bh = int(bar_h * c)
            cv2.rectangle(frame, (x1, y0 + (bar_h - bh)), (x2, y0 + bar_h), col, -1)
            cv2.rectangle(frame, (x1, y0), (x2, y0 + bar_h), (60, 60, 60), 1)

        # Heading arrow from bottom-center toward the chosen open column.
        bx = int((plan["best"] + 0.5) * cw)
        cv2.arrowedLine(frame, (w // 2, h - 6), (bx, y0 - 10),
                        (0, 165, 255), 3, cv2.LINE_AA, tipLength=0.3)

        # Status text with the absolute forward distance in metres.
        status = "BLOCKED - STOP" if plan["blocked"] else "CLEAR"
        scol = (0, 0, 255) if plan["blocked"] else (0, 200, 0)
        txt = f"steer:{plan['steer']:+.2f}  ahead:{plan['center_reach']:.1f}m  {status}"
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    scol, 1, cv2.LINE_AA)

        # Small colorized metric-depth inset (top-right) for intuition.
        if show_depth_inset:
            dm = DepthEngine.colorize(depth)
            iw = w // 4
            ih = int(iw * h / w)
            inset = cv2.resize(dm, (iw, ih))
            frame[4:4 + ih, w - iw - 4:w - 4] = inset
            cv2.rectangle(frame, (w - iw - 4, 4), (w - 4, 4 + ih),
                          (255, 255, 255), 1)
        return frame
