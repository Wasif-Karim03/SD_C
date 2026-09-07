#!/usr/bin/env python3
"""
perception/freespace.py — metric-depth free-space navigation (front camera).

Ported from Camera Control/metric_nav.py. Turns a metric depth map into a driving
decision, class-agnostically (it doesn't care WHAT is ahead, only how far):

    free space = "how far can I see in this direction?"

Per vertical column of the drive zone we take a high percentile of the metric
depth = how far the view extends before something caps it. Open floor sees several
metres; a wall/box caps it close. We steer toward the column that sees farthest,
and raise `blocked` when even straight-ahead is capped short (a wall across the
path) — the case a single-plane LiDAR or MiDaS relative depth can miss.

  estimate(frame) -> HxW float32 metric depth (metres)
  plan(depth)     -> {steer, blocked, best, clearance, reach, center_reach, ...}
  draw(...)       -> annotated frame (bars + heading + status + depth inset)
"""
import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config                                  # noqa: E402
from perception.depth import DepthEngine       # noqa: E402


class FreeSpacePlanner:
    def __init__(self, device=0, n_cols=15, drive_top=0.45,
                 reach_pct=90, stop_m=None, clear_m=None,
                 reach_smooth=0.4, steer_power=3.0, steer_deadband=0.10):
        # stop_m/clear_m default from config (env-tuned): this model's ABSOLUTE
        # metres run compressed indoors, so an open corridor reads ~2.6 m — block
        # well below that. Stopping is safe; over-blocking means it never drives.
        self.eng = DepthEngine(device=device)
        self.n_cols = n_cols
        self.drive_top = drive_top
        self.reach_pct = reach_pct
        self.stop_m = config.FREESPACE_STOP_M if stop_m is None else stop_m
        self.clear_m = config.FREESPACE_CLEAR_M if clear_m is None else clear_m
        self.reach_smooth = reach_smooth
        self.steer_power = steer_power
        self.steer_deadband = steer_deadband
        self._reach_ema = None
        self._blocked = False
        self.stop_m_attr = stop_m   # exposed for speed-scaling in the loop

    def estimate(self, frame_bgr):
        return self.eng.infer(frame_bgr)

    def plan(self, depth):
        h, w = depth.shape
        top = int(h * self.drive_top)
        zone = depth[top:, :]
        cols = np.array_split(zone, self.n_cols, axis=1)
        reach = np.array([np.percentile(c, self.reach_pct) for c in cols])

        if self._reach_ema is None or self._reach_ema.shape != reach.shape:
            self._reach_ema = reach.copy()
        else:
            self._reach_ema += self.reach_smooth * (reach - self._reach_ema)
        reach = self._reach_ema

        center = (self.n_cols - 1) / 2.0
        cmid = self.n_cols // 2
        half = max(1, self.n_cols // 6)
        center_reach = float(reach[cmid - half: cmid + half + 1].mean())

        clearance = np.clip(reach / max(reach.max(), 1e-6), 0.0, 1.0)
        weights = clearance ** self.steer_power
        idx = np.arange(self.n_cols)
        centroid = float((idx * weights).sum() / weights.sum()) \
            if weights.sum() > 1e-6 else center
        steer = (centroid - center) / center
        if abs(steer) < self.steer_deadband:
            steer = 0.0
        steer = float(np.clip(steer, -1.0, 1.0))
        best = int(np.argmax(reach))

        # blocked WITH HYSTERESIS: stop below stop_m, resume only past clear_m
        if self._blocked:
            self._blocked = center_reach < self.clear_m
        else:
            self._blocked = center_reach < self.stop_m
        blocked = bool(self._blocked)
        return {"steer": float(steer), "blocked": blocked, "best": best,
                "clearance": clearance, "reach": reach,
                "center_reach": center_reach, "drive_top": top}

    def draw(self, frame, depth, plan, show_depth_inset=True):
        h, w = frame.shape[:2]
        n = self.n_cols
        bar_h = 60
        y0 = h - bar_h - 4
        cw = w / n
        for i in range(n):
            c = float(plan["clearance"][i])
            col = (0, int(255 * c), int(255 * (1 - c)))
            x1, x2 = int(i * cw) + 2, int((i + 1) * cw) - 2
            bh = int(bar_h * c)
            cv2.rectangle(frame, (x1, y0 + (bar_h - bh)), (x2, y0 + bar_h), col, -1)
            cv2.rectangle(frame, (x1, y0), (x2, y0 + bar_h), (60, 60, 60), 1)

        bx = int((plan["best"] + 0.5) * cw)
        cv2.arrowedLine(frame, (w // 2, h - 6), (bx, y0 - 10),
                        (0, 165, 255), 3, cv2.LINE_AA, tipLength=0.3)

        status = "BLOCKED - STOP" if plan["blocked"] else "CLEAR"
        scol = (0, 0, 255) if plan["blocked"] else (0, 200, 0)
        txt = f"steer:{plan['steer']:+.2f}  ahead:{plan['center_reach']:.1f}m  {status}"
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    scol, 1, cv2.LINE_AA)

        if show_depth_inset:
            dm = DepthEngine.colorize(depth)
            iw = w // 4
            ih = int(iw * h / w)
            inset = cv2.resize(dm, (iw, ih))
            frame[4:4 + ih, w - iw - 4:w - 4] = inset
            cv2.rectangle(frame, (w - iw - 4, 4), (w - 4, 4 + ih),
                          (255, 255, 255), 1)
        return frame
