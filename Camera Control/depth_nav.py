#!/usr/bin/env python3
"""
depth_nav.py — monocular-depth free-space navigation for indoor obstacle avoidance.

For indoor driving there usually are no lanes, so instead of "follow the line" the
car needs "where is the open space, and is anything too close ahead?" This module
estimates a relative depth map with MiDaS-small (GPU), then reasons about FREE
SPACE in a class-agnostic way (it doesn't care WHAT the obstacle is — wall, chair
leg, cable, box — only how near it is):

  - Split the lower "drive zone" of the frame into vertical columns.
  - Per column, measure how NEAR the closest obstacle is (high-percentile depth).
  - Steer toward the most OPEN column; raise a STOP flag if the path dead-ahead
    is blocked close-up.

Output is a steering offset in [-1, 1] (negative = steer left, positive = right)
plus a `blocked` flag — directly usable as a reactive avoidance controller.

MiDaS outputs INVERSE depth: larger value = NEARER. (No metric scale — this is
relative, which is all that free-space steering needs.)
"""

import numpy as np
import cv2
import torch


class DepthNavigator:
    def __init__(self, device=0, n_cols=9, drive_top=0.45,
                 near_percentile=85, block_ratio=0.80,
                 min_contrast=0.15):
        self.device = torch.device(f"cuda:{device}")
        self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small",
                                    trust_repo=True).to(self.device).eval()
        self.tf = torch.hub.load("intel-isl/MiDaS", "transforms",
                                 trust_repo=True).small_transform
        self.n_cols = n_cols              # number of steering columns
        self.drive_top = drive_top        # drive zone = below this frac of height
        self.near_pct = near_percentile   # percentile = "closest obstacle" in col
        self.block_ratio = block_ratio    # center nearness frac of max => blocked
        # MiDaS depth is relative/scale-free, and an OPEN floor always reads a
        # smooth center-peaked "nearness" hump (perspective: the floor right in
        # front of the car is the nearest thing). Two guards keep that geometry
        # from being mistaken for an obstacle:
        #   min_contrast  — min depth spread (span/level) before we react at all;
        #                   the open-floor hump is ~0.05-0.09, a box at stop range
        #                   is ~0.25+. Below this: clear + drive straight. Above:
        #                   steer toward open space, and BLOCK if center obstructed.
        self.min_contrast = min_contrast

    @torch.no_grad()
    def estimate(self, frame_bgr):
        """Return a HxW float depth map (inverse depth, larger = nearer)."""
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inp = self.tf(rgb).to(self.device)
        pred = self.midas(inp)
        pred = torch.nn.functional.interpolate(
            pred.unsqueeze(1), size=frame_bgr.shape[:2],
            mode="bicubic", align_corners=False).squeeze()
        return pred.detach().cpu().numpy()

    def plan(self, depth):
        """Return a planning dict from a depth map."""
        h, w = depth.shape
        top = int(h * self.drive_top)
        zone = depth[top:, :]                       # the area the car drives into
        cols = np.array_split(zone, self.n_cols, axis=1)
        # Nearness = how close the closest thing in this column is.
        nearness = np.array([np.percentile(c, self.near_pct) for c in cols])

        lo, hi = nearness.min(), nearness.max()
        span = hi - lo
        level = float(np.median(nearness))
        # Scale-free depth contrast. An open floor's smooth hump is tiny (~0.05);
        # a genuinely close obstacle stands out with much higher contrast.
        contrast = span / max(abs(level), 1e-6)

        # Per-column openness (1 = most open/far, 0 = nearest). Raw — NOT
        # de-trended: de-trending a low-order fit flattens a centered obstacle
        # (it looks just like the smooth floor hump), so the car wouldn't turn
        # away from a box dead ahead until it was almost touching it.
        clearance = 1.0 - (nearness - lo) / max(span, 1e-6)

        # --- steering ---
        # Gate on contrast instead of de-trending: on an OPEN floor the only
        # structure is the smooth perspective hump (contrast < min_contrast), so
        # ignore it and drive straight. Once a real obstacle raises the contrast,
        # steer toward the most-open column using raw depth — this reacts as soon
        # as the obstacle appears and never masks a centered one.
        center = (self.n_cols - 1) / 2.0
        if contrast >= self.min_contrast:
            best = int(np.argmax(clearance))        # most open column
        else:
            best = int(round(center))               # open floor -> straight
        steer = (best - center) / center            # -1..1

        # --- blocked: real depth structure AND the center cone is obstructed ---
        # (something close dead ahead). Off-center obstacles leave the center
        # clear -> not blocked, so the car steers around them instead of stopping.
        cmid = self.n_cols // 2
        center_near = nearness[max(0, cmid - 1): cmid + 2].mean()
        relatively_near = center_near >= lo + self.block_ratio * span
        blocked = bool(contrast >= self.min_contrast and relatively_near)

        return {"steer": float(steer), "blocked": blocked, "best": best,
                "clearance": clearance, "nearness": nearness,
                "contrast": float(contrast), "drive_top": top}

    def draw(self, frame, depth, plan, show_depth_inset=True):
        """Overlay depth inset + per-column clearance bars + heading + status."""
        h, w = frame.shape[:2]
        n = self.n_cols

        # Per-column clearance bars across the bottom (green=open, red=near).
        bar_h = 60
        y0 = h - bar_h - 4
        cw = w / n
        for i in range(n):
            c = float(plan["clearance"][i])
            col = (0, int(255 * c), int(255 * (1 - c)))   # BGR red->green
            x1, x2 = int(i * cw) + 2, int((i + 1) * cw) - 2
            bh = int(bar_h * c)
            cv2.rectangle(frame, (x1, y0 + (bar_h - bh)), (x2, y0 + bar_h),
                          col, -1)
            cv2.rectangle(frame, (x1, y0), (x2, y0 + bar_h), (60, 60, 60), 1)

        # Heading arrow from bottom-center toward the chosen open column.
        bx = int((plan["best"] + 0.5) * cw)
        cv2.arrowedLine(frame, (w // 2, h - 6), (bx, y0 - 10),
                        (0, 165, 255), 3, cv2.LINE_AA, tipLength=0.3)

        # Status text.
        status = "BLOCKED - STOP/TURN" if plan["blocked"] else "CLEAR"
        scol = (0, 0, 255) if plan["blocked"] else (0, 200, 0)
        txt = f"steer:{plan['steer']:+.2f}  {status}"
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    scol, 1, cv2.LINE_AA)

        # Small colorized depth inset (top-right) for intuition.
        if show_depth_inset:
            d = depth - depth.min()
            d = (d / max(d.max(), 1e-6) * 255).astype(np.uint8)
            dm = cv2.applyColorMap(d, cv2.COLORMAP_MAGMA)   # bright = near
            iw = w // 4
            ih = int(iw * h / w)
            inset = cv2.resize(dm, (iw, ih))
            frame[4:4 + ih, w - iw - 4:w - 4] = inset
            cv2.rectangle(frame, (w - iw - 4, 4), (w - 4, 4 + ih),
                          (255, 255, 255), 1)
        return frame
