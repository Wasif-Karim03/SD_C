#!/usr/bin/env python3
"""
control/follow_the_gap.py — the classical reactive baseline.

WHY THIS EXISTS, AND WHY IT IS FIRST
------------------------------------
Follow-the-Gap is F1TENTH Lab 4. It has no parameters to train, no data, no
GPU, and about a hundred lines. A 2024 benchmark of methods in f1tenth_gym
found end-to-end deep RL was no faster than it.

So it is not a stepping stone. It is three things at once:

  1. The BAR. Any learned policy has to beat this to justify existing, and a
     paper that does not report it will be asked why.
  2. The EXPERT. Run it in simulation across randomised tracks and it
     generates (scan -> action) pairs for free. That is your imitation
     training set, with no labelling and no driving.
  3. The FALLBACK. When a learned policy misbehaves, something has to take
     the wheel. This is what takes it.

THE ALGORITHM
-------------
  1. Take the ranges in a forward arc.
  2. Find the nearest return and zero out a "safety bubble" around it, so the
     car cannot choose a gap that clips the thing closest to it.
  3. Find the widest run of remaining free space -- the max gap.
  4. Aim at the deepest point in that gap.
  5. Steer toward it, proportionally.

WHAT IT CANNOT DO, stated plainly: it is purely reactive. It has no memory, no
map and no plan, so it will drive into a dead end and sit there oscillating.
That is not a bug to fix here -- it is the reason a planner exists.
"""
import math

import numpy as np


class FollowTheGap:
    """Reactive gap-following steering from a single LiDAR scan.

    Consumes the same nose-referenced range grid the recorder and reader
    produce: bin 0 straight ahead, bins increasing with scanner angle, 0.0
    meaning "no return".
    """

    def __init__(self, arc_deg=100.0, bubble_m=0.25, max_range_m=6.0,
                 steer_gain=1.0, disparity_m=0.35, min_gap_bins=4,
                 half_width_m=0.16, min_gap_depth_m=0.60):
        self.arc_deg = float(arc_deg)          # how much of the world to consider
        self.bubble_m = float(bubble_m)        # safety radius around the nearest point
        self.max_range_m = float(max_range_m)  # clip: far is far, the difference is noise
        self.steer_gain = float(steer_gain)
        self.disparity_m = float(disparity_m)  # step in range that marks an edge
        # Disparity extension widens an edge by the car's HALF-WIDTH, not by
        # the whole safety bubble. Using the bubble for both meant the two
        # mechanisms compounded and could close the entire arc.
        self.half_width_m = float(half_width_m)
        self.min_gap_bins = int(min_gap_bins)
        # A run of free bins is not a gap if you cannot drive into it. Boxed in
        # at 0.4 m on every bearing, the bubble clears the middle of the arc and
        # leaves the edges "free" -- and the car cheerfully aims at a wall it is
        # already touching. A gap has to be DEEP as well as WIDE.
        self.min_gap_depth_m = float(min_gap_depth_m)

    # ------------------------------------------------------------------ #

    def plan(self, grid, speed=None):
        """grid: (bins,) ranges, bin 0 = straight ahead, 0.0 = no return.

        Returns a dict. `steer` is normalised [-1, 1] in the same convention
        the rest of this codebase uses; None means "no gap, do not move".
        """
        n = len(grid)
        if n == 0:
            return self._nothing("empty scan")
        deg_per_bin = 360.0 / n
        half = int(round((self.arc_deg / 2.0) / deg_per_bin))

        # forward arc, unwrapped so it is contiguous and centred on the nose
        idx = np.arange(-half, half + 1) % n
        r = np.asarray(grid, dtype=np.float64)[idx].copy()
        ang = np.arange(-half, half + 1) * deg_per_bin

        # A zero means the beam returned nothing, which for a LiDAR means
        # "nothing out there within range" -- i.e. FREE, not blocked. Treating
        # it as an obstacle is the classic way to make this algorithm refuse
        # to drive down an open corridor.
        r[r <= 1e-6] = self.max_range_m
        r = np.clip(r, 0.0, self.max_range_m)

        # ── disparity extension ────────────────────────────────────────────
        # A pillar edge reads as a sudden step in range. Without this the car
        # aims at the deep gap just past a doorframe and takes the mirror off.
        # Extend the nearer side of every step across the angular width the
        # car needs to fit.
        r = self._extend_disparities(r, ang)

        # ── safety bubble around the closest return ────────────────────────
        j = int(np.argmin(r))
        nearest = float(r[j])
        if nearest > 1e-6:
            span = math.degrees(math.atan2(self.bubble_m, max(nearest, 0.05)))
            mask = np.abs(ang - ang[j]) <= span
            r[mask] = 0.0

        # ── choose among the runs of free space ────────────────────────────
        # Textbook Follow-the-Gap takes the WIDEST run. That picks a broad
        # shallow shelf of wall over a narrower opening that actually leads
        # somewhere, and on a symmetric scene it breaks ties by array order --
        # which is how the unit test caught it steering into a wall because the
        # wall's side happened to come first.
        #
        # Score by depth first, then by how little steering it asks for. Depth
        # is what makes a gap drivable; among equally deep options the one
        # straight ahead is both safer and less surprising to watch.
        runs = [(lo, hi) for lo, hi in self._runs(r > 0.0)
                if (hi - lo) >= self.min_gap_bins]
        scored = []
        for lo, hi in runs:
            seg = r[lo:hi]
            depth = float(seg.max())
            if depth < self.min_gap_depth_m:
                continue
            top = np.nonzero(seg >= depth - 0.05 * max(depth, 1e-6))[0]
            k = lo + int(round(0.5 * (top[0] + top[-1])))
            scored.append((depth, -abs(float(ang[k])), lo, hi, k))
        if not scored:
            return self._nothing(
                f"no gap at least {self.min_gap_bins} bins wide and "
                f"{self.min_gap_depth_m:.2f} m deep")
        scored.sort(reverse=True)
        _, _, lo, hi, k = scored[0]
        # Aim at the CENTRE OF THE DEEPEST PLATEAU, not at the first bin that
        # happens to hold the maximum. On open ground every bin is at max range
        # and argmax returns index 0 -- the leftmost bin of the arc -- so the
        # car swerves hard left across an empty room. Found by the unit test
        # below, which is exactly why it is there.
        target_deg = float(ang[k])

        steer = max(-1.0, min(1.0, self.steer_gain * target_deg / (self.arc_deg / 2.0)))
        return {"steer": steer,
                "target_deg": target_deg,
                "gap_deg": float(ang[hi - 1] - ang[lo]),
                "gap_depth_m": float(r[k]),
                "nearest_m": nearest,
                "note": ""}

    # ------------------------------------------------------------------ #

    def _extend_disparities(self, r, ang):
        out = r.copy()
        d = np.diff(r)
        for i in np.nonzero(np.abs(d) > self.disparity_m)[0]:
            near_i, near_r = (i, r[i]) if r[i] < r[i + 1] else (i + 1, r[i + 1])
            if near_r < 0.05:
                continue
            span = math.degrees(math.atan2(self.half_width_m, near_r))
            mask = np.abs(ang - ang[near_i]) <= span
            out[mask] = np.minimum(out[mask], near_r)
        return out

    @staticmethod
    def _runs(free):
        out, start = [], None
        for i, v in enumerate(free):
            if v and start is None:
                start = i
            elif not v and start is not None:
                out.append((start, i)); start = None
        if start is not None:
            out.append((start, len(free)))
        return out

    @staticmethod
    def _nothing(why):
        return {"steer": None, "target_deg": None, "gap_deg": 0.0,
                "gap_depth_m": 0.0, "nearest_m": 0.0, "note": why}
