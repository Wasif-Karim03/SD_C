#!/usr/bin/env python3
"""
perception/planner.py — localize on a saved map + plan a path (A*).

Two pieces the navigation needs:
  Localizer — scan-to-map ICP: match the live LiDAR scan against the saved map's
              walls to get the car's pose ON that map.
  astar     — shortest path over the occupancy grid's free space (obstacles
              inflated by the car's radius), from the car cell to a goal cell.

Traversable = not a wall and not within the inflation margin. Unknown cells are
allowed (the live camera+LiDAR brain handles anything actually there while driving).
"""
import os
import sys
import math
import heapq

import numpy as np
from scipy.ndimage import binary_dilation

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from perception.slam import RES, SIZE, ORIGIN, scan_to_xy, icp   # noqa: E402

OCC_LOGODDS = 0.8      # grid cell counts as wall above this log-odds
ROBOT_CELLS = 4        # inflate walls by ~0.2 m so the path keeps clearance
MAX_EXPAND = 80000     # A* safety cap


def occupied_cloud(grid):
    ys, xs = np.where(grid > OCC_LOGODDS)
    return np.column_stack([(xs - ORIGIN) * RES, (ys - ORIGIN) * RES])


def obstacle_mask(grid):
    occ = grid > OCC_LOGODDS
    return binary_dilation(occ, iterations=ROBOT_CELLS)


FREE_LOGODDS = -0.1       # cell counts as observed-FREE below this (one free pass
                          # = -0.4, so this includes lightly-seen free cells too)


def traversable_mask(grid):
    """True where the car may go: observed-free AND clear of inflated walls.
    (Unknown/unmapped cells are NOT traversable — you can only route where mapped.)"""
    occ_inflated = binary_dilation(grid > OCC_LOGODDS, iterations=ROBOT_CELLS)
    free = grid < FREE_LOGODDS
    return free & ~occ_inflated


def snap_to_traversable(trav, cell, max_r=60):
    """Nearest traversable cell to `cell` (so a click near a wall / the car's cell
    still plans). Returns None if nothing traversable is within max_r."""
    r0, c0 = cell
    R, C = trav.shape
    if 0 <= r0 < R and 0 <= c0 < C and trav[r0, c0]:
        return cell
    for rad in range(1, max_r):
        for dr in range(-rad, rad + 1):
            for dc in (-rad, rad):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < R and 0 <= c < C and trav[r, c]:
                    return (r, c)
        for dc in range(-rad, rad + 1):
            for dr in (-rad, rad):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < R and 0 <= c < C and trav[r, c]:
                    return (r, c)
    return None


def world_to_cell(x, y):
    return int(round(y / RES)) + ORIGIN, int(round(x / RES)) + ORIGIN   # (row, col)


def cell_to_world(r, c):
    return (c - ORIGIN) * RES, (r - ORIGIN) * RES


class Localizer:
    """Scan-to-map ICP pose tracker.

    HEALTH MATTERS: icp() reports whether it actually converged onto the map, and
    an unhealthy pose is far more dangerous than no pose at all — a follower that
    believes a diverged pose will confidently steer somewhere wrong. So the ok flag
    is kept (it used to be discarded), consecutive failures are counted, and the
    last pose is FROZEN rather than overwritten on a failed match. Callers should
    check `healthy()` before acting on `pose` and stop the car when it goes False.
    """

    MAX_FAILS = 5          # ~0.25 s at the cockpit's 20 Hz localize rate

    def __init__(self, grid, start_pose=(0.0, 0.0, 0.0)):
        self.cloud = occupied_cloud(grid)
        self.pose = start_pose
        self.ok = False            # did the LAST update converge?
        self.fails = 0             # consecutive failed/skipped updates
        self.updates = 0           # successful matches since construction

    def healthy(self):
        """True once we've matched at least once and aren't in a failure streak."""
        return self.updates > 0 and self.fails < self.MAX_FAILS

    def update(self, scan):
        pts = scan_to_xy(scan)
        if len(pts) < 15 or len(self.cloud) < 15:
            self.ok = False
            self.fails += 1
            return self.pose
        pose, ok = icp(pts, self.cloud, self.pose)
        self.ok = bool(ok)
        if ok:
            self.pose = pose       # only trust a converged match
            self.fails = 0
            self.updates += 1
        else:
            self.fails += 1        # freeze the old pose; let healthy() go False
        return self.pose


def astar(blocked, start, goal):
    """blocked: bool grid (True=cannot enter). start/goal: (row,col). -> [cells] or None."""
    R, C = blocked.shape
    if not (0 <= start[0] < R and 0 <= start[1] < C):
        return None
    if not (0 <= goal[0] < R and 0 <= goal[1] < C):
        return None
    if blocked[goal] or blocked[start]:
        return None

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    openq = [(h(start, goal), 0.0, start)]
    came = {}
    g = {start: 0.0}
    expanded = 0
    while openq and expanded < MAX_EXPAND:
        _, gc, cur = heapq.heappop(openq)
        expanded += 1
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dr, dc in nbrs:
            nr, nc = cur[0] + dr, cur[1] + dc
            if 0 <= nr < R and 0 <= nc < C and not blocked[nr, nc]:
                ng = gc + math.hypot(dr, dc)
                if (nr, nc) not in g or ng < g[(nr, nc)]:
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    heapq.heappush(openq, (ng + h((nr, nc), goal), ng, (nr, nc)))
    return None
