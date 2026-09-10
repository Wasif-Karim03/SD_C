#!/usr/bin/env python3
"""
control/test_follow_the_gap.py — scenes with an unarguable right answer.

These are not a formality. Every one of them caught a real bug on the first
run of the algorithm they test:

  open field          steered hard LEFT across an empty room, because argmax
                      over a flat array returns index 0 -- the leftmost bin.
  wall ahead+right    steered INTO the wall, because textbook Follow-the-Gap
                      takes the widest run and broke a tie by array order.
  boxed in at 0.4 m   found a "gap" in a wall it was nearly touching, because
                      a gap was required to be wide but not deep.

Run: python3 control/test_follow_the_gap.py       (exit 0 = all pass)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.follow_the_gap import FollowTheGap          # noqa: E402


def scene(**kw):
    g = np.full(360, kw.get("base", 3.0), dtype=np.float64)
    for lo, hi, v in kw.get("patches", []):
        g[lo:hi] = v
    return g


CASES = [
    ("open field — nothing in range",
     scene(base=0.0),
     lambda s: s is not None and abs(s) < 0.10,
     "drive straight; an empty world is not a reason to turn"),

    ("wall ahead and to +bins",
     scene(patches=[(0, 45, 0.9)]),
     lambda s: s is not None and s < -0.10,
     "steer away from the wall, toward the open side"),

    ("wall ahead and to -bins",
     scene(patches=[(315, 360, 0.9)]),
     lambda s: s is not None and s > 0.10,
     "mirror image of the above"),

    ("boxed in at 0.4 m on every bearing",
     scene(base=0.4),
     lambda s: s is None,
     "refuse: there is nowhere to go, and saying so beats inventing a gap"),

    ("corridor with the opening straight ahead",
     scene(base=1.2, patches=[(350, 360, 6.0), (0, 10, 6.0)]),
     lambda s: s is not None and abs(s) < 0.15,
     "go through it, not at a wall beside it"),

    ("narrow slot, too shallow to enter",
     scene(base=0.45, patches=[(0, 6, 0.55)]),
     lambda s: s is None,
     "a 10 cm recess in a wall is not a gap"),
]


def main():
    ftg = FollowTheGap()
    fails = 0
    print(f"\n  FollowTheGap — {len(CASES)} scenes\n")
    for name, grid, want, why in CASES:
        p = ftg.plan(grid)
        s = p["steer"]
        good = want(s)
        fails += 0 if good else 1
        print(f"  [{'ok  ' if good else 'FAIL'}] {name}")
        print(f"         steer={'None' if s is None else format(s, '+.3f')}"
              f"  depth={p['gap_depth_m']:.2f} m"
              f"{'  (' + p['note'] + ')' if p['note'] else ''}")
        if not good:
            print(f"         expected: {why}")
    print(f"\n  {len(CASES) - fails}/{len(CASES)} passed\n")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
