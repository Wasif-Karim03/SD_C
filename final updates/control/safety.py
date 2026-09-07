#!/usr/bin/env python3
"""
control/safety.py — small, reusable safety primitives for the driving loop.

Keep the safety logic tiny and obvious so it's easy to trust:
  Watchdog   — trips if it isn't "pet" within a timeout (stale perception -> stop).
  clamp/ramp — bounded, smooth actuator changes (no jerks, never exceed a cap).

The driving philosophy (from the perception report): throttle starts DISARMED,
every actuator fails safe on exit, and a 'blocked' reading forces throttle to zero
regardless of what the planner wants. This module holds the shared helpers; the
loop wires them together.
"""
import time


class Watchdog:
    """Trips stale() if pet() hasn't been called within `timeout` seconds."""

    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self._t = time.monotonic()

    def pet(self):
        self._t = time.monotonic()

    def stale(self):
        return (time.monotonic() - self._t) > self.timeout


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def ramp(current, target, step):
    """Move `current` toward `target` by at most `step` (smooth accel/decel)."""
    if current < target:
        return min(target, current + step)
    return max(target, current - step)
