#!/usr/bin/env python3
"""Shared compass-calibration loader for the IST8310 readers.

`compass_calibrate.py` writes `compass_cal.json` (hard-iron offset + soft-iron
scale per axis). The readers (`compass_read.py`, `gps_web.py`) call `load()`
once and `apply()` on every raw sample before computing heading.

Calibration model (per axis i):
    corrected[i] = (raw[i] - offset[i]) * scale[i]

If the file is missing or unreadable, `load()` returns identity values
(offset 0, scale 1) so the readers fall back to raw behaviour unchanged.
"""
import json
import os

CAL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "compass_cal.json")

IDENTITY = {"offset": [0.0, 0.0, 0.0], "scale": [1.0, 1.0, 1.0]}


def load(path=CAL_PATH):
    """Return a calibration dict {'offset':[3], 'scale':[3]}.

    Always returns a usable dict — identity (no-op) if the file is absent or
    malformed, so callers never need to guard for missing calibration.
    """
    try:
        with open(path) as f:
            c = json.load(f)
        off = [float(v) for v in c["offset"]]
        scl = [float(v) for v in c["scale"]]
        if len(off) == 3 and len(scl) == 3:
            return {"offset": off, "scale": scl}
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return {"offset": list(IDENTITY["offset"]), "scale": list(IDENTITY["scale"])}


def apply(mx, my, mz, cal):
    """Apply `cal` (from load()) to a raw (x, y, z) sample -> corrected tuple."""
    off, scl = cal["offset"], cal["scale"]
    return ((mx - off[0]) * scl[0],
            (my - off[1]) * scl[1],
            (mz - off[2]) * scl[2])


def is_identity(cal):
    """True if `cal` does nothing (no calibration file was loaded)."""
    return (cal["offset"] == IDENTITY["offset"]
            and cal["scale"] == IDENTITY["scale"])
