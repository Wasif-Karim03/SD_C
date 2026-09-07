#!/usr/bin/env python3
"""Lightweight Jetson system-stats reader (no jtop / psutil dependency).

Reads CPU%, GPU%, SoC temperature, fan speed and memory straight from /proc
and sysfs, so it can run inside the web dashboard with zero extra packages and
without conflicting with the OLED service's jtop client. Every reader returns
None on failure rather than raising.

    s = SysStats()
    snap = s.read()   # call ~once per second; CPU% needs the delta between calls
"""
import glob
import os


# ---- generic sysfs helpers (mirrors oled_stats.py) ------------------------
def _read_int(path):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def _read_thermal_zone(type_name):
    for zone in glob.glob("/sys/devices/virtual/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zone, "type")) as f:
                if f.read().strip() != type_name:
                    continue
        except OSError:
            continue
        milli = _read_int(os.path.join(zone, "temp"))
        if milli is not None:
            return milli / 1000.0
    return None


def _hottest_zone():
    """Hottest of all thermal zones (C) — a safe 'system temp' fallback."""
    best = None
    for zone in glob.glob("/sys/devices/virtual/thermal/thermal_zone*"):
        milli = _read_int(os.path.join(zone, "temp"))
        if milli is not None:
            c = milli / 1000.0
            if best is None or c > best:
                best = c
    return best


def _hwmon_path(name):
    for hw in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            with open(os.path.join(hw, "name")) as f:
                if f.read().strip() == name:
                    return hw
        except OSError:
            continue
    return None


def gpu_percent():
    """GPU load % from the Tegra sysfs 'load' node (per-mille), or None."""
    for path in (glob.glob("/sys/devices/platform/*/[0-9]*.gpu/load")
                 + glob.glob("/sys/devices/platform/*.gpu/load")
                 + glob.glob("/sys/devices/gpu.0/load")):
        raw = _read_int(path)
        if raw is not None:
            return raw / 10.0
    return None


def temp_c():
    """SoC/CPU temperature in C. Tries common Jetson zone names, else hottest."""
    for name in ("cpu-thermal", "CPU-therm", "tj-thermal", "tj-therm"):
        v = _read_thermal_zone(name)
        if v is not None:
            return v
    return _hottest_zone()


def fan():
    """Return (rpm_or_None, pwm_pct_or_None)."""
    rpm = None
    tach = _hwmon_path("pwm_tach")
    if tach is not None:
        rpm = _read_int(os.path.join(tach, "rpm"))
    pct = None
    pwm = _hwmon_path("pwmfan") or _hwmon_path("pwm_fan")
    if pwm is not None:
        raw = _read_int(os.path.join(pwm, "pwm1"))
        if raw is not None:
            pct = raw / 255.0 * 100.0
    return rpm, pct


def mem():
    """Return (used_mb, total_mb, used_pct) from /proc/meminfo, or (None,)*3."""
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                info[k] = int(v.split()[0])   # value is in kB
        total = info.get("MemTotal")
        avail = info.get("MemAvailable")
        if total and avail is not None:
            used = total - avail
            return (used / 1024.0, total / 1024.0, 100.0 * used / total)
    except (OSError, ValueError, KeyError, IndexError):
        pass
    return (None, None, None)


class _CPUSampler:
    """Overall CPU% from /proc/stat deltas between successive reads."""

    def __init__(self):
        self._prev = None  # (idle, total)

    def percent(self):
        try:
            with open("/proc/stat") as f:
                parts = f.readline().split()
        except OSError:
            return None
        if not parts or parts[0] != "cpu":
            return None
        try:
            vals = [int(x) for x in parts[1:]]
        except ValueError:
            return None
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)  # idle + iowait
        total = sum(vals)
        prev = self._prev
        self._prev = (idle, total)
        if prev is None:
            return None                       # first call primes the delta
        d_total = total - prev[1]
        d_idle = idle - prev[0]
        if d_total <= 0:
            return None
        return max(0.0, min(100.0, 100.0 * (d_total - d_idle) / d_total))


class SysStats:
    """Holds the CPU delta state; `read()` returns a fresh snapshot dict."""

    def __init__(self):
        self._cpu = _CPUSampler()

    def read(self):
        rpm, pwm = fan()
        used_mb, total_mb, mem_pct = mem()
        return {
            "cpu": self._cpu.percent(),
            "gpu": gpu_percent(),
            "temp": temp_c(),
            "fan_rpm": rpm,
            "fan_pct": pwm,
            "mem_used": used_mb,
            "mem_total": total_mb,
            "mem_pct": mem_pct,
        }


if __name__ == "__main__":
    import time
    s = SysStats()
    s.read()                 # prime CPU
    while True:
        time.sleep(1)
        print(s.read())
