#!/usr/bin/env python3
"""
oled_stats.py — Live Jetson Orin Nano system stats on a 128x64 SSD1306 I2C OLED.

Shows: SoC/CPU temperature, overall CPU%, GPU%, fan speed (RPM or PWM%),
and GPS lock status + signal strength (Radiolink SE100 on the header UART).

Usage metrics (CPU, GPU) are drawn as scrolling ECG / heartbeat waveforms whose
beat rate and spike height scale with load (idle = slow, shallow beats; heavy
load = fast, tall spikes). Temperature and fan are shown as readouts along the
bottom. A small pulse dot in the title beats in sync with the CPU trace.

Stats are *sampled* ~once per second, but the screen is *rendered* at a higher
frame rate so the waveforms scroll smoothly and values ease between samples.

Data sources, in order of preference:
  1. jetson-stats `jtop` Python API  (best for GPU / temp / fan on Jetson)
  2. Plain Linux fallbacks           (psutil for CPU, sysfs for temp/gpu/fan)

If any single value can't be read it shows "N/A" rather than crashing.

Rendering uses luma.oled (SSD1306) + PIL fonts.

----------------------------------------------------------------------------
ADDING A NEW METRIC (still trivial):
  Write a provider returning (text, percent_or_None), then register a Metric
  with a `kind`:

      def metric_power():
          return ("4.2W", 35.0)            # (text, percent)

      # "wave" -> scrolling ECG trace driven by the percent (a usage metric)
      METRICS.append(Metric("Pwr", metric_power, kind="wave"))
      # "text" -> plain readout along the bottom row (percent may be None)
      METRICS.append(Metric("Mem", metric_mem,   kind="text"))

  The renderer stacks every "wave" metric into the waveform region and spreads
  every "text" metric across the bottom row, so adding either just works.
----------------------------------------------------------------------------
"""

import glob
import math
import os
import signal
import sys
import threading
import time

# =====================  CONFIG  =============================================
I2C_BUS = 7            # /dev/i2c-7 -> 40-pin header pins 3 (SDA) / 5 (SCL)
I2C_ADDRESS = 0x3C     # SSD1306 default address (some modules use 0x3D)
REFRESH_SECONDS = 1.0  # how often the *values* are re-sampled
RENDER_FPS = 25        # how often the *screen* is redrawn (smooth scrolling)
TITLE = "ORIN NANO"    # title-bar text

# Bar/readout full-scale references.
TEMP_MAX_C = 90.0      # (kept for reference / future bars)
FAN_MAX_RPM = 6000.0

# GPS (Radiolink SE100 on the 40-pin header UART). NMEA at 38400 8N1.
# NOTE: only one process can own this serial port at a time, so don't run
# gps_web.py and this script against the same port simultaneously.
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUD = 38400
SNR_MAX = 50.0         # dB-Hz treated as "full strength" for the bar/percent
# ===========================================================================

import psutil
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont

try:
    import serial          # pyserial — used only for the GPS reader thread
    HAVE_SERIAL = True
except Exception:
    HAVE_SERIAL = False

JETSON = None          # live jtop handle, or None when unavailable
_STOP = False          # set by SIGINT/SIGTERM to request a clean shutdown


def _request_stop(*_):
    """Signal handler: ask the render loop to exit so the display gets cleared."""
    global _STOP
    _STOP = True

WIDTH, HEIGHT = 128, 64
EASE = 0.25            # per-frame easing for the usage intensity (0..1)
ECG_SCROLL_PX = 2      # pixels the waveform scrolls per rendered frame
ECG_BUF = WIDTH + 8    # ring-buffer length for each waveform


# =====================  LOW-LEVEL READ HELPERS  ============================
def _read_int(path):
    """Read a single integer from a sysfs file, or None on any failure."""
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def _read_thermal_zone(type_name):
    """Return the temperature (C) of the thermal zone whose 'type' matches."""
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


def _hwmon_path(name):
    """Return the /sys/class/hwmon/hwmonN dir whose 'name' equals `name`.

    hwmon numbers are not stable across boots, so we look them up by name.
    """
    for hw in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            with open(os.path.join(hw, "name")) as f:
                if f.read().strip() == name:
                    return hw
        except OSError:
            continue
    return None


# =====================  METRIC PROVIDERS  ==================================
# Each provider returns (text, percent_or_None).

def metric_temperature():
    """SoC/CPU temperature in Celsius (jtop preferred, sysfs fallback)."""
    temp = None
    if JETSON is not None:
        try:
            temps = JETSON.temperature
            for key in ("CPU", "cpu", "Tj", "tj", "TJ", "SOC0", "soc0"):
                if key in temps:
                    entry = temps[key]
                    val = entry.get("temp") if isinstance(entry, dict) else entry
                    if val is not None:
                        temp = float(val)
                        break
        except Exception:
            temp = None
    if temp is None:
        temp = _read_thermal_zone("cpu-thermal")
    if temp is None:
        temp = _read_thermal_zone("tj-thermal")
    if temp is None:
        return ("N/A", None)
    return ("%.0fC" % temp, temp / TEMP_MAX_C * 100.0)


def metric_cpu():
    """Overall CPU usage %. psutil is reliable on all Jetsons, so use it always."""
    pct = psutil.cpu_percent(interval=None)  # non-blocking; primed in main()
    return ("%.0f%%" % pct, pct)


def metric_gpu():
    """GPU usage % (jtop preferred, sysfs 'load' per-mille fallback)."""
    if JETSON is not None:
        try:
            gpu = JETSON.gpu
            for entry in gpu.values():
                if isinstance(entry, dict):
                    status = entry.get("status", entry)
                    load = status.get("load") if isinstance(status, dict) else None
                    if load is not None:
                        return ("%.0f%%" % load, float(load))
        except Exception:
            pass
    for path in glob.glob("/sys/devices/platform/*/[0-9]*.gpu/load") + \
            glob.glob("/sys/devices/platform/*.gpu/load"):
        raw = _read_int(path)
        if raw is not None:
            pct = raw / 10.0
            return ("%.0f%%" % pct, pct)
    return ("N/A", None)


def metric_fan():
    """Fan speed: RPM if a tachometer exists, else PWM duty %."""
    if JETSON is not None:
        try:
            fan = JETSON.fan
            for entry in fan.values():
                if not isinstance(entry, dict):
                    continue
                rpm = entry.get("rpm")
                if isinstance(rpm, (list, tuple)) and rpm:
                    rpm = rpm[0]
                if rpm:
                    return ("%drpm" % int(rpm), float(rpm) / FAN_MAX_RPM * 100.0)
                speed = entry.get("speed")
                if isinstance(speed, (list, tuple)) and speed:
                    speed = speed[0]
                if speed is not None:
                    return ("%.0f%%" % speed, float(speed))
        except Exception:
            pass
    tach = _hwmon_path("pwm_tach")
    if tach is not None:
        rpm = _read_int(os.path.join(tach, "rpm"))
        if rpm is not None:
            return ("%drpm" % rpm, rpm / FAN_MAX_RPM * 100.0)
    pwm = _hwmon_path("pwmfan")
    if pwm is not None:
        raw = _read_int(os.path.join(pwm, "pwm1"))
        if raw is not None:
            pct = raw / 255.0 * 100.0
            return ("%.0f%%" % pct, pct)
    return ("N/A", None)


# =====================  GPS (background NMEA reader)  ======================
# A serial reader can't live inside a metric provider: readline() would block
# the render loop. Instead a daemon thread continuously parses NMEA and
# publishes a snapshot here; the GPS metric providers just read this dict.
GPS_LOCK = threading.Lock()
GPS_STATE = {
    "link": "off",      # "off" (no serial / no data), "stale", or "ok"
    "locked": False,    # valid 2D/3D fix (RMC status 'A' and GGA fix != 0)
    "in_view": 0,       # satellites reported across all constellations
    "used": "0",        # satellites used in the current fix (GGA field 7)
    "best": None,       # strongest satellite SNR in dB-Hz, or None
}


def _nmea_checksum_ok(line):
    """True if an NMEA sentence's trailing *HH XOR checksum matches."""
    if "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def gps_reader_loop():
    """Daemon thread: read NMEA from GPS_PORT and publish into GPS_STATE.

    Resilient by design — serial errors trigger a reconnect, the link goes
    "stale" then "off" when sentences stop, and nothing here ever raises into
    the render loop. If pyserial is missing the link simply stays "off".
    """
    if not HAVE_SERIAL:
        return
    last_rx = 0.0
    fix, status, used = "0", "V", "0"
    sats, acc = {}, {}          # per-constellation lists of SNRs (None if blank)
    ser = None
    while not _STOP:
        try:
            if ser is None:
                ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
            raw = ser.readline().decode(errors="replace").strip()
        except Exception:
            ser = None
            with GPS_LOCK:
                GPS_STATE["link"] = "off"
            time.sleep(1)
            continue

        now = time.monotonic()
        if raw.startswith("$") and _nmea_checksum_ok(raw):
            last_rx = now
            f = raw.split(",")
            typ = f[0][3:6]
            talker = f[0][1:3]
            if typ == "GGA" and len(f) >= 8:
                fix, used = f[6], f[7]
            elif typ == "RMC" and len(f) >= 3:
                status = f[2]
            elif typ == "GSV" and len(f) >= 4:
                if f[2] == "1":          # first page of this constellation
                    acc[talker] = []
                i = 4
                while i + 3 < len(f):
                    snr_s = f[i + 3].split("*")[0]
                    snr = int(snr_s) if snr_s.isdigit() else None
                    if f[i]:             # has a PRN
                        acc.setdefault(talker, []).append(snr)
                    i += 4
                if f[2] == f[1]:         # last page -> commit this constellation
                    sats[talker] = acc.get(talker, [])

        snrs = [v for lst in sats.values() for v in lst if v is not None]
        in_view = sum(len(lst) for lst in sats.values())
        link = "off"
        if last_rx:
            link = "ok" if (now - last_rx) < 3 else "stale"
        with GPS_LOCK:
            GPS_STATE.update({
                "link": link,
                "locked": status == "A" and fix not in ("", "0"),
                "in_view": in_view,
                "used": used,
                "best": max(snrs) if snrs else None,
            })


def metric_gps():
    """GPS lock status + satellite count (text readout)."""
    with GPS_LOCK:
        st = dict(GPS_STATE)
    if st["link"] == "off":
        return ("no link", None)
    if st["locked"]:
        try:
            used = int(st["used"])
        except ValueError:
            used = st["used"]
        return ("LOCK %ss" % used, 100.0)
    return ("SRCH %ds" % st["in_view"], 0.0)


def metric_gps_signal():
    """Strongest satellite SNR in dB-Hz (text readout). Percent for future bars."""
    with GPS_LOCK:
        st = dict(GPS_STATE)
    if st["link"] == "off":
        return ("--", None)
    best = st["best"]
    if best is None:
        return ("0dB", 0.0)
    return ("%ddB" % best, min(100.0, best / SNR_MAX * 100.0))


# =====================  ECG WAVEFORM  ======================================
def _ecg_sample(t):
    """One point of a stylized ECG (PQRST) beat for phase t in [0, 1).

    Returns roughly -0.25 .. 1.0 (R-spike peaks at 1.0).
    """
    def bump(center, width):
        return math.exp(-((t - center) / width) ** 2)
    return (0.10 * bump(0.15, 0.025)    # P wave
            - 0.12 * bump(0.27, 0.012)  # Q dip
            + 1.00 * bump(0.30, 0.010)  # R spike
            - 0.24 * bump(0.34, 0.016)  # S dip
            + 0.22 * bump(0.52, 0.045)) # T wave


class ECGTrace:
    """A scrolling ECG buffer whose rate/amplitude follow a 0..1 intensity."""

    def __init__(self):
        self.samples = [0.0] * ECG_BUF
        self.phase = 0.0
        self.intensity = 0.0

    def update(self, dt, intensity):
        self.intensity = max(0.0, min(1.0, intensity))
        # Heart rate rises with load: ~0.8 Hz idle -> ~3 Hz busy.
        rate = 0.8 + self.intensity * 2.2
        for _ in range(ECG_SCROLL_PX):
            self.phase = (self.phase + rate * dt / ECG_SCROLL_PX) % 1.0
            self.samples.append(_ecg_sample(self.phase))
            self.samples.pop(0)

    def pulse(self):
        """Current beat height (0..1) — used to drive the title pulse dot."""
        return max(0.0, _ecg_sample(self.phase))

    def draw(self, draw, x0, y0, x1, y1):
        base = y1 - 1                       # baseline near the bottom of the band
        amp = 2.0 + self.intensity * (y1 - y0 - 3)
        w = x1 - x0
        n = len(self.samples)
        pts = []
        for i in range(w):
            v = self.samples[n - w + i]
            y = base - v * amp
            y = max(y0, min(y1, y))         # clamp inside the band
            pts.append((x0 + i, y))
        if len(pts) > 1:
            draw.line(pts, fill=255)


# =====================  METRIC REGISTRY  ===================================
class Metric:
    """A named stat. `kind` is 'wave' (ECG trace) or 'text' (bottom readout).

    `read()` never raises — failures become ('N/A', None). For wave metrics,
    `shown` eases toward the sampled percent and drives the trace intensity.
    """

    def __init__(self, label, provider, kind="text"):
        self.label = label
        self.provider = provider
        self.kind = kind
        self.text = "..."
        self.target = 0.0
        self.shown = 0.0
        self.trace = ECGTrace() if kind == "wave" else None

    def sample(self):
        try:
            text, pct = self.provider()
        except Exception:
            text, pct = ("N/A", None)
        self.text = text
        if pct is not None:
            self.target = max(0.0, min(100.0, pct))

    def ease(self):
        self.shown += (self.target - self.shown) * EASE

    def animate(self, dt):
        """Advance per-frame animation (waveform scroll) for wave metrics."""
        if self.trace is not None:
            self.trace.update(dt, self.shown / 100.0)


# To add a metric, append one line (see the recipe in the header comment).
METRICS = [
    Metric("CPU", metric_cpu,         kind="wave"),
    Metric("GPU", metric_gpu,         kind="wave"),
    Metric("Tmp", metric_temperature, kind="text"),
    Metric("Fan", metric_fan,         kind="text"),
    Metric("GPS", metric_gps,         kind="text"),
    Metric("Sig", metric_gps_signal,  kind="text"),
]


# =====================  RENDERING  =========================================
def load_fonts():
    """Return (big_font, small_font), falling back to PIL's builtin font."""
    bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    reg = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    if os.path.exists(bold) and os.path.exists(reg):
        return ImageFont.truetype(bold, 11), ImageFont.truetype(reg, 9)
    f = ImageFont.load_default()
    return f, f


def _text_w(draw, text, font):
    try:
        return int(draw.textlength(text, font=font))
    except AttributeError:
        bbox = draw.textbbox((0, 0), text, font=font)
        return bbox[2] - bbox[0]


def render(draw, fonts, metrics, _phase):
    """Draw one animated frame onto a luma/PIL canvas."""
    big, small = fonts
    waves = [m for m in metrics if m.kind == "wave"]
    texts = [m for m in metrics if m.kind == "text"]

    # ---- Title bar: inverted strip + pulse dot synced to the CPU beat ----
    draw.rectangle((0, 0, WIDTH - 1, 11), fill=255)
    draw.text((2, 0), TITLE, font=big, fill=0)
    if waves:
        p = waves[0].trace.pulse()
        r = 2 + int(round(p * 2))
        cx, cy = WIDTH - 7, 5
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=0)

    # ---- Bottom readout rows (text metrics, two per row, growing upward) ----
    # Each pair is laid out left + right; an odd final metric sits on the left.
    row_h = 11
    n_rows = (len(texts) + 1) // 2 if texts else 0
    text_top = HEIGHT - n_rows * row_h
    for r in range(n_rows):
        ry = text_top + r * row_h
        left = texts[2 * r]
        draw.text((0, ry), "%s %s" % (left.label, left.text), font=small, fill=255)
        if 2 * r + 1 < len(texts):
            right = texts[2 * r + 1]
            s1 = "%s %s" % (right.label, right.text)
            draw.text((WIDTH - _text_w(draw, s1, small), ry), s1, font=small, fill=255)
    if texts:
        draw.line((0, text_top - 2, WIDTH - 1, text_top - 2), fill=255)  # separator

    # ---- Waveform region: stack each wave metric into its own band ----
    top = 13
    bottom = (text_top if texts else HEIGHT) - 3
    if waves:
        band_h = (bottom - top) // len(waves)
        # Label + value share one line ("CPU 45%"); align every trace start to
        # the widest header so the CPU/GPU waveforms line up vertically.
        headers = ["%s %s" % (m.label, m.text) for m in waves]
        trace_x = max(_text_w(draw, h, small) for h in headers) + 4
        for i, m in enumerate(waves):
            by0 = top + i * band_h
            by1 = by0 + band_h - 1
            # Header line, vertically centred in the band.
            ty = by0 + max(0, (band_h - 9) // 2)
            draw.text((0, ty), headers[i], font=small, fill=255)
            # ECG trace fills the full band height to the right of the header.
            m.trace.draw(draw, trace_x, by0, WIDTH - 1, by1)


# =====================  MAIN LOOP  =========================================
def _init_display():
    serial = i2c(port=I2C_BUS, address=I2C_ADDRESS)
    return ssd1306(serial, width=WIDTH, height=HEIGHT)


def _try_start_jtop():
    """Start jtop if available; return a live handle or None. Never raises."""
    try:
        from jtop import jtop
    except Exception:
        return None
    try:
        jet = jtop()
        jet.start()
        for _ in range(20):
            if jet.ok():
                break
            time.sleep(0.1)
        return jet
    except Exception:
        return None


def main():
    global JETSON

    try:
        device = _init_display()
    except Exception as exc:
        sys.stderr.write(
            "ERROR: could not open SSD1306 on /dev/i2c-%d @ 0x%02X: %s\n"
            "Check wiring and run: i2cdetect -y -r %d\n"
            % (I2C_BUS, I2C_ADDRESS, exc, I2C_BUS))
        return 1

    fonts = load_fonts()
    JETSON = _try_start_jtop()
    if JETSON is None:
        sys.stderr.write("jtop unavailable; using psutil/sysfs fallbacks.\n")

    psutil.cpu_percent(interval=None)  # prime psutil's first (bogus) reading

    # GPS runs in its own daemon thread so its blocking serial reads never
    # stall the render loop; it publishes into GPS_STATE for the metrics.
    if HAVE_SERIAL:
        threading.Thread(target=gps_reader_loop, daemon=True).start()
    else:
        sys.stderr.write("pyserial missing; GPS readout disabled.\n")

    # Catch both Ctrl+C (SIGINT) and kill / `timeout` / systemd stop (SIGTERM)
    # so the `finally` block always clears the panel instead of leaving it frozen.
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    frame_dt = 1.0 / RENDER_FPS
    start = time.monotonic()
    last_sample = -1e9
    try:
        while not _STOP:
            now = time.monotonic()
            if now - last_sample >= REFRESH_SECONDS:
                for m in METRICS:
                    m.sample()
                last_sample = now
            for m in METRICS:
                m.ease()
                m.animate(frame_dt)
            with canvas(device) as draw:
                render(draw, fonts, METRICS, now - start)
            time.sleep(frame_dt)
    except KeyboardInterrupt:
        pass
    finally:
        if JETSON is not None:
            try:
                JETSON.close()
            except Exception:
                pass
        try:
            device.clear()
            device.hide()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
