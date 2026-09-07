#!/usr/bin/env python3
"""Browser GPS dashboard for the Radiolink SE100 on the Jetson header UART.

Reads NMEA in a background thread and serves a live web page (no external
deps beyond pyserial). Open the printed URL in any browser on the network.

Usage: python3 gps_web.py [port] [baud] [http_port]
Defaults: /dev/ttyTHS1 @ 38400, served on :8080
Ctrl-C to stop.
"""
import os
import sys
import json
import time
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import math

import serial

# The steering servo lives in the sibling ../servo package. Make it importable
# so the dashboard can move the wheel through the same ServoController the
# autonomy code uses (no duplicate serial logic).
SERVO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "servo"))
if SERVO_DIR not in sys.path:
    sys.path.insert(0, SERVO_DIR)
try:
    from servo_control import ServoController, norm_to_angle, clamp, CENTER
    HAVE_SERVO = True
except Exception as _servo_err:  # pragma: no cover - import-time only
    HAVE_SERVO = False
    CENTER = 90
    _SERVO_IMPORT_ERR = str(_servo_err)

    def clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def norm_to_angle(norm):
        return int(round(CENTER + clamp(float(norm), -1.0, 1.0) * 45))

# The drive motor (VESC) lives in ../Camera Control. Reuse the same VESC class
# the autonomy/test code uses so there's one serial implementation.
VESC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Camera Control"))
if VESC_DIR not in sys.path:
    sys.path.insert(0, VESC_DIR)
try:
    from vesc_driver import VESC
    HAVE_VESC = True
except Exception as _vesc_err:  # pragma: no cover - import-time only
    HAVE_VESC = False
    _VESC_IMPORT_ERR = str(_vesc_err)

try:
    from smbus2 import SMBus
    HAVE_I2C = True
except ImportError:
    HAVE_I2C = False

import compass_cal
import sys_stats

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 38400
HTTP_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 8080

# IST8310 magnetometer (Radiolink SE100 compass), shared I2C bus with the OLED.
MAG_BUS = 7
MAG_ADDR = 0x0E
MAG_SENS = 0.3  # microtesla per LSB
COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

CONSTEL = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
           "GB": "BeiDou", "GQ": "QZSS", "GN": "Mixed"}
FIX_QUALITY = {"0": "no fix", "1": "GPS fix", "2": "DGPS fix",
               "4": "RTK fixed", "5": "RTK float", "6": "estimated"}

# ---- shared state, updated by reader thread, read by HTTP handler ----
LOCK = threading.Lock()
STATE = {
    "link": "waiting", "link_age": None, "sentences": 0, "uptime": 0,
    "fix": "0", "fix_text": "no fix", "status": "V", "locked": False,
    "utc": "", "used": "0", "hdop": "", "alt": "",
    "lat": None, "lon": None,
    "in_view": 0, "strong": 0, "best": None, "avg": None,
    "constellations": {},   # name -> [{prn, snr}]
    # compass
    "mag_link": "off", "heading": None, "cardinal": "", "mag_cal": "raw",
    "mx": None, "my": None, "mz": None,
    # system (Jetson Orin Nano)
    "cpu": None, "gpu": None, "temp": None,
    "fan_rpm": None, "fan_pct": None,
    "mem_used": None, "mem_total": None, "mem_pct": None,
}


class ServoManager:
    """Owns the steering-servo serial link for the dashboard's manual control.

    The link is opened lazily: enabling the toggle in the UI opens the port
    (which resets the Nano, ~2 s), centers the wheel, and starts holding the
    last commanded angle. Disabling centers, detaches, and closes the port so
    the autonomy code can grab it later. All serial access is serialized by an
    internal lock since the HTTP server is multi-threaded.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._ctrl = None
        self.norm = 0.0
        self.error = ""

    def status(self):
        with self._lock:
            on = self._ctrl is not None
            return {
                "servo_avail": HAVE_SERVO,
                "servo_enabled": on,
                "servo_norm": round(self.norm, 3),
                "servo_angle": norm_to_angle(self.norm),
                "servo_port": getattr(self._ctrl, "port", None) if on else None,
                "servo_error": self.error,
            }

    def enable(self):
        if not HAVE_SERVO:
            raise RuntimeError(
                "servo_control not importable: %s" % globals().get(
                    "_SERVO_IMPORT_ERR", "unknown"))
        with self._lock:
            if self._ctrl is None:
                self._ctrl = ServoController()   # opens + waits for READY
                self.norm = 0.0
                self.error = ""
                self._ctrl.center()

    def disable(self):
        with self._lock:
            if self._ctrl is not None:
                try:
                    self._ctrl.close()           # centers, detaches, closes
                finally:
                    self._ctrl = None
            self.norm = 0.0

    def set_norm(self, norm):
        norm = clamp(float(norm), -1.0, 1.0)
        with self._lock:
            self.norm = norm
            if self._ctrl is not None:
                self._ctrl.steer(norm)


SERVO = ServoManager()


# ---- throttle (VESC drive motor) ----
VESC_PORT = "/dev/ttyACM0"
MAX_DUTY = 0.20        # hard ceiling for manual driving (20% of battery voltage)
DUTY_STEP = 0.01       # +/- button increment
DEFAULT_DUTY = 0.05    # the 5% we bench-tested
DEADMAN_S = 0.5        # cut throttle if no drive keepalive within this window


class ThrottleManager:
    """Deadman drive control for the dashboard, backed by the VESC.

    Safety model, in priority order:
      * E-STOP  — latched; forces locked + zero throttle until cleared.
      * LOCK    — when locked (not armed) the motor cannot move; the VESC port
                  is released so the autonomy/test code can use it.
      * DEADMAN — the motor only spins while the browser keeps sending drive
                  keepalives (it does so while you hold the lever). If the page,
                  network, or tab dies, throttle drops to zero within DEADMAN_S.

    A single background thread owns the serial link: it opens the VESC when
    armed, streams the commanded duty at ~20 Hz (well under the VESC's ~1 s
    command timeout), reads telemetry, and closes the port when locked.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.armed = False
        self.estop = False
        self.level = DEFAULT_DUTY
        self.direction = 0          # -1 reverse, 0 stop, +1 forward
        self._last_drive = 0.0
        self.link = "off"           # off / ok / error
        self.error = ""
        self.telem = {}
        threading.Thread(target=self._loop, daemon=True).start()

    def status(self):
        with self._lock:
            t = self.telem
            return {
                "thr_avail": HAVE_VESC,
                "thr_armed": self.armed,
                "thr_estop": self.estop,
                "thr_level_pct": round(self.level * 100, 1),
                "thr_max_pct": round(MAX_DUTY * 100, 1),
                "thr_dir": self.direction,
                "thr_link": self.link,
                "thr_error": self.error,
                "thr_duty": t.get("duty"),
                "thr_erpm": t.get("erpm"),
                "thr_vin": t.get("v_in"),
                "thr_motor_i": t.get("motor_current"),
                "thr_fault": t.get("fault"),
            }

    def arm(self, on):
        with self._lock:
            if self.estop:
                return                      # must clear the e-stop first
            self.armed = bool(on)
            if not on:
                self.direction = 0

    def trigger_estop(self):
        with self._lock:
            self.estop = True
            self.armed = False
            self.direction = 0

    def clear_estop(self):
        with self._lock:
            self.estop = False              # stays locked (armed False)

    def set_level(self, duty):
        with self._lock:
            self.level = clamp(float(duty), 0.0, MAX_DUTY)

    def step_level(self, n):
        with self._lock:
            self.level = clamp(self.level + n * DUTY_STEP, 0.0, MAX_DUTY)

    def drive(self, direction):
        """Momentary: held by the lever, refreshes the deadman timestamp."""
        with self._lock:
            if not self.armed or self.estop:
                self.direction = 0
                return
            self.direction = 1 if direction > 0 else -1 if direction < 0 else 0
            self._last_drive = time.time()

    def _loop(self):
        if not HAVE_VESC:
            with self._lock:
                self.link = "off"
                self.error = globals().get("_VESC_IMPORT_ERR", "vesc_driver missing")
            return
        v = None
        last_telem = 0.0
        while True:
            with self._lock:
                want = self.armed and not self.estop
                direction = self.direction
                level = self.level
                fresh = (time.time() - self._last_drive) < DEADMAN_S
            try:
                if want and v is None:
                    v = VESC(VESC_PORT)
                    with self._lock:
                        self.link = "ok"
                        self.error = ""
                if (not want) and v is not None:
                    v.stop()
                    v.close()
                    v = None
                    with self._lock:
                        self.link = "off"
                if v is not None:
                    duty = direction * level if (direction != 0 and fresh) else 0.0
                    if duty == 0.0:
                        v.set_current(0.0)
                    else:
                        v.set_duty(duty)
                    if time.time() - last_telem > 0.5:
                        tv = v.get_values()
                        if tv:
                            with self._lock:
                                self.telem = tv
                        last_telem = time.time()
            except Exception as exc:
                with self._lock:
                    self.error = str(exc)
                    self.link = "error"
                    self.armed = False
                    self.direction = 0
                if v is not None:
                    try:
                        v.close()
                    except Exception:
                        pass
                    v = None
                time.sleep(0.3)
            time.sleep(0.05)


THROTTLE = ThrottleManager()


def mag_loop():
    """Poll the IST8310 and publish heading into STATE."""
    if not HAVE_I2C:
        with LOCK:
            STATE["mag_link"] = "off"
        return
    cal = compass_cal.load()
    cal_state = "raw" if compass_cal.is_identity(cal) else "calibrated"
    bus = None
    while True:
        try:
            if bus is None:
                bus = SMBus(MAG_BUS)
                if bus.read_byte_data(MAG_ADDR, 0x00) != 0x10:
                    raise OSError("not IST8310")
                bus.write_byte_data(MAG_ADDR, 0x41, 0x24)  # 16x averaging
                bus.write_byte_data(MAG_ADDR, 0x42, 0xC0)  # pulse duration
            bus.write_byte_data(MAG_ADDR, 0x0A, 0x01)      # single measurement
            for _ in range(20):
                if bus.read_byte_data(MAG_ADDR, 0x02) & 0x01:
                    break
                time.sleep(0.002)
            d = bus.read_i2c_block_data(MAG_ADDR, 0x03, 6)

            def s16(lo, hi):
                v = (hi << 8) | lo
                return v - 65536 if v & 0x8000 else v
            mx = s16(d[0], d[1]) * MAG_SENS
            my = s16(d[2], d[3]) * MAG_SENS
            mz = s16(d[4], d[5]) * MAG_SENS
            mx, my, mz = compass_cal.apply(mx, my, mz, cal)
            heading = math.degrees(math.atan2(my, mx)) % 360.0
            card = COMPASS[int((heading + 22.5) // 45) % 8]
            with LOCK:
                STATE.update({
                    "mag_link": "ok", "heading": round(heading, 1),
                    "cardinal": card, "mag_cal": cal_state,
                    "mx": round(mx, 1), "my": round(my, 1), "mz": round(mz, 1),
                })
        except (OSError, IOError):
            bus = None
            with LOCK:
                STATE["mag_link"] = "off"
            time.sleep(1)
        time.sleep(0.15)


def sys_loop():
    """Poll Jetson CPU/GPU/temp/fan/memory ~1 Hz and publish into STATE."""
    stats = sys_stats.SysStats()
    stats.read()          # prime the CPU% delta
    while True:
        time.sleep(1.0)
        snap = stats.read()
        with LOCK:
            STATE.update(snap)


def dm_to_deg(val, hemi):
    if not val:
        return None
    dot = val.find(".")
    deg = float(val[:dot - 2])
    minutes = float(val[dot - 2:])
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def checksum_ok(line):
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


def reader_loop():
    started = time.time()
    last_rx = 0.0
    sentences = 0
    s = {"fix": "0", "status": "V", "lat": None, "lon": None,
         "alt": "", "used": "0", "hdop": "", "utc": ""}
    sats = {}
    acc = {}
    ser = None
    while True:
        try:
            if ser is None:
                ser = serial.Serial(PORT, BAUD, timeout=1)
            raw = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            ser = None
            time.sleep(1)
            continue
        now = time.time()
        if raw.startswith("$") and checksum_ok(raw):
            last_rx = now
            sentences += 1
            f = raw.split(",")
            typ = f[0][3:6]
            talker = f[0][1:3]
            if typ == "GGA" and len(f) >= 10:
                s["utc"] = f[1][:6]
                s["fix"] = f[6]
                s["used"] = f[7]
                s["hdop"] = f[8]
                s["alt"] = f[9]
                lat = dm_to_deg(f[2], f[3])
                lon = dm_to_deg(f[4], f[5])
                if lat is not None:
                    s["lat"], s["lon"] = lat, lon
            elif typ == "RMC" and len(f) >= 3:
                s["status"] = f[2]
            elif typ == "GSV" and len(f) >= 4:
                name = CONSTEL.get(talker, talker)
                if f[2] == "1":
                    acc[name] = []
                i = 4
                while i + 3 < len(f):
                    prn = f[i]
                    snr_s = f[i + 3].split("*")[0]
                    snr = int(snr_s) if snr_s.isdigit() else None
                    if prn:
                        acc.setdefault(name, []).append({"prn": prn, "snr": snr})
                    i += 4
                if f[2] == f[1]:
                    sats[name] = acc.get(name, [])

        # publish snapshot
        all_sats = [x for lst in sats.values() for x in lst]
        snrs = [x["snr"] for x in all_sats if x["snr"] is not None]
        link_age = (now - last_rx) if last_rx else None
        link = "waiting"
        if last_rx:
            link = "ok" if link_age < 3 else "stale"
        with LOCK:
            STATE.update({
                "link": link,
                "link_age": round(link_age, 1) if link_age is not None else None,
                "sentences": sentences,
                "uptime": round(now - started),
                "fix": s["fix"],
                "fix_text": FIX_QUALITY.get(s["fix"], s["fix"]),
                "status": s["status"],
                "locked": s["status"] == "A" and s["fix"] not in ("", "0"),
                "utc": s["utc"], "used": s["used"], "hdop": s["hdop"],
                "alt": s["alt"], "lat": s["lat"], "lon": s["lon"],
                "in_view": len(all_sats),
                "strong": sum(1 for v in snrs if v >= 25),
                "best": max(snrs) if snrs else None,
                "avg": round(sum(snrs) / len(snrs), 1) if snrs else None,
                "constellations": sats.copy(),
            })


PAGE = """<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Jetson Dashboard</title>
<style>
 :root{--bg:#0d1117;--card:#161b22;--line:#30363d;--fg:#e6edf3;--mut:#8b949e;
       --ok:#2ea043;--warn:#d29922;--bad:#f85149;--accent:#58a6ff;}
 *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--fg);
   font-family:ui-monospace,Menlo,Consolas,monospace}
 header{padding:14px 18px;border-bottom:1px solid var(--line);display:flex;
   align-items:center;gap:12px;flex-wrap:wrap}
 h1{font-size:18px;margin:0} .dot{width:12px;height:12px;border-radius:50%}
 .grid{display:grid;gap:14px;padding:16px;grid-template-columns:repeat(auto-fit,minmax(240px,1fr))}
 .card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
 .card h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut);margin:0 0 10px}
 .big{font-size:30px;font-weight:700} .sub{color:var(--mut);font-size:12px;margin-top:4px}
 .row{display:flex;justify-content:space-between;padding:3px 0;font-size:14px}
 .row span:first-child{color:var(--mut)}
 table{width:100%;border-collapse:collapse;font-size:13px}
 td,th{padding:4px 6px;text-align:left} th{color:var(--mut);font-weight:500;border-bottom:1px solid var(--line)}
 .bar{height:8px;border-radius:4px;background:#21262d;overflow:hidden}
 .bar>i{display:block;height:100%}
 .pill{padding:2px 10px;border-radius:999px;font-size:12px;font-weight:600}
 .ok{background:rgba(46,160,67,.18);color:var(--ok)}
 .warn{background:rgba(210,153,34,.18);color:var(--warn)}
 .bad{background:rgba(248,81,73,.18);color:var(--bad)}
 a{color:var(--accent)}
 /* manual steering control */
 .sw{position:relative;display:inline-block;width:46px;height:24px}
 .sw input{opacity:0;width:0;height:0}
 .sw .sl{position:absolute;inset:0;background:#30363d;border-radius:999px;
   transition:.2s;cursor:pointer}
 .sw .sl:before{content:"";position:absolute;height:18px;width:18px;left:3px;
   bottom:3px;background:#e6edf3;border-radius:50%;transition:.2s}
 .sw input:checked+.sl{background:var(--accent)}
 .sw input:checked+.sl:before{transform:translateX(22px)}
 .steer-row{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:6px}
 .steer-slider{-webkit-appearance:none;appearance:none;width:100%;height:10px;
   border-radius:6px;background:linear-gradient(90deg,#f85149,#30363d 50%,#2ea043);
   outline:none}
 .steer-slider:disabled{opacity:.4}
 .steer-slider::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;
   width:26px;height:26px;border-radius:50%;background:var(--accent);
   border:3px solid #0d1117;cursor:pointer;box-shadow:0 0 0 1px var(--line)}
 .steer-slider::-moz-range-thumb{width:26px;height:26px;border-radius:50%;
   background:var(--accent);border:3px solid #0d1117;cursor:pointer}
 .steer-labels{display:flex;justify-content:space-between;color:var(--mut);
   font-size:12px;margin-top:4px}
 .btn{background:#21262d;color:var(--fg);border:1px solid var(--line);
   border-radius:8px;padding:6px 12px;font:inherit;cursor:pointer}
 .btn:hover{border-color:var(--accent)}
 .btn:disabled{opacity:.4;cursor:default}
 /* throttle / drive */
 .thr-wrap{display:flex;gap:22px;flex-wrap:wrap;align-items:center}
 .thr-controls{flex:1;min-width:240px}
 .estop{width:100%;background:var(--bad);color:#fff;border:none;border-radius:10px;
   padding:16px;font-size:20px;font-weight:800;letter-spacing:.06em;cursor:pointer;
   margin-bottom:12px;box-shadow:0 3px 0 #8b1a14}
 .estop:active{transform:translateY(2px);box-shadow:none}
 .thr-level{display:flex;align-items:center;gap:16px;margin:12px 0}
 .thr-level .btn{font-size:24px;width:48px;height:48px;line-height:1;padding:0}
 .lever{position:relative;width:88px;height:210px;border-radius:16px;flex:none;
   background:linear-gradient(#1b2330,#10151c);border:1px solid var(--line);
   display:flex;flex-direction:column;justify-content:space-between;align-items:center;
   padding:10px 0;touch-action:none;user-select:none;-webkit-user-select:none;cursor:grab}
 .lever.locked{opacity:.35;cursor:not-allowed}
 .lever:active{cursor:grabbing}
 .lever-label{color:var(--mut);font-size:12px;font-weight:700;pointer-events:none}
 .lever-knob{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
   width:66px;height:58px;border-radius:11px;background:#30363d;color:#0d1117;
   font-weight:800;font-size:12px;display:flex;align-items:center;justify-content:center;
   box-shadow:0 3px 10px rgba(0,0,0,.45);transition:top .12s,background .1s;pointer-events:none}
 .lever-knob.fwd{background:var(--ok)} .lever-knob.rev{background:var(--warn)}
</style></head>
<body>
<header>
 <span id="ld" class="dot" style="background:#888"></span>
 <h1>Jetson Dashboard</h1>
 <span id="link" class="pill warn">connecting…</span>
 <span class="sub" id="meta"></span>
</header>
<div class="grid">
 <div class="card" style="grid-column:1/-1">
  <h2>Manual steering</h2>
  <div class="steer-row">
   <label class="sw"><input type="checkbox" id="servoSw"><span class="sl"></span></label>
   <span id="servoState" class="pill warn">off</span>
   <span class="sub">flip on, then drag or scroll the bar to turn the wheel</span>
   <span style="margin-left:auto" class="big" id="servoAngle">—</span>
  </div>
  <input type="range" id="servoSlider" class="steer-slider" min="-100" max="100"
         step="1" value="0" disabled>
  <div class="steer-labels">
   <span>◀ full left</span>
   <button class="btn" id="servoCenter" disabled>center</button>
   <span>full right ▶</span>
  </div>
 </div>
 <div class="card" style="grid-column:1/-1">
  <h2>Throttle — drive motor</h2>
  <div class="thr-wrap">
   <div class="thr-controls">
    <button class="estop" id="estop">■ EMERGENCY STOP</button>
    <div class="steer-row">
     <button class="btn" id="thrLock">🔒 LOCKED</button>
     <span id="thrState" class="pill warn">locked</span>
     <span class="sub">unlock, set throttle, then hold the lever ▶</span>
    </div>
    <div class="thr-level">
     <button class="btn" id="thrMinus" disabled>−</button>
     <div>
      <div class="big" id="thrLevel">5%</div>
      <div class="sub">throttle (max <span id="thrMax">20</span>%)</div>
     </div>
     <button class="btn" id="thrPlus" disabled>+</button>
    </div>
    <div class="sub" id="thrTelem">motor link off</div>
   </div>
   <div class="lever locked" id="lever">
    <div class="lever-label">▲ FORWARD</div>
    <div class="lever-knob" id="leverKnob">HOLD</div>
    <div class="lever-label">▼ REVERSE</div>
   </div>
  </div>
 </div>
 <div class="card">
  <h2>System — Orin Nano</h2>
  <div class="row"><span>CPU</span><b id="cpu">—</b></div>
  <div class="bar" style="margin:3px 0 9px"><i id="cpubar" style="width:0%"></i></div>
  <div class="row"><span>GPU</span><b id="gpu">—</b></div>
  <div class="bar" style="margin:3px 0 9px"><i id="gpubar" style="width:0%"></i></div>
  <div class="row"><span>Memory</span><b id="mem">—</b></div>
  <div class="bar" style="margin:3px 0 9px"><i id="membar" style="width:0%"></i></div>
  <div class="row"><span>Temperature</span><b id="temp">—</b></div>
  <div class="row"><span>Fan</span><b id="fan">—</b></div>
 </div>
 <div class="card">
  <h2>Fix status</h2>
  <div class="big" id="fix">—</div>
  <div class="sub" id="fixsub">waiting for data…</div>
 </div>
 <div class="card">
  <h2>Satellites</h2>
  <div class="big" id="inview">0</div>
  <div class="sub">in view · <b id="used">0</b> used in fix</div>
  <div class="row" style="margin-top:8px"><span>Strong (&ge;25 dB)</span><b id="strong">0</b></div>
  <div class="row"><span>HDOP</span><b id="hdop">—</b></div>
 </div>
 <div class="card">
  <h2>Compass (IST8310)</h2>
  <div style="display:flex;align-items:center;gap:14px">
   <svg viewBox="-60 -60 120 120" width="104" height="104" style="flex:none">
    <circle cx="0" cy="0" r="54" fill="#0d1117" stroke="#30363d" stroke-width="2"/>
    <text x="0" y="-40" fill="#8b949e" font-size="11" text-anchor="middle">N</text>
    <text x="44" y="4" fill="#8b949e" font-size="11" text-anchor="middle">E</text>
    <text x="0" y="48" fill="#8b949e" font-size="11" text-anchor="middle">S</text>
    <text x="-44" y="4" fill="#8b949e" font-size="11" text-anchor="middle">W</text>
    <g id="needle" style="transition:transform .15s linear">
     <polygon points="0,-40 9,8 0,0 -9,8" fill="#f85149"/>
     <polygon points="0,40 9,-8 0,0 -9,-8" fill="#8b949e"/>
    </g>
    <circle cx="0" cy="0" r="4" fill="#e6edf3"/>
   </svg>
   <div>
    <div class="big" id="hdg">—</div>
    <div class="sub" id="hdgsub">waiting…</div>
   </div>
  </div>
 </div>
 <div class="card">
  <h2>Signal</h2>
  <div class="row"><span>Best SNR</span><b id="best">—</b></div>
  <div class="row"><span>Average SNR</span><b id="avg">—</b></div>
  <div class="sub">dB-Hz — 30+ is a solid signal</div>
 </div>
 <div class="card">
  <h2>Position</h2>
  <div class="big" id="pos" style="font-size:18px">no fix yet</div>
  <div class="sub" id="possub"></div>
  <div class="sub" id="maplink"></div>
 </div>
 <div class="card" style="grid-column:1/-1">
  <h2>Satellites in view</h2>
  <table><thead><tr><th>Const.</th><th>PRN</th><th style="width:60%">SNR</th><th>dB</th></tr></thead>
  <tbody id="sats"><tr><td colspan="4" style="color:#8b949e">none yet…</td></tr></tbody></table>
 </div>
</div>
<script>
const $=id=>document.getElementById(id);
function setPill(el,cls,txt){el.className='pill '+cls;el.textContent=txt;}
function snrColor(s){if(s==null)return '#444';if(s>=30)return '#2ea043';if(s>=20)return '#d29922';return '#f85149';}
function useColor(p){if(p==null)return '#30363d';if(p>=85)return '#f85149';if(p>=60)return '#d29922';return '#2ea043';}
function setBar(id,p){const el=$(id);el.style.width=(p==null?0:Math.min(100,p))+'%';el.style.background=useColor(p);}
async function tick(){
 try{
  const r=await fetch('/data',{cache:'no-store'});const d=await r.json();
  // link
  if(d.link==='ok'){$('ld').style.background='#2ea043';setPill($('link'),'ok','LINK OK');}
  else if(d.link==='stale'){$('ld').style.background='#d29922';setPill($('link'),'warn','STALE '+d.link_age+'s');}
  else{$('ld').style.background='#f85149';setPill($('link'),'bad','NO DATA');}
  $('meta').textContent=`${d.sentences} sentences · up ${d.uptime}s · UTC ${d.utc||'--'}`;
  // system
  $('cpu').textContent=d.cpu!=null?Math.round(d.cpu)+'%':'—';setBar('cpubar',d.cpu);
  $('gpu').textContent=d.gpu!=null?Math.round(d.gpu)+'%':'—';setBar('gpubar',d.gpu);
  if(d.mem_pct!=null){$('mem').textContent=Math.round(d.mem_pct)+'%  ('+Math.round(d.mem_used)+'/'+Math.round(d.mem_total)+' MB)';setBar('membar',d.mem_pct);}
  else{$('mem').textContent='—';setBar('membar',null);}
  $('temp').textContent=d.temp!=null?d.temp.toFixed(1)+' °C':'—';
  $('temp').style.color=d.temp==null?'#e6edf3':(d.temp>=80?'#f85149':(d.temp>=65?'#d29922':'#e6edf3'));
  $('fan').textContent=d.fan_rpm!=null?d.fan_rpm+' rpm':(d.fan_pct!=null?Math.round(d.fan_pct)+'%':'—');
  // fix
  $('fix').textContent=d.locked?d.fix_text.toUpperCase():'SEARCHING';
  $('fix').style.color=d.locked?'#2ea043':'#d29922';
  $('fixsub').textContent=d.locked?'valid fix':`status ${d.status} · need ~${Math.max(0,4-d.strong)} more strong sats`;
  // sats
  $('inview').textContent=d.in_view;$('used').textContent=d.used;
  $('strong').textContent=d.strong;$('hdop').textContent=d.hdop||'—';
  $('best').textContent=d.best!=null?d.best+' dB':'—';
  $('avg').textContent=d.avg!=null?d.avg+' dB':'—';
  // compass
  if(d.mag_link==='ok'&&d.heading!=null){
   $('needle').style.transform='rotate('+d.heading+'deg)';
   $('hdg').textContent=Math.round(d.heading)+'° '+d.cardinal;
   $('hdg').style.color='#e6edf3';
   $('hdgsub').textContent=`X ${d.mx} Y ${d.my} Z ${d.mz} µT · ${d.mag_cal}`;
  }else{
   $('hdg').textContent='—';$('hdg').style.color='#8b949e';
   $('hdgsub').textContent='compass not detected';
  }
  // position
  if(d.locked&&d.lat!=null){
   $('pos').textContent=d.lat.toFixed(6)+', '+d.lon.toFixed(6);
   $('possub').textContent='alt '+(d.alt||'?')+' m';
   $('maplink').innerHTML=`<a target="_blank" href="https://www.openstreetmap.org/?mlat=${d.lat}&mlon=${d.lon}#map=18/${d.lat}/${d.lon}">open in map ↗</a>`;
  }else{$('pos').textContent='no fix yet';$('possub').textContent='';$('maplink').textContent='';}
  // table
  const rows=[];const order=['GPS','GLONASS','Galileo','BeiDou','QZSS'];
  for(const name of order){const lst=d.constellations[name];if(!lst)continue;
   for(const s of lst){const pct=s.snr!=null?Math.min(100,s.snr*2):0;
    rows.push(`<tr><td>${name}</td><td>${s.prn}</td><td><div class="bar"><i style="width:${pct}%;background:${snrColor(s.snr)}"></i></div></td><td>${s.snr!=null?s.snr:'—'}</td></tr>`);}}
  $('sats').innerHTML=rows.length?rows.join(''):'<tr><td colspan="4" style="color:#8b949e">none yet…</td></tr>';
  // servo / manual steering link state (don't fight the user while dragging)
  if(d.servo_avail===false){setPill($('servoState'),'bad','unavailable');}
  else if(!servoBusy && document.activeElement!==sl){
   sw.checked=!!d.servo_enabled;
   sl.disabled=$('servoCenter').disabled=!d.servo_enabled;
   if(!d.servo_enabled){sl.value=0;}
   setServoState(d.servo_enabled);
  }
  $('servoAngle').textContent=d.servo_angle!=null?d.servo_angle+'°':'—';
  // throttle
  applyThr(d);
 }catch(e){setPill($('link'),'bad','server gone');}
}

// ---- manual steering control ----
const sw=$('servoSw'),sl=$('servoSlider'),cen=$('servoCenter');
let servoBusy=false,lastSend=0,pending=null,timer=null;
function setServoState(on){setPill($('servoState'),on?'ok':'warn',on?'LIVE':'off');}
function postServo(o){return fetch('/servo',{method:'POST',
  headers:{'Content-Type':'application/json'},body:JSON.stringify(o)})
  .then(r=>r.json()).catch(()=>null);}
function flushNorm(n){lastSend=Date.now();postServo({norm:n}).then(d=>{
  if(d&&d.servo_angle!=null)$('servoAngle').textContent=d.servo_angle+'°';});}
function sendNorm(n){            // throttle to ~16 Hz, always send the final value
  const now=Date.now();
  if(now-lastSend>=60){flushNorm(n);}
  else{pending=n;if(!timer)timer=setTimeout(()=>{timer=null;
    if(pending!=null){flushNorm(pending);pending=null;}},60);}
}
function onSlide(){if(sw.checked)sendNorm((+sl.value)/100);}
sw.addEventListener('change',async()=>{
  servoBusy=true;
  sl.disabled=cen.disabled=!sw.checked;
  setPill($('servoState'),'warn',sw.checked?'connecting…':'off');
  const d=await postServo({enable:sw.checked,norm:0});
  if(sw.checked){sl.value=0;}
  if(d&&d.ok===false){setPill($('servoState'),'bad','error');
   sw.checked=false;sl.disabled=cen.disabled=true;alert('Servo error: '+d.error);}
  else{setServoState(sw.checked);if(d&&d.servo_angle!=null)$('servoAngle').textContent=d.servo_angle+'°';}
  servoBusy=false;
});
sl.addEventListener('input',onSlide);
sl.addEventListener('wheel',e=>{           // scroll the bar to nudge the wheel
  if(!sw.checked)return;
  e.preventDefault();
  const step=(e.deltaY<0?1:-1)*(e.shiftKey?5:2);
  sl.value=Math.max(-100,Math.min(100,(+sl.value)+step));
  onSlide();
},{passive:false});
cen.addEventListener('click',()=>{if(sw.checked){sl.value=0;onSlide();}});

// ---- throttle / drive motor (deadman lever) ----
const estop=$('estop'),thrLock=$('thrLock'),thrPlus=$('thrPlus'),thrMinus=$('thrMinus'),
      lever=$('lever'),knob=$('leverKnob');
let thrArmed=false,thrEstop=false,leverActive=false,curDir=0,driveTimer=null;
function postThr(o){return fetch('/throttle',{method:'POST',
  headers:{'Content-Type':'application/json'},body:JSON.stringify(o)})
  .then(r=>r.json()).catch(()=>null);}
function applyThr(d){
  if(!d)return;
  thrArmed=!!d.thr_armed;thrEstop=!!d.thr_estop;
  if(d.thr_avail===false)setPill($('thrState'),'bad','no VESC');
  else if(thrEstop)setPill($('thrState'),'bad','E-STOP');
  else if(thrArmed)setPill($('thrState'),'ok','UNLOCKED');
  else setPill($('thrState'),'warn','locked');
  thrLock.textContent=thrEstop?'⟲ CLEAR E-STOP':(thrArmed?'🔓 UNLOCKED':'🔒 LOCKED');
  const usable=thrArmed&&!thrEstop;
  thrPlus.disabled=thrMinus.disabled=!usable;
  if(!leverActive)lever.classList.toggle('locked',!usable);
  if(d.thr_level_pct!=null)$('thrLevel').textContent=d.thr_level_pct+'%';
  if(d.thr_max_pct!=null)$('thrMax').textContent=d.thr_max_pct;
  if(d.thr_link==='ok'){
   $('thrTelem').textContent='duty '+(d.thr_duty!=null?Math.round(d.thr_duty*100)+'%':'—')
     +' · erpm '+(d.thr_erpm!=null?d.thr_erpm:'—')
     +' · '+(d.thr_vin!=null?d.thr_vin.toFixed(1)+'V':'')
     +' · I '+(d.thr_motor_i!=null?d.thr_motor_i.toFixed(1)+'A':'—')
     +' · fault '+(d.thr_fault!=null?d.thr_fault:'—');
  }else if(d.thr_error){$('thrTelem').textContent='err: '+d.thr_error;}
  else{$('thrTelem').textContent='motor link off';}
}
estop.addEventListener('click',()=>{endLever();postThr({estop:true}).then(applyThr);});
thrLock.addEventListener('click',()=>{
  if(thrEstop){postThr({clear:true}).then(applyThr);return;}
  postThr({arm:!thrArmed}).then(applyThr);
});
thrPlus.addEventListener('click',()=>postThr({step:1}).then(applyThr));
thrMinus.addEventListener('click',()=>postThr({step:-1}).then(applyThr));
function setKnob(dir){
  curDir=dir;
  knob.classList.toggle('fwd',dir>0);knob.classList.toggle('rev',dir<0);
  knob.textContent=dir>0?'FWD':dir<0?'REV':'HOLD';
  knob.style.top=dir>0?'20%':dir<0?'80%':'50%';
}
function leverDir(ev){
  const r=lever.getBoundingClientRect();
  const frac=(r.top+r.height/2-ev.clientY)/(r.height/2);  // up = positive
  return frac>0.2?1:frac<-0.2?-1:0;
}
function updateLever(ev){const d=leverDir(ev);if(d!==curDir){setKnob(d);postThr({drive:d}).then(applyThr);}}
lever.addEventListener('pointerdown',e=>{
  if(!thrArmed||thrEstop)return;
  e.preventDefault();leverActive=true;
  try{lever.setPointerCapture(e.pointerId);}catch(_){}
  updateLever(e);
  if(!driveTimer)driveTimer=setInterval(()=>{if(leverActive)postThr({drive:curDir});},150);
});
lever.addEventListener('pointermove',e=>{if(leverActive){e.preventDefault();updateLever(e);}});
function endLever(e){
  if(!leverActive&&curDir===0&&!driveTimer)return;
  leverActive=false;
  if(driveTimer){clearInterval(driveTimer);driveTimer=null;}
  setKnob(0);postThr({drive:0}).then(applyThr);
}
lever.addEventListener('pointerup',endLever);
lever.addEventListener('pointercancel',endLever);
// safety: stop driving if the tab is hidden or the window loses focus
document.addEventListener('visibilitychange',()=>{if(document.hidden)endLever();});
window.addEventListener('blur',endLever);

tick();setInterval(tick,1000);
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def _send_json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path.startswith("/servo"):
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                req = json.loads(raw or b"{}")
            except ValueError:
                req = {}
            resp = {"ok": True}
            try:
                if "enable" in req:
                    SERVO.enable() if req["enable"] else SERVO.disable()
                if "norm" in req and req["norm"] is not None:
                    SERVO.set_norm(req["norm"])
            except Exception as exc:
                SERVO.error = str(exc)
                resp = {"ok": False, "error": str(exc)}
            resp.update(SERVO.status())
            self._send_json(resp)
        elif self.path.startswith("/throttle"):
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                req = json.loads(raw or b"{}")
            except ValueError:
                req = {}
            resp = {"ok": True}
            try:
                if req.get("estop"):
                    THROTTLE.trigger_estop()
                if req.get("clear"):
                    THROTTLE.clear_estop()
                if "arm" in req:
                    THROTTLE.arm(req["arm"])
                if "level" in req and req["level"] is not None:
                    THROTTLE.set_level(req["level"])
                if "step" in req:
                    THROTTLE.step_level(req["step"])
                if "drive" in req and req["drive"] is not None:
                    THROTTLE.drive(req["drive"])
            except Exception as exc:
                resp = {"ok": False, "error": str(exc)}
            resp.update(THROTTLE.status())
            self._send_json(resp)
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        if self.path.startswith("/data"):
            with LOCK:
                snap = dict(STATE)
            snap.update(SERVO.status())
            snap.update(THROTTLE.status())
            self._send_json(snap)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main():
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=mag_loop, daemon=True).start()
    threading.Thread(target=sys_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    ip = lan_ip()
    print(f"GPS web dashboard reading {PORT} @ {BAUD}")
    print(f"  Open in browser:  http://localhost:{HTTP_PORT}")
    print(f"  From phone/PC on same network:  http://{ip}:{HTTP_PORT}")
    print("  Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped.")
        srv.shutdown()


if __name__ == "__main__":
    main()
