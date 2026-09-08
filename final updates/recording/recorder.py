"""
recording/recorder.py — fixed-rate session logger for the car.

WHY THIS EXISTS
---------------
Nothing downstream of here works without a timestamped record of what the car
sensed and what it was commanded to do:

  * imitation learning  needs (observation -> action) pairs from real driving
  * system identification needs (duty, steer) -> (speed, turn radius) over time
  * simulator tuning     needs real sensor traces to compare against
  * failure analysis     needs a replayable record of the seconds before a crash
  * evaluation           needs the same route logged for two different policies

DESIGN RULES
------------
1. NEVER raise into the caller. A logging fault must not affect driving.
   Every public method swallows its own exceptions and counts them.
2. NEVER block the caller. Samples go onto a bounded queue drained by a writer
   thread. If the queue fills we drop and count; we do not stall a control loop.
3. OWN NO HARDWARE. Only one process may hold a serial port, so a recorder that
   opened its own would be unusable while driving. The Recorder is *fed* by
   whichever app already owns the ports (apps/cockpit.py, apps/record.py).
4. CRASH-SAFE. Everything is appended and flushed as it arrives, so a session
   killed with SIGKILL is still readable up to the last flush.

SESSION LAYOUT  (one directory per run, default under `final updates/logs/`)
---------------------------------------------------------------------------
    logs/2026-09-08T14-30-05/
      meta.json      schema, git sha, config + calibration snapshot, counters
      telemetry.csv  one row per tick (default 20 Hz) - state + commanded action
      scans.bin      appended raw LiDAR revolutions (format below)
      scans.csv      index: seq, t, n, offset  -> random access into scans.bin
      frames.csv     index: seq, t, which, path      (only when frames are on)
      frames/        front_000123.jpg, rear_000123.jpg

scans.bin record format (little-endian, packed, no padding):
      magic  u4   0x314e4353  ('SCN1' as LE bytes)
      seq    u4
      t      f8   monotonic seconds since session start
      n      u4   number of points
      then n * (angle_deg f4, range_m f4)

Angles are the RAW scanner bearing exactly as the driver reports them - NOT
re-referenced to the car's nose. Applying `config.LIDAR_FORWARD_DEG` is the
reader's job, so re-calibrating the forward angle later does not invalidate
recordings made before it.

Time base: every `t` is seconds since `time.monotonic()` at session start.
Wall-clock is recorded once in meta.json plus per-row as `t_wall` so a session
can be correlated with external events, but deltas should always use `t`.
"""

from __future__ import annotations

import json
import os
import queue
import struct
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)          # "final updates/"
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

SCHEMA = 1
SCAN_MAGIC = 0x314E4353
_SCAN_HEADER = struct.Struct("<IIdI")   # magic, seq, t, n
_SCAN_POINT = struct.Struct("<ff")      # angle_deg, range_m

DEFAULT_LOG_ROOT = os.path.join(_ROOT, "logs")

# Column order is part of the on-disk schema. APPEND ONLY - never reorder or
# remove a column without bumping SCHEMA, or old sessions become unreadable.
TELEMETRY_COLUMNS = [
    "seq", "t", "t_wall",
    # --- the ACTION: what the controller commanded this tick --------------
    "cmd_duty", "cmd_steer",
    # --- mode / safety state ----------------------------------------------
    "mode", "armed", "estop", "follow_on",
    # --- proprioception (VESC telemetry) ----------------------------------
    "duty_actual", "erpm", "tach", "tach_abs", "v_in",
    "motor_current", "input_current", "temp_mos", "temp_motor", "fault",
    "speed_mps",
    # --- pose (SLAM / localizer, when a map is loaded) ---------------------
    "pose_x", "pose_y", "pose_yaw",
    # --- lidar link + derived ---------------------------------------------
    "scan_seq", "scan_age", "near_m", "blocked",
    # --- camera link -------------------------------------------------------
    "front_seq", "rear_seq",
    # --- gps ----------------------------------------------------------------
    "gps_lat", "gps_lon", "gps_fix", "gps_sats", "gps_speed", "gps_course",
    # --- compass -------------------------------------------------------------
    "heading_deg", "heading_cal",
]


def _git_sha(path):
    try:
        out = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=3)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return None


def _config_snapshot():
    """Freeze the calibration that was in force for this run.

    A recording is only as meaningful as the constants it was taken under - a
    session logged at LIDAR_FORWARD_DEG=340 is not comparable with one at 354.6.
    """
    try:
        import config
    except Exception:
        return {}
    keys = [
        "STEER_LEFT", "STEER_CENTER", "STEER_RIGHT",
        "METERS_PER_TACH", "WHEELBASE_M", "MAX_STEER_ANGLE_RAD",
        "MAX_DUTY", "MIN_MOVE_DUTY",
        "LIDAR_FORWARD_DEG", "STEER_SIGN", "STEER_SIGN_VERIFIED",
        "LIDAR_FRONT_ARC_DEG", "LIDAR_STEER_ARC_DEG",
        "LIDAR_STOP_M", "LIDAR_CLEAR_M", "LIDAR_MIN_M",
        "FREESPACE_STOP_M", "FREESPACE_CLEAR_M", "FREESPACE_SLOW_REACH",
        "CAM_WIDTH", "CAM_HEIGHT", "CAM_REAR_ROTATE",
    ]
    snap = {}
    for k in keys:
        try:
            snap[k] = getattr(config, k)
        except Exception:
            pass
    try:
        snap["_self_mask_bins"] = sum(1 for v in (config.load_self_mask() or []) if v)
    except Exception:
        pass
    return snap


def _fmt(v):
    """CSV cell. None -> empty; bool -> 0/1; float -> trimmed."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        if v != v:
            return ""            # NaN -> missing
        if v == float("inf"):
            return "inf"         # "nothing in the cone" is information, not missing
        if v == float("-inf"):
            return "-inf"
        return repr(round(v, 6))
    return str(v)


class Recorder:
    """Append-only session logger. Fed by the app that owns the hardware.

    Typical use inside a control loop:

        rec = Recorder(note="stand test").start()
        ...
        sseq = rec.log_scan(scan)                  # when a NEW revolution lands
        rec.log({"cmd_duty": duty, "cmd_steer": steer, "scan_seq": sseq, ...})
        ...
        rec.stop()

    Unknown keys passed to log() are ignored (with a counter), so an app can be
    updated to publish more fields before the schema catches up.
    """

    def __init__(self, root=None, name=None, note="", source="unknown",
                 frames_hz=0.0, jpeg_quality=80, queue_max=4000,
                 flush_every_s=1.0):
        self.root = root or DEFAULT_LOG_ROOT
        self.name = name or time.strftime("%Y-%m-%dT%H-%M-%S")
        self.note = note
        self.source = source
        self.frames_hz = float(frames_hz)
        self.jpeg_quality = int(jpeg_quality)
        self.flush_every_s = float(flush_every_s)

        self.dir = os.path.join(self.root, self.name)
        self.running = False
        self.t0 = None
        self.t0_wall = None

        self._q = queue.Queue(maxsize=int(queue_max))
        self._thread = None
        self._tel_f = None
        self._scan_f = None
        self._scan_idx_f = None
        self._frame_idx_f = None
        self._scan_bytes = 0

        self._tel_seq = 0
        self._scan_seq = 0
        self._frame_seq = 0
        self._last_frame_t = {}

        # counters - surfaced in meta.json and by stats(); a session that
        # dropped samples is a session you should not train on unquestioned.
        self.n_tel = 0
        self.n_scan = 0
        self.n_frame = 0
        self.n_dropped = 0
        self.n_errors = 0
        self.unknown_keys = set()

    # ---------------------------------------------------------------- start --
    def start(self, meta_extra=None):
        try:
            os.makedirs(self.dir, exist_ok=True)
            self.t0 = time.monotonic()
            self.t0_wall = time.time()

            self._tel_f = open(os.path.join(self.dir, "telemetry.csv"), "w",
                               buffering=1 << 16)
            self._tel_f.write(",".join(TELEMETRY_COLUMNS) + "\n")

            self._scan_f = open(os.path.join(self.dir, "scans.bin"), "wb",
                                buffering=1 << 18)
            self._scan_idx_f = open(os.path.join(self.dir, "scans.csv"), "w",
                                    buffering=1 << 14)
            self._scan_idx_f.write("seq,t,n,offset\n")

            if self.frames_hz > 0:
                os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)
                self._frame_idx_f = open(os.path.join(self.dir, "frames.csv"), "w",
                                         buffering=1 << 14)
                self._frame_idx_f.write("seq,t,which,path\n")

            self._write_meta(meta_extra, final=False)

            self.running = True
            self._thread = threading.Thread(target=self._writer, daemon=True,
                                            name="recorder")
            self._thread.start()
            print(f"  [rec] recording -> {self.dir}")
        except Exception as e:                                   # noqa: BLE001
            self.running = False
            self.n_errors += 1
            print(f"  [rec] FAILED to start: {e}")
        return self

    def _write_meta(self, extra=None, final=True):
        meta = {
            "schema": SCHEMA,
            "name": self.name,
            "note": self.note,
            "source": self.source,
            "started_wall": self.t0_wall,
            "started_iso": time.strftime("%Y-%m-%dT%H:%M:%S",
                                         time.localtime(self.t0_wall or time.time())),
            "git_sha": _git_sha(os.path.dirname(_ROOT)),
            "python": sys.version.split()[0],
            "frames_hz": self.frames_hz,
            "columns": TELEMETRY_COLUMNS,
            "config": _config_snapshot(),
            "counts": {"telemetry": self.n_tel, "scans": self.n_scan,
                       "frames": self.n_frame, "dropped": self.n_dropped,
                       "errors": self.n_errors},
            "complete": bool(final),
        }
        if final and self.t0 is not None:
            meta["duration_s"] = round(time.monotonic() - self.t0, 3)
        if self.unknown_keys:
            meta["unknown_keys"] = sorted(self.unknown_keys)
        if extra:
            meta["extra"] = extra
        try:
            with open(os.path.join(self.dir, "meta.json"), "w") as f:
                json.dump(meta, f, indent=2, default=str)
        except Exception:                                        # noqa: BLE001
            self.n_errors += 1

    # ------------------------------------------------------------------ log --
    def now(self):
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def log(self, sample):
        """Queue one telemetry row. Cheap, non-blocking, never raises."""
        if not self.running:
            return
        try:
            row = dict(sample)
            for k in row:
                if k not in _COLSET:
                    self.unknown_keys.add(k)
            row.setdefault("t", self.now())
            row.setdefault("t_wall", time.time())
            row["seq"] = self._tel_seq
            self._tel_seq += 1
            self._q.put_nowait(("tel", row))
        except queue.Full:
            self.n_dropped += 1
        except Exception:                                        # noqa: BLE001
            self.n_errors += 1

    def log_scan(self, scan, t=None):
        """Queue one LiDAR revolution. Returns its scan_seq (or None).

        `scan` is the driver's native list of (quality, angle_deg, dist_mm).
        Call this only when a NEW revolution arrives - logging the same scan
        twice makes the recording claim a sensor rate the C1 cannot deliver.
        """
        if not self.running or not scan:
            return None
        try:
            seq = self._scan_seq
            self._scan_seq += 1
            tt = self.now() if t is None else t
            # Serialise in the caller: deterministic, and it means the writer
            # thread never holds a reference to driver-owned data.
            buf = bytearray(_SCAN_HEADER.pack(SCAN_MAGIC, seq, tt, len(scan)))
            for p in scan:
                buf += _SCAN_POINT.pack(float(p[1]), float(p[2]) / 1000.0)
            self._q.put_nowait(("scan", seq, tt, len(scan), bytes(buf)))
            return seq
        except queue.Full:
            self.n_dropped += 1
            return None
        except Exception:                                        # noqa: BLE001
            self.n_errors += 1
            return None

    def log_frame(self, which, jpg_bytes, t=None):
        """Queue one camera frame (already JPEG-encoded). Rate-limited.

        Returns the frame seq, or None if throttled/off. Frames dominate disk:
        at 640x480 a JPEG is ~30-60 kB, so 10 Hz on two cameras is ~1 GB/hour.
        Default frames_hz=0 means cameras are NOT recorded unless asked for.
        """
        if not self.running or self.frames_hz <= 0 or not jpg_bytes:
            return None
        try:
            tt = self.now() if t is None else t
            last = self._last_frame_t.get(which, -1e9)
            if tt - last < (1.0 / self.frames_hz):
                return None
            self._last_frame_t[which] = tt
            seq = self._frame_seq
            self._frame_seq += 1
            self._q.put_nowait(("frame", seq, tt, str(which), bytes(jpg_bytes)))
            return seq
        except queue.Full:
            self.n_dropped += 1
            return None
        except Exception:                                        # noqa: BLE001
            self.n_errors += 1
            return None

    # --------------------------------------------------------------- writer --
    def _writer(self):
        last_flush = time.monotonic()
        while True:
            try:
                item = self._q.get(timeout=0.25)
            except queue.Empty:
                if not self.running:
                    break
                item = None
            if item is not None:
                try:
                    self._write_one(item)
                except Exception:                                # noqa: BLE001
                    self.n_errors += 1
            now = time.monotonic()
            if now - last_flush >= self.flush_every_s:
                self._flush()
                last_flush = now
        self._flush()

    def _write_one(self, item):
        kind = item[0]
        if kind == "tel":
            row = item[1]
            self._tel_f.write(",".join(_fmt(row.get(c)) for c in TELEMETRY_COLUMNS) + "\n")
            self.n_tel += 1
        elif kind == "scan":
            _, seq, tt, n, blob = item
            self._scan_idx_f.write(f"{seq},{tt:.6f},{n},{self._scan_bytes}\n")
            self._scan_f.write(blob)
            self._scan_bytes += len(blob)
            self.n_scan += 1
        elif kind == "frame":
            _, seq, tt, which, blob = item
            rel = os.path.join("frames", f"{which}_{seq:06d}.jpg")
            with open(os.path.join(self.dir, rel), "wb") as f:
                f.write(blob)
            self._frame_idx_f.write(f"{seq},{tt:.6f},{which},{rel}\n")
            self.n_frame += 1

    def _flush(self):
        for f in (self._tel_f, self._scan_f, self._scan_idx_f, self._frame_idx_f):
            try:
                if f is not None:
                    f.flush()
            except Exception:                                    # noqa: BLE001
                self.n_errors += 1

    # ----------------------------------------------------------------- stop --
    def stop(self):
        """Drain, close, finalise meta.json. Safe to call twice."""
        if not self.running:
            return self.stats()
        self.running = False
        try:
            if self._thread is not None:
                self._thread.join(timeout=3.0)
        except Exception:                                        # noqa: BLE001
            self.n_errors += 1
        for f in (self._tel_f, self._scan_f, self._scan_idx_f, self._frame_idx_f):
            try:
                if f is not None:
                    f.close()
            except Exception:                                    # noqa: BLE001
                self.n_errors += 1
        self._tel_f = self._scan_f = self._scan_idx_f = self._frame_idx_f = None
        self._write_meta(final=True)
        s = self.stats()
        print(f"  [rec] stopped: {s}")
        return s

    def stats(self):
        return {"dir": self.dir, "telemetry": self.n_tel, "scans": self.n_scan,
                "frames": self.n_frame, "dropped": self.n_dropped,
                "errors": self.n_errors,
                "duration_s": round(self.now(), 1)}

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False


_COLSET = set(TELEMETRY_COLUMNS)


if __name__ == "__main__":
    # Self-test with synthetic data - no hardware needed.
    import math
    import random
    import tempfile
    rec = Recorder(root=tempfile.mkdtemp(prefix="rec_selftest_"), name="session",
                   note="synthetic self-test", source="recorder.__main__",
                   frames_hz=2.0).start()
    for i in range(100):
        if i % 2 == 0:
            scan = [(15, a * 0.8, 1000 + 500 * math.sin(a / 30.0))
                    for a in range(450)]
            sseq = rec.log_scan(scan)
        rec.log({"cmd_duty": 0.07, "cmd_steer": 0.2 * math.sin(i / 10.0),
                 "mode": "drive", "armed": True, "estop": False,
                 "erpm": 1400 + random.randint(-40, 40), "tach": i * 7,
                 "v_in": 11.9, "speed_mps": 0.34, "scan_seq": sseq,
                 "near_m": float("inf"), "blocked": False,
                 "bogus_field": 1})
        if i % 10 == 0:
            rec.log_frame("front", b"\xff\xd8\xff\xd9")
        time.sleep(0.005)
    print(rec.stop())
