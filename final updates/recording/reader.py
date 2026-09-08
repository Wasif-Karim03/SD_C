"""
recording/reader.py — load a recorded session back into numpy.

    from recording.reader import Session
    s = Session("logs/2026-09-08T14-30-05")
    s.summary()                       # human check: rates, gaps, drops
    t   = s.tel["t"]
    act = s.actions()                 # (N, 2) [duty, steer]  <- the labels
    obs = s.scan_grid(bins=360)       # (M, 360) metres, 0 = no return  <- the input
    idx = s.scan_index_for_telemetry()# align each telemetry row to a scan

CLI:
    python3 reader.py <session_dir>            # summary
    python3 reader.py <session_dir> --plot out.png
    python3 reader.py --list                   # every session under logs/

Design notes
------------
* Nothing here is lazy-loaded from a database; a session is plain files and
  this module is deliberately dependency-light (numpy only, matplotlib only
  for --plot) so it runs on the Jetson, on a laptop, and inside a training job.
* `scan_grid` is the function that turns a recording into a training tensor.
  It re-references raw scanner bearings to the car's nose using the
  LIDAR_FORWARD_DEG that was IN FORCE FOR THAT SESSION (read from meta.json),
  not the current one - which is the whole reason the raw angle is stored.
"""

from __future__ import annotations

import json
import os
import struct
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
DEFAULT_LOG_ROOT = os.path.join(_ROOT, "logs")

SCAN_MAGIC = 0x314E4353
_SCAN_HEADER = struct.Struct("<IIdI")
_HDR = _SCAN_HEADER.size

# Columns that are text by definition. Any other column that fails to parse as
# a float falls back to text automatically, so an app publishing a string into a
# numeric column degrades instead of making the whole session unreadable.
_TEXT_COLS = {"mode", "fault"}


class Session:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        if not os.path.isdir(self.path):
            raise FileNotFoundError(self.path)
        self.meta = self._load_meta()
        self.tel = self._load_telemetry()
        self.scan_idx = self._load_scan_index()
        self._scan_f = None

    # ------------------------------------------------------------- loading --
    def _load_meta(self):
        p = os.path.join(self.path, "meta.json")
        try:
            with open(p) as f:
                return json.load(f)
        except Exception:
            return {}

    def _load_telemetry(self):
        p = os.path.join(self.path, "telemetry.csv")
        if not os.path.exists(p):
            return {}
        with open(p) as f:
            header = f.readline().strip().split(",")
            rows = [ln.rstrip("\n").split(",") for ln in f if ln.strip()]
        out = {}
        for ci, name in enumerate(header):
            col = [r[ci] if ci < len(r) else "" for r in rows]
            if name in _TEXT_COLS:
                out[name] = np.array(col, dtype=object)
                continue
            try:
                out[name] = np.array([np.nan if c == "" else float(c) for c in col],
                                     dtype=np.float64)
            except ValueError:
                out[name] = np.array(col, dtype=object)
        return out

    def _load_scan_index(self):
        p = os.path.join(self.path, "scans.csv")
        if not os.path.exists(p):
            return {"seq": np.zeros(0), "t": np.zeros(0),
                    "n": np.zeros(0, dtype=np.int64), "offset": np.zeros(0, dtype=np.int64)}
        seq, t, n, off = [], [], [], []
        with open(p) as f:
            f.readline()
            for ln in f:
                parts = ln.strip().split(",")
                if len(parts) != 4:
                    continue
                seq.append(int(parts[0])); t.append(float(parts[1]))
                n.append(int(parts[2])); off.append(int(parts[3]))
        return {"seq": np.array(seq, dtype=np.int64), "t": np.array(t),
                "n": np.array(n, dtype=np.int64), "offset": np.array(off, dtype=np.int64)}

    # --------------------------------------------------------------- scans --
    @property
    def n_scans(self):
        return len(self.scan_idx["seq"])

    def scan(self, i):
        """Return (angles_deg, ranges_m) for scan i, as raw scanner bearings."""
        if self._scan_f is None:
            self._scan_f = open(os.path.join(self.path, "scans.bin"), "rb")
        off = int(self.scan_idx["offset"][i])
        n = int(self.scan_idx["n"][i])
        self._scan_f.seek(off)
        head = self._scan_f.read(_HDR)
        magic, seq, t, hn = _SCAN_HEADER.unpack(head)
        if magic != SCAN_MAGIC:
            raise ValueError(f"scan {i}: bad magic 0x{magic:08x} at offset {off}")
        if hn != n:
            raise ValueError(f"scan {i}: index says n={n}, record says n={hn}")
        raw = np.frombuffer(self._scan_f.read(8 * n), dtype=np.float32)
        pts = raw.reshape(-1, 2)
        return pts[:, 0].astype(np.float64), pts[:, 1].astype(np.float64)

    def forward_deg(self):
        """The LIDAR_FORWARD_DEG in force when this session was recorded."""
        return float(self.meta.get("config", {}).get("LIDAR_FORWARD_DEG", 0.0))

    def scan_grid(self, bins=360, indices=None, forward_deg=None,
                  min_m=0.0, max_m=12.0, fill=0.0):
        """(M, bins) float32 of ranges, re-referenced to the car's NOSE.

        Bin 0 is straight ahead; bins increase in the direction of increasing
        raw scanner angle. Empty bins get `fill` (default 0.0 = "no return"),
        which is the convention most 2D-LiDAR policies expect. Where several
        points land in one bin the NEAREST wins - a policy must never be shown
        a farther reading than the sensor actually saw.
        """
        fwd = self.forward_deg() if forward_deg is None else float(forward_deg)
        idx = range(self.n_scans) if indices is None else list(indices)
        out = np.full((len(idx), bins), float(fill), dtype=np.float32)
        step = 360.0 / bins
        for row, i in enumerate(idx):
            a, r = self.scan(i)
            ok = (r > min_m) & (r < max_m) & np.isfinite(r)
            if not ok.any():
                continue
            a, r = a[ok], r[ok]
            b = np.floor((np.mod(a - fwd, 360.0)) / step).astype(np.int64) % bins
            # nearest-wins: sort descending by range so the smallest lands last
            order = np.argsort(-r)
            out[row][b[order]] = r[order].astype(np.float32)
        return out

    # ------------------------------------------------------------ alignment --
    def scan_index_for_telemetry(self):
        """For each telemetry row, the index of the most recent scan at/before it.

        -1 where no scan had arrived yet. This is what pairs an observation with
        the action that was commanded while it was the current observation.
        """
        tt = self.tel.get("t")
        st = self.scan_idx["t"]
        if tt is None or len(st) == 0:
            return np.full(0 if tt is None else len(tt), -1, dtype=np.int64)
        return np.searchsorted(st, tt, side="right") - 1

    def actions(self):
        """(N, 2) [cmd_duty, cmd_steer] - the supervision signal for cloning."""
        d = self.tel.get("cmd_duty")
        s = self.tel.get("cmd_steer")
        if d is None or s is None:
            return np.zeros((0, 2), dtype=np.float32)
        return np.stack([np.nan_to_num(d), np.nan_to_num(s)], axis=1).astype(np.float32)

    def moving_mask(self, min_speed=0.05):
        """Rows where the car was actually rolling - the only rows worth cloning."""
        sp = self.tel.get("speed_mps")
        if sp is None:
            return np.zeros(0, dtype=bool)
        return np.nan_to_num(np.abs(sp)) > float(min_speed)

    # -------------------------------------------------------------- summary --
    def summary(self, out=print):
        m, tel, si = self.meta, self.tel, self.scan_idx
        out(f"session : {os.path.basename(self.path)}")
        out(f"  source: {m.get('source','?')}   note: {m.get('note','') or '-'}")
        out(f"  started {m.get('started_iso','?')}   schema {m.get('schema','?')}"
            f"   git {str(m.get('git_sha') or '-')[:8]}   complete={m.get('complete')}")
        cfg = m.get("config", {})
        if cfg:
            out(f"  calib : forward={cfg.get('LIDAR_FORWARD_DEG')}  "
                f"steer_sign={cfg.get('STEER_SIGN')}  "
                f"m/tach={cfg.get('METERS_PER_TACH')}  "
                f"wheelbase={cfg.get('WHEELBASE_M')}  self_mask_bins={cfg.get('_self_mask_bins')}")
        t = tel.get("t")
        if t is not None and len(t) > 1:
            dur = t[-1] - t[0]
            dt = np.diff(t)
            out(f"  telem : {len(t)} rows over {dur:.1f}s "
                f"({len(t)/max(dur,1e-9):.1f} Hz, max gap {dt.max()*1000:.0f} ms)")
        if len(si["t"]) > 1:
            sd = si["t"][-1] - si["t"][0]
            sdt = np.diff(si["t"])
            out(f"  scans : {len(si['t'])} revs ({len(si['t'])/max(sd,1e-9):.1f} Hz, "
                f"{si['n'].mean():.0f} pts avg, max gap {sdt.max()*1000:.0f} ms)")
        else:
            out("  scans : none")
        c = m.get("counts", {})
        if c.get("dropped") or c.get("errors"):
            out(f"  !! dropped={c.get('dropped')} errors={c.get('errors')} "
                f"- do not train on this session without checking why")
        if m.get("unknown_keys"):
            out(f"  note  : app published unknown fields {m['unknown_keys']}")
        mv = self.moving_mask()
        if len(mv):
            out(f"  motion: {mv.sum()} / {len(mv)} rows above 0.05 m/s "
                f"({100.0*mv.mean():.0f}% useful for cloning)")
        arm = tel.get("armed")
        if arm is not None and len(arm):
            out(f"  armed : {int(np.nansum(arm))} rows;  "
                f"estop rows {int(np.nansum(tel.get('estop', np.zeros(1))))}")
        return self

    def close(self):
        if self._scan_f is not None:
            self._scan_f.close()
            self._scan_f = None


def list_sessions(root=None):
    root = root or DEFAULT_LOG_ROOT
    if not os.path.isdir(root):
        return []
    return sorted(os.path.join(root, d) for d in os.listdir(root)
                  if os.path.isdir(os.path.join(root, d)))


def _plot(sess, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = sess.tel.get("t")
    fig, ax = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    ax[0].plot(t, sess.tel.get("cmd_duty"), label="cmd_duty")
    ax[0].plot(t, sess.tel.get("duty_actual"), label="duty_actual", alpha=.6)
    ax[0].set_ylabel("duty"); ax[0].legend(loc="upper right"); ax[0].grid(alpha=.3)
    ax[1].plot(t, sess.tel.get("speed_mps"), color="tab:green")
    ax[1].set_ylabel("m/s"); ax[1].grid(alpha=.3)
    ax[2].plot(t, sess.tel.get("cmd_steer"), color="tab:orange")
    ax[2].set_ylabel("steer"); ax[2].set_xlabel("t (s)"); ax[2].grid(alpha=.3)
    fig.suptitle(os.path.basename(sess.path))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print("wrote", out_png)


def main(argv):
    if not argv or argv[0] == "--list":
        for p in list_sessions():
            try:
                Session(p).summary(); print()
            except Exception as e:                               # noqa: BLE001
                print(f"{p}: unreadable ({e})\n")
        return 0
    s = Session(argv[0]).summary()
    if "--plot" in argv:
        _plot(s, argv[argv.index("--plot") + 1])
    s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
