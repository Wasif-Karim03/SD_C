# recording/ — session capture

Nothing downstream works without this. Imitation learning needs
(observation → action) pairs. System identification needs (duty, steer) →
(speed, turn radius) over time. Simulator tuning needs real sensor traces to
match against. Failure analysis needs the seconds before the crash. Evaluation
needs the same route logged for two different policies. All of it is this file.

## Record

In **cockpit**: press **● REC** in the top bar. It turns red and shows elapsed
seconds and scan count. Press again to stop. Sessions land in
`final updates/logs/<timestamp>/` (gitignored).

From anywhere on the LAN:

```bash
curl -X POST "http://<jetson-ip>:8080/cmd?rec=1&note=stand%20test%20run%203"
curl -X POST "http://<jetson-ip>:8080/cmd?rec=0"
```

The `note` is worth setting — six sessions from one afternoon are
indistinguishable by timestamp alone.

## Read

```bash
cd "final updates"
python3 recording/reader.py --list                       # every session
python3 recording/reader.py logs/2026-09-08T14-30-05      # one summary
python3 recording/reader.py logs/2026-09-08T14-30-05 --plot run3.png
```

```python
from recording.reader import Session
s = Session("logs/2026-09-08T14-30-05").summary()

obs = s.scan_grid(bins=360)          # (M, 360) metres, bin 0 = straight ahead
act = s.actions()                    # (N, 2) [cmd_duty, cmd_steer]
idx = s.scan_index_for_telemetry()   # pair each action with the scan in force
use = s.moving_mask()                # rows where the car was actually rolling
```

## What is in a session

```
logs/2026-09-08T14-30-05/
  meta.json      schema, git sha, and the CALIBRATION SNAPSHOT for this run
  telemetry.csv  one row per 20 Hz control tick
  scans.bin      raw LiDAR revolutions, append-only
  scans.csv      index into scans.bin (seq, t, n, byte offset)
  frames.csv     camera index      (only when frames are enabled)
  frames/        front_000123.jpg  (only when frames are enabled)
```

**`meta.json` carries the calibration in force.** A session recorded at
`LIDAR_FORWARD_DEG = 340` is not comparable with one at `354.6`, and after a
re-calibration you need to know which is which. `reader.Session.scan_grid()`
re-references bearings using *that session's* forward angle, not today's.

**Scan angles are stored raw.** Exactly as the driver reports them, not rotated
to the nose. Re-calibrating the forward angle later does not invalidate old
recordings — the reader applies it at load time.

**Time base.** Every `t` is seconds since `time.monotonic()` at session start.
Use `t` for deltas, always. `t_wall` exists only to correlate with the outside
world; it can step.

## Design constraints worth knowing

- **The recorder owns no hardware.** Only one process may hold a serial port,
  so a recorder that opened its own would be unusable while driving. It is fed
  by whichever app already owns the ports.
- **It cannot break driving.** Every method swallows its own exceptions and
  counts them. The queue is bounded — on overflow it drops and counts rather
  than stalling a control loop.
- **It is crash-safe.** Everything is appended and flushed as it arrives. A
  session killed with SIGKILL is readable up to the last flush; only a clean
  stop sets `"complete": true` in `meta.json`.
- **Check `dropped` and `errors` before you train on a session.** The reader's
  summary shouts about them. A session that dropped samples has silent holes.

## Known gaps (deliberate, and worth fixing in this order)

1. **VESC columns are carried forward.** The action stream is logged at the
   20 Hz control rate, but VESC telemetry only refreshes at ~3 Hz, so
   `erpm` / `tach` / `speed_mps` repeat between reads. Fine for behaviour
   cloning; **not** fine for system identification — for that, raise the
   telemetry rate or log a dedicated fast tach channel.
2. **GPS and compass columns are never filled by cockpit** — it does not open
   either device. The columns exist so the schema does not have to change when
   they are wired up. Indoors this costs nothing (no GPS fix indoors anyway,
   and the compass is still uncalibrated — no `compass_cal.json` exists).
3. **Cameras are off by default** (`frames_hz=0`). Two 640×480 JPEG streams at
   10 Hz is roughly 1 GB/hour. Turn them on only for a run you intend to use.
4. **`speed_mps` is a finite difference of `tach`** computed in cockpit's
   telemetry loop, so it inherits that loop's 3 Hz quantisation.

## Next milestone this unblocks

**System identification.** Drive scripted patterns while recording, then fit:

- `duty → steady-state speed` — there is currently no such mapping anywhere in
  the codebase, and a simulator cannot be built without it.
- throttle rise time (the motor's first-order response to a duty step).
- `steer_norm → actual turn radius`, which yields the *measured*
  `WHEELBASE_M` and `MAX_STEER_ANGLE_RAD` — both are guesses today
  (`config.py` marks them `# TODO measure on the real chassis`) and both feed
  live into `control/odometry.py`.
- per-sensor latency, from the `scan_age` column.

Those four numbers are what make a simulated car behave like this car.
