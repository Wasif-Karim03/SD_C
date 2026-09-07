# Camera Lab

Scratch workspace for camera experiments on the Jetson Orin Nano, kept separate
from the working `../Camera Control` code so nothing driving-related breaks.

## Hardware / baseline
- USB camera, YUYV 640x480 ~22fps, manual exposure pinned (see project memory).
- Perception stack already proven in `../Camera Control` (MiDaS depth, YOLO, lane).

## Layout
- Put experiment scripts here. Prefix throwaway captures with `scratch_`.

## Pipeline (inspired by LingBot-Map, decomposed for the Orin Nano)
LingBot-Map (arXiv 2604.14141) does single-camera streaming 3D reconstruction with
one 4.63 GB transformer — too heavy for an 8 GB Orin Nano. We rebuild the same job
from light parts that each fit, and use the car's real sensors for what the big
model has to learn:

  1. Depth   : Depth-Anything V2 metric-small  -> per-frame METERS   [DONE]
  2. Pose    : VESC wheel odometry + GPS (+ optional visual odometry)
  3. Fusion  : accumulate depth point clouds into one map (TSDF/voxel)
  4. Polish  : drift / loop-closure

## Files
- `depth_engine.py` — Depth-Anything V2 metric depth wrapper + `--bench` FPS test.
- `live_depth.py` — live camera+depth preview, center-pixel distance readout.
- `pointcloud.py` — Step 2: depth frame -> colored 3D point cloud (.ply) + headless
  previews (bird's-eye floor plan + angled 3D render). No Open3D needed.

## Step 1 results (2026-07-01, Orin Nano)
- Model: `Depth-Anything-V2-Metric-Indoor-Small-hf` (max_depth 20 m), fp16.
- **12.4 FPS, 81 ms/frame, 0.18 GB VRAM** at 640x480 — big real-time headroom.
- Needs **transformers >= 4.45** for the metric head (4.44 outputs all zeros).
- Depth spatial structure validated on a real desk/room scene (near→far correct).
- Absolute scale is compressed (~0.4–0.8 m indoors) — expected; will be calibrated
  from VESC/GPS odometry in Step 2/3. Relative geometry is what matters for now.
- Camera capture: reuse `../Camera Control/threaded_camera.py` — MUST call
  `ThreadedCamera().start()`; `read()` returns `(frame, seq)`.

## Step 2 results (2026-07-01)
- `pointcloud.py` back-projects metric depth through a pinhole model into a colored
  3D cloud. 76,800 pts (stride 2), writes binary .ply + BEV + angled render.
- Intrinsics are ESTIMATED (fx=fy~554, 60 deg HFOV, cx/cy = center). For accurate
  geometry run `../Camera Control/calibrate_camera.py` (checkerboard) and pass
  --fx/--fy/--cx/--cy.
- Confirmed: absolute depth scale is not just small, it DRIFTS per frame
  (one frame 0.19-0.36 m, another up to 1.3 m). => Step 3 must lock scale to
  VESC/GPS odometry, not trust the model's meters frame-to-frame.

## Step 3 results (2026-07-01) — fusion engine
- `mapper.py record` captures a timestamped frame sequence; `mapper.py build`
  fuses sequence + pose track into ONE voxel map (.ply + BEV + angled render).
- Scale-lock = per-frame median normalization (median depth -> --scale meters).
  VALIDATED: 24 static frames (822k pts) fused to only 6,886 voxels — frames stay
  mutually consistent (drift would smear -> far more voxels). It didn't.
- Pose placement VALIDATED with `--synthetic` drive: map extended z 0.9->5.5 m,
  BEV span 0.6 m -> 9.2 m as virtual cam advanced. Transform math correct.
- CAVEAT: synthetic motion over a STATIC scene only proves the plumbing, not real
  mapping. Real map needs real camera motion + real poses (VESC/GPS/IMU).

## What's left
- Step 3b: feed REAL poses. Wire VESC wheel odometry (distance) + a heading source
  (IMU yaw, or GPS course) into an (x z yaw) pose track; record while driving the
  car; `mapper.py build --poses track.txt`.
- Calibrate intrinsics (`../Camera Control/calibrate_camera.py`) for true geometry.
- Optional: live incremental mapping loop + simple loop-closure for drift.

## LIVE mapping (2026-07-01) — live_map.py
- Poor-man's monocular SLAM: metric visual odometry (ORB matches + depth-backed
  solvePnPRansac) estimates camera motion IN METERS from the images alone, no
  external sensors. Frames scale-locked (median->scale) then fused into VoxelMap.
- Weak-VO frames (few matches / PnP fail) are SKIPPED, not smeared.
- Run:  python3 live_map.py --out mymap   (live windows if DISPLAY, else rolling
  mymap_bev/view/depth.png). Ctrl-C/q saves mymap.ply.
- Offline test: python3 live_map.py --replay scratch_seq  — on the ~static recorded
  seq VO correctly reported ~0 motion (traj 0.03 m/24 frames, map stayed 3.5k
  voxels, 0 smearing). Confirms VO doesn't hallucinate motion.
- VoxelMap.add() vectorized (packed int64 keys + np.unique/bincount) for live speed.
- Move tips: translate through space (not pure rotation), slow/smooth (fixed-focus
  cam motion-blurs), keep textured+lit surfaces. Expect drift on long paths — that's
  what real VESC/GPS odometry (Step 3b) will fix.

## Robustness pass (2026-07-01) — live_map.py gates
Live monocular VO first diverged (bad matches on motion blur -> PnP blow-up).
Added three gates; live-validated:
- MOTION gate (VisualOdometry): reject per-frame |t|>0.5 m or rot>25 deg, or
  inliers<12 / ratio<0.35. Caught real blow-ups live (t=2819 m!) -> skipped,
  tracking self-recovers. THIS is what stops divergence.
- BLUR gate: skip frames with Laplacian-variance < 0.5*recent-median (runs before
  the depth step -> also a speedup). ~10/1300 frames skipped in a dark room.
- KEYFRAME fusion: only add to map after moving >4 cm or >4 deg since last
  keyframe. 11 keyframes instead of ~1300 redundant frames -> much cleaner map.
New knobs: --blur-frac --kf-dist --kf-rot --exposure.

## Environment bottleneck (measured)
Exposure sweep: room too dark to shorten the 31 ms exposure (exposure_time_absolute
=312). This camera has NO gain control (only brightness/gamma), so lowering exposure
-> near-black. So motion -> blur -> frames skipped -> map only grows when near-still.
=> BIGGEST real win now is MORE LIGHT (then --exposure ~50 = 5 ms kills blur), or a
camera with gain/autofocus. Software is now robust; physics is the limit.

## Low-blur fix (2026-07-01) — short exposure + DIGITAL gain
Root cause of blur: icspring sensor is insensitive with NO analog gain, so even a
bright room can't brighten a short exposure (auto-exposure maxes at 31 ms for only
bright~97). Fix = short exposure (less motion blur) + software brightness gain so
depth+ORB still see structure. Measured ORB keypoints (boosted x3): 31ms=631,
12ms=336, 6ms=113 — 12 ms keeps plenty. live_map.py new args: --gain (alpha) and
--gain-beta, applied to each frame before depth/tracking.
BEST RUN so far: `--exposure 120 --gain 3.0` -> continuous motion (NO pausing),
37 keyframes, 1.83 m trajectory, 0->18 blur-skips, L-shaped room ~2.8 m. Recommended
default for this camera. Compare: 31 ms needed move-and-pause, only 1.0 m.

## Calibration TODO (needs the user + checkerboard)
Estimated intrinsics (fx~554) are still in use. To fix geometry: print
`../Camera Control/checkerboard_9x6_25mm.pdf`, run that folder's
`calibrate_camera.py`, hold board at ~10 angles; pass results via --fx/--fy/--cx/--cy.

## Notes
(add findings as you go)
