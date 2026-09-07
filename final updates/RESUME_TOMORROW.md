# RoboCar — resume notes (as of 2026-08-09)

Self-driving RC car: Traxxas Slash 4x4 + Jetson Orin Nano (8 GB). Hardware is fixed
and already wired/soldered — we only change code. Everything lives under
`final updates/`. Config is the single source of truth: `config.py`.

## The ONE app to run now: apps/cockpit.py  (Mission Control)
Replaces mapper_web / navigate_web / control_center — it owns the hardware, so run
only one at a time.

    pkill -f navigate_web.py ; pkill -f mapper_web.py ; pkill -f control_center.py ; pkill -f cockpit.py
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 cockpit.py
    # browser: http://<jetson-ip>:8080   (hard-reload Ctrl-Shift-R)

Design: "Tactical Mission Control" (dark charcoal + cyan HUD, monospace).
Memory-safe by design: only ONE heavy pipeline runs at a time (the old cockpit
OOM-crashed running everything at once). Four modes share a light always-on core
(LiDAR + telemetry + steering + E-STOP):
  - DRIVE       manual drive + top-down LiDAR radar + pseudo-3D LiDAR view
  - MAP         2D LiDAR SLAM builds the room live; SAVE MAP -> maps/room.npy/.png
  - NAVIGATE    load saved map, localize, click a goal -> A* route -> GO auto-drives
                it (pure-pursuit) with live LiDAR obstacle stopping
  - PERCEPTION  YOLO object detection on front (+ optional rear) camera; the
                detector loads only in this mode and is freed on exit

Controls (made user-friendly for driving while mapping):
  - Keyboard anywhere: W/S = fwd/rev, A/D = steer (springs back to center),
    SPACE = E-STOP. First drive input AUTO-ENABLES driving (auto-arm).
  - On-screen drive pad (▲▼◄►) with live THR/STEER readout; steering trim slider.
  - Every panel has a ⤢ button to expand it fullscreen.

## What works / verified
- Click-to-goal on the map now plans routes reliably. Root causes fixed:
  (1) planner blocked mask now = walls only (obstacle_mask), so A* may route
      through UNKNOWN space; requiring observed-free fragmented the graph.
  (2) the map click handler had been named `click` (collided with the element's
      built-in .click()); now bound via addEventListener.
- navigate_web.py "GO" and cockpit NAVIGATE both auto-drive the planned route
  (pure-pursuit + LiDAR safety). Verified in code + headless browser; NOT yet
  verified on the physical floor.
- cockpit.py front-end fully verified in a real headless Chromium: zero JS errors,
  all controls fire, telemetry renders, keyboard drive + spring-back steer + auto-arm
  + deadman throttle all confirmed.

## PENDING (do next / tomorrow)
1. FIRST DRIVE = WHEELS-UP ON A STAND. Confirm two things that can't be known till
   the wheels move:
     - steering direction: if it steers the WRONG way, flip FOLLOW_STEER_SIGN in
       BOTH apps (cockpit.py and navigate_web.py): -1.0 <-> +1.0.
     - the pure-pursuit heading reference (FOLLOW_FORWARD_DEG = config.LIDAR_FORWARD_DEG=340).
   Then a gentle floor test.
2. Layer camera VISION into auto-drive decisions (person/obstacle stop) — currently
   NAVIGATE drives on LiDAR + map only. PERCEPTION mode already runs YOLO; next is
   feeding it into the follow loop (kept out so far to protect the 8 GB budget).
3. Tesla-style 3D: fuse camera DEPTH (Depth-Anything) into the 3D view (right now the
   3D view is a pseudo-3D projection of the 2D LiDAR only).
4. Multi-waypoint routing (place -> place -> place).
5. Deferred calibration: odometry tape recheck (meters_per_tach), measure wheelbase_m
   + max_steer_angle_rad, camera intrinsics, compass 360 cal.

## Key tuning knobs (top of cockpit.py / navigate_web.py)
  FOLLOW_DUTY / DUTY_INDOOR=0.07 / DUTY_OUTDOOR=0.09, LOOKAHEAD_M=0.55,
  GOAL_TOL_M=0.25, STEER_GAIN=1.8, FOLLOW_STEER_SIGN=-1.0, REACT_M=1.3,
  BLOCK_GIVEUP_S=6.0. MAX_DUTY=0.20, MIN_MOVE_DUTY=0.06 (in config.py).
