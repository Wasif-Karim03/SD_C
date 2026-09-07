#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# drive.sh — one-command launcher for the camera-autonomy stack (autonav.py).
#
#   * Forces DISPLAY=:0 (the desktop shown over NoMachine — where VS Code lives).
#     autonav's window otherwise opens on whatever display the shell inherited,
#     which may not be the one you're looking at.
#   * Starts autonav.py in the foreground (you get live telemetry + Ctrl-C).
#   * A background helper waits for the "AutoNav" window to appear, then moves
#     it to the front so it can't hide behind a maximized VS Code.
#
# Any extra args are passed straight through to autonav.py, e.g.:
#   drive.sh --dry                 # perception only, no motors
#   drive.sh --max-duty 0.05       # gentler throttle cap
#
# Keys once the window is up:  a = arm throttle   SPACE = e-stop   q = quit
# TEST ON A STAND (wheels up) before the floor.
# ---------------------------------------------------------------------------
set -uo pipefail

CAM_DIR="/home/wasif/Documents/Self Driving Car/Camera Control"

# The desktop you view over NoMachine is :0. Force it so the window lands there.
export DISPLAY=:0

# Background helper: pull the camera window in front of VS Code once it exists.
# The metric-depth model takes ~20-30 s to load before the window appears, so
# poll for up to 60 s, then raise it once and exit.
(
  for _ in $(seq 1 60); do
    wid=$(xdotool search --name "AutoNav" 2>/dev/null | head -1)
    if [ -n "$wid" ]; then
      xdotool windowmove   "$wid" 300 200 2>/dev/null
      xdotool windowactivate "$wid" 2>/dev/null
      xdotool windowraise  "$wid" 2>/dev/null
      break
    fi
    sleep 1
  done
) &

cd "$CAM_DIR" || exit 1
echo "Starting camera autonomy (autonav.py) on DISPLAY=:0 ..."
echo "Window will auto-raise in front of VS Code once the depth model loads (~25 s)."
echo "Keys:  a = arm   SPACE = e-stop   q = quit    (wheels up on a stand first!)"
exec python3 autonav.py "$@"
