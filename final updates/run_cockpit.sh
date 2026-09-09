#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════
# RoboCar — start Mission Control on the Jetson.
#
#   ./run_cockpit.sh            preflight, then start
#   ./run_cockpit.sh --pull     git pull first, then the above
#   ./run_cockpit.sh --check    preflight only, start nothing
#
# The path is derived from where THIS FILE lives, not hard-coded, so the same
# script works on the Jetson, on a laptop, and out of a fresh clone in any
# directory. Ctrl-C stops the cockpit and safes the motor on the way out.
# ══════════════════════════════════════════════════════════════════════════
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="$HERE/$(basename "${BASH_SOURCE[0]}")"   # absolute: we cd below
APPDIR="$HERE/apps"
cd "$APPDIR" || { echo "cannot cd to $APPDIR"; exit 1; }

PULL=0
CHECK_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --pull)  PULL=1 ;;
    --check) CHECK_ONLY=1 ;;
    -h|--help) sed -n '3,11p' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg   (try --help)"; exit 2 ;;
  esac
done

if [ "$PULL" = "1" ]; then
  echo "updating from git ..."
  if ! git -C "$HERE/.." pull --ff-only; then
    echo
    echo "git pull failed. If you have local edits on the car, either commit them"
    echo "or stash them:   git -C \"$HERE/..\" stash"
    exit 1
  fi
  echo
fi

# Nothing else may hold the serial ports or the port. These are the other
# entry points in this repo that open the same hardware; leaving one running
# is the usual reason the cockpit comes up with no LiDAR.
echo "stopping anything already holding the hardware or port 8080 ..."
for p in navigate_web.py mapper_web.py control_center.py cockpit.py dashboard.py mock_cockpit.py; do
  pkill -f "$p" 2>/dev/null
done
sleep 1

python3 preflight.py
STATUS=$?

if [ "$CHECK_ONLY" = "1" ]; then
  exit $STATUS
fi

if [ "$STATUS" != "0" ]; then
  # Start anyway. The cockpit is built to degrade — a missing LiDAR gives you
  # a screen that says so, which is more useful than no screen at all. But
  # pause long enough that the report above is actually read.
  echo "starting in DEGRADED mode in 4 s — Ctrl-C now to fix the above first."
  sleep 4
  echo
fi

echo "starting RoboCar Mission Control ...   (Ctrl-C to stop)"
echo
exec python3 cockpit.py
