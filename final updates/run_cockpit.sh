#!/usr/bin/env bash
# RoboCar — launch Mission Control (cockpit.py).
# Kills any other app that would fight for port 8080 / the hardware, then starts.
set -u
APPDIR="/home/wasif/Documents/Self Driving Car/final updates/apps"
cd "$APPDIR" || { echo "cannot cd to $APPDIR"; exit 1; }

echo "stopping any app already using the hardware / port 8080 ..."
pkill -f navigate_web.py   2>/dev/null
pkill -f mapper_web.py     2>/dev/null
pkill -f control_center.py 2>/dev/null
pkill -f cockpit.py        2>/dev/null
sleep 1

echo "starting RoboCar Mission Control ..."
echo "open a browser to:  http://localhost:8080   (or http://<jetson-ip>:8080)"
echo "press Ctrl-C here to stop."
exec python3 cockpit.py
