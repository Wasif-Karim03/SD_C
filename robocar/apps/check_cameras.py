#!/usr/bin/env python3
"""
check_cameras.py — probe every camera node and capture a frame from each.

Purpose: we now have TWO identical icSpring USB cameras (front + rear). Because
they are the SAME model, /dev/serial-style by-id can't tell them apart, and the
/dev/videoN numbers can swap on replug. This script:

  1. Lists every /dev/video* node.
  2. Shows the STABLE /dev/v4l/by-path/ mapping (physical USB port -> videoN) —
     this is how we pin "front" vs "rear" reliably.
  3. Opens each node (headless, no display needed), grabs a frame, and saves it
     as cam_probe/<node>.jpg so we can SEE what each camera is looking at.
  4. Writes cam_probe/report.txt with everything (also printed to the terminal).

Run on the Jetson (both cameras plugged in):
    cd "/home/wasif/Documents/Self Driving Car/robocar/apps"
    python3 check_cameras.py

Then the captured JPEGs + report.txt appear in ./cam_probe/ .
No X display required. Uses the system OpenCV (cv2) already on the Jetson.
"""
import os
import glob
import subprocess
import datetime

try:
    import cv2
except ImportError:
    raise SystemExit("cv2 not importable — run with the Jetson system python3 "
                     "(the one that has the JetPack OpenCV).")

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cam_probe")
os.makedirs(OUT_DIR, exist_ok=True)

lines = []


def log(msg=""):
    print(msg)
    lines.append(msg)


def run(cmd):
    """Run a shell command, return stdout (or a note if it isn't available)."""
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                             timeout=10)
        return (out.stdout or "") + (out.stderr or "")
    except Exception as exc:  # noqa: BLE001
        return f"(could not run '{cmd}': {exc})"


def by_path_map():
    """Map /dev/videoN -> the stable /dev/v4l/by-path symlink pointing at it."""
    mapping = {}
    bp = "/dev/v4l/by-path"
    if os.path.isdir(bp):
        for name in sorted(os.listdir(bp)):
            link = os.path.join(bp, name)
            try:
                real = os.path.realpath(link)
                mapping.setdefault(real, []).append(name)
            except OSError:
                pass
    return mapping


log("=" * 70)
log(f"camera probe @ {datetime.datetime.now().isoformat(timespec='seconds')}")
log("=" * 70)

log("\n--- /dev/video* nodes ---")
nodes = sorted(glob.glob("/dev/video*"),
               key=lambda p: int("".join(filter(str.isdigit, p)) or -1))
log(", ".join(nodes) if nodes else "  (none found — are the cameras plugged in?)")

log("\n--- v4l2-ctl --list-devices (which nodes belong to which camera) ---")
log(run("v4l2-ctl --list-devices"))

bpm = by_path_map()
log("--- stable physical-port map (/dev/v4l/by-path -> videoN) ---")
if bpm:
    for real, names in sorted(bpm.items()):
        log(f"  {real}  <=  {', '.join(names)}")
else:
    log("  (/dev/v4l/by-path not present)")

log("\n--- opening each node + grabbing a frame ---")
captured = []
for node in nodes:
    idx = int("".join(filter(str.isdigit, node)) or -1)
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        log(f"  {node}: could NOT open (busy, or a metadata-only node)")
        cap.release()
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    frame = None
    for _ in range(8):          # warm up: first frames are often stale/blank
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    if frame is None:
        log(f"  {node}: opened but NO frame ({w}x{h}) — likely a metadata node")
        continue
    # tag the saved file with the physical port so front/rear is obvious
    port = ""
    if bpm.get(os.path.realpath(node)):
        port = "__" + bpm[os.path.realpath(node)][0].replace("/", "_")
    out = os.path.join(OUT_DIR, f"video{idx}{port}.jpg")
    cv2.imwrite(out, frame)
    log(f"  {node}: CAPTURED {w}x{h} -> {os.path.basename(out)}")
    captured.append(out)

log("\n--- summary ---")
log(f"  {len(captured)} camera(s) produced a real image:")
for c in captured:
    log(f"    {c}")
log("\nIf you see TWO images, both cameras work. The filename's by-path tag tells")
log("us which physical USB port each is on — that's how we pin front vs rear.")

report = os.path.join(OUT_DIR, "report.txt")
with open(report, "w") as fh:
    fh.write("\n".join(lines) + "\n")
print(f"\nreport written -> {report}")
