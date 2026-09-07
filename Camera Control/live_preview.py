#!/usr/bin/env python3
"""
live_preview.py — USB camera live preview with alignment overlay.

Foundation for an autonomous-car perception system on Jetson Orin Nano.

- Detects which /dev/videoN is the USB camera and prints its supported
  resolutions before opening it.
- Shows a live preview with an alignment guide: center crosshair, a horizontal
  "horizon" reference line, and a faint 3x3 grid, so the camera's framing/tilt
  can be eyeballed and reproduced when mounting on a car.
- Shows live FPS in a corner.
- 'q' to quit, 's' to save the current annotated frame as a JPG.

Uses the system OpenCV (JetPack). No pip installs.
"""

import glob
import os
import re
import subprocess
import sys
import time
from collections import deque

# --------------------------------------------------------------------------- #
# Camera tuning
# --------------------------------------------------------------------------- #
# This UVC webcam defaults to auto-exposure (Aperture Priority), which lengthens
# exposure time in low light and silently drops FPS (e.g. 15 -> 9). For a
# perception system we want a STABLE, high frame rate, so we pin manual exposure.
#
# FPS is capped by the camera/USB (~22-23 fps for uncompressed YUYV at 640x480),
# and the real frame period (~44 ms) is far longer than the exposure time, so
# exposure can go fairly high before it limits FPS. EXPOSURE is in units of
# 100 us; 312 ~= 31 ms gives a usable indoor-lit image at no FPS cost. Lower it
# (e.g. 156) to cut motion blur on a moving car; raise it for a brighter image.
USE_MANUAL_EXPOSURE = True
EXPOSURE = 312          # exposure_time_absolute, units of 100us (valid 1..5000)

try:
    import cv2
except ImportError:
    sys.exit(
        "ERROR: OpenCV (cv2) is not importable. On JetPack, install the "
        "ARM/JetPack build of OpenCV — do NOT 'pip install opencv-python'.\n"
        "Stopping so you can install the correct version."
    )


# --------------------------------------------------------------------------- #
# Camera detection
# --------------------------------------------------------------------------- #
def _run_v4l2(args):
    """Run v4l2-ctl and return stdout, or None if the tool is unavailable."""
    try:
        out = subprocess.run(
            ["v4l2-ctl", *args],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


def parse_resolutions(list_formats_ext):
    """Pull a sorted, de-duplicated set of WxH from --list-formats-ext output."""
    res = set()
    for w, h in re.findall(r"Size:\s+Discrete\s+(\d+)x(\d+)", list_formats_ext):
        res.add((int(w), int(h)))
    return sorted(res, key=lambda wh: wh[0] * wh[1])


def probe_device(dev):
    """Return a dict describing a /dev/videoN node, or None if not a camera."""
    info = _run_v4l2(["-d", dev, "--info"])
    if info is None:
        return None  # v4l2-ctl missing; caller falls back

    card = re.search(r"Card type\s*:\s*(.+)", info)
    driver = re.search(r"Driver name\s*:\s*(.+)", info)
    bus = re.search(r"Bus info\s*:\s*(.+)", info)

    fmts = _run_v4l2(["-d", dev, "--list-formats-ext"]) or ""
    resolutions = parse_resolutions(fmts)
    pixel_formats = sorted(set(re.findall(r"\]:\s*'(\w+)'", fmts)))

    return {
        "dev": dev,
        "card": card.group(1).strip() if card else "?",
        "driver": driver.group(1).strip() if driver else "?",
        "bus": bus.group(1).strip() if bus else "?",
        "resolutions": resolutions,
        "pixel_formats": pixel_formats,
        # A real capture node exposes at least one capture format/resolution.
        "is_capture": bool(resolutions or pixel_formats),
        "is_usb": bool(bus and "usb" in bus.group(1).lower()) if bus else False,
    }


def detect_usb_camera():
    """Find the USB camera node and print what we discovered.

    Returns (device_path, info_dict_or_None).
    """
    nodes = sorted(glob.glob("/dev/video*"),
                   key=lambda p: int(re.sub(r"\D", "", p) or 0))
    if not nodes:
        sys.exit("ERROR: No /dev/video* devices found. Is the USB camera plugged in?")

    print("Scanning video devices...\n")
    candidates = []
    for dev in nodes:
        info = probe_device(dev)
        if info is None:
            # No v4l2-ctl available — can't introspect; treat first node as fallback.
            print(f"  {dev}: (v4l2-ctl unavailable, cannot introspect)")
            continue
        tags = []
        if info["is_usb"]:
            tags.append("USB")
        if info["is_capture"]:
            tags.append("capture")
        print(f"  {dev}: {info['card']}  [{info['driver']}]  {info['bus']}"
              + (f"  <{', '.join(tags)}>" if tags else ""))
        if info["is_capture"]:
            candidates.append(info)

    # Prefer a USB capture node; otherwise any capture node.
    usb_caps = [c for c in candidates if c["is_usb"]]
    chosen = (usb_caps or candidates or [None])[0]

    if chosen is None:
        # v4l2-ctl unavailable or nothing introspectable: fall back to first node.
        fallback = nodes[0]
        print(f"\nCould not introspect devices; falling back to {fallback}")
        return fallback, None

    print(f"\nSelected USB camera: {chosen['dev']}  ({chosen['card']})")
    if chosen["pixel_formats"]:
        print(f"  Pixel formats : {', '.join(chosen['pixel_formats'])}")
    if chosen["resolutions"]:
        pretty = ", ".join(f"{w}x{h}" for w, h in chosen["resolutions"])
        print(f"  Resolutions   : {pretty}")
    else:
        print("  Resolutions   : (none reported)")
    print()
    return chosen["dev"], chosen


def apply_manual_exposure(dev, exposure):
    """Pin manual exposure so FPS doesn't sag in low light.

    Uses v4l2-ctl (OpenCV's V4L2 exposure mapping is unreliable on UVC).
    Best-effort: warns but does not fail if controls are unavailable.
    """
    if _run_v4l2(["-d", dev, "-c", "auto_exposure=1"]) is None:
        print("NOTE: v4l2-ctl unavailable; leaving camera auto-exposure as-is.")
        return
    _run_v4l2(["-d", dev, "-c", f"exposure_time_absolute={exposure}"])
    read = _run_v4l2(["-d", dev, "--get-ctrl",
                      "auto_exposure,exposure_time_absolute"]) or ""
    mode = "manual" if "auto_exposure: 1" in read else "auto(?)"
    got = re.search(r"exposure_time_absolute:\s*(\d+)", read)
    print(f"Exposure: {mode}, exposure_time_absolute="
          f"{got.group(1) if got else '?'} (FPS now stable in low light)")


# --------------------------------------------------------------------------- #
# Overlay
# --------------------------------------------------------------------------- #
def draw_overlay(frame, fps):
    """Draw alignment guide + FPS onto frame in place."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    faint = (120, 120, 120)   # 3x3 grid
    accent = (0, 255, 0)      # crosshair
    horizon = (0, 200, 255)   # horizon line (amber)

    # Faint 3x3 grid (thirds).
    for i in (1, 2):
        x = w * i // 3
        y = h * i // 3
        cv2.line(frame, (x, 0), (x, h), faint, 1, cv2.LINE_AA)
        cv2.line(frame, (0, y), (w, y), faint, 1, cv2.LINE_AA)

    # Horizon reference line across the middle.
    cv2.line(frame, (0, cy), (w, cy), horizon, 1, cv2.LINE_AA)

    # Center crosshair with a small gap at the center.
    gap, arm = 6, 22
    cv2.line(frame, (cx - arm, cy), (cx - gap, cy), accent, 1, cv2.LINE_AA)
    cv2.line(frame, (cx + gap, cy), (cx + arm, cy), accent, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - arm), (cx, cy - gap), accent, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy + gap), (cx, cy + arm), accent, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 2, accent, -1, cv2.LINE_AA)

    # FPS in top-left corner (shadow for readability over any background).
    label = f"FPS: {fps:5.1f}"
    org = (12, 28)
    cv2.putText(frame, label, (org[0] + 1, org[1] + 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, label, org,
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)

    # Hint in bottom-left corner.
    hint = "q: quit   s: save"
    cv2.putText(frame, hint, (13, h - 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, hint, (12, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    dev, info = detect_usb_camera()

    # Open with the V4L2 backend explicitly (avoids GStreamer guesswork).
    index = int(re.sub(r"\D", "", dev) or 0)
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(
            f"ERROR: Could not open {dev} with the V4L2 backend.\n"
            "If OpenCV was built without V4L2 support, you likely have a "
            "non-JetPack build — install the correct ARM/JetPack OpenCV."
        )

    # Request a sensible resolution: largest supported up to 1280x720, else default.
    if info and info["resolutions"]:
        target = max((r for r in info["resolutions"] if r[0] <= 1280 and r[1] <= 720),
                     default=info["resolutions"][-1])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, target[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target[1])

    # Pin manual exposure so FPS stays stable as ambient light changes.
    if USE_MANUAL_EXPOSURE:
        apply_manual_exposure(dev, EXPOSURE)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Streaming {dev} at {actual_w}x{actual_h}.  "
          "Press 'q' to quit, 's' to save a frame.")

    save_dir = os.path.dirname(os.path.abspath(__file__))
    win = "Live Preview - Alignment"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    # Rolling FPS over the last ~30 frame intervals.
    times = deque(maxlen=30)
    saved = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("WARNING: dropped frame (camera read failed). Retrying...")
                if cv2.waitKey(50) & 0xFF == ord("q"):
                    break
                continue

            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

            draw_overlay(frame, fps)
            cv2.imshow(win, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(save_dir, f"frame_{ts}_{saved:03d}.jpg")
                if cv2.imwrite(path, frame):
                    saved += 1
                    print(f"Saved {path}")
                else:
                    print(f"ERROR: failed to write {path}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"Done. Saved {saved} frame(s).")


if __name__ == "__main__":
    main()
