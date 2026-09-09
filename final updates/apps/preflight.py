#!/usr/bin/env python3
"""
apps/preflight.py — say what the car will and will not be able to do, BEFORE
cockpit.py opens a single serial port.

Why this exists: cockpit.py is deliberately forgiving. Every device it brings
up is wrapped in try/except so that a missing LiDAR or a flat motor battery
degrades the session instead of ending it. That is right for a control
program and useless for a person standing next to the car wondering why it
will not drive. This script is the other half: it is deliberately noisy, it
opens nothing, and it names the fix for each thing it finds.

Exit status: 0 if the car can drive, 1 if something in the drive path is
missing. Advisory findings never fail the run — GNSS is missing indoors by
definition, and that is not an error.

  python3 preflight.py            report and exit
  python3 preflight.py --quiet    only problems
"""
import os
import shutil
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

PORT = 8080
QUIET = "--quiet" in sys.argv

# ── output ────────────────────────────────────────────────────────────────
BOLD, DIM, RED, YEL, GRN, OFF = "\033[1m", "\033[2m", "\033[31m", "\033[33m", "\033[32m", "\033[0m"
if not sys.stdout.isatty():
    BOLD = DIM = RED = YEL = GRN = OFF = ""

_rows = []


def row(name, level, detail, fix=None):
    """level: ok | warn | fail. warn never fails the run."""
    _rows.append((name, level, detail, fix))


def _emit():
    mark = {"ok": GRN + "  ok  " + OFF, "warn": YEL + " warn " + OFF, "fail": RED + " FAIL " + OFF}
    for name, level, detail, fix in _rows:
        if QUIET and level == "ok":
            continue
        print(f"[{mark[level]}] {name:<22} {detail}")
        if fix and level != "ok":
            for line in fix.strip().split("\n"):
                print(f"          {DIM}{line}{OFF}")


# ── checks ────────────────────────────────────────────────────────────────

def check_imports():
    """The heavy dependencies. A missing cv2 is a five-second answer here and a
    confusing traceback thirty seconds into cockpit start-up."""
    for mod, why, fix in [
        ("numpy", "arrays", "pip3 install numpy"),
        ("cv2", "camera + map rendering", "pip3 install opencv-python"),
        ("serial", "every serial device", "pip3 install pyserial"),
        ("scipy", "SLAM / ICP", "pip3 install scipy"),
    ]:
        try:
            __import__(mod)
            row(f"python: {mod}", "ok", why)
        except ImportError as e:
            row(f"python: {mod}", "fail", f"missing — {why}", f"{fix}\n({e})")


def _dev(path, name, why, fix):
    if not os.path.exists(path):
        row(name, "fail", f"not present — {why}", fix)
        return False
    real = os.path.realpath(path)
    if not os.access(real, os.R_OK | os.W_OK):
        row(name, "fail", f"{real} present but not writable",
            "you are probably not in the dialout group:\n"
            "  sudo usermod -aG dialout $USER\n"
            "then log out and back in (a new SSH session is enough)")
        return False
    row(name, "ok", real if real != path else path)
    return True


def check_serial():
    import config
    ok = True
    ok &= _dev(config.vesc_port(), "VESC (throttle)", "no drive, no telemetry",
               "motor battery off, or USB unplugged.\n"
               "check:  ls -l /dev/serial/by-id/")
    ok &= _dev(config.steering_port(), "steering (Nano)", "steering commands go nowhere",
               "check:  ls -l /dev/serial/by-id/")
    ok &= _dev(config.lidar_port(), "RPLIDAR C1", "no obstacle stopping, no SLAM",
               "check:  ls -l /dev/serial/by-id/")
    return ok


def check_gps():
    """Advisory only. Indoors there is no fix, and that is not a fault."""
    import config
    p = config.GPS_PORT
    if not os.path.exists(p):
        row("GNSS (SE100)", "warn", f"{p} not present — POSITION screen will show ABSENT")
        return
    # The classic Jetson trap: the serial-console getty owns ttyTHS1, so the
    # GPS opens and then reads nothing at all.
    holder = ""
    try:
        out = subprocess.run(["systemctl", "is-active", "nvgetty"],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        if out == "active":
            holder = "nvgetty"
    except Exception:                                        # noqa: BLE001
        pass
    if holder:
        row("GNSS (SE100)", "warn", f"{p} is held by {holder} (serial console)",
            "the GPS will open the port and read nothing. To free it:\n"
            "  sudo systemctl stop nvgetty && sudo systemctl disable nvgetty")
    elif not os.access(p, os.R_OK | os.W_OK):
        row("GNSS (SE100)", "warn", f"{p} not readable",
            "sudo usermod -aG dialout $USER   (then re-login)")
    else:
        row("GNSS (SE100)", "ok", p)


def check_cameras():
    import config
    for label, path in (("front", config.CAM_FRONT_BYPATH), ("rear", config.CAM_REAR_BYPATH)):
        if os.path.exists(path):
            row(f"camera {label}", "ok", os.path.realpath(path))
        else:
            row(f"camera {label}", "warn", "not at its USB port path — check the plug order",
                "cameras are pinned by physical port (front 2.1, rear 2.3), never /dev/videoN.\n"
                "check:  ls -l /dev/v4l/by-path/")


def check_web():
    d = os.path.join(HERE, "web")
    need = ["index.html", "app.css", "core.js", "widgets.js", "radar.js", "vision.js", "position.js"]
    missing = [f for f in need if not os.path.isfile(os.path.join(d, f))]
    if missing:
        row("front end", "fail", "web/ incomplete: " + ", ".join(missing),
            "git pull, or the working tree is partial")
    else:
        n = len(os.listdir(os.path.join(d, "fonts"))) if os.path.isdir(os.path.join(d, "fonts")) else 0
        row("front end", "ok", f"web/ complete · {n} font files")


def check_port():
    s = socket.socket()
    try:
        s.bind(("0.0.0.0", PORT))
        row(f"port {PORT}", "ok", "free")
        return True
    except OSError:
        who = ""
        if shutil.which("lsof"):
            try:
                who = subprocess.run(["lsof", "-ti", f"tcp:{PORT}"],
                                     capture_output=True, text=True, timeout=4).stdout.strip()
            except Exception:                                # noqa: BLE001
                pass
        row(f"port {PORT}", "fail", "already in use" + (f" by pid {who}" if who else ""),
            "run_cockpit.sh normally clears this. By hand:\n"
            f"  pkill -f cockpit.py")
        return False
    finally:
        s.close()


def check_disk():
    """Recording writes continuously. Finding out it stopped is a lost run."""
    try:
        st = os.statvfs(ROOT)
        free = st.f_bavail * st.f_frsize / 1e9
    except Exception:                                        # noqa: BLE001
        return
    if free < 1.0:
        row("disk", "fail", f"{free:.1f} GB free — recording will fail",
            "clear space under logs/ before driving")
    elif free < 5.0:
        row("disk", "warn", f"{free:.1f} GB free — a long session may fill this")
    else:
        row("disk", "ok", f"{free:.0f} GB free")


def check_git():
    try:
        sha = subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                      stderr=subprocess.DEVNULL, timeout=4).decode().strip()
        dirty = subprocess.check_output(["git", "-C", ROOT, "status", "--porcelain"],
                                        stderr=subprocess.DEVNULL, timeout=4).decode().strip()
        branch = subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--abbrev-ref", "HEAD"],
                                         stderr=subprocess.DEVNULL, timeout=4).decode().strip()
        row("git", "warn" if dirty else "ok",
            f"{branch} @ {sha}" + (" · TREE DIRTY" if dirty else " · clean"),
            "uncommitted changes on the car — the session log will say TREE DIRTY,\n"
            "which is correct but means the run is not reproducible from a commit."
            if dirty else None)
    except Exception:                                        # noqa: BLE001
        row("git", "warn", "not a git checkout — the cockpit cannot stamp runs with a commit")


def addresses():
    """Every way to reach this machine. The hostname is the one worth learning:
    it does not change when DHCP hands out a different lease."""
    host = socket.gethostname().split(".")[0]
    ips = []
    for fam, kind in ((socket.AF_INET, "8.8.8.8"),):
        s = socket.socket(fam, socket.SOCK_DGRAM)
        try:
            s.connect((kind, 80))
            ips.append(s.getsockname()[0])
        except Exception:                                    # noqa: BLE001
            pass
        finally:
            s.close()
    return host, ips


def main():
    print(f"\n{BOLD}RoboCar preflight{OFF}  {DIM}(opens nothing; reports only){OFF}\n")
    check_imports()
    drive_ok = check_serial()
    check_gps()
    check_cameras()
    check_web()
    port_ok = check_port()
    check_disk()
    check_git()
    _emit()

    host, ips = addresses()
    print(f"\n{BOLD}reach the cockpit at{OFF}")
    print(f"  http://localhost:{PORT}          {DIM}(on the Jetson itself){OFF}")
    for ip in ips:
        print(f"  http://{ip}:{PORT}")
    print(f"  http://{host}.local:{PORT}       {DIM}(same address every time — "
          f"needs avahi-daemon running){OFF}")

    fails = [r for r in _rows if r[1] == "fail"]
    warns = [r for r in _rows if r[1] == "warn"]
    print()
    if fails:
        print(f"{RED}{BOLD}{len(fails)} blocking{OFF} · {len(warns)} advisory — "
              f"the cockpit will still start, but see above.\n")
        return 1
    print(f"{GRN}ready{OFF}" + (f" · {len(warns)} advisory" if warns else "") + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
