#!/usr/bin/env python3
"""Live GPS dashboard for the Radiolink SE100 on the Jetson header UART.

Refreshes a single in-place panel showing link status, fix state, satellites
in view / used per constellation, signal strength and (once locked) position.

Usage: python3 gps_monitor.py [port] [baud]
Defaults: /dev/ttyTHS1 @ 38400
Ctrl-C to stop.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 38400

# GSV talker id -> constellation name
CONSTEL = {
    "GP": "GPS",
    "GL": "GLONASS",
    "GA": "Galileo",
    "GB": "BeiDou",
    "GQ": "QZSS",
    "GN": "Mixed",
}
FIX_QUALITY = {
    "0": "no fix", "1": "GPS fix", "2": "DGPS fix",
    "4": "RTK fixed", "5": "RTK float", "6": "estimated",
}


def dm_to_deg(val, hemi):
    if not val:
        return None
    dot = val.find(".")
    deg = float(val[:dot - 2])
    minutes = float(val[dot - 2:])
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def checksum_ok(line):
    if "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def snr_bar(snr):
    """Tiny text bar for 0-50 dB-Hz SNR."""
    if snr is None:
        return "    -"
    n = min(10, max(0, int(snr / 5)))
    return ("#" * n).ljust(10)


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {PORT} @ {BAUD}: {e}")
        sys.exit(1)

    # state, rebuilt continuously
    state = {
        "fix": "0", "status": "V", "lat": None, "lon": None,
        "alt": "", "used": "0", "hdop": "", "utc": "",
    }
    # per-constellation list of (prn, snr); GSV may span multiple sentences
    sats = {}          # committed view, keyed by constellation
    sats_acc = {}      # accumulating current GSV burst
    last_rx = 0.0
    sentences = 0
    started = time.time()

    def render():
        # total / strong satellite counts across constellations
        all_sats = [s for lst in sats.values() for s in lst]
        in_view = len(all_sats)
        strong = sum(1 for _, snr in all_sats if snr is not None and snr >= 25)
        snrs = [snr for _, snr in all_sats if snr is not None]
        best = max(snrs) if snrs else None
        avg = (sum(snrs) / len(snrs)) if snrs else None

        link_age = time.time() - last_rx
        if last_rx == 0:
            link = "WAITING (no bytes yet)"
        elif link_age < 3:
            link = "OK  (streaming)"
        else:
            link = f"STALE ({link_age:.0f}s since last data)"

        fixq = FIX_QUALITY.get(state["fix"], state["fix"])
        locked = state["status"] == "A" and state["fix"] not in ("", "0")

        out = []
        out.append("=" * 52)
        out.append(f" GPS MONITOR  {PORT} @ {BAUD}   up {time.time()-started:5.0f}s")
        out.append("=" * 52)
        out.append(f" Link    : {link}")
        out.append(f" Sentences read : {sentences}")
        out.append("-" * 52)
        flag = "LOCKED" if locked else "searching..."
        out.append(f" FIX     : {fixq:<10} [{flag}]   UTC {state['utc'] or '--'}")
        out.append(f" Sats in view  : {in_view:<3}   used in fix: {state['used']}")
        out.append(f" Strong(>=25)  : {strong:<3}   HDOP: {state['hdop'] or '--'}")
        if best is not None:
            out.append(f" Signal  : best {best:.0f}  avg {avg:.0f} dB-Hz")
        else:
            out.append(" Signal  : -- (no satellites detected)")
        if locked and state["lat"] is not None:
            out.append(f" POSITION: lat {state['lat']:.6f}  lon {state['lon']:.6f}")
            out.append(f"           alt {state['alt'] or '?'} m")
        else:
            need = max(0, 4 - strong)
            out.append(f" POSITION: --   (need ~{need} more strong sats)")
        out.append("-" * 52)
        out.append(" Per constellation (PRN:SNR):")
        if not sats:
            out.append("   (none yet)")
        for name in ("GPS", "GLONASS", "Galileo", "BeiDou", "QZSS"):
            lst = sats.get(name)
            if not lst:
                continue
            parts = []
            for prn, snr in lst[:8]:
                parts.append(f"{prn}:{snr if snr is not None else '-'}")
            out.append(f"   {name:<8} {len(lst):>2}  " + " ".join(parts))
        out.append("-" * 52)
        out.append(" strongest:")
        top = sorted(all_sats, key=lambda x: (x[1] is None, -(x[1] or 0)))[:4]
        if top:
            for prn, snr in top:
                out.append(f"   {prn:<4} {snr_bar(snr)} {snr if snr is not None else '-'} dB")
        else:
            out.append("   --")
        out.append("=" * 52)
        out.append(" Ctrl-C to stop")

        # clear screen + home, then paint
        sys.stdout.write("\033[2J\033[H" + "\n".join(out) + "\n")
        sys.stdout.flush()

    last_paint = 0.0
    try:
        while True:
            raw = ser.readline().decode(errors="replace").strip()
            now = time.time()
            if raw.startswith("$") and checksum_ok(raw):
                last_rx = now
                sentences += 1
                f = raw.split(",")
                typ = f[0][3:6]   # e.g. GGA, RMC, GSV
                talker = f[0][1:3]

                if typ == "GGA" and len(f) >= 10:
                    state["utc"] = f[1][:6]
                    state["fix"] = f[6]
                    state["used"] = f[7]
                    state["hdop"] = f[8]
                    state["alt"] = f[9]
                    lat = dm_to_deg(f[2], f[3])
                    lon = dm_to_deg(f[4], f[5])
                    if lat is not None:
                        state["lat"], state["lon"] = lat, lon
                elif typ == "RMC" and len(f) >= 3:
                    state["status"] = f[2]
                elif typ == "GSV" and len(f) >= 4:
                    name = CONSTEL.get(talker, talker)
                    msg_num = f[2]
                    total_msgs = f[1]
                    if msg_num == "1":
                        sats_acc[name] = []
                    # satellite blocks: prn,elev,azim,snr (4 fields each) from idx 4
                    i = 4
                    while i + 3 < len(f):
                        prn = f[i]
                        snr_s = f[i + 3].split("*")[0]
                        snr = int(snr_s) if snr_s.isdigit() else None
                        if prn:
                            sats_acc.setdefault(name, []).append((prn, snr))
                        i += 4
                    if msg_num == total_msgs:
                        sats[name] = sats_acc.get(name, [])

            # repaint at most ~2x/sec
            if now - last_paint > 0.5:
                render()
                last_paint = now
    except KeyboardInterrupt:
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()
        print("stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
