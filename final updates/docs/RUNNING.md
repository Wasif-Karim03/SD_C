# Running the cockpit on the Jetson

## Every session, two commands

```
ssh wasif@<jetson>
cd "~/Documents/Self Driving Car/final updates" && ./run_cockpit.sh --pull
```

`--pull` fetches whatever was pushed from the Mac, then runs preflight, then
starts. Drop `--pull` when nothing has changed. Ctrl-C stops it and safes the
motor on the way out.

Then open, from any machine on the same network:

```
http://<jetson-hostname>.local:8080
```

The hostname address does not change when DHCP hands out a different lease,
which the IP does. Preflight prints all of them; if `.local` does not resolve,
see **mDNS** below.

## What preflight tells you

`./run_cockpit.sh --check` runs the checks and starts nothing. It opens no
serial ports, so it is safe to run while something else is using the car.

It separates two kinds of finding, and the difference matters:

- **FAIL** — something in the drive path is missing. VESC, steering, LiDAR,
  the front end, the port. The cockpit will still start (it is built to
  degrade, and a screen that says "no LiDAR" beats no screen), but it pauses
  four seconds first so you actually read the report.
- **warn** — advisory. GNSS is missing indoors *by definition*; that is not a
  fault and never blocks a run.

Each finding names its own fix. The three you will hit most:

| Symptom | Cause | Fix |
|---|---|---|
| all three serial devices FAIL | motor battery off, or USB hub unplugged | check `ls -l /dev/serial/by-id/` |
| device present but not writable | not in `dialout` | `sudo usermod -aG dialout $USER`, then re-login |
| GNSS opens but reads nothing | `nvgetty` owns `/dev/ttyTHS1` | `sudo systemctl stop nvgetty && sudo systemctl disable nvgetty` |

## First time on a fresh Jetson

```
cd ~/Documents
git clone <repo> "Self Driving Car"
cd "Self Driving Car/final updates"
./run_cockpit.sh --check
```

Fix whatever it lists, then run it without `--check`.

## mDNS — so the address never changes

Ubuntu on the Jetson normally has this already. To confirm:

```
systemctl is-active avahi-daemon
```

If it is not active:

```
sudo apt install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
hostnamectl                 # the hostname is what goes before .local
```

macOS resolves `.local` natively, so nothing is needed on your Mac.

## Driving somewhere with other people on the network

The cockpit binds `0.0.0.0:8080` with no authentication by default. On a bench
that is right — you want to open it from your laptop without ceremony. On a
university network it means anyone who finds the port can arm the motor of a
vehicle they are not standing next to.

Set a token before you take the car anywhere shared:

```
ROBOCAR_TOKEN=somethinglongandrandom ./run_cockpit.sh
```

then open `http://<host>:8080/?k=somethinglongandrandom`. The page keeps it for
that tab only, and appends it to every command. Telemetry stays readable
without it — someone watching the numbers is harmless, and being able to see
what the car is doing is itself a safety property. What the token stops is
someone else driving it.

Be clear-eyed about what this is: a shared secret over plain HTTP. It is a
lock on a door, not a security system, and it is worth exactly that much. The
real isolation is a network only you are on.

## Reaching it from outside your network

`docs/REMOTE_ACCESS.md` covers both cases: Tailscale for your own devices from
anywhere, and a Cloudflare quick tunnel for a link you can send someone. The
short version: monitor from anywhere, drive on the LAN in the same room. The
deadman is 500 ms and internet jitter will trip it constantly, which is the
safety system working correctly and awful to drive through.

## Working on the interface without the car

```
cd "final updates/apps"
python3 mock_cockpit.py       # http://localhost:8099
```

Serves a simulated run — corridor, doorway, a 30 cm box at 1.24 m, a
controller lagging its plan. Same page, same renderers, no hardware. The
cockpit reads `web/` from disk on every request, so edit and reload; no
restart, and no dropping serial ports on the real car either.

## If something looks wrong on screen

- **Numbers struck through, "NO LINK TO VEHICLE"** — the browser stopped
  getting `/state`. The values shown are the last ones received and are marked
  as such rather than left looking live.
- **The scene is mirrored** — the scanner's positive lateral is the other side
  from what is assumed. Add `?mirror=1` to the URL to confirm, then say so and
  it gets fixed properly in `radar.js` and `vision.js`.
- **STEER shows only a magenta marker** — correct. The servo is open loop;
  there is no encoder, so there is no measured steering angle to draw.
- **WHEELBASE_M / MAX_STEER_ANGLE_RAD marked EST on DIAGNOSE** — also correct.
  Both are still estimates, and both feed the predicted path on VISION. Measure
  them and the white ribbon gets honest.
