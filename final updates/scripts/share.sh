#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════
# RoboCar — put the RUNNING cockpit behind a temporary public HTTPS URL, so
# you can send someone a link without them joining your network.
#
#   ROBOCAR_TOKEN=... ./run_cockpit.sh      (terminal 1, on the Jetson)
#   ./scripts/share.sh                       (terminal 2, same machine)
#
# This script REFUSES to run if the cockpit will accept commands from an
# unauthenticated caller. That is not caution for its own sake: the thing on
# the other end of this URL is a motor, and the URL is on the public internet
# where it will be found by scanners within minutes.
#
# It verifies that EMPIRICALLY — it sends a command and checks it is refused —
# rather than trusting that the environment variable made it into the right
# process. An env var set in this shell says nothing about the process that
# was started in the other one.
# ══════════════════════════════════════════════════════════════════════════
set -u

PORT="${PORT:-8080}"
LOCAL="http://localhost:$PORT"

command -v cloudflared >/dev/null 2>&1 || {
  cat <<'MSG'
cloudflared is not installed. On the Jetson (arm64):

  curl -fsSLo /tmp/cloudflared.deb \
    https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
  sudo dpkg -i /tmp/cloudflared.deb

No Cloudflare account is needed for a temporary link.
MSG
  exit 1
}

# ── is the cockpit even up? ───────────────────────────────────────────────
if ! curl -sf -m 4 "$LOCAL/state" >/dev/null; then
  echo "nothing is answering on $LOCAL"
  echo "start it first:   ROBOCAR_TOKEN=<something-long> ./run_cockpit.sh"
  exit 1
fi

# ── would an anonymous caller be able to drive it? ────────────────────────
# A harmless probe: no recognised command keys, so a 200 here changes nothing
# on the car — it only tells us the door is unlocked.
CODE="$(curl -s -o /dev/null -m 4 -w '%{http_code}' -X POST "$LOCAL/?ping=1")"
if [ "$CODE" != "403" ]; then
  cat <<MSG

REFUSING to publish this cockpit.

An unauthenticated POST to $LOCAL returned $CODE, not 403. That means anyone
who opens the public link could arm the motor and drive the car.

Stop the cockpit and start it with a token:

  ROBOCAR_TOKEN=\$(head -c 18 /dev/urandom | base64 | tr -d '/+=') ./run_cockpit.sh

Then run this script again. Keep the token; you will need it to drive.

MSG
  exit 1
fi

if [ -z "${ROBOCAR_TOKEN:-}" ]; then
  echo "note: ROBOCAR_TOKEN is not set in THIS shell, so the driving link"
  echo "      below cannot be printed. The read-only link still works."
  echo
fi

cat <<MSG
The cockpit is token-protected. Opening a temporary public URL ...

  send people:   <url>/?nocam=1        watch only, no camera video
  keep for you:  <url>/?k=${ROBOCAR_TOKEN:-<your-token>}   full control

Two things worth knowing before you send it:

  · Anyone with the link can WATCH — telemetry, the LiDAR scene, position.
    They cannot command the car. That asymmetry is the whole point.
  · Without ?nocam=1 the link is also a live view of whatever room the car
    is standing in. Cloudflare's temporary tunnels cap at 200 in-flight
    requests and do not handle long-lived streams well; two camera feeds are
    exactly that. Send the ?nocam=1 form.

The URL dies when you Ctrl-C this. It is different every time.

MSG

exec cloudflared tunnel --url "$LOCAL"
