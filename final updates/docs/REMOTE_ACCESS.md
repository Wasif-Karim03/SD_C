# Reaching the cockpit from outside your network

Two different problems, two different tools. Do not use the second one to
solve the first.

| I want to... | Use | Who can reach it |
|---|---|---|
| open my own cockpit from campus, home, or my phone | **Tailscale** | only your own devices |
| send someone a link so they can watch | **Cloudflare quick tunnel** | anyone with the link |

## Read this before either one

**Do not drive the car over the internet.** Not because the transport is
unsafe — because the deadman is 500 ms. Add internet latency and jitter to a
500 ms timeout and the car stutters and stops constantly. That is the safety
system working exactly as designed, and it makes for miserable driving.

More importantly you would be commanding a physical vehicle you cannot see.
Monitor from anywhere. Drive on the LAN, in the same room, close enough to
pick the car up.

---

## Tailscale — your own devices, from anywhere

A private encrypted network between machines you own. The Jetson is never
exposed to the public internet; there is no port to find and nothing to
scan. Free for personal use.

**On the Jetson** (arm64 is supported; the script detects it):

```
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

It prints a URL. Open it, sign in, and the Jetson joins your tailnet.

**On your Mac and phone:** install Tailscale, sign in with the same account.

**Then, from anywhere:**

```
http://jetson:8080
```

(substitute whatever name the Jetson shows in your Tailscale admin panel —
MagicDNS gives each machine a short name). `tailscale ip` on the Jetson gives
the numeric address if you would rather use that.

This address works on hotel Wi-Fi, on cellular, from another country. It does
not change when your home router hands out a different lease.

---

## Cloudflare quick tunnel — a link you can send someone

A temporary public HTTPS URL that forwards to the cockpit. No account, no
domain, no port forwarding. The URL is random, changes every run, and dies
when you stop the tunnel.

**Install once, on the Jetson:**

```
curl -fsSLo /tmp/cloudflared.deb https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
sudo dpkg -i /tmp/cloudflared.deb
```

**Every time you want to share:**

```
# terminal 1
ROBOCAR_TOKEN=$(head -c 18 /dev/urandom | base64 | tr -d '/+=') ./run_cockpit.sh

# terminal 2
./scripts/share.sh
```

`share.sh` **refuses to run** unless the cockpit is token-protected, and it
checks that by actually sending a command and confirming it is refused —
not by trusting that the environment variable reached the right process. An
env var set in one shell says nothing about a process started in another.

The thing on the other end of that URL is a motor, and a public URL is found
by scanners within minutes. This is the one place in the project where a
script says no.

### What to send, and what to keep

```
send people:   https://<random>.trycloudflare.com/?nocam=1
keep for you:  https://<random>.trycloudflare.com/?k=<your-token>
```

Anyone with the link can **watch** — telemetry, the LiDAR scene, position,
the event log. They cannot command the car. That asymmetry is deliberate:
being able to see what the car is doing is itself a safety property, and it
costs nothing to share. Driving is what the token gates.

### Why `?nocam=1`

Two reasons, both real:

1. Cloudflare caps quick tunnels at 200 in-flight requests and states plainly
   that they do not handle long-lived streams well. Each camera panel holds a
   connection open for as long as it is visible. Two feeds per viewer is
   exactly the traffic shape they warn about.
2. A public link without it is a live video feed of whatever room the car is
   standing in.

With `?nocam=1` the camera panels say *"video off for this link"* — the
scenes, telemetry, map and logs all still work, because they are ordinary
JSON polling.

### Limits worth knowing

Quick tunnels are documented as being for testing and development: random URL
each time, no persistence, 200 concurrent requests. For something permanent
on your own domain you need a free Cloudflare account and a named tunnel —
worth doing if you end up demoing this regularly.

---

## Which one for what

- **Working on the car from your desk across campus** — Tailscale.
- **Showing your advisor the LiDAR scene while you drive it in the lab** —
  quick tunnel with `?nocam=1`.
- **A link on a resume or a portfolio** — neither. Both need the car powered
  on and the Jetson awake. That wants a static page replaying a recorded
  session, which is a different build.

Sources: [Tailscale Linux install](https://tailscale.com/kb/1031/install-linux) ·
[Cloudflare quick tunnels](https://developers.cloudflare.com/cloudflare-one/networks/connectors/cloudflare-tunnel/do-more-with-tunnels/trycloudflare/) ·
[cloudflared downloads](https://developers.cloudflare.com/cloudflare-one/networks/connectors/cloudflare-tunnel/downloads/)
