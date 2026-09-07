# Jetson Orin Nano — OLED System Stats

Live system stats on a 128x64 **SSD1306 I2C OLED**, refreshed once per second:

| Row | Metric        | Source (preferred → fallback)                          |
|-----|---------------|--------------------------------------------------------|
| Tmp | SoC/CPU temp  | `jtop` → `/sys/.../thermal_zone*/temp`                 |
| CPU | Overall CPU % | `psutil` (with a tiny bar)                              |
| GPU | GPU usage %   | `jtop` → `/sys/devices/platform/.../*.gpu/load` (bar)  |
| Fan | RPM or PWM %  | `jtop` → `pwm_tach`/`pwmfan` hwmon                      |

If a value can't be read it shows `N/A` instead of crashing. The whole metric
set is a registry list, so adding a new stat is a 2-line change (see the header
comment in `oled_stats.py`).

---

## 1. Wiring (40-pin header)

| OLED pin | Jetson pin            |
|----------|-----------------------|
| VCC      | Pin 1  (3.3V)         |
| GND      | Pin 6  (GND)          |
| SDA      | Pin 3  (I2C1 SDA)     |
| SCL      | Pin 5  (I2C1 SCL)     |

Pins 3/5 are exposed on the Orin Nano as **`/dev/i2c-7`** (the default in the
script). Use 3.3V (pin 1), **not** 5V, for an SSD1306 module.

---

## 2. Enable / verify I2C

I2C on the 40-pin header is enabled by default on JetPack. Confirm the bus and
that your display is detected:

```bash
# Install i2c-tools if needed:
sudo apt-get update && sudo apt-get install -y i2c-tools

# List buses (you should see i2c-7 among them):
ls /dev/i2c-*

# Make sure your user can use I2C without sudo:
sudo usermod -aG i2c $USER     # then log out / back in

# Scan bus 7 for the OLED — expect a device at 0x3c (or 0x3d):
i2cdetect -y -r 7
```

Expected output has `3c` in the grid:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
```

If `0x3c` does **not** appear: re-check wiring (especially SDA/SCL not swapped),
confirm 3.3V on VCC, and try the other address `0x3d` (set `I2C_ADDRESS` in the
script). If your module is on a different bus, set `I2C_BUS` accordingly.

---

## 3. Install dependencies

### a) Python rendering + fallback libs (no sudo needed)

```bash
cd oled_stats
pip3 install -r requirements.txt
```

This installs `luma.oled` (which brings in `luma.core`, `Pillow`/PIL and
`smbus2`) plus `psutil`.

### b) jetson-stats (`jtop`) — recommended, needs sudo + reboot

`jtop` gives the best GPU / temperature / fan readings on Jetson:

```bash
sudo pip3 install -U jetson-stats
sudo reboot          # jtop installs a system service that needs a reboot
```

After reboot, verify the service is up:

```bash
systemctl status jtop.service
jtop                  # interactive UI; press 'q' to quit
```

> The script works **without** jtop too — it automatically falls back to
> `psutil` + sysfs. You'll just see a one-line note on stderr.

---

## 4. Run

```bash
cd oled_stats
python3 oled_stats.py
```

Press **Ctrl+C** to stop — the display is cleared and powered off on exit.

Config lives at the top of `oled_stats.py`:

```python
I2C_BUS         = 7       # /dev/i2c-7
I2C_ADDRESS     = 0x3C    # try 0x3D if 0x3C isn't detected
REFRESH_SECONDS = 1.0
```

---

## 5. Auto-start on boot (optional, systemd)

A unit file `oled_stats.service` is included. It is preconfigured for user
`wasif` and this folder — **edit `User=`, `WorkingDirectory=`, and `ExecStart=`
if your paths/user differ**, then:

```bash
# Copy the unit into place:
sudo cp oled_stats/oled_stats.service /etc/systemd/system/

# Reload systemd and enable at boot:
sudo systemctl daemon-reload
sudo systemctl enable --now oled_stats.service

# Check it / view logs:
systemctl status oled_stats.service
journalctl -u oled_stats.service -f
```

Manage it:

```bash
sudo systemctl stop oled_stats.service       # stop (clears the display)
sudo systemctl disable oled_stats.service    # don't start at boot
```

---

## 6. Add your own metric

In `oled_stats.py`:

```python
# 1) Write a provider returning (text, percent_or_None):
def metric_power():
    return ("4.2W", None)        # None  -> no bar
    # return ("47%", 47.0)       # 0..100 -> draws a bar

# 2) Register it (one line):
METRICS.append(Metric("Pwr", metric_power))
```

The render loop iterates `METRICS`, so nothing else needs to change. (Keep the
list at ~4–5 rows so it fits the 64px-tall panel.)

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `could not open SSD1306 ... Errno 2` | Wrong bus — check `ls /dev/i2c-*` and `I2C_BUS`. |
| `Remote I/O error` / nothing on screen | `0x3c` not detected — see step 2; try `0x3d`. |
| `Permission denied` on `/dev/i2c-7` | Add user to `i2c` group (step 2) and re-login. |
| GPU/Fan show `N/A` | Install jetson-stats (step 3b) and reboot. |
| Garbled/half screen | Module may be SH1106 — install `luma.oled` and use `sh1106` device instead of `ssd1306`. |
