#!/usr/bin/env python3
"""
drivers/oled.py — SSD1306 128x64 I2C OLED (status display).

Thin wrapper over luma.oled. The panel is on /dev/i2c-7 @ 0x3c (40-pin header pins
3/SDA, 5/SCL), VCC on 3.3V (pin 1). Shares the bus with the IST8310 compass (0x0e).

  open()          -> init the panel
  text(lines)     -> draw a list of text rows
  clear()         -> blank the panel
  close()         -> blank + release

NOTE: if the oled_stats systemd service is running it already owns the panel and
this will fight it (garbled screen). Stop it first:
    sudo systemctl stop oled_stats.service
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

BUS = config.I2C_BUS
ADDR = config.OLED_ADDR
WIDTH = 128
HEIGHT = 64


class OLED:
    def __init__(self, bus=BUS, addr=ADDR):
        self.bus = bus
        self.addr = addr
        self.device = None

    def open(self):
        from luma.core.interface.serial import i2c
        from luma.oled.device import ssd1306
        serial = i2c(port=self.bus, address=self.addr)
        self.device = ssd1306(serial, width=WIDTH, height=HEIGHT)
        return self

    def text(self, lines):
        """Draw a list of strings, one per row (top-down)."""
        from luma.core.render import canvas
        with canvas(self.device) as draw:
            y = 0
            for line in lines[:6]:
                draw.text((0, y), str(line), fill="white")
                y += 11

    def clear(self):
        from luma.core.render import canvas
        with canvas(self.device):
            pass

    def close(self):
        if self.device is not None:
            try:
                self.clear()
            except Exception:
                pass
            self.device = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()
