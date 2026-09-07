/*
 * steering_esp32.ino — steering-servo driver (ESP32 version).
 *
 * Same role and serial protocol as the Arduino version: the Jetson sends a
 * steering value over USB serial and the board generates the servo pulse.
 * Target: Seeed XIAO ESP32S3 (servo on GPIO2 = pad "D1"). 3.3V logic, native
 * USB-CDC serial, WiFi/BLE-capable for later.
 *
 * Protocol (one message per line):
 *   float in [-1.0, +1.0]  ->  -1 = full left, 0 = center, +1 = full right
 * Replies "ok <us>". Failsafe: re-centers if no command for TIMEOUT_MS.
 *
 * Needs the ESP32Servo library (handles the LEDC peripheral for servo PWM).
 */
#include <ESP32Servo.h>

const int SERVO_PIN      = 2;     // GPIO2 = XIAO ESP32S3 pad "D1" (3.3V PWM)
const int CENTER_US      = 1500;  // straight-ahead (calibrated on the Uno)
const int STEER_RANGE_US = 450;   // +/- from center (full left 1050, full right 1950)
const unsigned long TIMEOUT_MS = 1000;

Servo steering;
unsigned long lastCmd = 0;

void setup() {
  Serial.begin(115200);
  steering.setPeriodHertz(50);            // standard 50 Hz servo frame
  steering.attach(SERVO_PIN, 1000, 2000); // safe pulse bounds
  steering.writeMicroseconds(CENTER_US);
  lastCmd = millis();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      float s = line.toFloat();
      if (s < -1.0) s = -1.0;
      if (s >  1.0) s =  1.0;
      int us = CENTER_US + (int)(s * STEER_RANGE_US);
      steering.writeMicroseconds(us);
      lastCmd = millis();
      Serial.print("ok ");
      Serial.println(us);
    }
  }
  if (millis() - lastCmd > TIMEOUT_MS) {
    steering.writeMicroseconds(CENTER_US);  // failsafe: straighten
  }
}
