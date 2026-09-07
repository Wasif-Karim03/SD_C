/*
 * steering_arduino.ino — steering-servo driver for the self-driving car.
 *
 * The VESC can't output a servo signal on this firmware, so the Arduino
 * generates the steering pulse instead. It receives a steering command from
 * the Jetson over USB serial and drives the steering servo on pin D9.
 *
 * Protocol (one message per line, newline-terminated):
 *   a float in [-1.0, +1.0]   ->  -1.0 = full left, 0 = center, +1.0 = full right
 * Replies "ok <microseconds>" so the Jetson can confirm.
 *
 * Failsafe: if no command arrives for TIMEOUT_MS, the servo re-centers, so the
 * car straightens out if the Jetson link drops. Throttle stays on the VESC.
 */
#include <Servo.h>

const int SERVO_PIN      = 9;     // servo signal wire goes here
const int CENTER_US      = 1500;  // neutral pulse — CALIBRATE to your car's straight
const int STEER_RANGE_US = 450;   // +/- from center (calibrated: -1=left, center straight)
const unsigned long TIMEOUT_MS = 1000;

Servo steering;
unsigned long lastCmd = 0;

void setup() {
  Serial.begin(115200);
  steering.attach(SERVO_PIN, 1000, 2000);   // safe pulse bounds
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
  // Failsafe: re-center if the Jetson stops sending commands.
  if (millis() - lastCmd > TIMEOUT_MS) {
    steering.writeMicroseconds(CENTER_US);
  }
}
