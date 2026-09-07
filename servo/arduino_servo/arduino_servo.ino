/*
 * arduino_servo.ino — steering-servo driver for the Self-Driving Car.
 *
 * The Jetson can't cleanly generate a 50 Hz servo PWM, and the VESC firmware
 * wouldn't drive the servo, so an Arduino Nano sits in between: the Jetson
 * sends an angle over USB serial and the Nano outputs the servo pulse on D9.
 *
 * Wiring:
 *   Servo signal (usually orange/white) -> D9
 *   Servo +  (red)                       -> 5V power  (SEE NOTE)
 *   Servo -  (brown/black)               -> GND  (shared with Arduino GND)
 *
 *   NOTE on power: a steering servo can pull >1 A when it hits a stop or fights
 *   the wheels. Pulling that through the Arduino's 5V (fed from the Jetson USB)
 *   can brown out the Nano or the Jetson port and cause resets. For anything
 *   beyond a quick free-air test, power the servo from a separate 5-6 V BEC and
 *   just share GND between the BEC, the servo, and the Arduino.
 *
 * Serial protocol (115200 baud, newline-terminated). Human-typable so you can
 * also drive it straight from the Arduino IDE Serial Monitor:
 *   "90"   -> set angle to 90 degrees (clamped to the safe steering range)
 *   "c"    -> center (90)
 *   "s"    -> sweep across the safe range and back (proves it physically moves)
 *   "d"    -> detach: stop sending pulses so the servo relaxes (no hold buzz)
 *   "?"    -> print status
 * The Nano echoes a line back ("ANGLE 90", "SWEEP done", ...) so the host can
 * confirm the command was understood.
 *
 * STEERING SAFETY: this is a STEERING servo bolted to a linkage, not a free
 * servo. The wheels hit a mechanical stop well before 0 or 180 deg, and driving
 * the servo into that stop makes it stall — it buzzes, pulls >1 A, heats up, and
 * can strip its gears. So we clamp every command to STEER_MIN..STEER_MAX.
 * These were bench-proven to move cleanly with no binding, and are ASYMMETRIC
 * (the linkage swings further left than right). If you confirm the wheels can
 * swing further without the connector binding, widen the range; if they bind
 * sooner, narrow it. HARD_MIN/HARD_MAX are the servo's absolute electrical
 * limits. Keep STEER_MIN/STEER_MAX in sync with LEFT_LIMIT/RIGHT_LIMIT in
 * servo_control.py.
 */
#include <Servo.h>

const int SERVO_PIN  = 9;
const int CENTER     = 90;
const int HARD_MIN   = 0;     // servo's absolute travel — never exceed
const int HARD_MAX   = 180;

const int STEER_MIN  = 60;    // most-left  safe angle (bench-tested)
const int STEER_MAX  = 115;   // most-right safe angle (bench-tested)

Servo steer;
int  currentAngle = CENTER;
bool attached = false;
String buf;

void ensureAttached() {
  if (!attached) { steer.attach(SERVO_PIN); attached = true; }
}

void setAngle(int a) {
  // Clamp to the soft steering range so we can never drive into the linkage stop.
  a = constrain(a, STEER_MIN, STEER_MAX);
  ensureAttached();
  steer.write(a);
  currentAngle = a;
  Serial.print("ANGLE ");
  Serial.println(a);
}

void detach() {
  // Stop the pulse train so the servo stops actively holding (less buzz/heat
  // when idle). Any move command re-attaches automatically.
  if (attached) { steer.detach(); attached = false; }
  Serial.println("DETACHED");
}

void sweep() {
  ensureAttached();
  for (int a = STEER_MIN; a <= STEER_MAX; a += 5) { steer.write(a); delay(25); }
  for (int a = STEER_MAX; a >= STEER_MIN; a -= 5) { steer.write(a); delay(25); }
  setAngle(CENTER);
  Serial.println("SWEEP done");
}

void handle(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c0 = s.charAt(0);
  if (c0 == 'c' || c0 == 'C') { setAngle(CENTER); return; }
  if (c0 == 's' || c0 == 'S') { sweep(); return; }
  if (c0 == 'd' || c0 == 'D') { detach(); return; }
  if (c0 == '?') {
    Serial.print("STATUS angle="); Serial.print(currentAngle);
    Serial.print(" range="); Serial.print(STEER_MIN);
    Serial.print(".."); Serial.print(STEER_MAX);
    Serial.print(" attached="); Serial.println(attached ? 1 : 0);
    return;
  }

  bool numeric = true;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) { numeric = false; break; }
  }
  if (numeric) setAngle(s.toInt());
  else { Serial.print("ERR ?"); Serial.println(s); }
}

void setup() {
  Serial.begin(115200);
  setAngle(CENTER);   // attaches and centers
  Serial.print("READY servo-driver D9, range ");
  Serial.print(STEER_MIN); Serial.print(".."); Serial.print(STEER_MAX);
  Serial.println(", send angle / c / s / d / ?");
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { handle(buf); buf = ""; }
    else if (buf.length() < 16)   { buf += ch; }
  }
}
