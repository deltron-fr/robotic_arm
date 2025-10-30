// robot_controller.ino
#include <Servo.h>

// -----------------------------
// Configuration
// -----------------------------
const int NUM_JOINTS = 6;
int jointPins[NUM_JOINTS] = {2,3,4,5,6,7};   // PWM-capable pins for servos
Servo joints[NUM_JOINTS];

const int GRIPPER_PIN = 8; // PWM servo; if you have a digital gripper, change logic below
Servo gripper;

int homeDeg[NUM_JOINTS] = {0, -30, 60, 0, 30, 0};    // must match Python HOME
// Servo library wants 0..180; clamp where needed, or remap per joint if offsets/gearing
int minDeg[NUM_JOINTS]  = {0,  0,  0,  0,  0,  0};
int maxDeg[NUM_JOINTS]  = {180,180,180,180,180,180};

// Trapezoid profile params
// Max speed/accel in deg/s and deg/s^2 for servo interpolation (these are “virtual” limits; Servo moves are position-only).
float MAX_SPEED_DPS  = 120.0;  // tune
float MAX_ACCEL_DPS2 = 400.0;  // tune

// Gripper open/close angles
int GRIP_OPEN_DEG  = 20;
int GRIP_CLOSE_DEG = 80;

// Serial
const unsigned long SERIAL_BAUD = 115200;

// -----------------------------
// State
// -----------------------------
int currentDeg[NUM_JOINTS] = {0};
char lineBuf[128];

// -----------------------------
// Utils
// -----------------------------
int clampDeg(int d, int idx) {
  if (d < minDeg[idx]) return minDeg[idx];
  if (d > maxDeg[idx]) return maxDeg[idx];
  return d;
}

void setJointAngle(int idx, int deg) {
  deg = clampDeg(deg, idx);
  joints[idx].write(deg);
  currentDeg[idx] = deg;
}

void goHome() {
  const unsigned long T = 1200; // ms
  moveTrapezoid(homeDeg, T);
}

// Compute per-joint trapezoid and step synchronously so all joints finish together
void moveTrapezoid(const int targetDeg[NUM_JOINTS], unsigned long totalTimeMs) {
  // Convert to float
  float start[NUM_JOINTS], goal[NUM_JOINTS], dist[NUM_JOINTS];
  float maxDist = 0.0;
  for (int i=0; i<NUM_JOINTS; ++i) {
    start[i] = currentDeg[i];
    goal[i]  = clampDeg(targetDeg[i], i);
    dist[i]  = goal[i] - start[i];
    float ad = fabs(dist[i]);
    if (ad > maxDist) maxDist = ad;
  }

  // If no movement, just OK
  if (maxDist < 0.5f) {
    Serial.println("OK");
    return;
  }

  // Determine profile so that the longest-distance joint finishes exactly at totalTimeMs
  float T = totalTimeMs / 1000.0f;
  // Symmetric trapezoid: accel, cruise, decel. Solve for feasible vmax given T and dist.
  // For simplicity, use a time-scaling factor on a [0,1] S-curve (smoothstep-like) so we avoid jerk.
  // This is simpler and robust with hobby servos.

  const unsigned long dt_ms = 20; // 50 Hz update
  unsigned long startMs = millis();
  while (true) {
    unsigned long now = millis();
    if (now - startMs >= totalTimeMs) break;
    float t = float(now - startMs) / float(totalTimeMs); // 0..1
    // Smoothstep (cubic) for gentle acceleration: 3t^2 - 2t^3
    float s = t*t*(3.0f - 2.0f*t);

    for (int i=0; i<NUM_JOINTS; ++i) {
      float pos = start[i] + s * dist[i];
      setJointAngle(i, (int)round(pos));
    }
    delay(dt_ms);
  }

  // Ensure final exact goal
  for (int i=0; i<NUM_JOINTS; ++i) setJointAngle(i, (int)round(goal[i]));
  Serial.println("OK");
}

bool parseMove(char* line, int outDeg[NUM_JOINTS], unsigned long &Tms) {
  // Format: MOVE a1,a2,a3,a4,a5,a6 T=1500
  // We'll split before 'T='
  char *tpart = strstr(line, "T=");
  if (!tpart) return false;

  // Parse time
  int tval = atoi(tpart + 2);
  if (tval <= 0) return false;
  Tms = (unsigned long)tval;

  // Temporarily terminate the string before T
  *tpart = '\0';

  // After "MOVE "
  char *args = line + 5;
  // Remove leading spaces
  while (*args == ' ') args++;
  // Parse 6 integers separated by commas
  for (int i=0; i<NUM_JOINTS; ++i) {
    char *nextComma = (i < NUM_JOINTS-1) ? strchr(args, ',') : NULL;
    if (i < NUM_JOINTS-1 && !nextComma) return false;

    if (nextComma) *nextComma = '\0';
    outDeg[i] = atoi(args);
    if (nextComma) args = nextComma + 1;
  }
  return true;
}

// -----------------------------
// Setup / Loop
// -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  for (int i=0; i<NUM_JOINTS; ++i) {
    joints[i].attach(jointPins[i]);
    currentDeg[i] = 90;      // neutral on boot; adjust per joint
    joints[i].write(currentDeg[i]);
  }
  gripper.attach(GRIPPER_PIN);
  gripper.write(GRIP_OPEN_DEG);
  delay(500);
}

void loop() {
  // Read line
  if (Serial.available()) {
    size_t n = Serial.readBytesUntil('\n', lineBuf, sizeof(lineBuf)-1);
    lineBuf[n] = '\0';

    // Trim CR
    if (n > 0 && lineBuf[n-1] == '\r') lineBuf[n-1] = '\0';

    if (strncmp(lineBuf, "MOVE", 4) == 0) {
      int target[NUM_JOINTS];
      unsigned long Tms = 0;
      if (parseMove(lineBuf, target, Tms)) {
        moveTrapezoid(target, Tms);
      } else {
        // Bad command
        Serial.println("OK"); // still ack to keep PC from stalling; you can print ERR instead
      }
    } else if (strncmp(lineBuf, "GRIP", 4) == 0) {
      if (strstr(lineBuf, "OPEN")) {
        gripper.write(GRIP_OPEN_DEG);
        delay(250);
        Serial.println("OK");
      } else if (strstr(lineBuf, "CLOSE")) {
        gripper.write(GRIP_CLOSE_DEG);
        delay(400);
        Serial.println("OK");
      } else {
        Serial.println("OK");
      }
    } else if (strncmp(lineBuf, "HOME", 4) == 0) {
      goHome();
    } else {
      // Unknown → acknowledge to avoid Python timeouts, or print an error if you prefer
      Serial.println("OK");
    }
  }
}
