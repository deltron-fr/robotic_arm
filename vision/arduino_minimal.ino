#include <Servo.h>

const int NUM_JOINTS = 6;
int jointPins[NUM_JOINTS] = {2,3,4,5,6,7};
Servo joints[NUM_JOINTS];

const int GRIPPER_PIN = 8;
Servo gripper;

int currentDeg[NUM_JOINTS] = {90,90,90,90,90,90};

void setup() {
  Serial.begin(115200);
  for (int i=0; i<NUM_JOINTS; ++i) {
    joints[i].attach(jointPins[i]);
    joints[i].write(currentDeg[i]);
  }
  gripper.attach(GRIPPER_PIN);
  gripper.write(20); // open
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.startsWith("MOVE")) {
      int angles[NUM_JOINTS];
      int parsed = sscanf(cmd.c_str(), "MOVE %d,%d,%d,%d,%d,%d", 
                          &angles[0], &angles[1], &angles[2], 
                          &angles[3], &angles[4], &angles[5]);
      if (parsed == NUM_JOINTS) {
        for (int i=0; i<NUM_JOINTS; i++) {
          joints[i].write(angles[i]);
          currentDeg[i] = angles[i];
        }
        Serial.println("OK");
      }
    } 
    else if (cmd.startsWith("GRIP OPEN")) {
      gripper.write(20);
      Serial.println("OK");
    } 
    else if (cmd.startsWith("GRIP CLOSE")) {
      gripper.write(80);
      Serial.println("OK");
    }
  }
}
