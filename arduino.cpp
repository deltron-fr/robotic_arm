#include <Servo.h>

#define NUM_SERVOS 6
Servo servos[NUM_SERVOS];
Servo gripper;

int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};  // Change pins if needed
int gripperPin = 13;  // Example pin for gripper

void setup() {
  Serial.begin(9600);

  // Attach each servo
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }

  gripper.attach(gripperPin);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Handle gripper commands
    if (input == "CLOSE") {
      gripper.write(30);  // Adjust value for closed position
      delay(1000);  // Allow time to close
      Serial.println("OK");
      return;
    } else if (input == "OPEN") {
      gripper.write(90);  // Adjust value for open position
      delay(1000);  // Allow time to open
      Serial.println("OK");
      return;
    }

    // Handle joint angles
    int angles[NUM_SERVOS];
    int index = 0;

    while (input.length() > 0 && index < NUM_SERVOS) {
      int commaIndex = input.indexOf(',');
      String part = (commaIndex != -1) ? input.substring(0, commaIndex) : input;

      angles[index] = part.toInt();
      index++;

      if (commaIndex == -1) break;
      input = input.substring(commaIndex + 1);
    }

    // Move servos to target angles
    if (index == NUM_SERVOS) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(angles[i]);
      }

      delay(1500);  // Wait for motion to complete
      Serial.println("OK");
    } else {
      Serial.println("ERR");  // Invalid input
    }
  }
}
