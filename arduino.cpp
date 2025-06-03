#include <Servo.h>

Servo servo1, servo2, servo3;

void setup() {
  Serial.begin(9600);
  servo1.attach(0);
  servo2.attach(1);
  servo3.attach(2);
  servo3.attach(3);
  servo3.attach(4);
  servo3.attach(5);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int angles[6];
    int idx = 0;

    char *ptr = strtok((char*)input.c_str(), ",");
    while (ptr != NULL && idx < 3) {
      angles[idx++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    servo1.write(angles[0]);
    servo2.write(angles[1]);
    servo3.write(angles[2]);
    servo3.write(angles[3]);
    servo3.write(angles[4]);
    servo3.write(angles[5]);
  }
}
