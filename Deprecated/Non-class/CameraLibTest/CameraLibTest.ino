#include <Arduino.h>

#include "Camera.h"

angleSet angles = { angles.xAngle = 90, angles.yAngle = 180 };
angleSet angles2 = { angles2.xAngle = 0, angles.yAngle = 90 };
angleSet angles3 = { angles3.xAngle = 180, angles.yAngle = 0 };

void setup() {
  Serial.begin(115200);
  cameraSetup();
  delay(3000);
}

void loop() {
  move(angles);
  delay(1500);
  move(angles2);
  delay(1500);
  move(angles3);
  delay(1500);
}
