#include <Arduino.h>

#include <Camera.h>

Camera camera;

angleSet angles = { angles.xAngle = 90, angles.yAngle = 180 };
angleSet angles2 = { angles2.xAngle = 0, angles.yAngle = 90 };
angleSet angles3 = { angles3.xAngle = 180, angles.yAngle = 0 };

void setup() {
  Serial.begin(115200);
  delay(3000);
}

void loop() {
  camera.move(angles);
  delay(1500);
  camera.move(angles2);
  delay(1500);
  camera.move(angles3);
  delay(1500);
}