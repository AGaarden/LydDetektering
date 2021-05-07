#include <Arduino.h>

#include <Camera.h>

Camera camera; // Same as: Camera camera(19, 18, 50, 3);

angleSet resetAngles = { resetAngles.xAngle = 0, resetAngles.yAngle = 0};

angleSet angles = { angles.xAngle = 0, angles.yAngle = 0 };
angleSet angles2 = { angles2.xAngle = 2, angles2.yAngle = 2 };
angleSet angles3 = { angles3.xAngle = 3, angles3.yAngle = 3 };
angleSet angles4 = { angles4.xAngle = 20, angles4.yAngle = 20 };

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
  camera.move(angles4);
  delay(1500);
  
  // camera.move(resetAngles);

  // camera.testServos();
}