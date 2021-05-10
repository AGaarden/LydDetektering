#include <Arduino.h>

#include <Camera.h>

Camera camera; // Same as: Camera camera(19, 18, 50, 3);

angleSet resetAngles = { resetAngles.xAngle = 0, resetAngles.yAngle = 0};

angleSet angles{ 0, 0 };
angleSet angles2{ 2, 2 };
angleSet angles3{ 3, 3 };
angleSet angles4{ 20, 20 };

angleSet angles5{ 110, 110 };
angleSet angles6{ 80, 80 };
angleSet angles7{ 88, 88 };

void setup() {
  Serial.begin(115200);
  delay(3000);
}

void loop() {
  /*
  camera.move(angles);
  delay(1500);
  camera.move(angles2);
  delay(1500);
  camera.move(angles3);
  delay(1500);
  camera.move(angles4);
  delay(1500);
  */

  // camera.move(resetAngles);

  // camera.testServos();

  /* This first tests 20 degrees in steps, then 30 in the opposite direction, then 8 steps to see if it goes to normal move */
  
  camera.stepMove(angles5);
  delay(1500);
  camera.stepMove(angles6);
  delay(1500);
  camera.stepMove(angles7);
  delay(1500);
  
}