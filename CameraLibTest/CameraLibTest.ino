#include <Arduino.h>
#include <ESP32Servo.h> /* This is added for the CameraMovement header file */

#include "CameraMovement.h"

void setup() {
  servoSetup();
}

void loop() {
  testServos();
}
