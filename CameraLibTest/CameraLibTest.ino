#include <Arduino.h>

#include "Camera.h"

void setup() {
  Serial.begin(115200);
  servoSetup();
}

void loop() {
  delay(1500);
  testServos();
}
