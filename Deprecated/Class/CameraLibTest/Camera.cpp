/*
   Camera.cpp - Library for moving a camera with a set of two servos
   Created by Sander Gaarden, April 26, 2021
*/

/* Include necessary libraries */
#include <esp32-hal-ledc.h>
#include <Esp.h>
#include <stdint.h>

/* Include header file */
#include "Camera.h"

/* RESOLUTION RANGE LOOKUP TABLE (From 0-180 on MG996R Servo motors. Parentheses are minimum and maximum outside of 0-180 range)
 * 8 bit: 5-30 (5-33)
 * 10 bit: 20-119 (18-134)
 * 11 bit: 39-238 (36-268) (+5 for map, 2.1%)
 * 12 bit: 78-477 (71-537) (+5 for map, 1%)
 * 16 bit: 1220-7600 (1134-8600) (+152 for map, 2%)
 */

/*
 * Constructor
 * Output: None
 * Remarks:
 * The following function sets up servos as per the ledc library
 * Base servo pins are 19 and 18, and base period hertz is 50
 * NOTE: Servos only available on pins 2, 4, 5, 12-19, 21-23, 25-27, 32-33
 */
Camera::Camera(uint8_t servo_pin_1, uint8_t servo_pin_2, uint16_t period_hertz) {
  /* Set up PWM channels */
  ledcSetup(PWM_CHANNEL_1, period_hertz, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, period_hertz, PWM_RESOLUTION);

  /* Attach servos to channels */
  ledcAttachPin(servo_pin_1, PWM_CHANNEL_1);
  ledcAttachPin(servo_pin_2, PWM_CHANNEL_2);

  /* Put both servos at their middle positions */
  ledcWrite(PWM_CHANNEL_1, ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN);
  ledcWrite(PWM_CHANNEL_2, ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN);
}

/* 
 *  Name: angleToPwm
 *  Input: Angle in integers
 *  Output: PWM value in integers
 *  Remarks:
 *  The PWM_MAP_EXTRA variable is added arbitrarily to make sure map reaches 180 degrees
 */
uint16_t Camera::angleToPwm(uint8_t angle) {
  return map(angle, 0, 180, PWM_MIN, PWM_MAX + PWM_MAP_EXTRA);
}

/* 
 *  Name: pwmToAngle
 *  Input: PWM value in integers
 *  Output: Angle in integers
 *  Remarks:
 *  None
 */
uint8_t Camera::pwmToAngle(uint16_t pwm) {
  return map(pwm, PWM_MIN, PWM_MAX, 0, 180);
}

/*
   Name: move
   Input: angleSet
   Output: None
   Remarks:
   Moves the camera into the position given
   Has no inherent error correction
*/
void Camera::move(angleSet angles) {
  ledcWrite(PWM_CHANNEL_1, angleToPwm(round(angles.xAngle)));
  ledcWrite(PWM_CHANNEL_2, angleToPwm(round(angles.yAngle)));
}

/*
   Name: testServos
   Output: None
   Remarks:
   Tests the servos by moving them through all 180 positions
*/
void Camera::testServos() {
  ledcWrite(PWM_CHANNEL_1, angleToPwm(0));
  ledcWrite(PWM_CHANNEL_2, angleToPwm(0));

  /* Move servo 1 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    ledcWrite(PWM_CHANNEL_1, angleToPwm(pos));

    delay(15);
  }
  delay(3000);

  /* Move servo 2 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    ledcWrite(PWM_CHANNEL_2, angleToPwm(pos));
    delay(15);
  }
  delay(3000);

  /* Move servo 1 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    ledcWrite(PWM_CHANNEL_1, angleToPwm(pos));
    delay(15);
  }
  delay(3000);

  /* Move servo 2 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    ledcWrite(PWM_CHANNEL_2, angleToPwm(pos));
    delay(15);
  }
  delay(1000);
}
