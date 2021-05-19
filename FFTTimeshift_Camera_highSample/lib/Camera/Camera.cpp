/*
   Camera.cpp - Library for moving a camera with a set of two servos
   Created by Sander Gaarden, April 26, 2021
*/

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
Camera::Camera(uint8_t servo_pin_1, uint8_t servo_pin_2, uint16_t period_hertz, uint8_t angle_variation) {
  /* Set variables given to constructor */
  _servo_pin_1 = servo_pin_1;
  _servo_pin_2 = servo_pin_2;
  _pwm_freq = period_hertz;
  _angle_variation = angle_variation;
  
  /* Set up PWM channels */
  ledcSetup(PWM_CHANNEL_1, _pwm_freq, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, _pwm_freq, PWM_RESOLUTION);

  /* Attach servos to channels */
  ledcAttachPin(_servo_pin_1, PWM_CHANNEL_1);
  ledcAttachPin(_servo_pin_2, PWM_CHANNEL_2);

  /* Put both servos at their middle positions */
  ledcWrite(PWM_CHANNEL_1, ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN);
  ledcWrite(PWM_CHANNEL_2, ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN);

  current_servo_pos_1 = ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN;
  current_servo_pos_2 = ((PWM_MAX - PWM_MIN) / 2) + PWM_MIN;

  Serial.println("Camera setup complete.");
}

/* 
 *  Name: angleToPwm
 *  Input: Angle in integers
 *  Output: PWM value in integers
 *  Remarks:
 *  The PWM_MAP_EXTRA variable is added arbitrarily to make sure map reaches 180 degrees
 */
uint16_t Camera::angleToPwm(uint8_t angle) {
  uint8_t angle_to_pwm = map(angle, 0, 180, PWM_MIN, PWM_MAX + PWM_MAP_EXTRA);
  
  //return map(angle_to_pwm, 0, 180, 180, 0);
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
   Output: A boolean value representing whether or not the camera has moved anything
   Remarks:
   Moves the camera into the position given
   Only moves if the new position is +-3 degrees in difference
   Has no inherent error correction
*/
bool Camera::move(angleSet angles) {
  uint8_t angle_1 = round(angles.xAngle);
  uint8_t angle_2 = round(angles.yAngle);

  bool moved_1 = false;
  bool moved_2 = false;

  if(abs(angle_1 - current_servo_pos_1) > _angle_variation) {
    ledcWrite(PWM_CHANNEL_1, angleToPwm(angle_1));
    current_servo_pos_1 = angle_1;
    moved_1 = true;
  }

  if(abs(angle_2 - current_servo_pos_2) > _angle_variation) {
    ledcWrite(PWM_CHANNEL_2, angleToPwm(angle_2));
    current_servo_pos_2 = angle_2;
    moved_2 = true;
  }

  if(!moved_1 && !moved_2) {
    return false;
  }
  else {
    return true;
  }
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