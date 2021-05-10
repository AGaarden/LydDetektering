/*
 * Camera.h - Library for moving a camera with a set of two servos
 * Created by Sander Gaarden, April 26, 2021
 */

/* RESOLUTION RANGE LOOKUP TABLE (From 0-180 on MG996R Servo motors. Parentheses are minimum and maximum outside of 0-180 range)
 * 8 bit: 5-30 (5-33)
 * 10 bit: 20-119 (18-134)
 * 11 bit: 39-238 (36-268) (+5 for map, 2.1%)
 * 12 bit: 78-477 (71-537) (+5 for map, 1%)
 * 16 bit: 1220-7600 (1134-8600) (+152 for map, 2%)
 */

/* Include necessary libraries */
#include <esp32-hal-ledc.h>
#include <Esp.h>
#include <stdint.h>

#ifndef Camera_h
#define Camera_h

// make a structure with the necessary angles to point towards an occurrence
typedef struct {
  double xAngle;
  double yAngle;
} angleSet;

class Camera{
  private:
    /* PWM properties */
    static const uint8_t BASE_PWM_FREQ = 50;
    static const uint8_t PWM_CHANNEL_1 = 0; /* Zero indexed */
    static const uint8_t PWM_CHANNEL_2 = 1; /* Zero indexed */
    static const uint8_t PWM_RESOLUTION = 11;

    /* Values based on PWM_RESOLUTION, check lookup table */
    static const uint16_t PWM_MIN = 39;
    static const uint16_t PWM_MAX = 238;
    static const uint8_t PWM_MAP_EXTRA = 5; /* Arbitrary value for getting closer to 180 degrees when using map */

    /* Base servo pins */
    static const uint8_t BASE_SERVO_PIN_1 = 19;
    static const uint8_t BASE_SERVO_PIN_2 = 18;

    /* The current position of the camera gets saved to these variables */
    uint8_t current_servo_pos_1;
    uint8_t current_servo_pos_2;

    /* Base variable for plusminus calculations */
    static const uint8_t BASE_ANGLE_VARIATION = 3;

    /* The variables that get set by constructor */
    uint8_t _pwm_freq;
    uint8_t _servo_pin_1;
    uint8_t _servo_pin_2;
    uint8_t _angle_variation;

    /* The functions */
    uint16_t angleToPwm(uint8_t angle);
    uint8_t pwmToAngle(uint16_t pwm);
  public:
    /* The functions */
    Camera(uint8_t servo_pin_1 = BASE_SERVO_PIN_1, uint8_t servo_pin_2 = BASE_SERVO_PIN_2, uint16_t period_hertz = BASE_PWM_FREQ, uint8_t angle_variation = BASE_ANGLE_VARIATION);
    bool move(angleSet angles);
    void testServos();
};

#endif