#include <Arduino.h>
#include <eventdetect.h>
#include <direction.h>
#include <Camera.h>
#include <math.h>

static float adc0buff[2048];
static float adc3buff[2048];
static float adc6buff[2048];
static float adc7buff[2048];

unsigned int start_b, end_b;        // Variable for execution measurement
angleSet angles;
Point debugPoints;
Camera camera;

void setup() {
  Serial.begin(921600);
  i2s_init();

  Serial.println("\n\nSystem started...");
}

void loop() {
  while (1) {

    if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
      double shift0_3, shift0_6, shift0_7;

      start_b = esp_timer_get_time();          

      shift0_3 = fft_timeshift(&adc0buff[0], &adc3buff[0]);
      shift0_6 = fft_timeshift(&adc0buff[0], &adc6buff[0]);
      shift0_7 = fft_timeshift(&adc0buff[0], &adc7buff[0]);

      if (((shift0_3 < 0.0004) && (shift0_3 > -0.0004)) && ((shift0_6 < 0.0004) && (shift0_6 > -0.0004)) && ((shift0_7 < 0.0004) && (shift0_7 > -0.0004))) {
        if (fabs(shift0_3) != fabs(shift0_6)) {
          angles = direction_angle(shift0_3, shift0_6, shift0_7);
          debugPoints = direction(shift0_3, shift0_6, shift0_7);
          camera.move(angles);

          end_b = esp_timer_get_time();  

          printf("%i ms \t xy: %.1f \t zy: %.1f \t a: %.6f \t b: %.6f \t c: %.6f \t x: %.2f \t y: %.2f \t z: %.2f \n", (end_b-start_b)/1000, angles.xAngle, angles.yAngle, shift0_3, shift0_6, shift0_7, debugPoints.x, debugPoints.y, debugPoints.z);
        }
      }
    }
  }
}