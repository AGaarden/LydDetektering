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

  Serial.println("System started...");
}

void loop() {
  while (1) {

    if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
      double shift0_3, shift0_6, shift0_7;

      start_b = esp_timer_get_time();          

      shift0_3 = fft_timeshift(&adc0buff[0], &adc3buff[0]);
      shift0_6 = fft_timeshift(&adc0buff[0], &adc6buff[0]);
      shift0_7 = fft_timeshift(&adc0buff[0], &adc7buff[0]);

      angles = direction_angle(shift0_3, shift0_6, shift0_7);
      debugPoints = direction(shift0_3, shift0_6, shift0_7);
      camera.move(angles);

      // // Initiate sample shift calculation on the existing buffers.
      // shift0_3 = (float) (calc_sample_shift(&adc0buff[0], &adc3buff[0]))*0.00002;
      // shift0_6 = (float) (calc_sample_shift(&adc0buff[0], &adc6buff[0]))*0.00002;
      // shift0_7 = (float) (calc_sample_shift(&adc0buff[0], &adc7buff[0]))*0.00002;
      // angle = direction(shift0_3, shift0_6, shift0_7);
      // camera.move(angle);
      end_b = esp_timer_get_time();  

      //printf("Angle \txy: %f \tyz: %f\n", angle.xAngle, angle.yAngle);
      //printf("[Detected] 0-3: %.6f s \t 0-6: %.6f s \t 0-7: %.6f s \n", shift0_3, shift0_6, shift0_7);
      //printf("Performance timer: %i ms\n", (end_b-start_b)/1000);

      // for (int i = 0; i < 2048; i++) {
      //   //printf("%i;%f;%f;%f\n", i, adc0buff[i], real_fft_plan0->output[i], real_ifft_plan0->output[i]);
      //   printf("%i;%f;%f;%f;%f;%f;%f;%f\n", i, adc0buff[i], real_fft_plan0->output[i], real_ifft_plan0->output[i],   adc3buff[i], real_fft_plan1->output[i], real_ifft_plan1->output[i], real_ifft_foldning->output[i]);
      // }
      //printf("%f ms\t %f ms\t %f ms\t %f.2°\t %f.2°\n", shift0_3, shift0_6, shift0_7, angles.X, angles.Y);
      printf("%i ms \t xy: %.1f \t zy: %.1f \t a: %.6f \t b: %.6f \t c: %.6f \t x: %.2f \t y: %.2f \t z: %.2f \n", (end_b-start_b)/1000, angles.xAngle, angles.yAngle, shift0_3, shift0_6, shift0_7, debugPoints.x, debugPoints.y, debugPoints.z);
    }
  }
}