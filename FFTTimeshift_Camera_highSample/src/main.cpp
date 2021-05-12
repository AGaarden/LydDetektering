#include <Arduino.h>
#include <eventdetect.h>
#include <direction.h>
#include <Camera.h>
#include <math.h>

#define avgCnt 10

#define exportSamples 0

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
  //Serial.begin(1500000);
  i2s_init();

  Serial.println("\n\nSystem started...");

  // angles = direction_angle( -0.000324297751982, 0.000331477068266, 0.000279886724913);  
  // debugPoints = direction( -0.000324297751982, 0.000331477068266, 0.000279886724913);
  // printf("xang: %.2f \tyang: %.2f \tx: %.2f \ty: %.2f \tz: %.2f \n", angles.xAngle, angles.yAngle, debugPoints.x, debugPoints.y, debugPoints.z);

  // delay(1000);
}

void loop() {
  static angleSet average[avgCnt];
  static angleSet sum;
  static angleSet avg;
  static int avg_pos = 0;

  while (1) {
#if exportSamples
    sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0]);

    for (int i = 0; i < 2048; i++) {
      printf("%.4f;%.4f;%.4f;%.4f\n", adc0buff[i], adc3buff[i], adc6buff[i], adc7buff[i]);
    }
#else
    if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
      double shift0_3, shift0_6, shift0_7;

      start_b = esp_timer_get_time();          

      // FFT based timeshift calculation.
      shift0_3 = fft_timeshift(&adc0buff[0], &adc3buff[0]) * (-1);
      shift0_6 = fft_timeshift(&adc0buff[0], &adc6buff[0]) * (-1);
      shift0_7 = fft_timeshift(&adc0buff[0], &adc7buff[0]) * (-1);

      // Old sample shift calculation method.
      // shift0_3 = calc_sample_shift(&adc0buff[0], &adc3buff[0]) * 1.0/200000;
      // shift0_6 = calc_sample_shift(&adc0buff[0], &adc6buff[0]) * 1.0/200000;
      // shift0_7 = calc_sample_shift(&adc0buff[0], &adc7buff[0]) * 1.0/200000;

      if (((shift0_3 < 0.0005) && (shift0_3 > -0.0005)) && ((shift0_6 < 0.0005) && (shift0_6 > -0.0005)) && ((shift0_7 < 0.0005) && (shift0_7 > -0.0005))) {
        if (fabs(shift0_3) != fabs(shift0_6)) {
          angles = direction_angle(shift0_3, shift0_6, shift0_7);

          if ((!isnan(angles.xAngle)) && (!isnan(angles.yAngle))) {
            debugPoints = direction(shift0_3, shift0_6, shift0_7);

            // Avg code starts here. TBD: Make into function.
            sum.xAngle = sum.xAngle - average[avg_pos].xAngle + angles.xAngle;
            average[avg_pos].xAngle = angles.xAngle;

            sum.yAngle = sum.yAngle - average[avg_pos].yAngle + angles.yAngle;
            average[avg_pos].yAngle = angles.yAngle;

            avg.xAngle = 180-(sum.xAngle / avgCnt);
            avg.yAngle = (sum.yAngle / avgCnt);

            avg_pos++;
            if (avg_pos >= avgCnt){
              avg_pos = 0;
            }
            // Avg code ends here. To get average:
            // "sum.xAngle / avgCnt"

            camera.move(avg);

            end_b = esp_timer_get_time();  

            // Serial debug spamming:
            //printf("%i ms \t xy: %.1f \t zy: %.1f \t Avg_xy: %.1f \t avg_zy: %.1f \t a: %.6f \t b: %.6f \t c: %.6f \t x: %.2f \t y: %.2f \t z: %.2f \n", (end_b-start_b)/1000, angles.xAngle, angles.yAngle, sum.xAngle/avgCnt, sum.yAngle/avgCnt, shift0_3, shift0_6, shift0_7, debugPoints.x, debugPoints.y, debugPoints.z);

            // MegunoLink debug spamming:
            printf("{XYPLOT|DATA|angle|%.1f|%.1f}{XYPLOT|DATA|angleAVG|%.1f|%.1f}{XYPLOT|DATA|0_3|135|%.1f}{XYPLOT|DATA|0_6|140|%.1f}{XYPLOT|DATA|0_7|145|%.1f}{XYPLOT|DATA|x|165|%.2f}{XYPLOT|DATA|y|170|%.2f}{XYPLOT|DATA|z|175|%.2f}\n", angles.xAngle, angles.yAngle, sum.xAngle/avgCnt, sum.yAngle/avgCnt, shift0_3*1000000, shift0_6*1000000, shift0_7*1000000, debugPoints.x, debugPoints.y, debugPoints.z);
            //printf("%i\t {XYPLOT|DATA|angle|%.1f|%.1f}{XYPLOT|DATA|0_3|135|%.1f}{XYPLOT|DATA|0_6|140|%.1f}{XYPLOT|DATA|0_7|145|%.1f}\n", (end_b-start_b)/1000, angles.xAngle, angles.yAngle, shift0_3*1000000, shift0_6*1000000, shift0_7*1000000);
          }
        }
      }
    }
#endif
  }
}