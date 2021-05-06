#include <Arduino.h>
#include <eventdetect.h>
#include <direction.h>

static int32_t adc0buff[2048];
static int32_t adc3buff[2048];
static int32_t adc6buff[2048];
static int32_t adc7buff[2048];

unsigned int start_b, end_b;        // Variable for execution measurement
angleSet angle;
Camera camera;

void setup() {
  Serial.begin(921600);
  i2s_init();

  Serial.println("System started...");
}

void loop() {
  if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
    float shift0_3, shift0_6, shift0_7;

    start_b = esp_timer_get_time();            
    // Initiate sample shift calculation on the existing buffers.
    shift0_3 = (float) (calc_sample_shift(&adc0buff[0], &adc3buff[0]))*0.00002;
    shift0_6 = (float) (calc_sample_shift(&adc0buff[0], &adc6buff[0]))*0.00002;
    shift0_7 = (float) (calc_sample_shift(&adc0buff[0], &adc7buff[0]))*0.00002;
    
    angle = direction(shift0_3, shift0_6, shift0_7);

    camera.move(angle);
    end_b = esp_timer_get_time();            
    //printf("Angle \txy: %f \tyz: %f\n", angle.xAngle, angle.yAngle);
    //printf("[Detected] 0-3: %.6f s \t 0-6: %.6f s \t 0-7: %.6f s \n", shift0_3, shift0_6, shift0_7);
    //printf("Performance timer: %i ms\n", (end_b-start_b)/1000);
    printf("%i ms\n", (end_b-start_b)/1000);
  }
}