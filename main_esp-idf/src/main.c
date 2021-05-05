/*************************
 * ESP32 I2S ADC 2Mpbs sampling code for 4 ADC channels
 * 
 * Made by Stephan (2021-04-27)
 * Based on code from amedes, located at https://github.com/espressif/esp-idf/pull/1991
 * 
 * *************************/
#include <stdio.h>                  // This library is used for some datatypes like uint16_t etc.
#include "freertos/FreeRTOS.h"      // This library is used for stuff like printf to serial terminal.
#include "freertos/task.h"          // This library is used for some timing stuff like Arduino delay(10) which here is vTaskDelay(10 / portTICK_PERIOD_MS)
#include "esp_system.h"             // ESP specific stuff like debug commands to view heap size.
#include "esp_log.h"                // ESP 'debug' logging system, eg. the one from i2s read.

#include <math.h>

#include "eventdetect.h"
#include "direction.h"

unsigned int start_b, end_b;        // Variable for execution measurement

void app_main(void)
{
    // Four buffers, one for each ADC.
    static int32_t adc0buff[ADC_LEN];
    static int32_t adc3buff[ADC_LEN];
    static int32_t adc6buff[ADC_LEN];
    static int32_t adc7buff[ADC_LEN];
    
    angles angle;

    i2s_init();

    while (1) {
    // Do forever...
        if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
            float shift0_3, shift0_6, shift0_7;

            start_b = esp_timer_get_time();            
            // Initiate sample shift calculation on the existing buffers.
            shift0_3 = (float) (calc_sample_shift(&adc0buff[0], &adc3buff[0]))*0.00002;
            shift0_6 = 0.0; //calc_sample_shift(&adc0buff[0], &adc6buff[0]);
            shift0_7 = 0.0; //calc_sample_shift(&adc0buff[0], &adc7buff[0]);
            end_b = esp_timer_get_time();            

            //angle = direction(shift0_3, shift0_6 +0.00001, shift0_7);
            angle = direction(shift0_3, shift0_6, shift0_7);
            printf("Angle \txy: %f \tyz: %f\n", angle.xAngle, angle.yAngle);

            // Print the measured sample shift and also scaled to time based on samplerate.
            printf("\n[Detected] 0-3: %.6f s \t 0-6: %.6f s \t 0-7: %.6f s \n", shift0_3, shift0_6, shift0_7);
            printf("Performance timer: %i ms\n", (end_b-start_b)/1000);
        }
    }
}

