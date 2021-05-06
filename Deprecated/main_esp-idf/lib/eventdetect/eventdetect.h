#include <stdio.h>                  // This library is used for some datatypes like uint16_t etc.
#include "freertos/FreeRTOS.h"      // This library is used for stuff like printf to serial terminal.
#include "freertos/task.h"          // This library is used for some timing stuff like Arduino delay(10) which here is vTaskDelay(10 / portTICK_PERIOD_MS)
#include "esp_system.h"             // ESP specific stuff like debug commands to view heap size.
#include "esp_log.h"                // ESP 'debug' logging system, eg. the one from i2s read.

#include "driver/i2s.h"             
#include "driver/adc.h"             
#include "driver/dac.h"
#include "soc/syscon_periph.h"      // Stuff included 'to allow' write directly to some registers
#include "soc/i2s_periph.h"         // Stuff included 'to allow' write directly to some registers
#include "soc/sens_periph.h"        // Stuff included 'to allow' to write directly to some registers

#define SAMPLE_RATE (200 * 1000)
#define BUF_LEN 1024
#define BUF_CNT 8
#define ADC_LEN ((BUF_LEN*BUF_CNT) / 4)

#define DEBUG 0
#define PRINT_SIZE 2048
#define PRINT_START 0

#define DCOFFSET 1600

#define TAG "I2S"

esp_err_t i2s_init(void);
void corr_s32(const int32_t *Signal, const int siglen, const int32_t *Pattern, const int patlen, int32_t *dest);
int16_t calc_sample_shift(int32_t *pBuf1, int32_t *pBuf2);
bool sample_checkamplitude(int32_t *buff0, int32_t *buff1, int32_t *buff2, int32_t *buff3);
