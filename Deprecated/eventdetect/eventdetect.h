#include <Arduino.h>
#include <stdio.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include <soc/i2s_reg.h>
#include <soc/dport_reg.h>
#include <soc/dport_access.h>
#include <driver/periph_ctrl.h>

#define DCOFFSET 1900
#define BUF_CNT 8
#define BUF_LEN 1024
#define ADC_LEN ((1024*8)/4)

#define I2S0_CLKM_CONF_REG 0x3FF4F0AC
#define I2S0_SAMPLE_RATE_CONF_REG 0x3FF4F0B0

const int32_t upper_threshold = 300;
const int32_t lower_threshold = -300;

void i2s_init();
bool sample_checkamplitude(int32_t *buff0, int32_t *buff1, int32_t *buff2, int32_t *buff3);
int16_t calc_sample_shift(int32_t *pBuf1, int32_t *pBuf2);
void corr_s32(const int32_t *Signal, const int siglen, const int32_t *Pattern, const int patlen, int32_t *dest);