#include <Arduino.h>
#include <stdio.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include <soc/i2s_reg.h>
#include <soc/dport_reg.h>
#include <soc/dport_access.h>
#include <driver/periph_ctrl.h>
#include <fft.h>

float foldning[4096];

#define DCOFFSET 1.3
#define BUF_CNT 8
#define BUF_LEN 1024
#define ADC_LEN ((1024*8)/4)

#define I2S0_CLKM_CONF_REG 0x3FF4F0AC
#define I2S0_SAMPLE_RATE_CONF_REG 0x3FF4F0B0

const float upper_threshold = 0.3;
const float lower_threshold = -0.3;

typedef enum {
    REAL_FORWARD,
    REAL_BACKWARD
} fft_type_dir;

void i2s_init();
bool sample_checkamplitude(float *buff0, float *buff1, float *buff2, float *buff3);
int16_t calc_sample_shift(int32_t *pBuf1, int32_t *pBuf2);
void corr_s32(const int32_t *Signal, const int siglen, const int32_t *Pattern, const int patlen, int32_t *dest);
void real_fft(fft_config_t *fft_plan, float *input_arr);
void conj_array(fft_config_t *fft_plan);
void convolve(fft_config_t *fft_plan_0, fft_config_t *fft_plan_1);
float fft_timeshift(float *arr_0, float *arr_1);
int argmax(fft_config_t *fft_plan, float *input_array);
float arg_to_sec(int arg, int fs);