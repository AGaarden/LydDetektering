#include <stdio.h>
#include "eventdetect.h"

float foldning[4096];

void corr_s32(const float *Signal, const int siglen, const float *Pattern, const int patlen, float *dest)
{
    for (size_t n = 0; n < (siglen - patlen); n++) {
        float k_corr = 0;
        for (size_t m = 0; m < patlen; m++) {
            k_corr += Signal[n + m] * Pattern[m];
        }
        dest[n] = k_corr;
    }
}

// pBuf1 & pBuf2 should be arrays with the length of ADC_LEN.
int16_t calc_sample_shift(float *pBuf1, float *pBuf2) {
    static float corr_buffer[ADC_LEN];    // Variable to temporarily store correlated signal

    corr_s32(&pBuf1[0], ADC_LEN, &pBuf2[63], ADC_LEN-128, &corr_buffer[0]);

    // Find the shift based on the correlated signal
    int i, max_i = 0;
    float max = corr_buffer[0];
    for (i = 0; i < 128; ++i) {
        if (corr_buffer[i] > max) {
            max = corr_buffer[i];
            max_i = i - 63;                  // Remember offset since we start at indice 63
        }
    }
    // Return the sample shift.
    return -max_i; 
}

void real_fft(fft_config_t *fft_plan, float *input_arr) {
    fft_plan->input = input_arr;
    fft_execute(fft_plan);
}

void conj_array(fft_config_t *fft_plan) {
    for (int i = 3; i < 4096; i += 2) {
          fft_plan->output[i] = fft_plan->output[i] * (-1);
      }
}

void convolve(fft_config_t *fft_plan_0, fft_config_t *fft_plan_1) {
    foldning[0] = fft_plan_0->output[0] * fft_plan_1->output[0];
    foldning[1] = fft_plan_0->output[1] * fft_plan_1->output[1];
    for (int i = 2; i < 4096; i += 2) {
          foldning[i] = (fft_plan_0->output[i]*fft_plan_1->output[i] - fft_plan_0->output[i+1]*fft_plan_1->output[i+1]);
          foldning[i+1] = (fft_plan_0->output[i]*fft_plan_1->output[i+1] + fft_plan_1->output[i]*fft_plan_0->output[i+1]);
      }
}

int argmax(fft_config_t *fft_plan) {
    int i, max_i = 1023;
    float max = fft_plan->output[1023];
    for (i = 1024; i < 3071; i +=1) {
        if (fft_plan->output[i] > max) {
            max = fft_plan->output[i];
            max_i = i;
        }
    }

    return max_i - 2048;
}

float arg_to_sec(int arg, int fs) {
    return arg * (1.0 / fs);
}

float fft_timeshift(float *arr_0, float *arr_1) {
    static fft_config_t *real_fft_plan0 = fft_init(4096, FFT_REAL, FFT_FORWARD, NULL, NULL);
    static fft_config_t *real_fft_plan1 = fft_init(4096, FFT_REAL, FFT_FORWARD, NULL, NULL);
    static fft_config_t *real_ifft_foldning = fft_init(4096, FFT_REAL, FFT_BACKWARD, NULL, NULL);
    // 1. Calculate FFT of the two arrays
    real_fft(real_fft_plan0, arr_0);
    real_fft(real_fft_plan1, arr_1);
    // 2. Conjugate the second array
    conj_array(real_fft_plan1);
    // 3. Convolute the two arrays
    convolve(real_fft_plan0, real_fft_plan1);
    // 4. Inverse FFT of the convolution
    real_fft(real_ifft_foldning, foldning);
    for (int i = 0; i < 4096; i++) {
        printf("%f\n", real_ifft_foldning->output[i]);
    }
    // 5. Find index of max value in convolution
    int max_arg = argmax(real_ifft_foldning);
    // 6. Find corresponding time of index
    //return arg_to_sec(max_arg, 50000);
    return (float) max_arg;
}