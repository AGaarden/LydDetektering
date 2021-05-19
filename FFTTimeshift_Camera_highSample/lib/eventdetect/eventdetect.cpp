#include <Arduino.h>
#include <stdio.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include <soc/i2s_reg.h>
#include <soc/dport_reg.h>
#include <soc/dport_access.h>
#include <driver/periph_ctrl.h>
#include <eventdetect.h>

float foldning[4096];



void i2s_init()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  1000000,                        // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = BUF_CNT,
    .dma_buf_len = BUF_LEN,
    .use_apll = false,
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
  i2s_adc_enable(I2S_NUM_0);

  adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_3);
  adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_6);
  adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_7);

  delay(100);

  // Clear the max measurement register so it is 0.
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MAX_MEAS_NUM_M);
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MEAS_NUM_LIMIT_M);

  // Configure the adc sampling pattern.
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_SAR1_PATT_TAB1_REG, SYSCON_SARADC_SAR1_PATT_TAB1_M);
  SET_PERI_REG_BITS(SYSCON_SARADC_SAR1_PATT_TAB1_REG, SYSCON_SARADC_SAR1_PATT_TAB1_V, 0xf3f6f7f, SYSCON_SARADC_SAR1_PATT_TAB1_S);
  
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_CTRL_REG,SYSCON_SARADC_SAR1_PATT_LEN);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN_V, 0x3, SYSCON_SARADC_SAR1_PATT_LEN_S); //0xF0000

  // Configure ADC sampletime for 2 msps
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR_CLK_DIV, 0x2, SYSCON_SARADC_SAR_CLK_DIV_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_SAMPLE_CYCLE, 0x2, SYSCON_SARADC_SAMPLE_CYCLE_S);

  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_DATA_TO_I2S_V, 1, SYSCON_SARADC_DATA_TO_I2S_S);

  // I2S registers
  DPORT_SET_PERI_REG_BITS(I2S0_CLKM_CONF_REG, I2S_CLKM_DIV_NUM, 200, I2S_CLKM_DIV_NUM_S);
  //DPORT_SET_PERI_REG_BITS(I2S0_CLKM_CONF_REG, I2S_CLKM_DIV_NUM, 100, I2S_CLKM_DIV_NUM_S);
  //DPORT_SET_PERI_REG_BITS(I2S0_CLKM_CONF_REG, I2S_CLKM_DIV_NUM, 50, I2S_CLKM_DIV_NUM_S);    // 200 ksps pr channel.
  DPORT_SET_PERI_REG_BITS(I2S0_CLKM_CONF_REG, I2S_CLKM_DIV_B, 0, I2S_CLKM_DIV_B_S);
  DPORT_SET_PERI_REG_BITS(I2S0_CLKM_CONF_REG, I2S_CLKM_DIV_A, 1, I2S_CLKM_DIV_A_S);
  DPORT_SET_PERI_REG_BITS(I2S0_SAMPLE_RATE_CONF_REG, I2S_RX_BCK_DIV_NUM, 4, I2S_RX_BCK_DIV_NUM_S);

  delay(1000);
}

// Substraction in time domain.
void subs_s32(const float *Signal, const int siglen, const float *Pattern, const int patlen, float *dest)
{
    for (size_t n = 0; n < (siglen - patlen); n++) {
        float k_corr = 0;
        for (size_t m = 0; m < patlen; m++) {
            k_corr += Signal[n + m] - Pattern[m];
        }
        dest[n] = k_corr;
    }
}

// Correlation in time domain.
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

// Deprecated (slow) method to calculate sample shift.
// pBuf1 & pBuf2 should be arrays with the length of ADC_LEN.
int16_t calc_sample_shift(float *pBuf1, float *pBuf2) {
    static float corr_buffer[ADC_LEN];    // Variable to temporarily store correlated signal

//    corr_s32(&pBuf1[0], ADC_LEN, &pBuf2[63], ADC_LEN-128, &corr_buffer[0]);
    corr_s32(&pBuf1[0], ADC_LEN, &pBuf2[127], ADC_LEN-256, &corr_buffer[0]);

    //Debugging code to dump ADC data to serial terminal in CSV format.
#if DEBUG == 1
    printf("\nDumping data: ");
    for (int y = PRINT_START; y < (PRINT_START + PRINT_SIZE); y++)
    {
        printf("\n%i;%i;%i;%i", y, signed_buf1[y], signed_buf2[y], corr_buffer[y]);
    }
#endif

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
    return max_i; 
}

bool sample_checkamplitude(float *buff0, float *buff1, float *buff2, float *buff3)
{
    static uint16_t buf[BUF_LEN*BUF_CNT];   // Buffer for i2s data.
    size_t bytes_read;                      // Size var to store # of bytes readed.
    int i;                                  // Counter for debug spamming.  
    bool eventFlag = false;                 // Event detection flag.

    // Read I2S 0 data  and store in 'buf' variabel. Give error message if failed.
    i2s_read(I2S_NUM_0, buf, BUF_LEN * BUF_CNT * sizeof(uint16_t), &bytes_read, portMAX_DELAY);

    int cnt_0 = 0, cnt_3 = 0, cnt_6 = 0, cnt_7 = 0;

    // Extract the 4 ADC channels from the I2S buffer and store in an array for each.
    // This is necessary since we do not know precisely what time empty the buffer and
    // thus do not know the order of the sampled ADC data.
    for (i = 0; i < (BUF_LEN * BUF_CNT); i++) {
        int ch_sel;
        
        ch_sel = buf[i] >> 12;                            // Shift 12 bits to the right to extract channel number.
        //printf("{TIMEPLOT|DATA|%i|T|%i }\n", ch_sel, buf[i] &0x0FFF);
        if (ch_sel == 0) { 
            buff0[cnt_0] = (float)((buf[i] & 0x0fff)*0.0008) - DCOFFSET0;  // Bitwise AND to extract the first 12 bits containing ADC data.
            //Serial.println(buff0[cnt_0]);
            // Check whether amplitude threshold is exceded
            if ((buff0[cnt_0] < upper_threshold) && (buff0[cnt_0] > lower_threshold)) { 
                // Do nothing
            } else { 
                eventFlag = true; 
            } // Amplitude threshold exceeded, event detected. Also time performance measurement.

            cnt_0++; // Count this specific channel index up as we now have copied a sample.
        }
        
        if (ch_sel == 3) { 
            buff1[cnt_3] = (float)((buf[i] & 0x0fff)*0.0008) - DCOFFSET1;  // Bitwise AND to extract the first 12 bits containing ADC data.
            //Serial.println(buff0[cnt_0]);
            // Check whether amplitude threshold is exceded
            if ((buff1[cnt_3] < upper_threshold) && (buff1[cnt_3] > lower_threshold)) { 
                // Do nothing
            } else { 
                eventFlag = true; 
            } // Amplitude threshold exceeded, event detected. Also time performance measurement.

            cnt_3++; // Count this specific channel index up as we now have copied a sample.
        }

        if (ch_sel == 6) { 
            buff2[cnt_6] = (float)((buf[i] & 0x0fff)*0.0008) - DCOFFSET2;  // Bitwise AND to extract the first 12 bits containing ADC data.
            //Serial.println(buff0[cnt_0]);
            // Check whether amplitude threshold is exceded
            if ((buff2[cnt_6] < upper_threshold) && (buff2[cnt_6] > lower_threshold)) { 
                // Do nothing
            } else { 
                eventFlag = true; 
            } // Amplitude threshold exceeded, event detected. Also time performance measurement.

            cnt_6++; // Count this specific channel index up as we now have copied a sample.
        }

        if (ch_sel == 7) { 
            buff3[cnt_7] = (float)((buf[i] & 0x0fff)*0.0008) - DCOFFSET3;  // Bitwise AND to extract the first 12 bits containing ADC data.
            //Serial.println(buff0[cnt_0]);
            // Check whether amplitude threshold is exceded
            if ((buff3[cnt_7] < upper_threshold) && (buff3[cnt_7] > lower_threshold)) { 
                // Do nothing
            } else { 
                eventFlag = true; 
            } // Amplitude threshold exceeded, event detected. Also time performance measurement.

            cnt_7++; // Count this specific channel index up as we now have copied a sample.
        }
    }
    return eventFlag;
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