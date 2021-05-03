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

static int32_t upper_threshold = 100;
static int32_t lower_threshold = -100;

unsigned int start_b, end_b;     // Variable for execution measurement

esp_err_t i2s_init(void)
{
    // I2S driver configuration.
    i2s_config_t i2s_conf = {
    	.mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
	    .sample_rate = 1000000,
        //.sample_rate = 48000,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Original deprecated setting: I2S_COMM_FORMAT_I2S_MSB
	    .intr_alloc_flags = 0,
	    .dma_buf_count = BUF_CNT,
	    .dma_buf_len = BUF_LEN,
	    .use_apll = false
    };

    // Configure the I2S driver to use internal ADC, based on configuration above.
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_conf, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0));
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_NUM_0));

    // Delay for I2S bug workaround
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ***IMPORTANT*** enable continuous adc sampling
    SYSCON.saradc_ctrl2.meas_num_limit = 0;

    // Configuring the ADC sampling pattern, their bit resolution and attenuation.
    // Sample in this order: first ADC0, then ADC3, then ADC6 and then ADC7.
    // All set to 12bit (max resolution) and 11dB attenuation (to achieve 0-3.3V scale).
    SYSCON.saradc_sar1_patt_tab[0] = ( (ADC1_CHANNEL_0 << 4) | (ADC_WIDTH_BIT_12 << 2) | (ADC_ATTEN_DB_11 <<0) ) << 24;
    SYSCON.saradc_sar1_patt_tab[0] = SYSCON.saradc_sar1_patt_tab[0] | (( (ADC1_CHANNEL_3 << 4) | (ADC_WIDTH_BIT_12 << 2) | (ADC_ATTEN_DB_11 <<0) ) << 16);
    SYSCON.saradc_sar1_patt_tab[0] = SYSCON.saradc_sar1_patt_tab[0] | (( (ADC1_CHANNEL_6 << 4) | (ADC_WIDTH_BIT_12 << 2) | (ADC_ATTEN_DB_11 <<0) ) << 8);
    SYSCON.saradc_sar1_patt_tab[0] = SYSCON.saradc_sar1_patt_tab[0] | (( (ADC1_CHANNEL_7 << 4) | (ADC_WIDTH_BIT_12 << 2) | (ADC_ATTEN_DB_11 <<0) ) );
    SYSCON.saradc_ctrl.sar1_patt_len = 3;

    // Configuring the ADC sample time for 2Msps
    SYSCON.saradc_ctrl.sar_clk_div = 2;
    SYSCON.saradc_fsm.sample_cycle = 2;

    // Configuring the I2S clock dividers for sampling rate 
    I2S0.clkm_conf.clkm_div_num = 200; // with 200 division 50ksps pr channel is achieved when bck is 2.
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.sample_rate_conf.rx_bck_div_num = 2;

    return ESP_OK;
}

void corr_s16(const int32_t *Signal, const int siglen, const int32_t *Pattern, const int patlen, int32_t *dest)
{
    for (size_t n = 0; n < (siglen - patlen); n++) {
        int32_t k_corr = 0;
        for (size_t m = 0; m < patlen; m++) {
            k_corr += Signal[n + m] * Pattern[m];
        }
        dest[n] = k_corr;
    }
}

// pBuf1 & pBuf2 should be arrays with the length of ADC_LEN.
int16_t calc_sample_shift(int32_t *pBuf1, int32_t *pBuf2) {
    static int32_t corr_buffer[ADC_LEN];    // Variable to temporarily store correlated signal

    corr_s16(&pBuf1[0], ADC_LEN, &pBuf2[63], ADC_LEN-128, &corr_buffer[0]);

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
    int32_t max = corr_buffer[0];
    for (i = 0; i < 128; ++i) {
        if (corr_buffer[i] > max) {
            max = corr_buffer[i];
            max_i = i - 62;                  // Remember offset since we start at indice 63
        }
    }
    // Return the sample shift.
    return max_i; 
}

// This function emptys the DMA into four buffers, removes DC offset and checkes the amplitude.
bool sample_checkamplitude(int32_t *buff0, int32_t *buff1, int32_t *buff2, int32_t *buff3)
{
    static uint16_t buf[BUF_LEN*BUF_CNT]; // Buffer for i2s data.
    size_t bytes_read; // Size var to store # of bytes readed.
    int i; // Counter for debug spamming.  
    bool eventFlag = false; // Event detection flag.

    // Read I2S 0 data  and store in 'buf' variabel. Give error message if failed.
    if (i2s_read(I2S_NUM_0, buf, BUF_LEN * BUF_CNT * sizeof(uint16_t), &bytes_read, portMAX_DELAY) != ESP_OK)
    {
        ESP_LOGW(TAG, "i2s_read() fail");
    }
    int cnt_0 = 0, cnt_3 = 0, cnt_6 = 0, cnt_7 = 0;

    // Extract the 4 ADC channels from the I2S buffer and store in an array for each.
    // This is necessary since we do not know precisely what time empty the buffer and
    // thus do not know the order of the sampled ADC data.
    for (i = 0; i < BUF_LEN * BUF_CNT; i++) {
        int ch_sel;
        
        ch_sel = buf[i] >> 12;  // Shift 12 bits to the right to extract channel number.
        if (ch_sel == 0) { 
            buff0[cnt_0] = (buf[i] & 0x0fff) - DCOFFSET; // Bitwise AND to extract the first 12 bits containing ADC data.
            // Check whether amplitude threshold is exceded
            if ((buff0[cnt_0] < upper_threshold) && (buff0[cnt_0] > lower_threshold)) { 
                // Do nothing
            } else { 
                eventFlag = true; 
            } // Amplitude threshold exceeded, event detected. Also time performance measurement.
            cnt_0++; // Count this specific channel index up as we now have copied a sample.
        }
        
        if (ch_sel == 3) { buff1[cnt_3] = (buf[i] & 0x0fff) - DCOFFSET; cnt_3++; } // Same as above without lvl detection,
        if (ch_sel == 6) { buff2[cnt_6] = (buf[i] & 0x0fff) - DCOFFSET; cnt_6++; } // done for the rest of the ADCs.
        if (ch_sel == 7) { buff3[cnt_7] = (buf[i] & 0x0fff) - DCOFFSET; cnt_7++; }
    }
    return eventFlag;
}

void app_main(void)
{
    // Four buffers, one for each ADC.
    static int32_t adc0buff[ADC_LEN];
    static int32_t adc3buff[ADC_LEN];
    static int32_t adc6buff[ADC_LEN];
    static int32_t adc7buff[ADC_LEN];

    i2s_init();
    while (1) {
    // Do forever...
        if (sample_checkamplitude(&adc0buff[0], &adc3buff[0], &adc6buff[0], &adc7buff[0])) {
            int16_t shift0_3, shift0_6, shift0_7;

            start_b = esp_timer_get_time();
            // Initiate sample shift calculation on the existing buffers.
            shift0_3 = calc_sample_shift(&adc0buff[0], &adc3buff[0]);
            shift0_6 = calc_sample_shift(&adc0buff[0], &adc6buff[0]);
            shift0_7 = calc_sample_shift(&adc0buff[0], &adc7buff[0]);
            end_b = esp_timer_get_time();

            // Print the measured sample shift and also scaled to time based on samplerate.
            printf("\n[Dtctd] Smpls %i \t %.2f us \t Prfrm: %i ms\n", shift0_3, shift0_3 * 20.0, (end_b-start_b)/1000);
        }
    }
}

