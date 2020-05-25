#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "dig_i2s_adc.h"
#include "dac-cosine.h"

#define I2S_ADC_UNIT            (ADC_UNIT_1)

float ch6_samples[1000] = {0};
float ch7_samples[1000] = {0};

/*
This example set up I2S ADC mode, The ADC scans 4 channels (ADC unit0: ch3, ch0, ch7 and ch6) in sequence, and the sampling frequency is 2M. 
each sample takes 2 bytes, low 12bit is the result of AD conversion and the high 4bit is the channel num.

        |-500K-|
         __     __     __     __     __     __     __     __     __  
WS    __|  |___|  |___|  |___|  |___|  |___|  |___|  |___|  |___|  |__

       CH3   CH0   CH7   CH6   CH3   CH0   CH7   CH6  ...

receive buffer: [ CH0 ][ CH3 ][ CH6 ][ CH7 ]...

*/

//Out put WS signal from gpio18(only for debug mode)
void i2s_adc_check_clk(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[4], PIN_FUNC_GPIO);
    gpio_set_direction(4, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(4, I2S0I_WS_OUT_IDX, 0, 0);
}

esp_err_t i2s_adc_setup(void)
{
    // adc_channel_t channel[] = {
    //     ADC1_CHANNEL_3,
    //     ADC1_CHANNEL_0,
    //     ADC1_CHANNEL_7,
    //     ADC1_CHANNEL_6,
    // };
    adc_channel_t channel[] = {
        ADC1_CHANNEL_7,
        ADC1_CHANNEL_6,
    };
    if (i2s_adc_init(I2S_NUM_0) != ESP_OK) {
        printf("i2s adc init fail\n");
        return ESP_FAIL;
    }
    //Configuring scan channels
    //adc_i2s_scale_mode_init(I2S_ADC_UNIT, channel, 4);
    adc_i2s_scale_mode_init(I2S_ADC_UNIT, channel, 2);
    //rate = 2M
    //ADC sampling rate = 4000,000 / clkm_num (4,000,000 should be divisible by clkm_num)
    // 40kHz = 4Mhz/100. That should be 20kHz per channel
    i2s_adc_set_clk(I2S_NUM_0, 100);
    if (i2s_adc_driver_install(I2S_NUM_0, NULL, NULL) != ESP_OK){
        printf("driver install fail\n");
        return ESP_FAIL;
    }
    //Uncomment this line only in debug mode.
    i2s_adc_check_clk();
    return ESP_OK;
}

/*
 * Main task that let you test CW parameters in action
 *
*/
void dactask(void* arg)
{
    while(1){

        // frequency setting is common to both channels
        dac_frequency_set(clk_8m_div, frequency_step);

        /* Tune parameters of channel 2 only
         * to see and compare changes against channel 1
         */
        dac_scale_set(DAC_CHANNEL_2, scale);
        dac_offset_set(DAC_CHANNEL_2, offset);
        dac_invert_set(DAC_CHANNEL_2, invert);

        float frequency = RTC_FAST_CLK_FREQ_APPROX / (1 + clk_8m_div) * (float) frequency_step / 65536;
        printf("clk_8m_div: %d, frequency step: %d, frequency: %.0f Hz\n", clk_8m_div, frequency_step, frequency);
        printf("DAC2 scale: %d, offset %d, invert: %d\n", scale, offset, invert);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

void app_main()
{
    /* Generate a sine waveform on both DAC channels:
    *
    * DAC_CHANNEL_1 - GPIO25
    * DAC_CHANNEL_2 - GPIO26
    */

    dac_cosine_enable(DAC_CHANNEL_1);
    dac_cosine_enable(DAC_CHANNEL_2);

    dac_frequency_set(clk_8m_div, frequency_step);
    dac_scale_set(DAC_CHANNEL_2, scale);
    dac_offset_set(DAC_CHANNEL_2, offset);
    dac_invert_set(DAC_CHANNEL_2, 2);

    // dac_output_enable(DAC_CHANNEL_1);
    // dac_output_enable(DAC_CHANNEL_2);
    
    //xTaskCreate(dactask, "dactask", 1024*3, NULL, 10, NULL);

    //Calculate clocks
    float frequency_dac = RTC_FAST_CLK_FREQ_APPROX / (1 + clk_8m_div) * (float) frequency_step / 65536;
    float adc_sample_rate = 40000;

    uint32_t samples_per_period = (uint32_t)(adc_sample_rate/frequency_dac);
    samples_per_period = round(samples_per_period / 2) * 2; //Round to nearest even integer.

    //Make sure adc_sample_len is even!? Since we have 2 channels. Need equal amount of samples per channel.
    uint32_t adc_sample_len = samples_per_period*2;     //sample 2 periods (for both channels)
    uint32_t adc_sample_len_bytes = sizeof(uint16_t) * adc_sample_len;

    ESP_LOGI(__func__, "Setting up ADC samplig. Number of samples to take: %d", adc_sample_len);

    uint16_t *buf = malloc(adc_sample_len_bytes);

    if (!buf) {
        printf("buffer malloc fail\n");
        goto error;
    }
    if (i2s_adc_setup() != ESP_OK) {
        printf("i2s adc setup fail\n");
        goto error;
    }

    while(1) {

        for (size_t j = 10; j > 0; j--)
        {
            ESP_LOGI(__func__, "Starting ADC sampling in %d", j);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }

        if (i2s_adc_start(I2S_NUM_0, buf, adc_sample_len_bytes) != ESP_OK) {
            goto error;
        }

        dac_output_enable(DAC_CHANNEL_1);
        dac_output_enable(DAC_CHANNEL_2);

        i2s_wait_adc_done(I2S_NUM_0, portMAX_DELAY);

        uint32_t ch, sample6, sample7;
        sample6 = 0;
        sample7 = 0;
        for (int i = 0; i < adc_sample_len; i++) {

            ch = (buf[i] >> 12);
            if(ch == 6){
                ch6_samples[sample6] = 3.3 - (3.14* (buf[i] & 0xfff)  / 4095);
                sample6++;
            }   
            else if(ch == 7){
                ch7_samples[sample7] = 3.3 - (3.14* (buf[i] & 0xfff)  / 4095);
                sample7++;
            }
            else
            {
                ESP_LOGE(__func__, "Invalid channel sample! Channel: %d", ch);
            }

            //printf("ch %d  %fv\n", (buf[i] >> 12), 3.3 - (3.14* (buf[i] & 0xfff)  / 4095) );
        }

        //print samples
        //Print header
        printf("sample,channel 6,channel 7\r\n");
        for (size_t k = 0; k < adc_sample_len/2; k++)
        {
            printf("%d\t", k);
            printf("%f\t", ch6_samples[k]);
            printf("%f\r\n", ch7_samples[k]);
        }

        printf("------------------------\n");
        memset(buf, 0, adc_sample_len_bytes);

        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
error:
    while(1) vTaskDelay(1000/portTICK_PERIOD_MS);
}