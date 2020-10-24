#include <string.h>
#include <math.h>
#include <esp_types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/xtensa_api.h"
#include "soc/dport_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "soc/efuse_reg.h"
#include "soc/syscon_struct.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/rtc_io.h"
#include "driver/dac.h"
#include "esp_intr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "dig_i2s_adc.h"

#define I2S_ADC_CHECK(a, str, ret) if (!(a)) {\
        ESP_LOGE(I2S_ADC_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);\
        return (ret);\
        }

#define I2S_ADC_ENTER_CRITICAL(i2s_num)   portENTER_CRITICAL(&i2s_adc_spinlock[i2s_num])
#define I2S_ADC_EXIT_CRITICAL(i2s_num)    portEXIT_CRITICAL(&i2s_adc_spinlock[i2s_num])

#define LOG_CONV_TIME 1
#if LOG_CONV_TIME == 1
    uint64_t conv_start_time;
#endif

static const char* I2S_ADC_TAG = "I2S ADC";
static portMUX_TYPE i2s_adc_spinlock[I2S_NUM_MAX] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};

static i2s_adc_handle_t* i2s_adc_handle[2] = {NULL, NULL};

extern void adc_power_always_on();

/* ========================================================================= */
/* [PFUN] Private functions implementations                                  */
/* ========================================================================= */

//Out put WS signal from gpio4 (only for debug mode)
static void s_i2s_adc_check_clk(void)
{
    //Todo. Connect to the used I2S port!
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[4], PIN_FUNC_GPIO);
    gpio_set_direction(4, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(4, I2S0I_WS_OUT_IDX, 0, 0);
}

static void IRAM_ATTR s_i2s_adc_isr(void *arg)
{
    i2s_adc_handle_t *self = (i2s_adc_handle_t*) arg;
    uint8_t i2s_num = self->i2s_num;
    portBASE_TYPE high_priority_task_awoken = 0;
    uint32_t inr_st = self->I2S[i2s_num]->int_st.val;
    if(inr_st == 0) return;
    if (inr_st & (I2S_IN_DSCR_ERR_INT_ST_M || inr_st & I2S_IN_ERR_EOF_INT_ST_M)) {
        ESP_EARLY_LOGE(I2S_ADC_TAG, "dma error, interrupt status: 0x%08x", inr_st);
    }
    if (inr_st & I2S_IN_SUC_EOF_INT_ST_M) {
        #if LOG_CONV_TIME == 1
            self->conversion_time = esp_timer_get_time() - conv_start_time;
        #endif
        self->I2S[i2s_num]->conf.rx_reset = 1;
        self->I2S[i2s_num]->lc_conf.ahbm_fifo_rst = 1;
        xSemaphoreGiveFromISR(self->done_mux, &high_priority_task_awoken);
        if(self->cb) {
            self->cb(self->param);
        }


    }
    if (high_priority_task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    self->I2S[i2s_num]->int_clr.val = inr_st;
}

static esp_err_t s_i2s_adc_intr_ena(i2s_adc_handle_t *self)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    I2S_ADC_ENTER_CRITICAL(self->i2s_num);
    self->I2S[self->i2s_num]->int_ena.val = I2S_IN_DSCR_ERR_INT_ENA_M | I2S_IN_ERR_EOF_INT_ENA_M | I2S_IN_SUC_EOF_INT_ENA_M;
    I2S_ADC_EXIT_CRITICAL(self->i2s_num);
    return ESP_OK;
}

static esp_err_t s_i2s_adc_intr_dis(i2s_adc_handle_t *self)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    I2S_ADC_ENTER_CRITICAL(self->i2s_num);
    self->I2S[self->i2s_num]->int_ena.val = 0;
    self->I2S[self->i2s_num]->int_clr.val = ~0;
    I2S_ADC_EXIT_CRITICAL(self->i2s_num);
    return ESP_OK;
}

static esp_err_t s_i2s_adc_setup_handle(i2s_adc_handle_t *self)
{
    ESP_LOGD(I2S_ADC_TAG, "Setting the handle to default state");

    if(self == NULL) {
        self = (i2s_adc_handle_t *)calloc(1, sizeof(i2s_adc_handle_t));
        if(self == NULL){
            ESP_LOGE(I2S_ADC_TAG, "failed to install i2s_adc handle");
            s_i2s_adc_driver_uninstall(self);
            return ESP_FAIL;
        }
    }
    else {
        ESP_LOGD(I2S_ADC_TAG, "driver aleardy installed");
        return ESP_OK;
    }

    self->I2S[0] = &I2S0;
    self->I2S[1] = &I2S1;

    return ESP_OK;
}

static esp_err_t s_i2s_adc_set_clk(i2s_adc_handle_t *self, uint8_t clkm)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    I2S_ADC_ENTER_CRITICAL(self->i2s_num);
    self->I2S[self->i2s_num]->clkm_conf.clka_en = 0;
    self->I2S[self->i2s_num]->clkm_conf.clkm_div_a = 0;
    self->I2S[self->i2s_num]->clkm_conf.clkm_div_b = 0;
    self->I2S[self->i2s_num]->clkm_conf.clkm_div_num = clkm;
    I2S_ADC_EXIT_CRITICAL(self->i2s_num);
    return ESP_OK;
}


static esp_err_t s_i2s_adc_driver_uninstall(i2s_adc_handle_t *self)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    if(self != NULL) {
        s_i2s_adc_intr_dis(self->i2s_num);
        if (self->i2s_num == I2S_NUM_0) {
            periph_module_disable(PERIPH_I2S0_MODULE);
        } else if (self->i2s_num == I2S_NUM_1) {
            periph_module_disable(PERIPH_I2S1_MODULE);
        }
        if(self->done_mux) {
            vSemaphoreDelete(self->done_mux); 
        }
        esp_intr_free(self->i2s_isr_handle);
        free(self);
    }
    return ESP_OK;
}

static esp_err_t s_i2s_adc_driver_install(i2s_adc_handle_t *self, adc_done_cb cb, void *param)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);

    self->done_mux = xSemaphoreCreateMutex();
    xSemaphoreTake(self->done_mux, portMAX_DELAY);
    if(self->done_mux == NULL){
        ESP_LOGE(I2S_ADC_TAG, "failed to retrieve done_mux");
        s_i2s_adc_driver_uninstall(self);
        return ESP_FAIL;       
    }
    self->param = param;
    self->dma_handle.descs->owner = 1;
    self->dma_handle.descs->eof = 0;
    self->dma_handle.descs->sosf = 0;
    self->dma_handle.descs->length = 0;
    self->dma_handle.descs->size = 0;
    self->dma_handle.descs->buf = NULL;
    self->dma_handle.descs->offset = 0;
    self->dma_handle.descs->empty = NULL;
    self->cb = cb;
    self->param = param;
    s_i2s_adc_intr_ena(self->i2s_num);
    return esp_intr_alloc(ETS_I2S0_INTR_SOURCE + self->i2s_num, 0, &s_i2s_adc_isr, (void *)self, &(self->i2s_isr_handle));
}

static esp_err_t s_i2s_adc_dma_init(i2s_adc_handle_t *self, size_t sample_len, size_t list_num)
{

    //sample_len is the length of the rx buffer for each dma transfer. 

    //Calculate the DMA transfer length
    uint32_t dmachunklen;

    //Receive needs DMA length rounded to next 32-bit boundary
    //dmachunklen = (mspi_dma_config->dma_trans_len + 3) & (~3);
    dmachunklen = (sample_len >> 2) << 2;

    //Set up DMA buffer
    uint32_t dma_buffer_len = dmachunklen * list_num;

    ESP_LOGD(TAG, "Allocating DMA buffer with %d bytes", dma_buffer_len);
    handle->dma_handle.dma_buffer = (uint32_t *)heap_caps_malloc(dma_buffer_len, MALLOC_CAP_DMA);       	//For DMA
    
    memset(handle->dma_handle.dma_buffer, 0, handle->dma_handle.buffer_len);
    ESP_LOGD(TAG, "Successfully allocated spi_buffer on address: %p", handle->dma_handle.dma_buffer);
 

    //Setup DMA descriptors
    handle->dma_handle.descs = (lldesc_t *)calloc(mspi_dma_config->list_num, sizeof(lldesc_t));
    uint8_t *data = (uint8_t *)handle->dma_handle.dma_buffer;

    for (size_t i = 0; i < mspi_dma_config->list_num; i++)
    {
        handle->dma_handle.descs[i].owner = 1;
        handle->dma_handle.descs[i].eof = 1;      //Hard-coded to 1 for now. This makes the SPI_IN_SUC_EOF_DES_ADDR_REG updated at each dma transfer
        handle->dma_handle.descs[i].sosf = 0; 

        handle->dma_handle.descs[i].length = 2;
        handle->dma_handle.descs[i].size = dmachunklen;
        
        handle->dma_handle.descs[i].qe.stqe_next = &handle->dma_handle.descs[i+1];
        handle->dma_handle.descs[i].buf = data;

        ESP_LOGD(TAG, "DMA desc %d address: %p", i,(void*) &handle->dma_handle.descs[i]);
        ESP_LOGD(TAG, "DMA buffer %d address: %p", i,(void*) handle->dma_handle.descs[i].buf);
       
        data += dmachunklen;    //Increment buffer pointer
    }

    handle->dma_handle.descs[mspi_dma_config->list_num - 1].eof = 1;             //Mark last DMA desc as end of stream.

    if(mspi_dma_config->linked_list_circular)
    {
        handle->dma_handle.descs[mspi_dma_config->list_num - 1].qe.stqe_next = &handle->dma_handle.descs[0]; //Point to the first descriptor to get a circular array of descriptors
    }
    else
    {
        handle->dma_handle.descs[mspi_dma_config->list_num - 1].qe.stqe_next = NULL; //current linked list item is last of list
    }
    
    //Reset SPI DMA
    handle->hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    handle->hw->dma_out_link.start  		= 0;
    handle->hw->dma_in_link.start   		= 0;
    handle->hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    if(mspi_dma_config->isrx == 0){
        //Configure outlink descriptor
        handle->hw->dma_out_link.addr           = (int)(handle->dma_handle.descs) & 0xFFFFF;
        handle->hw->dma_conf.outdscr_burst_en   = 1;
        handle->hw->dma_conf.out_data_burst_en  = 0;
        //handle->hw->dma_out_link.start		    = 1;	
    }
    else
    {
        //Configure inlink descriptor
        handle->hw->dma_in_link.addr            = (int)(handle->dma_handle.descs) & 0xFFFFF; 
        handle->hw->dma_conf.indscr_burst_en    = 1;
        //handle->hw->dma_in_link.start           = 1;
    }

    //Register any DMA interrupts if needed
    //s_mspi_register_interrupt_dmatrans(handle);

    return ESP_OK;
}



static esp_err_t s_i2s_adc_init(i2s_adc_config_t *i2s_config, i2s_adc_handle_t *self)
{
    esp_err_t ret;

    s_i2s_adc_setup_handle(self);

    self->i2s_num = i2s_config->i2s_num;

    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    if (self->i2s_num == I2S_NUM_1) {
        periph_module_enable(PERIPH_I2S1_MODULE);
    } else {
        periph_module_enable(PERIPH_I2S0_MODULE);
    }

    adc_power_always_on();
    I2S_ADC_ENTER_CRITICAL(self->i2s_num);

    //Disable interrupt
    self->I2S[self->i2s_num]->int_ena.val = 0;
    self->I2S[self->i2s_num]->int_clr.val = ~0;
    self->I2S[self->i2s_num]->conf.val = 0;
    self->I2S[self->i2s_num]->conf.rx_reset = 1;
    self->I2S[self->i2s_num]->conf.rx_reset = 0;
    self->I2S[self->i2s_num]->conf.rx_msb_right = 1;    //Set this to place right-channel data at the MSB in the receive FIFO.
    self->I2S[self->i2s_num]->conf.rx_right_first = 0;  //Set this bit to receive right-channel data first.
    //Reset fifio
    self->I2S[self->i2s_num]->conf.rx_fifo_reset = 1;
    self->I2S[self->i2s_num]->conf.rx_fifo_reset = 0;
    //Disable pcm
    self->I2S[self->i2s_num]->conf1.rx_pcm_bypass = 1;  //set this bit to bypass the Compress/Decompress module for the received data.
    //Enable and configure DMA
    self->I2S[self->i2s_num]->lc_conf.val = 0;
    self->I2S[self->i2s_num]->lc_conf.ahbm_fifo_rst = 1;
    self->I2S[self->i2s_num]->lc_conf.ahbm_fifo_rst = 0;
    self->I2S[self->i2s_num]->lc_conf.ahbm_rst = 1;
    self->I2S[self->i2s_num]->lc_conf.ahbm_rst = 0;
    self->I2S[self->i2s_num]->lc_conf.in_rst = 1;
    self->I2S[self->i2s_num]->lc_conf.in_rst = 0;
    self->I2S[self->i2s_num]->lc_conf.indscr_burst_en = 1;
    //Enable paral mode
    self->I2S[self->i2s_num]->conf2.val = 0;
    self->I2S[self->i2s_num]->conf2.lcd_en = 1;
    //Configure fifo
    self->I2S[self->i2s_num]->fifo_conf.val = 0;
    self->I2S[self->i2s_num]->fifo_conf.rx_fifo_mod = 1;    //16-bit single channel data (mode 1) p.315
    self->I2S[self->i2s_num]->fifo_conf.rx_data_num = 32;
    self->I2S[self->i2s_num]->fifo_conf.rx_fifo_mod_force_en = 1;   //The bit should always be set to 1.
    self->I2S[self->i2s_num]->fifo_conf.dscr_en = 1;//connect dma to fifo
    self->I2S[self->i2s_num]->conf_chan.rx_chan_mod = 1;
    self->I2S[self->i2s_num]->pdm_conf.val = 0;
    self->I2S[self->i2s_num]->clkm_conf.clk_en = 1;
    self->I2S[self->i2s_num]->sample_rate_conf.rx_bck_div_num = 20; //'M' coefficient in master receive mode
    //16 bit mode
    self->I2S[self->i2s_num]->sample_rate_conf.rx_bits_mod = 16;    //Set the bits to configure the bit length of I²S receiver channel.
    self->I2S[self->i2s_num]->conf.rx_start = 1;
    I2S_ADC_EXIT_CRITICAL(self->i2s_num);


    //Configuring scan channels
    adc_channel_t channel[] = {
        ADC1_CHANNEL_7,
        ADC1_CHANNEL_6,
    };
    ret = adc_i2s_scale_mode_init(ADC_UNIT_1, channel, 2);
    if (ret != ESP_OK)
    {
		ESP_LOGE(__func__, "adc_i2s_scale_mode_init(): returned %d", ret);
        return ESP_FAIL;
    }

    ret = s_i2s_adc_set_clk(self, 100);
    if (ret != ESP_OK)
    {
		ESP_LOGE(__func__, "s_i2s_adc_set_clk(): returned %d", ret);
        return ESP_FAIL;
    }

    ret = s_i2s_adc_driver_install(self, NULL, NULL) != ESP_OK)
    if (ret != ESP_OK)
    {
		ESP_LOGE(__func__, "s_i2s_adc_driver_install(): returned %d", ret);
        return ESP_FAIL;
    }

    //Uncomment this line only in debug mode.
    s_i2s_adc_check_clk();   

    return ESP_OK;
}


// esp_err_t s_i2s_adc_get_dma_rxdata(mspi_transaction_t *mspi_trans_p, mspi_device_handle_t handle)
// {
//     if(handle->initiated == false){
//         ESP_LOGE(TAG, "mspi not initiated! Init first");
//         return ESP_FAIL;    
//     }
//     if(handle->dma_handle.dmaChan == 0){
//         ESP_LOGE(TAG, "DMA not used! Init DMA first with channel 1 or 2");
//         return ESP_FAIL;
//     }
//     //Check register SPI_IN_SUC_EOF_DES_ADDR_REG to get "The last inlink descriptor address when SPI DMA encountered EOF. (RO)"
//     //This way, we should be able to get the address of the buffer containing the most recent data.

//     //TODO - Check so that we actually have some data received!

//     uint32_t dma_in_eof_addr_internal;
//     lldesc_t* last_inlink_desc_eof;
//     uint32_t dma_data_size;
//     uint8_t *dma_data_buf;

//     //Get the last inlink descriptor address when SPI DMA encountered EOF
//     dma_in_eof_addr_internal = handle->hw->dma_in_suc_eof_des_addr;
    
//     if(dma_in_eof_addr_internal == 0){
//         //No EOF encountered yet.
//         ESP_LOGW(TAG, "No DMA data ready yet");
//         return ESP_OK;
//     }

//     //Assign pointer to the address
//     last_inlink_desc_eof = (lldesc_t *)dma_in_eof_addr_internal;
    
//     //Get the corresponding data buffer pointer
//     dma_data_buf = (uint8_t *) last_inlink_desc_eof->buf;
//     dma_data_size = last_inlink_desc_eof->length;

//     //ESP_LOGD(TAG, "last_inlink_desc_eof:%p", last_inlink_desc_eof);
//     //ESP_LOGD(TAG, "dma_data_buf address:%p. Value:[0x%02x, 0x%02x]", dma_data_buf, *dma_data_buf, *(dma_data_buf+1));
//     //ESP_LOGD(TAG, "dma_data_size:%d", dma_data_size);

//     for (size_t i = 0; i < dma_data_size; i++)
//     {
//         mspi_trans_p->rxdata[i] = *(dma_data_buf+i);
//     }
    
//     return ESP_OK;
// }

/* ========================================================================= */
/* [FUNC] Functions implementations                                          */
/* ========================================================================= */

esp_err_t i2s_adc_init(i2s_adc_handle_t* handle)
{
    esp_err_t ret = ESP_OK;
    i2s_adc_config_t config;
    config.i2s_num = I2S_NUM_0;

    handle = i2s_adc_handle[0];

    ret = s_i2s_adc_init(&config, handle);
    if (ret != ESP_OK)
    {
		ESP_LOGE(__func__, "s_i2s_adc_init(): returned %d", ret);
        return ESP_FAIL;
    }
 
}

esp_err_t i2s_adc_start(i2s_adc_handle_t* self, void *buf, size_t len)
{
    I2S_ADC_CHECK((self->i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);

    I2S_ADC_CHECK((self != NULL), "driver not installed", ESP_ERR_INVALID_ARG);
    I2S_ADC_CHECK((buf != NULL), "buffer null", ESP_ERR_INVALID_ARG);

    s_i2s_adc_dma_init(self, len, 1);
    // uint32_t rd_len = (len >> 2) << 2;
    // //Configure DMA link
    // i2s_adc_obj[i2s_num]->desc.owner = 1;
    // i2s_adc_obj[i2s_num]->desc.eof = 1;
    // i2s_adc_obj[i2s_num]->desc.length = rd_len;
    // i2s_adc_obj[i2s_num]->desc.size = rd_len;
    // i2s_adc_obj[i2s_num]->desc.buf = (uint8_t *)buf;


    I2S_ADC_ENTER_CRITICAL(self->i2s_num);
    //Reset adc scan table pointer
    SYSCON.saradc_ctrl.sar1_patt_p_clear = 1;
    self->I2S[self->i2s_num]->lc_conf.ahbm_fifo_rst = 0;
    //I2S[i2s_num]->lc_conf.in_rst = 0;
    self->I2S[self->i2s_num]->conf.rx_reset = 0;
    //Set up DMA link
    self->I2S[self->i2s_num]->in_link.addr = &i2s_adc_obj[i2s_num]->desc;
    self->I2S[self->i2s_num]->rx_eof_num = rd_len / 4;
    SYSCON.saradc_ctrl.sar1_patt_p_clear = 0;
    //Start receive ADC data
    I2S[i2s_num]->in_link.start = 1;
    I2S[i2s_num]->conf.rx_start = 1;

    #if LOG_CONV_TIME == 1
        conv_start_time = esp_timer_get_time();
    #endif

    I2S_ADC_EXIT_CRITICAL();
    return ESP_OK;
}

esp_err_t i2s_wait_adc_done(i2s_port_t i2s_num, TickType_t ticks_to_wait)
{
    I2S_ADC_CHECK((i2s_num < I2S_NUM_MAX), "i2s_num error", ESP_ERR_INVALID_ARG);
    I2S_ADC_CHECK((i2s_adc_obj[i2s_num] != NULL), "driver not installed", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_FAIL;
    if(xSemaphoreTake(i2s_adc_obj[i2s_num]->done_mux, ticks_to_wait) == pdTRUE) {
        ret = ESP_OK;
    }
    #if LOG_CONV_TIME == 1
        ESP_LOGI(I2S_ADC_TAG, "Conversion done! Total conversion time: %lld", i2s_adc_obj[i2s_num]->conversion_time);
    #endif
    return ret;
}