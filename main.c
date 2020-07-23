/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "nrf_drv_adc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
   
#include "nordic_common.h"

#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
   
//#define NRF_LOG_MODULE_NAME "APP"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

// LSM6DS3 defines
#define WHO_AM_I                0x0F

#define CTRL1_XL                0x10
#define CTRL6_C                 0x15

#define OUTX_L_XL               0x28

#define STATUS_REG              0x1E

#define XL_POWER_DOWN           0x00
#define XL_52Hz                 0x30
#define XL_HM_MODE              0x08

#define VERBOSE                 false

uint8_t tx_data[2];
uint8_t rx_data[6];
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_rx_buf - 1);        /**< Transfer length. */

// ADC defines
#define ADC_BUFFER_SIZE                 8                                            //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.
#define ADC_SAMPLE_RATE     		1000                                        //ADC sampling frequencyng frequency in ms

static nrf_adc_value_t                  adc_buffer[ADC_BUFFER_SIZE];                /**< ADC buffer. */
static nrf_ppi_channel_t                m_ppi_channel;
static const nrf_drv_timer_t            m_timer = NRF_DRV_TIMER_INSTANCE(2);
static uint8_t                          adc_event_counter = 0;
static uint32_t                         number_of_adc_channels;



// LSM6DS3 section

/**@brief SPI user event handler.
 * 
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (VERBOSE && rx_data != 0)
    {
        uint32_t result = rx_data[0];
        NRF_LOG_INFO("EVENT_Received: %d\r\n", result);
    }
}

void send(uint8_t sizetx, uint8_t sizerx)
{
  memset(rx_data, 0, sizeof(rx_data));
  spi_xfer_done = false;
  
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, sizetx, rx_data, sizerx));
  
  while (!spi_xfer_done)
  {
    __WFE();
  }
  
  if(sizerx!=0)
  {
//	NRF_LOG_INFO("Received[0]: %x %x %x %x %x %x \n\r",m_rx_buf[0],m_rx_buf[1],m_rx_buf[2],m_rx_buf[3],m_rx_buf[4],m_rx_buf[5]);
//	NRF_LOG_INFO("Received[1]: %x %x %x %x %x %x \n\r",m_rx_buf[6],m_rx_buf[7],m_rx_buf[8],m_rx_buf[9],m_rx_buf[10],m_rx_buf[11]);
    NRF_LOG_INFO("Received: %x\r\n", rx_data[0]);
  }
  
  NRF_LOG_FLUSH();
}

void set_accel_freq(void)
{
    // Config accel to 52Hz
    tx_data[0] = CTRL1_XL;      // 0x10
    tx_data[1] = XL_52Hz;   // 0x30
    send(2, 1);
}

void set_accel_low_power(void)
{
  // Enable low power mode
    tx_data[0] = CTRL6_C;       // 0x15
    tx_data[1] = XL_HM_MODE;    // 0x08
    send(2, 1);
}

void power_down_accel(void)
{
  // Power down
      tx_data[0] = CTRL1_XL;            // 0x10
      tx_data[1] = XL_POWER_DOWN;       // 0x00    
      send(2, 1);
}

void who_am_i(void)
{
    // New who_am_i
    tx_data[0] = WHO_AM_I;      // 0x0F
    send(1, 1);
    
    if (rx_data[0] == 0x69)
    {
      NRF_LOG_INFO("Correct who_am_i: 0x69\r\n");
    }
    else
    {
      NRF_LOG_INFO("Unknown device: %x\r\n", rx_data[0]);
    }
}

void LSM6DS3_init(void)
{ 
    NRF_LOG_INFO("LSM6DS3 init begin...\r\n");
    
    who_am_i();
    
    // Enable low power mode
    set_accel_low_power();
    
    // Config accel to 52Hz
    set_accel_freq();   
}

void LSM6DS3_read_accel(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
{
  uint8_t status = 0;
  uint8_t data[6];
  
  tx_data[0] = STATUS_REG;
  
  // Ensures LSM6DS3 has data ready
  do {
    send(1,1);
    status = rx_data[0];
  } while(!(status & 0x01));
  
  // Read data from accel register
  tx_data[0] = OUTX_L_XL;
  send(1,6);
  
  power_down_accel();
  
  *data = *rx_data;
  *x_axis = (data[1] << 8) | data[0];
  *y_axis = (data[3] << 8) | data[2];
  *z_axis = (data[5] << 8) | data[4];
}



// ADC Section
/**@brief ADC interrupt handler.
 * 
 * Prints ADC results on hardware UART and over BLE via the NUS service.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    uint8_t adc_result[ADC_BUFFER_SIZE*2];
	
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        adc_event_counter++;
        NRF_LOG_INFO("  adc event counter: %d\r\n", adc_event_counter);
        for (uint32_t i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("ADC value channel %d: %d\r\n", (int)(i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
            adc_result[(i*2)] = p_event->data.done.p_buffer[i] >> 8;
            adc_result[(i*2)+1] = p_event->data.done.p_buffer[i];
        }
        
        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));

    }
    
    if (adc_result[0] == 0) 
    {
      NRF_LOG_INFO("huh? adc_result is zero...\r\n");
    }
//    NRF_LOG_INFO("ADC result from channel 1: %d%d\r\n", adc_result[0], adc_result[1]);
//    NRF_LOG_INFO("ADC result from channel 2: %d%d\r\n", adc_result[2], adc_result[3]);
}

/**@brief TIMER interrupt handler.
 * 
 */
void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
}

/**@brief ADC sampling event enable
  *
  */
void adc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/**@brief ADC initialization.
 * 
 */
static void adc_config(void)
{
    ret_code_t ret_code;
	
    // Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
    // Configure and enable ADC channel 0
    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); 
    m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_0_config);
	
    // Configure and enable ADC channel 1
    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_3); 
    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_1_config);
	
    // Configure and enable ADC channel 2
    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_0);	
    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_2_config);
    
    // Configure and enable ADC channel 3
    static nrf_drv_adc_channel_t m_channel_3_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_1);	
    m_channel_3_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_3_config);
	
    number_of_adc_channels = 4;    //Set equal to the number of configured ADC channels, for the sake of UART output.
}

/**@brief Setup ADC sampling events.
 * 
 */
void adc_sampling_event_init(void)
{
    // initialize PPI driver
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // create Timer instance and initialize
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer, ADC_SAMPLE_RATE);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t adc_sample_event_addr = nrf_drv_adc_start_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, adc_sample_event_addr);  //NRF_ADC->TASKS_START);
    APP_ERROR_CHECK(err_code);
}


// Main section
int main(void)
{
    int16_t *accel_x = 0;
    int16_t *accel_y = 0;
    int16_t *accel_z = 0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("\r\n\r\n\r\nSPI example\r\n");

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
    
    LSM6DS3_init();
    LSM6DS3_read_accel(accel_x, accel_y, accel_z);
    
    NRF_LOG_INFO("Accelerometer values read:\r\n");
    NRF_LOG_INFO("X: %d\t Y: %d\t Z: %d\t\r\n", *accel_x, *accel_y, *accel_z);
    
    adc_sampling_event_init();
    adc_config();
    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    adc_sampling_event_enable();
    
    // Continue sampling LSM6DS3 (and hopefully ADC too)
    for (int i = 0; i < 100; i++)
    {
      LSM6DS3_read_accel(accel_x, accel_y, accel_z);
      
      NRF_LOG_INFO("Accelerometer values read:\r\n");
      NRF_LOG_INFO("X: %d\t Y: %d\t Z: %d\t\r\n", *accel_x, *accel_y, *accel_z);
    }
      
    
    
    
    NRF_LOG_INFO("End of execution...\r\n");
    NRF_LOG_FLUSH();
    
    return 0;
}
