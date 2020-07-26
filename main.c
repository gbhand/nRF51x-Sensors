///* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
// *
// * The information contained herein is property of Nordic Semiconductor ASA.
// * Terms and conditions of usage are described in detail in NORDIC
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
// *
// * Licensees are granted free, non-transferable use of the information. NO
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
// * the file.
// *
// */
//
///** @file
// * @defgroup nrf_adc_example main.c
// * @{
// * @ingroup nrf_adc_example
// * @brief ADC Example Application main file.
// *
// * This file contains the source code for a sample application using the ADC driver.
// */
//
//#include "nrf.h"
//#include <stdbool.h>
//#include <stdint.h>
//#include <stdio.h>
//#include "nrf_drv_adc.h"
//#include "nordic_common.h"
//#include "boards.h"
//#define NRF_LOG_MODULE_NAME "APP"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "app_error.h"
//#include "app_util_platform.h"
//#include "nrf_ppi.h"
//#include "nrf_drv_timer.h"
//#include "nrf_drv_ppi.h"
//
//
//#define ADC_BUFFER_SIZE 6       //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.
//#define ADC_SAMPLE_RATE	1000     //Sets the sampling rate in ms
//
//static nrf_adc_value_t          adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
//static nrf_ppi_channel_t        m_ppi_channel;
//static const nrf_drv_timer_t    m_timer = NRF_DRV_TIMER_INSTANCE(2);
//static uint8_t                  adc_event_counter = 0;
//static uint8_t                  number_of_adc_channels;
//
///**
// * @brief ADC interrupt handler.
// */
//static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
//{
//    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
//    {
//        uint32_t i;
//        NRF_LOG_INFO("  adc event counter: %d\r\n", adc_event_counter);
//        for (i = 0; i < p_event->data.done.size; i++)
//        {
//            NRF_LOG_INFO("ADC value channel %d: %d\r\n", (i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
//        }
//    adc_event_counter++;
//    }
//    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
//
//    bsp_board_led_invert(BSP_BOARD_LED_0);
//}
//
//void timer_handler(nrf_timer_event_t event_type, void* p_context)
//{
//}
//
///**
// * @brief ADC initialization.
// */
//static void adc_config(void)
//{
//    ret_code_t ret_code;
//	
//    //Initialize ADC
//    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
//    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
//    APP_ERROR_CHECK(ret_code);
//	
//    //Configure and enable ADC channel 0
//    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); 
//    m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
//    nrf_drv_adc_channel_enable(&m_channel_0_config);
//	
//    //Configure and enable ADC channel 1
//    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_6); 
//    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
//    nrf_drv_adc_channel_enable(&m_channel_1_config);
//	
//    //Configure and enable ADC channel 2
//    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7);	
//    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
//    nrf_drv_adc_channel_enable(&m_channel_2_config);
//	
//    number_of_adc_channels = 3;    //Set equal to the number of configured ADC channels, for the sake of UART output.
//}
//
//void adc_sampling_event_enable(void)
//{
//    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
//    APP_ERROR_CHECK(err_code);
//}
//
//void adc_sampling_event_init(void)
//{
//    ret_code_t err_code;
//    err_code = nrf_drv_ppi_init();
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    timer_config.frequency = NRF_TIMER_FREQ_31250Hz;
//    err_code = nrf_drv_timer_init(&m_timer, &timer_config, timer_handler);
//    APP_ERROR_CHECK(err_code);
//
//    /* setup m_timer for compare event */
//    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer,ADC_SAMPLE_RATE);
//    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
//    nrf_drv_timer_enable(&m_timer);
//
//    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
//    uint32_t adc_sample_event_addr = nrf_drv_adc_start_task_get();
//
//    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
//    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, adc_sample_event_addr);  //NRF_ADC->TASKS_START);
//    APP_ERROR_CHECK(err_code);
//}
//
//
///**
// * @brief Function for main application entry.
// */
//int main(void)
//{
//    bsp_board_leds_init();
//
//    adc_sampling_event_init();
//    adc_config();
//    adc_sampling_event_enable();
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//
//    NRF_LOG_INFO("    ADC example\r\n");
//
//    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
//	
//    while (true)
//    {
//        __WFE();    
//        __SEV();
//        __WFE();
//        NRF_LOG_FLUSH();
//    }
//}
///** @} */






/** STATUS: ADC simple works
  *         SD Card code initializes, but not tested with hardware
  *         LSM via SPI is disabled
  *
  * NOTES:  Issues with sdk_config caused hangs
  *         watch out for conflicting defines and SPI instantiation
  *
  * TODO:   Expand ADC to all devices (and that IT WORKS)
  *         Configure ADC to be saved to SD card (do this later)
**/
 







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
   
#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nordic_common.h"

#include "nrf_drv_adc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"

#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
   


#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
   
//#define NRF_LOG_MODULE_NAME "APP"

//#define SPI_INSTANCE  0 /**< SPI instance index. */
//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
//static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
//
//// LSM6DS3 defines
//#define WHO_AM_I                0x0F
//
//#define CTRL1_XL                0x10
//#define CTRL6_C                 0x15
//
//#define OUTX_L_XL               0x28
//
//#define STATUS_REG              0x1E
//
//#define XL_POWER_DOWN           0x00
//#define XL_52Hz                 0x30
//#define XL_HM_MODE              0x08
//
//#define VERBOSE                 true
//#define LSM_ENABLE              false
//
//uint8_t tx_data[8];
//uint8_t rx_data[8];

//uint8_t spiCommandData[8];
//uint8_t spiReadData[8];
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_rx_buf - 1);        /**< Transfer length. */

// ADC defines
#define ADC_BUFFER_SIZE                 8                                            //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.
#define ADC_SAMPLE_RATE     		1000                                        //ADC sampling frequencyng frequency in ms
#define ADC_ENABLE                      true

static nrf_adc_value_t          adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_ppi_channel_t        m_ppi_channel;
static const nrf_drv_timer_t    m_timer = NRF_DRV_TIMER_INSTANCE(2);
static uint8_t                  adc_event_counter = 0;
static uint8_t                  number_of_adc_channels;

// SD Card defines
#define FILE_NAME                       "NORDIC.TXT"
#define TEST_STRING                     "SD card example.\r\n"
#define FATFS_EXAMPLE                   false
#define FATFS_ENABLE                    true

#define SDC_SCK_PIN                     9  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN                    8  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN                    10  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN                      13  ///< SDC chip select (CS) pin.

// ADC section
/**@brief ADC interrupt handler.
 * 
 * Prints ADC results on hardware UART and over BLE via the NUS service.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
  
    /** Old implementation -> NOT WORKING

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

    **/ //end
    
    /** New implementation -> TODO
    **/
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
        NRF_LOG_INFO("  adc event counter: %d\r\n", adc_event_counter);
        for (i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("ADC value channel %d: %d\r\n", (i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
        }
    adc_event_counter++;
    }
    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    
//    NRF_LOG_INFO("ADC result from channel 1: %d%d\r\n", adc_result[0], adc_result[1]);
//    NRF_LOG_INFO("ADC result from channel 2: %d%d\r\n", adc_result[2], adc_result[3]);
}

/**@brief TIMER interrupt handler.
 * 
 */
void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
}

/**@brief ADC initialization.
 * 
 */
static void adc_config(void)
{
    ret_code_t ret_code;
	
    //Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
    //Configure and enable ADC channel 0
    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); 
    m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_0_config);
	
    //Configure and enable ADC channel 1
    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_6); 
    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_1_config);
	
    //Configure and enable ADC channel 2
    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7);	
    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_2_config);
	
    number_of_adc_channels = 3;    //Set equal to the number of configured ADC channels, for the sake of UART output.
}

/**@brief ADC sampling event enable
  *
  */
void adc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/**@brief Setup ADC sampling events.
 * 
 */
void adc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_config.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&m_timer, &timer_config, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer,ADC_SAMPLE_RATE);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t adc_sample_event_addr = nrf_drv_adc_start_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, adc_sample_event_addr);  //NRF_ADC->TASKS_START);
    APP_ERROR_CHECK(err_code);
}



//// LSM6DS3 section
//
///**@brief SPI user event handler.
// * 
// * @param event
// */
//void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
//{
//    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.\r\n");
////    if (VERBOSE && rx_data != 0)
////    {
////        uint32_t result = rx_data[0];
////        NRF_LOG_INFO("EVENT_Received: %d\r\n", result);
////    }
//    if (VERBOSE && rx_data[1] != 0)
//    {
//        uint32_t result = rx_data[0];
//        NRF_LOG_INFO("EVENT_Received: %d\r\n", result);
//        result = rx_data[1];
//        NRF_LOG_INFO("EVENT_Received: %d\r\n", result);
//    }
//}
//
//void send(uint8_t sizetx, uint8_t sizerx)
//{
//  memset(rx_data, 0, sizeof(rx_data));
//  spi_xfer_done = false;
//  NRF_LOG_INFO("calling spi transfer...\r\n");
//  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, sizetx, rx_data, sizerx));
//  
//  while (!spi_xfer_done)
//  {
//    __WFE();
//  }
//  
//  if(sizerx!=0)
//  {
////	NRF_LOG_INFO("Received[0]: %x %x %x %x %x %x \n\r",m_rx_buf[0],m_rx_buf[1],m_rx_buf[2],m_rx_buf[3],m_rx_buf[4],m_rx_buf[5]);
////	NRF_LOG_INFO("Received[1]: %x %x %x %x %x %x \n\r",m_rx_buf[6],m_rx_buf[7],m_rx_buf[8],m_rx_buf[9],m_rx_buf[10],m_rx_buf[11]);
//    NRF_LOG_INFO("Received: %x\r\n", rx_data[0]);
//  }
//  
//  NRF_LOG_FLUSH();
//}
//
//void set_accel_freq(void)
//{
//    // Config accel to 52Hz
//    tx_data[0] = CTRL1_XL;      // 0x10
//    tx_data[1] = XL_52Hz;   // 0x30
//    send(2, 1);
//}
//
//void set_accel_low_power(void)
//{
//  // Enable low power mode
//    tx_data[0] = CTRL6_C;       // 0x15
//    tx_data[1] = XL_HM_MODE;    // 0x08
//    send(2, 1);
//}
//
//void power_down_accel(void)
//{
//  // Power down
//      tx_data[0] = CTRL1_XL;            // 0x10
//      tx_data[1] = XL_POWER_DOWN;       // 0x00    
//      send(2, 1);
//}
//
//void who_am_i(void)
//{
//    tx_data[0] = 0x8F;
//    tx_data[1] = 0x00;
//    memset(rx_data, 0x00, 8);
////    uint8_t spiReadData[] = {0x00, 0x00};
//    
//    nrf_drv_spi_transfer(&spi, tx_data, 2, rx_data, 2);
//    NRF_LOG_FLUSH();
//    
//    
//    // New who_am_i
//    tx_data[0] = (WHO_AM_I | 0x80);      // 0x0F
////    NRF_LOG_INFO("sending %x\r\n", tx_data[0]);
////    NRF_LOG_FLUSH();
//    send(1, 1);
//    tx_data[0] = 0x00;
////    NRF_LOG_INFO("sending %x\r\n", tx_data[0]);
////    NRF_LOG_FLUSH();
//    send(1,1);
//    
//    if (rx_data[0] == 0x69)
//    {
//      NRF_LOG_INFO("Correct who_am_i: 0x69\r\n");
//    }
//    else
//    {
//      NRF_LOG_INFO("Unknown device: %x\r\n", rx_data[0]);
//    }
//}
//
//void LSM6DS3_init(void)
//{ 
//    NRF_LOG_INFO("LSM6DS3 init begin...\r\n");
//    
////    who_am_i();
//    
//    // Enable low power mode
//    set_accel_low_power();
//    
//    // Config accel to 52Hz
//    set_accel_freq();   
//}
//
//void LSM6DS3_read_accel(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
//{
//  uint8_t status = 0;
//  uint8_t data[6];
//  
//  tx_data[0] = STATUS_REG;
//  
//  // Ensures LSM6DS3 has data ready
//  do {
//    send(1,1);
//    status = rx_data[0];
//  } while(!(status & 0x01));
//  
//  // Read data from accel register
//  tx_data[0] = OUTX_L_XL;
//  send(1,6);
//  
//  power_down_accel();
//  
//  *data = *rx_data;
//  *x_axis = (data[1] << 8) | data[0];
//  *y_axis = (data[3] << 8) | data[2];
//  *z_axis = (data[5] << 8) | data[4];
//}






// SD Card section
/**@brief  SDC block device definition
 * 
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/**@brief FATFS initialization
  *
  */
static void fatfs_init()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
//    static FIL file;

//    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.\r\n");
        return;
    }
    
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB\r\n", capacity);

    NRF_LOG_INFO("Mounting volume...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.\r\n");
        return;
    }

    NRF_LOG_INFO("\r\n Listing directory: /\r\n");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!\r\n");
        return;
    }
    
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s\r\n",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("\r\n");
    
}

/**@brief Write to FATFS
  *
  */
static void fatfs_write()
{
    static FIL file;
    
    uint32_t bytes_written;
    FRESULT ff_result;
    
  
    NRF_LOG_INFO("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.\r\n", bytes_written);
    }

    (void) f_close(&file);
    return;
}

/**@brief Function for demonstrating FAFTS usage.
 * 
 */
static void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;

    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.\r\n");
        return;
    }
    
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB\r\n", capacity);

    NRF_LOG_INFO("Mounting volume...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.\r\n");
        return;
    }

    NRF_LOG_INFO("\r\n Listing directory: /\r\n");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!\r\n");
        return;
    }
    
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s\r\n",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("\r\n");
    
    NRF_LOG_INFO("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.\r\n", bytes_written);
    }

    (void) f_close(&file);
    return;
}



// Main section
int main(void)
{
    adc_sampling_event_init();
    adc_config();
    adc_sampling_event_enable();
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    
    NRF_LOG_INFO("    ADC example\r\n");

    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    
    for (uint8_t i = 0; i < 100; i++)
    {
        __WFE();    
        __SEV();
        __WFE();
        NRF_LOG_FLUSH();
    }
    
    
//    int16_t *accel_x = 0;
//    int16_t *accel_y = 0;
//    int16_t *accel_z = 0;
//
//    
//    nrf_delay_ms(500);
//    NRF_LOG_INFO("\r\n\r\n I hope this works\r\n\r\n");
//    
//    
//    NRF_LOG_INFO("Skipping whoami...\r\n");
//    NRF_LOG_FLUSH();
//    
//    //who_am_i();
//    
//    if (LSM_ENABLE) 
//    {
//      if (rx_data[1] != 0x69)
//      {
//        NRF_LOG_INFO("Shutting down...\r\n");
//        NRF_LOG_FLUSH();
//        __WFE();
//        __SEV();
//        __WFE();
//      }
//    }
    
    
    if (FATFS_EXAMPLE && FATFS_ENABLE)
    {
        NRF_LOG_INFO("\r\nFATFS example.\r\n\r\n");
        
        fatfs_example();
    }
    else if (FATFS_ENABLE)
    {
      NRF_LOG_INFO("\r\nFATFS normal usage.\r\n\r\n");
      NRF_LOG_FLUSH();
      fatfs_init();
      fatfs_write();
      NRF_LOG_FLUSH();
    }
    

//    if (LSM_ENABLE)
//    {
//      NRF_LOG_INFO("\r\n\r\n\r\nSPI example\r\n");
//      NRF_LOG_FLUSH();
//      
//      nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//      spi_config.ss_pin   = SPI_SS_PIN;
//      spi_config.miso_pin = SPI_MISO_PIN;
//      spi_config.mosi_pin = SPI_MOSI_PIN;
//      spi_config.sck_pin  = SPI_SCK_PIN;
//      APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
//      
//      NRF_LOG_INFO("Initializing IMU\r\n");
//      NRF_LOG_FLUSH();
//      LSM6DS3_init();
//      nrf_delay_ms(500);
//      NRF_LOG_INFO("Reading IMU\r\n");
//      NRF_LOG_FLUSH();
//      LSM6DS3_read_accel(accel_x, accel_y, accel_z);
//      nrf_delay_ms(500);
//      
//      NRF_LOG_INFO("Accelerometer values read:\r\n");
//      NRF_LOG_INFO("X: %d\t Y: %d\t Z: %d\t\r\n", *accel_x, *accel_y, *accel_z);
//      NRF_LOG_FLUSH();
//    }
    
//    if (ADC_ENABLE)
//    {
//        adc_sampling_event_init();
//        adc_config();
////        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
//        adc_sampling_event_enable();
//        
//        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
//        
//        for (uint8_t i = 0; i < 100; i++)
//        {
//          __WFE();    
//          __SEV();
//          __WFE();
//          NRF_LOG_FLUSH();
//        }
//    }
    
//    // Continue sampling LSM6DS3 (and hopefully ADC too)
//    if (LSM_ENABLE)
//    {
//      for (int i = 0; i < 100; i++)
//      {
//        LSM6DS3_read_accel(accel_x, accel_y, accel_z);
//        
//        NRF_LOG_INFO("Accelerometer values read:\r\n");
//        NRF_LOG_INFO("X: %d\t Y: %d\t Z: %d\t\r\n", *accel_x, *accel_y, *accel_z);
//        NRF_LOG_FLUSH();
//        nrf_delay_ms(500);
//      }
//    }
      
    
    
    
    NRF_LOG_INFO("End of execution...\r\n");
    NRF_LOG_FLUSH();
    
    return 0;
}
