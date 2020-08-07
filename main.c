/** STATUS: ADC simple works
  *         SD Card code initializes
  *         LSM via SPI is WIP
  *
  * NOTES:  Issues with sdk_config caused hangs
  *         watch out for conflicting defines and SPI instantiation
  *
  * DONE:   Expand ADC to all devices (and that IT WORKS)
  *         Configure ADC to be saved to SD card
  *
  * TODO:   Successfully read WHO_AM_I on LSM6DS3
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

#include "app_lsm.h"
#include "nrf_drv_lsm.h"
   
//#include "nrf_drv_lsm.h"
//#include "app_lsm.h"
   
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

// Component enables
#define ADC_ENABLE                      false
#define FATFS_ENABLE                    false
#define LSM_ENABLE                      true


// ADC defines
#define ADC_BUFFER_SIZE                 4                                            //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.
#define ADC_SAMPLE_RATE     		10                                        //ADC sampling frequencyng frequency in ms

static nrf_adc_value_t          adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_ppi_channel_t        m_ppi_channel;
static const nrf_drv_timer_t    m_timer = NRF_DRV_TIMER_INSTANCE(2);
static uint32_t                  adc_event_counter = 0;
static uint8_t                  number_of_adc_channels;


// SD Card defines
#define FILE_NAME                       "NORDIC.TXT"
#define TEST_STRING                     "SD card example.\r\n"
#define TEST_DATA                       0x42
static uint8_t dt[2];
#define FATFS_EXAMPLE                   false


#define SDC_SCK_PIN                     9  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN                    8  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN                    10  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN                      13  ///< SDC chip select (CS) pin.

static void fatfs_write(uint8_t *to_write, uint8_t input_size);

// ADC section
/**@brief FatFs-enabled ADC interrupt handler.
 * 
 * Prints ADC results on hardware UART and over BLE via the NUS service.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    
    /** New implementation -> TODO **/
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
        NRF_LOG_RAW_INFO("Event %d |", adc_event_counter);
        for (i = 0; i < p_event->data.done.size; i++)
        {
            if (i < 4 && number_of_adc_channels)
            {
//                NRF_LOG_RAW_INFO("\t\tCh %d:\t\t%d", (i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
                NRF_LOG_RAW_INFO("\t\tCh %d:\t\t%d", i, p_event->data.done.p_buffer[i]);
                uint8_t arr[2];
                arr[0] = p_event->data.done.p_buffer[i] & 0xFF;
                arr[1] = (p_event->data.done.p_buffer[i] >> 8);
            
                fatfs_write(&arr[0], 1);
                //      TEMP:
//                if (i == 1)
//                {
//                    NRF_LOG_RAW_INFO("%d,\r\n", p_event->data.done.p_buffer[i]);
//                }
            }
        }
        NRF_LOG_RAW_INFO("\t\t\tidx = %d\r\n", i);
    adc_event_counter++;
    }
    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
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
    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_3); 
    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_1_config);
	
    //Configure and enable ADC channel 2
    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_0);	
    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_2_config);
    
    //Configure and enable ADC channel 3
    static nrf_drv_adc_channel_t m_channel_3_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_1);	
    m_channel_3_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_3_config);
	
    number_of_adc_channels = 4;    //Set equal to the number of configured ADC channels, for the sake of UART output.
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

/**@brief Write sample to FATFS
  *
  */
static void fatfs_write_test()
{
    static FIL file;
    
    uint32_t bytes_written;
    FRESULT ff_result;
    
  
    NRF_LOG_INFO("[TEST] Writing sample to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }
    
//    NRF_LOG_INFO("Attemping to write TEST_STRING:\r\n");
//    NRF_LOG_HEXDUMP_INFO(&TEST_STRING, sizeof(TEST_STRING));

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
    NRF_LOG_INFO("[TEST] Ending test...\r\n");
    return;
}

/**@brief Write to FATFS
  *
  */
static void fatfs_write(uint8_t *to_write, uint8_t input_size)
{
//    int[1] to_write = {42};
    
    static FIL file;
    
    uint32_t bytes_written;
    FRESULT ff_result;
    
//    NRF_LOG_INFO("\r\n\r\n\r\n\r\nTESTING DATA\r\n\r\ndata = %x\r\n", to_write[0]);
//    NRF_LOG_FLUSH();
    
    // testing
//    uint32_t arrsz = 4;
//    uint32_t input_size = sizeof(to_write) / sizeof(*to_write);
//    for(uint32_t i = 0; i < input_size; i++)
//    {
//      NRF_LOG_INFO("index: %d\r\n", i);
//      NRF_LOG_INFO("value: %x\r\n", to_write[i]);
//      NRF_LOG_FLUSH();
//    }
    
  
//    NRF_LOG_INFO("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_DEBUG("Unable to open or create file: " FILE_NAME ".\r\n");
        return; 
    }

//    ff_result = f_write(&file, to_write, 1, (UINT *) &bytes_written);
//    if (ff_result != FR_OK)
//    {
//        NRF_LOG_DEBUG("Write failed\r\n.");
//    }
//    else
//    {
//        NRF_LOG_DEBUG("%d bytes written.\r\n", bytes_written);
//    }
    
    for(uint32_t i = 0; i < input_size; i++)
    {
//        NRF_LOG_RAW_INFO(" (%X)", to_write[i]);
        NRF_LOG_DEBUG("Writing byte %d, value: %x\r\n.", i, to_write[i]);
        ff_result = f_write(&file, to_write, 1, (UINT *) &bytes_written);
        if (ff_result != FR_OK)
        {
            NRF_LOG_RAW_INFO(" +");
            NRF_LOG_DEBUG("Write failed for byte %d, value: %x\r\n.", i, to_write[i]);
        }
        else
        {
            NRF_LOG_RAW_INFO(" *");
            NRF_LOG_DEBUG("%d bytes written for byte %d, value: %x\r\n.", bytes_written, i, to_write[i]);
        }
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
    // ADC Section
    if (ADC_ENABLE)
    {
        adc_sampling_event_init();
        adc_config();
        adc_sampling_event_enable();
        NRF_LOG_INIT(NULL);
        NRF_LOG_INFO("ADC enabled\r\n");
    
//        NRF_LOG_INFO("    ADC example\r\n");

        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    }
    else
    {
        NRF_LOG_INIT(NULL);
        NRF_LOG_INFO("ADC disabled\r\n");
    }
    
    
    
    
    // FatFs Section
    if (FATFS_EXAMPLE && FATFS_ENABLE)
    {
        NRF_LOG_INFO("\r\nFATFS example.\r\n\r\n");
        
        fatfs_example();
    }
    else if (FATFS_ENABLE)
    {
      NRF_LOG_INFO("FatFs enabled\r\n");
      NRF_LOG_INFO("\r\nFATFS normal usage.\r\n\r\n");
      NRF_LOG_FLUSH();
      fatfs_init();
      fatfs_write_test();
      dt[0] = 0x42;
      dt[1] = 0x69;
      fatfs_write(&dt[0], 2);
      NRF_LOG_FLUSH();
    }
    else
    {
      NRF_LOG_INFO("FatFs disabled\r\n");
    }
    
    
    
    
    // LSM section
    if (LSM_ENABLE)
    {
        uint32_t err_code = 0;
        err_code = nrf_drv_lsm_init();
        if(err_code != NRF_SUCCESS) NRF_LOG_INFO("ERROR: 0x%X\r\n", err_code);
      
        
        NRF_LOG_INFO("LSM enabled\r\n");
        uint8_t result = 0;
        err_code = test_read(&result, 0x0F);
        if (err_code != NRF_SUCCESS)
        {
          NRF_LOG_INFO("ERROR: 0x%X\r\n", err_code);
        }
        
        NRF_LOG_INFO("Test read complete, output: 0x%X\r\n", result);
//        uint32_t lsm_ret = app_lsm_init();
//        if (lsm_ret == 0)
//        {
//            NRF_LOG_INFO("LSM init complete!\r\n");
//        }
//        else
//        {
//          NRF_LOG_INFO("LSM init failed, error code: %x\r\n", lsm_ret);
//        }
    }
    else 
    {
        NRF_LOG_INFO("LSM disabled\r\n");
    }
        NRF_LOG_FLUSH();
    
    
    
    NRF_LOG_INFO("\r\n\r\n*****\r\n\r\n\r\nBeginning main process\r\n\r\n\r\n*****\r\n\r\n");
    
    while (true)
    {
        __WFE();    
        __SEV();
        __WFE();
        NRF_LOG_FLUSH();
    }
}
