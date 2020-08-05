/**
  * NRF DRV LSM SPI 
  * Adapted from github.com/Martinsbl/nrf5-mpu-examples
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_spi.h"
#include "nrf_drv_lsm.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"

#define LSM_SPI_MISO_PIN        10
#define LSM_SPI_MOSI_PIN        8
#define LSM_SPI_SCL_PIN         9
#define LSM_SPI_CS_PIN          11

#define LSM_SPI_BUFFER_SIZE     12 // 12 bytes suffices for reading accelerometer and gyroscope values (2 * 3 + 2 * 3)
#define LSM_SPI_WRITE_BIT       0x00
#define LSM_SPI_READ_BIT        0x80
#define LSM_SPI_TIMEOUT         5000 // ms

static const nrf_drv_spi_t lsm_spi_instance = NRF_DRV_SPI_INSTANCE(1);
volatile static bool spi_tx_done = false;

uint8_t spi_tx_buffer[LSM_SPI_BUFFER_SIZE];
uint8_t spi_rx_buffer[LSM_SPI_BUFFER_SIZE];

/**@brief LSM6DS3 event handler (currently only sets xfer flag)
 *
 */
void nrf_drv_lsm_spi_event_handler(const nrf_drv_spi_evt_t *evt)
{
    if(evt->type == NRF_DRV_SPI_EVENT_DONE)
    {
        spi_tx_done = true;
    }
    else
    {
        // Something is wrong
    }
}

/**@brief Function for initializing SPI drivers for LSM6DS3 usage
 * 
 */
uint32_t nrf_drv_lsm_init(void)
{
    const nrf_drv_spi_config_t spi_lsm_config = {
        .sck_pin        = LSM_SPI_SCL_PIN,
        .mosi_pin       = LSM_SPI_MOSI_PIN,
        .miso_pin       = LSM_SPI_MISO_PIN,
        .ss_pin         = LSM_SPI_CS_PIN,
        .irq_priority   = APP_IRQ_PRIORITY_HIGH, 
        .orc            = 0xFF,                                    
        .frequency      = NRF_DRV_SPI_FREQ_1M,                     
        .mode           = NRF_DRV_SPI_MODE_0,                      
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         
    };  
    
    return nrf_drv_spi_init(&lsm_spi_instance, &spi_lsm_config, nrf_drv_lsm_spi_event_handler);
}

/**@brief Merges register address and existing data buffer (for writing)
 * 
 */
static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}

/**@brief Automatically merges register and buffer and writes to SPI instance
 * 
 */
uint32_t nrf_drv_lsm_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    
    if (length > LSM_SPI_BUFFER_SIZE - 1) // Must have space for register byte in buffer
    {
        return NRF_ERROR_DATA_SIZE;
    }
    
    uint32_t timeout = LSM_SPI_TIMEOUT;
    // Add write bit to register
    reg = reg | LSM_SPI_WRITE_BIT;
    
    merge_register_and_data(spi_tx_buffer, reg, p_data, length + 1);
    
    err_code = nrf_drv_spi_transfer(&lsm_spi_instance, spi_tx_buffer, length + 1, NULL, 0);
    if(err_code != NRF_SUCCESS) return err_code;
    
    while((!spi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    
    spi_tx_done = false;
    
    return err_code;
}

/**@brief Writes single byte to specified register
 * 
 */
uint32_t nrf_drv_lsm_write_single_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = LSM_SPI_TIMEOUT;
    
    uint8_t packet[2] = {reg, data};
    
    // Add write bit to register
    reg = reg | LSM_SPI_WRITE_BIT;
    
    err_code = nrf_drv_spi_transfer(&lsm_spi_instance, packet, 2, NULL, 0);
    if(err_code != NRF_SUCCESS) return err_code;
    
    while((!spi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    
    spi_tx_done = false;
    
    return err_code;
}

/**@brief Adds read bit (0x80) to register and reads address value (accounting for SPI artefacts)
 * 
 */
uint32_t nrf_drv_lsm_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = LSM_SPI_TIMEOUT;
    
    // Add read bit to register
    reg = reg | LSM_SPI_READ_BIT;
    
    // Read data over SPI and store incoming data in spi_rx_buffer
    err_code = nrf_drv_spi_transfer(&lsm_spi_instance, &reg, 1, spi_rx_buffer, length + 1); // Add 1 since register byte has to be clocked out before LSM returns data
    if (err_code != NRF_SUCCESS) return err_code;
    
    while((!spi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    
    spi_tx_done = false;
    
    // Copy data in spi_rx_buffer into p_data
    memcpy(p_data, &spi_rx_buffer[1], length);
    
    return NRF_SUCCESS;
}

/**@brief Another attempt at reading WHO_AM_I register using only native nRF functions
  *
  */
uint32_t test_read(uint8_t* outputPointer, uint8_t offset)
{
  // Previous attempt, disregard

//  uint32_t timeout = LSM_SPI_TIMEOUT;
//  uint8_t result = 0;
////  uint8_t numBytes = 1;
//  
//  uint8_t reg = offset | 0x80;
////  uint8_t ret_bit = 0x00; 
//  NRF_LOG_INFO("outputPointer: 0x%0X\r\noutputPointerValue: %d\r\noffset: %x\r\nreg: %x\r\n", (unsigned)&outputPointer, *outputPointer, offset, reg);
  
  
  
  // Hardcoded register attempt
  NRF_LOG_INFO("Begin transfer test\r\n");
  uint32_t err_code = 0;
  
  uint8_t tx;
  uint8_t rx;
  tx = 0x0F | 0x80;    // Address of WHO_AM_I register, should have value 0x69
                       // Read bit is 0x80
  
  err_code = nrf_drv_spi_transfer(&lsm_spi_instance, &tx, 1, &rx, 1);
  if (err_code != NRF_SUCCESS)
  {
        NRF_LOG_INFO(" Error during transfer.\r\n");
  }
  NRF_LOG_INFO("TRANSFER OUTPUT: %X\r\n", rx);
  
  // Reset RX buffer
  rx = 0;
  
  // Send dummy value to read response
  tx = 0x00;
  err_code = nrf_drv_spi_transfer(&lsm_spi_instance, &tx, 1, &rx, 1);
  if (err_code != NRF_SUCCESS)
  {
        NRF_LOG_INFO(" Error during transfer.\r\n");
  }
  NRF_LOG_INFO("TRANSFER OUTPUT: %d\r\n", rx);
  
//  err_code = nrf_drv_spi_transfer(&lsm_spi_instance, &reg, 1, &result, 1);
//  if (err_code != NRF_SUCCESS) return err_code;
//  
//  while((!spi_tx_done) && --timeout);
//  if(!timeout) return NRF_ERROR_TIMEOUT;
//  
//  spi_tx_done = false;
//  
//  NRF_LOG_INFO("result 1: %x\r\n", result);
////  
////  err_code = nrf_drv_spi_transfer(&lsm_spi_instance, &ret_bit, 1, &result, 1);
////  NRF_LOG_INFO("result 1: %x\r\n", result);
//  
//  *outputPointer = result;
  return err_code;
}