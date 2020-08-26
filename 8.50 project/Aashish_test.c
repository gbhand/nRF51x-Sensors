#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include <string.h>
#define NRF_LOG_MODULE_NAME "Dev"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// @amer Verify these correspond to the correct pins on the dev board
#define LSM_SPI_MISO_PIN        10
#define LSM_SPI_MOSI_PIN        8
#define LSM_SPI_SCL_PIN         9
#define LSM_SPI_CS_PIN          11

#define LSM_SPI_WRITE_MASK      0x00
#define LSM_SPI_READ_MASK       0x80
#define LSM_SPI_TIMEOUT         5000 // ms

// Registers from datasheet
#define LSM_WHO_AM_I            0x0F  // Value should be 0x69
#define LSM_CTRL1_XL            0x10  // Activates accl from powerdown | settings
#define LSM_CTRL2_G             0x11  // Activates gyro from powerdown | settings
#define LSM_STATUS_REG          0x1E  // 00000 | Temp | Gyro | Accel
#define LSM_CTRL6_C             0x15  // Angular rate power mode
#define LSM_CTRL7_G             0x16  // Gyro power mode
#define LSM_CTRL8_XL            0x17  // Accel power mode

#define LSM_OUTX_L_G            0x22  // Pitch (x) [7:0]
#define LSM_OUTX_H_G            0x23  // Pitch (x) [15:8]
#define LSM_OUTY_L_G            0x24  // Roll  (y) [7:0]
#define LSM_OUTY_H_G            0x25  // Roll  (y) [15:8]
#define LSM_OUTZ_L_G            0x26  // Yaw   (z) [7:0]
#define LSM_OUTZ_H_G            0x27  // Yaw   (z) [15:8]

#define LSM_OUTX_L_XL           0x28  // Accel (x) [7:0]
#define LSM_OUTX_H_XL           0x29  // Accel (x) [15:8]
#define LSM_OUTY_L_XL           0x2A  // Accel (y) [7:0]
#define LSM_OUTY_H_XL           0x2B  // Accel (y) [15:8]
#define LSM_OUTZ_L_XL           0x2C  // Accel (z) [7:0]
#define LSM_OUTZ_H_XL           0x2D  // Accel (z) [15:8]


#define SPI_INSTANCE 1
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
volatile static bool spi_tx_done = false;

#define LSM_SPI_BUFFER_LEN      12   // 12 bytes suffices for reading accelerometer and gyroscope values (2 * 3 + 2 * 3)
uint8_t spi_tx_buffer[LSM_SPI_BUFFER_SIZE];
uint8_t spi_rx_buffer[LSM_SPI_BUFFER_SIZE + 1]; // ensure termination
static const uint8_t spi_buffer_size = sizeof(spi_tx_buffer);

// Using this for the single register test
uint8_t spi_register_buffer;
static const uint8_t spi_register_buffer_size = sizeof(spi_register_buffer);


void lsm_spi_event_handler(const nrf_drv_spi_evt_t *event, void *context){
    if (event->type == NRF_DRV_SPI_EVENT_DONE){
        spi_tx_done = true;
        NRF_LOG_INFO(">> Received:\r\n");
        NRF_LOG_HEXDUMP_INFO(spi_rx_buffer, strlen((const char*)spi_rx_buffer));
    } else {
        NRF_LOG_INFO("SPI event error\r\n")
    }
}

void nrf_spi_init(void){
    NRF_LOG_INFO("LSM SPI init\r\n");
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = LSM_SPI_CS_PIN;
    spi_config.miso_pin = LSM_SPI_MISO_PIN;
    spi_config.mosi_pin = LSM_SPI_MOSI_PIN;
    spi_config.sck_pin  = LSM_SPI_SCL_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, lsm_spi_event_handler, NULL));
}

uint8_t test(void){
    uint8_t address;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_INFO("LSM TEST\r\n");
    
    nrf_spi_init();

    // Reset buffer and flag
    memset(spi_register_buffer, 0, spi_register_buffer_size);
    spi_tx_done = false;

    // Check who_am_i
    address = LSM_SPI_READ_MASK | LSM_WHO_AM_I
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &address, sizeof(address), &spi_register_buffer, spi_register_buffer_size));

    while (!spi_tx_done){
        __WFE();
    }

    NRF_LOG_FLUSH();
}
