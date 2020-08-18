#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include <string.h>
#define  NRF_LOG_MODULE_NAME "Dev"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

#define LSM_SPI_MISO_PIN        8     // BLE400: 8       Custom PCB: 10
#define LSM_SPI_MOSI_PIN        10    // BLE400: 10      Custom PCB: 9
#define LSM_SPI_SCL_PIN         9     // BLE400: 9       Custom PCB: 8
#define LSM_SPI_CS_PIN          11    // BLE400: 11      Custom PCB: 11

#define LSM_SPI_WRITE_MASK      0x00
#define LSM_SPI_READ_MASK       0x80
#define LSM_SPI_TIMEOUT         5000  // ms

// Registers from datasheet
#define LSM_WHO_AM_I            0x0F  // Value should be 0x69
#define LSM_CTRL1_XL            0x10  // Activates accl from powerdown | settings
#define LSM_CTRL2_G             0x11  // Activates gyro from powerdown | settings
#define LSM_CTRL3_C             0x12  // Software, interrupt, and endian controls
#define LSM_CTRL4_C             0x13  // Misc controls
#define LSM_CTRL5_C             0x14  // Rounding and self-test modes
#define LSM_CTRL6_C             0x15  // Angular rate power mode
#define LSM_CTRL7_G             0x16  // Gyro power mode
#define LSM_CTRL8_XL            0x17  // Accel power mode
#define LSM_CTRL9_XL            0x18  // Enable individual accel axes
#define LSM_CTRL10_C            0x19  // Enable individual gyro axes
#define LSM_STATUS_REG          0x1E  // 00000 | Temp | Gyro | Accel

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

#define LSM_SPI_BUFFER_SIZE      12   // 12 bytes suffices for reading accelerometer and gyroscope values (2 * 3 + 2 * 3)
uint8_t spi_tx_buffer[LSM_SPI_BUFFER_SIZE];
uint8_t spi_rx_buffer[LSM_SPI_BUFFER_SIZE + 1]; // ensure termination


void lsm_spi_event_handler(const nrf_drv_spi_evt_t *event){
    if (event->type == NRF_DRV_SPI_EVENT_DONE){
        spi_tx_done = true;
//        NRF_LOG_INFO(">> Received:\r\n");
//        NRF_LOG_HEXDUMP_INFO(spi_rx_buffer, strlen((const char*)spi_rx_buffer));
//        NRF_LOG_HEXDUMP_INFO(spi_rx_buffer, 2);
        
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
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    spi_config.mode      = NRF_DRV_SPI_MODE_3;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, lsm_spi_event_handler));
}

uint8_t read_register(uint8_t offset) {
    uint8_t address = LSM_SPI_READ_MASK | offset;
    
    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
    spi_tx_done = false;
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &address, 1, spi_rx_buffer, 2));
    while (!spi_tx_done){
        __WFE();
    }
    
    uint8_t value = spi_rx_buffer[1];
    return value;
}

uint16_t read_register_16bit(uint8_t offset) {
    uint8_t address = LSM_SPI_READ_MASK | offset;
    
    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
    spi_tx_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &address, 1, spi_rx_buffer, 3));
    while (!spi_tx_done){
        __WFE();
    }

    uint16_t value = spi_rx_buffer[1] | (spi_rx_buffer[2] << 8);
    return value;
}
    
uint8_t simple_write(uint8_t reg, uint8_t data) {
  uint8_t packet[2] = {reg, data};
  
  reg = reg | 0x00;
  
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, packet, 2, NULL, 0));
  while(!spi_tx_done);
  
  NRF_LOG_FLUSH();
  
  return 0;
}       

uint8_t lsm_init() {
    if (read_register(LSM_WHO_AM_I) != 0x69)
    {
        NRF_LOG_INFO("Bad WHO_AM_I\r\n");
    }
    else
    {
        NRF_LOG_INFO("WHO_AM_I Success!\r\n");
    }
    
    NRF_LOG_INFO("Current accelerometer config: 0x%X\r\n", read_register(LSM_CTRL1_XL));
    uint8_t xl_config = 0;
    // Set filter bandwidth to 100Hz
    xl_config |= 0x02;
    // Set scale to 16g
    xl_config |= 0x04;
    // Set ODR to 416Hz
    xl_config |= 0x60;
    
    NRF_LOG_INFO("Setting accelerometer config to 0x%X\r\n", xl_config);
    simple_write(LSM_CTRL1_XL, xl_config);
    NRF_LOG_INFO("New accelerometer config: 0x%X\r\n", read_register(LSM_CTRL1_XL));
    
    // Set ODR BW Scale 
    simple_write(LSM_CTRL4_C, 0x80);
    NRF_LOG_INFO("LSM_CTRL4_C: 0x%X\r\n", read_register(LSM_CTRL4_C));
    
    NRF_LOG_INFO("LSM init complete!\r\n");
    
    NRF_LOG_FLUSH();

    return 0;
}

float calc_accel(uint16_t input)
{
      float output = (float)input * 0.061 * (16 >> 1) / 1000;
      return output;
}

void print_accel_float(void)
{
    NRF_LOG_INFO("X: " NRF_LOG_FLOAT_MARKER "    Y: " NRF_LOG_FLOAT_MARKER "    Z: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(calc_accel(read_register_16bit(LSM_OUTX_L_XL))), NRF_LOG_FLOAT(calc_accel(read_register_16bit(LSM_OUTY_L_XL))), NRF_LOG_FLOAT(calc_accel(read_register_16bit(LSM_OUTZ_L_XL))));
    NRF_LOG_FLUSH();
}
    
int main(void) {
      APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
      nrf_spi_init();
      lsm_init();
      NRF_LOG_INFO("STATUS: 0x%X\r\n", read_register(LSM_STATUS_REG));
      
      nrf_delay_ms(500);
      
      uint8_t i = 15;
      while (i--) 
      {
          nrf_delay_ms(100);
          if (read_register(LSM_STATUS_REG) & 0x03 == 0x03)
          {
              print_accel_float();
          }
          else 
          {
              NRF_LOG_INFO("Bad status: 0x%X\r\n", read_register(LSM_STATUS_REG));
          }
          NRF_LOG_FLUSH();
      }
      
      nrf_delay_ms(100);
      NRF_LOG_INFO("XL config: 0x%X\r\n", read_register(LSM_CTRL1_XL));
      simple_write(LSM_CTRL1_XL, 0x00);
      nrf_delay_ms(100);
      NRF_LOG_INFO("XL config: 0x%X\r\n", read_register(LSM_CTRL1_XL));
      
      NRF_LOG_FLUSH();
      

}

































//      uint8_t i = 200;
//      while (--i);
//      i = 200;
//      NRF_LOG_INFO("X: %d    Y: %d    Z: %d\r\n", read_register(LSM_OUTX_L_XL), read_register(LSM_OUTY_L_XL), read_register(LSM_OUTZ_L_XL));
//      
//      while (--i);
//      i = 200;
//      NRF_LOG_INFO("X: %d    Y: %d    Z: %d\r\n", read_register(LSM_OUTX_L_XL), read_register(LSM_OUTY_L_XL), read_register(LSM_OUTZ_L_XL));
//      
      
      
      

      
//      NRF_LOG_INFO("\r\n\r\nPowering down IMU\r\n\r\n");
////      write_register(LSM_CTRL1_XL, 0x00);
//      read_register(LSM_CTRL1_XL);
////      read_register(LSM_OUTX_L_XL);
//      
//      NRF_LOG_INFO("END\r\n");
//      
//      NRF_LOG_FLUSH();
      
//      NRF_LOG_INFO("\r\n\r\nPowering up IMU\r\n\r\n");
////      startup();
////      write_register(LSM_CTRL1_XL, 0x40);
//      read_register(LSM_CTRL1_XL);
//      read_register(LSM_OUTX_L_XL);
//      
//      NRF_LOG_FLUSH();

//      while (1)
//      {
//          test();
//      }
//      0x104C
















// BAD FUNCTIONS

//uint8_t test(void){
//    uint8_t address;
//
////    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_INFO("LSM WHO_AM_I TEST\r\n");
//    
//    
//
//    // Reset buffer and flag
//    memset(&spi_register_buffer, 0, spi_register_buffer_size);
//    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
//    spi_tx_done = false;
//
//    // Check who_am_i
//    address = LSM_SPI_READ_MASK | LSM_WHO_AM_I;
//    
//    spi_tx_buffer[0] = address;
//    spi_tx_buffer[1] = 0x00;
//    
//    NRF_LOG_INFO("Address: 0x%X\r\n", address);
//    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buffer, 1, spi_rx_buffer, 2));
//
//    while (!spi_tx_done){
//        __WFE();
//    }
//    
//    NRF_LOG_INFO("Test complete\r\n");
//
//    NRF_LOG_FLUSH();
//    
//    return 1;
//}
//
//uint8_t readXLtest(void) {
//    uint8_t address;
//    
//    NRF_LOG_INFO("reading OUTX_L_XL\r\n");
//    
//    // Reset buffer and flag
//    memset(&spi_register_buffer, 0, spi_register_buffer_size);
//    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
//    spi_tx_done = false;
//  
//    address = LSM_SPI_READ_MASK | LSM_OUTX_L_XL;
//    
//    spi_tx_buffer[0] = address;
//    
//    NRF_LOG_INFO("Address: 0x%X\r\n", address);
//    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buffer, 1, spi_rx_buffer, 2));
//    
//    while (!spi_tx_done){
//        __WFE();
//    }
//    
//    NRF_LOG_FLUSH();
//    
//    return 1;
//}
//
//uint8_t startup(void) {
//    uint8_t address;
//    uint8_t data;
//    
////    NRF_LOG_INFO("setting LSM_CTRL1_XL to 0x40\r\n");
//    
//    // Reset buffer and flag
//    memset(&spi_register_buffer, 0, spi_register_buffer_size);
//    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
//    spi_tx_done = false;
//  
//    address = LSM_CTRL1_XL;
//    data = 0x40;
//    
//    spi_tx_buffer[0] = address;
//    spi_tx_buffer[1] = data;
//    
////    NRF_LOG_INFO("Address: 0x%X\r\n", address);
////    NRF_LOG_INFO("Data: 0x%X\r\n", data);
//    NRF_LOG_INFO("writing value 0x%X to register 0x%X\r\n", data, address);
//    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buffer, 2, spi_rx_buffer, 0));
//    
//    while (!spi_tx_done){
//        __WFE();
//    }
//    
//    NRF_LOG_FLUSH();
//    
//    return 1;
//}
//
//uint8_t write_register(uint8_t offset, uint8_t data) {
//  
//    NRF_LOG_INFO("writing value 0x%X to register 0x%X\r\n", data, offset);
//    NRF_LOG_FLUSH();
//    
//    memset(&spi_rx_buffer, 0, LSM_SPI_BUFFER_SIZE + 1);
////    memset(&spi_tx_buffer, 0, LSM_SPI_BUFFER_SIZE);
//    spi_tx_done = false;
//    
//    spi_tx_buffer[0] = offset;
//    spi_tx_buffer[1] = data;
//    
////    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buffer, 2, spi_rx_buffer, 2));
//    spi_tx_done = true;
//    
//    while (!spi_tx_done){
//        __WFE();
//    }
//    
//    NRF_LOG_FLUSH();
//    
//    return 0;
//}
