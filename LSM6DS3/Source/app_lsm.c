/**
  * APP_LSM.c
  * Adapted from github.com/Martinsbl/nrf5-mpu-examples
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_lsm.h"
#include "nrf_gpio.h"
#include "nrf_drv_lsm.h"
#include "nrf_error.h"
#include "nrf_peripherals.h"
#include "nrf_log.h"

uint32_t app_lsm_init(void)
{
    uint32_t err_code;
	
    // nRF-side configuration
    err_code = nrf_drv_lsm_init();
    if(err_code != NRF_SUCCESS) return err_code;

    // who are we talking to?
    uint8_t whois_value = 0;
    err_code = nrf_drv_lsm_read_registers(0x0F, &whois_value, 1);
    if (whois_value != 0x69)
    {
        NRF_LOG_INFO("Wrong WHO_AM_I code: %x\r\n", whois_value);
        return 1;
    }
    if(err_code != NRF_SUCCESS) return err_code;
    NRF_LOG_INFO("WHO_AM_I Successful!\r\n");
    
    // configure accelerometer register
    err_code = nrf_drv_lsm_write_single_register(0x10, 0x44);       // 01000100 104Hz operation, full-scale, 400Hz anti-aliasing
    if(err_code != NRF_SUCCESS) return err_code;
    
//    uint8_t reset_value = 7; // Resets gyro, accelerometer and temperature sensor signal paths.
//    err_code = nrf_drv_mpu_write_single_register(MPU_REG_SIGNAL_PATH_RESET, reset_value);
//    if(err_code != NRF_SUCCESS) return err_code;
//
//    // Chose  PLL with X axis gyroscope reference as clock source
//    err_code = nrf_drv_mpu_write_single_register(MPU_REG_PWR_MGMT_1, 1);
//    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}
