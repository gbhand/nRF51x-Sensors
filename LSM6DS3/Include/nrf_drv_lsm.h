/**
  * NRF DRV LSM Header 
  * Adapted from github.com/Martinsbl/nrf5-mpu-examples
  */

#ifndef NRF_DRV_LSM__
#define NRF_DRV_LSM__

#include <stdbool.h>
#include <stdint.h>



/**@brief Function to initiate SPI drivers
 *
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_lsm_init(void);



/**@brief Function for writing data to an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @param[in]   length          Number of bytes to write
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_lsm_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length);



/**@brief Function for writing one byte to an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_lsm_write_single_register(uint8_t reg, uint8_t data);



/**@brief Function for reading arbitrary register(s)
 *
 * @param[in]   reg             Register to read
 * @param[in]   p_data          Pointer to place to store value(s)
 * @param[in]   length          Number of registers to read
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_lsm_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);


uint32_t test_read(uint8_t* outputPointer, uint8_t offset);



#endif /* NRF_DRV_LSM__ */



