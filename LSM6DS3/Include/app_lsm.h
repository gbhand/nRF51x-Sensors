/**
  * APP_LSM.h 
  * Adapted from github.com/Martinsbl/nrf5-mpu-examples
  */
  
#ifndef APP_LSM_H__
#define APP_LSM_H__

#include <stdbool.h>
#include <stdint.h>
#include "nrf_peripherals.h"



/**@brief Enum defining Accelerometer's Full Scale range posibillities in Gs. */
enum accel_range {
    AFS_2G = 0,       // 2 G
    AFS_4G,           // 4 G
    AFS_8G,           // 8 G
    AFS_16G           // 16 G
};

/**@brief Enum defining Gyroscopes' Full Scale range posibillities in Degrees Pr Second. */
enum gyro_range {
    GFS_125DPS = 0,   // 125 deg/s
    GFS_250DPS,       // 250 deg/s
    GFS_500DPS,       // 500 deg/s
    GFS_1000DPS,      // 1000 deg/s
    GFS_2000DPS       // 2000 deg/s
};

/**@brief Structure to hold acceleromter values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}accel_values_t;


/**@brief Structure to hold gyroscope values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}gyro_values_t;

uint32_t app_lsm_init(void);

#endif
