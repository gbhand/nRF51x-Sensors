

#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H
// <<< Use Configuration Wizard in Context Menu >>>\n
#ifdef USE_APP_CONFIG
#include "app_config.h"
#endif
// <h> nRF_Drivers 

//==========================================================
// <e> ADC_ENABLED - nrf_drv_adc - Driver for ADC peripheral (nRF51)
//==========================================================
#ifndef ADC_ENABLED
#define ADC_ENABLED 1
#endif
#if  ADC_ENABLED
// <o> ADC_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 

#ifndef ADC_CONFIG_IRQ_PRIORITY
#define ADC_CONFIG_IRQ_PRIORITY 3
#endif

// <e> ADC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef ADC_CONFIG_LOG_ENABLED
#define ADC_CONFIG_LOG_ENABLED 0
#endif
#if  ADC_CONFIG_LOG_ENABLED
// <o> ADC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef ADC_CONFIG_LOG_LEVEL
#define ADC_CONFIG_LOG_LEVEL 3
#endif

// <o> ADC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef ADC_CONFIG_INFO_COLOR
#define ADC_CONFIG_INFO_COLOR 0
#endif

// <o> ADC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef ADC_CONFIG_DEBUG_COLOR
#define ADC_CONFIG_DEBUG_COLOR 0
#endif

#endif //ADC_CONFIG_LOG_ENABLED
// </e>

#endif //ADC_ENABLED
// </e>

// <e> PERIPHERAL_RESOURCE_SHARING_ENABLED - nrf_drv_common - Peripheral drivers common module
//==========================================================
#ifndef PERIPHERAL_RESOURCE_SHARING_ENABLED
#define PERIPHERAL_RESOURCE_SHARING_ENABLED 0
#endif
#if  PERIPHERAL_RESOURCE_SHARING_ENABLED
// <e> COMMON_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef COMMON_CONFIG_LOG_ENABLED
#define COMMON_CONFIG_LOG_ENABLED 0
#endif
#if  COMMON_CONFIG_LOG_ENABLED
// <o> COMMON_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef COMMON_CONFIG_LOG_LEVEL
#define COMMON_CONFIG_LOG_LEVEL 3
#endif

// <o> COMMON_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMMON_CONFIG_INFO_COLOR
#define COMMON_CONFIG_INFO_COLOR 0
#endif

// <o> COMMON_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMMON_CONFIG_DEBUG_COLOR
#define COMMON_CONFIG_DEBUG_COLOR 0
#endif

#endif //COMMON_CONFIG_LOG_ENABLED
// </e>

#endif //PERIPHERAL_RESOURCE_SHARING_ENABLED
// </e>

// <e> PPI_ENABLED - nrf_drv_ppi - PPI peripheral driver
//==========================================================
#ifndef PPI_ENABLED
#define PPI_ENABLED 1
#endif
#if  PPI_ENABLED
// <e> PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PPI_CONFIG_LOG_ENABLED
#define PPI_CONFIG_LOG_ENABLED 0
#endif
#if  PPI_CONFIG_LOG_ENABLED
// <o> PPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PPI_CONFIG_LOG_LEVEL
#define PPI_CONFIG_LOG_LEVEL 3
#endif

// <o> PPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_INFO_COLOR
#define PPI_CONFIG_INFO_COLOR 0
#endif

// <o> PPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_DEBUG_COLOR
#define PPI_CONFIG_DEBUG_COLOR 0
#endif

#endif //PPI_CONFIG_LOG_ENABLED
// </e>

#endif //PPI_ENABLED
// </e>

// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver
//==========================================================
#ifndef TIMER_ENABLED
#define TIMER_ENABLED 1
#endif
#if  TIMER_ENABLED
// <o> TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
 
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef TIMER_DEFAULT_CONFIG_FREQUENCY
#define TIMER_DEFAULT_CONFIG_FREQUENCY 0
#endif

// <o> TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef TIMER_DEFAULT_CONFIG_MODE
#define TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define TIMER_DEFAULT_CONFIG_BIT_WIDTH 0
#endif

// <o> TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 

#ifndef TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 3
#endif

// <q> TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef TIMER0_ENABLED
#define TIMER0_ENABLED 0
#endif

// <q> TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 0
#endif

// <q> TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef TIMER2_ENABLED
#define TIMER2_ENABLED 1
#endif

// <q> TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef TIMER3_ENABLED
#define TIMER3_ENABLED 0
#endif

// <q> TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef TIMER4_ENABLED
#define TIMER4_ENABLED 0
#endif

// <e> TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TIMER_CONFIG_LOG_ENABLED
#define TIMER_CONFIG_LOG_ENABLED 0
#endif
#if  TIMER_CONFIG_LOG_ENABLED
// <o> TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TIMER_CONFIG_LOG_LEVEL
#define TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_INFO_COLOR
#define TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_DEBUG_COLOR
#define TIMER_CONFIG_DEBUG_COLOR 0
#endif

#endif //TIMER_CONFIG_LOG_ENABLED
// </e>

#endif //TIMER_ENABLED
// </e>

// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver
//==========================================================
#ifndef UART_ENABLED
#define UART_ENABLED 1
#endif
#if  UART_ENABLED
// <o> UART_DEFAULT_CONFIG_HWFC  - Hardware Flow Control
 
// <0=> Disabled 
// <1=> Enabled 

#ifndef UART_DEFAULT_CONFIG_HWFC
#define UART_DEFAULT_CONFIG_HWFC 0
#endif

// <o> UART_DEFAULT_CONFIG_PARITY  - Parity
 
// <0=> Excluded 
// <14=> Included 

#ifndef UART_DEFAULT_CONFIG_PARITY
#define UART_DEFAULT_CONFIG_PARITY 0
#endif

// <o> UART_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate
 
// <323584=> 1200 baud 
// <643072=> 2400 baud 
// <1290240=> 4800 baud 
// <2576384=> 9600 baud 
// <3862528=> 14400 baud 
// <5152768=> 19200 baud 
// <7716864=> 28800 baud 
// <10289152=> 38400 baud 
// <15400960=> 57600 baud 
// <20615168=> 76800 baud 
// <30924800=> 115200 baud 
// <61865984=> 230400 baud 
// <67108864=> 250000 baud 
// <121634816=> 460800 baud 
// <251658240=> 921600 baud 
// <268435456=> 57600 baud 

#ifndef UART_DEFAULT_CONFIG_BAUDRATE
#define UART_DEFAULT_CONFIG_BAUDRATE 30924800
#endif

// <o> UART_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 

#ifndef UART_DEFAULT_CONFIG_IRQ_PRIORITY
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY 3
#endif

// <q> UART_EASY_DMA_SUPPORT  - Driver supporting EasyDMA
 

#ifndef UART_EASY_DMA_SUPPORT
#define UART_EASY_DMA_SUPPORT 1
#endif

// <q> UART_LEGACY_SUPPORT  - Driver supporting Legacy mode
 

#ifndef UART_LEGACY_SUPPORT
#define UART_LEGACY_SUPPORT 1
#endif

// <e> UART0_ENABLED - Enable UART0 instance
//==========================================================
#ifndef UART0_ENABLED
#define UART0_ENABLED 1
#endif
#if  UART0_ENABLED
// <q> UART0_CONFIG_USE_EASY_DMA  - Default setting for using EasyDMA
 

#ifndef UART0_CONFIG_USE_EASY_DMA
#define UART0_CONFIG_USE_EASY_DMA 1
#endif

#endif //UART0_ENABLED
// </e>

// <e> UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef UART_CONFIG_LOG_ENABLED
#define UART_CONFIG_LOG_ENABLED 0
#endif
#if  UART_CONFIG_LOG_ENABLED
// <o> UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef UART_CONFIG_LOG_LEVEL
#define UART_CONFIG_LOG_LEVEL 3
#endif

// <o> UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_INFO_COLOR
#define UART_CONFIG_INFO_COLOR 0
#endif

// <o> UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_DEBUG_COLOR
#define UART_CONFIG_DEBUG_COLOR 0
#endif

#endif //UART_CONFIG_LOG_ENABLED
// </e>

#endif //UART_ENABLED
// </e>

// </h> 
//==========================================================

// <h> nRF_Log 

//==========================================================
// <e> NRF_LOG_ENABLED - nrf_log - Logging
//==========================================================
#ifndef NRF_LOG_ENABLED
#define NRF_LOG_ENABLED 1
#endif
#if  NRF_LOG_ENABLED
// <e> NRF_LOG_USES_COLORS - If enabled then ANSI escape code for colors is prefixed to every string
//==========================================================
#ifndef NRF_LOG_USES_COLORS
#define NRF_LOG_USES_COLORS 0
#endif
#if  NRF_LOG_USES_COLORS
// <o> NRF_LOG_COLOR_DEFAULT  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_COLOR_DEFAULT
#define NRF_LOG_COLOR_DEFAULT 0
#endif

// <o> NRF_LOG_ERROR_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_ERROR_COLOR
#define NRF_LOG_ERROR_COLOR 0
#endif

// <o> NRF_LOG_WARNING_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_WARNING_COLOR
#define NRF_LOG_WARNING_COLOR 0
#endif

#endif //NRF_LOG_USES_COLORS
// </e>

// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL 3
#endif

// <e> NRF_LOG_DEFERRED - Enable deffered logger.

// <i> Log data is buffered and can be processed in idle.
//==========================================================
#ifndef NRF_LOG_DEFERRED
#define NRF_LOG_DEFERRED 1
#endif
#if  NRF_LOG_DEFERRED
// <o> NRF_LOG_DEFERRED_BUFSIZE - Size of the buffer for logs in words. 
// <i> Must be power of 2

#ifndef NRF_LOG_DEFERRED_BUFSIZE
#define NRF_LOG_DEFERRED_BUFSIZE 256
#endif

#endif //NRF_LOG_DEFERRED
// </e>

// <q> NRF_LOG_USES_TIMESTAMP  - Enable timestamping
 

// <i> Function for getting the timestamp is provided by the user

#ifndef NRF_LOG_USES_TIMESTAMP
#define NRF_LOG_USES_TIMESTAMP 0
#endif

#endif //NRF_LOG_ENABLED
// </e>

// <h> nrf_log_backend - Logging sink

//==========================================================
// <o> NRF_LOG_BACKEND_MAX_STRING_LENGTH - Buffer for storing single output string 
// <i> Logger backend RAM usage is determined by this value.

#ifndef NRF_LOG_BACKEND_MAX_STRING_LENGTH
#define NRF_LOG_BACKEND_MAX_STRING_LENGTH 256
#endif

// <o> NRF_LOG_TIMESTAMP_DIGITS - Number of digits for timestamp 
// <i> If higher resolution timestamp source is used it might be needed to increase that

#ifndef NRF_LOG_TIMESTAMP_DIGITS
#define NRF_LOG_TIMESTAMP_DIGITS 8
#endif

// <e> NRF_LOG_BACKEND_SERIAL_USES_UART - If enabled data is printed over UART
//==========================================================
#ifndef NRF_LOG_BACKEND_SERIAL_USES_UART
#define NRF_LOG_BACKEND_SERIAL_USES_UART 0
#endif
#if  NRF_LOG_BACKEND_SERIAL_USES_UART
// <o> NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE  - Default Baudrate
 
// <323584=> 1200 baud 
// <643072=> 2400 baud 
// <1290240=> 4800 baud 
// <2576384=> 9600 baud 
// <3862528=> 14400 baud 
// <5152768=> 19200 baud 
// <7716864=> 28800 baud 
// <10289152=> 38400 baud 
// <15400960=> 57600 baud 
// <20615168=> 76800 baud 
// <30924800=> 115200 baud 
// <61865984=> 230400 baud 
// <67108864=> 250000 baud 
// <121634816=> 460800 baud 
// <251658240=> 921600 baud 
// <268435456=> 57600 baud 

#ifndef NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE
#define NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE 30924800
#endif

// <o> NRF_LOG_BACKEND_SERIAL_UART_TX_PIN - UART TX pin 
#ifndef NRF_LOG_BACKEND_SERIAL_UART_TX_PIN
#define NRF_LOG_BACKEND_SERIAL_UART_TX_PIN 9
#endif

// <o> NRF_LOG_BACKEND_SERIAL_UART_RX_PIN - UART RX pin 
#ifndef NRF_LOG_BACKEND_SERIAL_UART_RX_PIN
#define NRF_LOG_BACKEND_SERIAL_UART_RX_PIN 11
#endif

// <o> NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN - UART RTS pin 
#ifndef NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN
#define NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN 8
#endif

// <o> NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN - UART CTS pin 
#ifndef NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN
#define NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN 10
#endif

// <o> NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL  - Hardware Flow Control
 
// <0=> Disabled 
// <1=> Enabled 

#ifndef NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL
#define NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL 0
#endif

// <o> NRF_LOG_BACKEND_UART_INSTANCE  - UART instance used
 
// <0=> 0 

#ifndef NRF_LOG_BACKEND_UART_INSTANCE
#define NRF_LOG_BACKEND_UART_INSTANCE 0
#endif

#endif //NRF_LOG_BACKEND_SERIAL_USES_UART
// </e>

// <e> NRF_LOG_BACKEND_SERIAL_USES_RTT - If enabled data is printed using RTT
//==========================================================
#ifndef NRF_LOG_BACKEND_SERIAL_USES_RTT
#define NRF_LOG_BACKEND_SERIAL_USES_RTT 1
#endif
#if  NRF_LOG_BACKEND_SERIAL_USES_RTT
// <o> NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE - RTT output buffer size. 
// <i> Should be equal or bigger than \ref NRF_LOG_BACKEND_MAX_STRING_LENGTH.
// <i> This value is used in Segger RTT configuration to set the buffer size
// <i> if it is bigger than default RTT buffer size.

#ifndef NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE
#define NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE 512
#endif

#endif //NRF_LOG_BACKEND_SERIAL_USES_RTT
// </e>

// </h> 
//==========================================================

// </h> 
//==========================================================

// <h> nRF_Segger_RTT 

//==========================================================
// <h> segger_rtt - SEGGER RTT

//==========================================================
// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_UP - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_UP
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 64
#endif

// <o> SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS
#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS 2
#endif

// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN 16
#endif

// <o> SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS
#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS 2
#endif

// </h> 
//==========================================================
// </h>

// Added by Geoffrey
// <h> nRF_Libraries 


//==========================================================
// <q> APP_FIFO_ENABLED  - app_fifo - Software FIFO implementation
 

#ifndef APP_FIFO_ENABLED
#define APP_FIFO_ENABLED 1
#endif

// <e> APP_SDCARD_ENABLED - app_sdcard - SD/MMC card support using SPI
//==========================================================
#ifndef APP_SDCARD_ENABLED
#define APP_SDCARD_ENABLED 1
#endif
#if  APP_SDCARD_ENABLED
// <o> APP_SDCARD_SPI_INSTANCE  - SPI instance used
 
// <0=> 0 
// <1=> 1 
// <2=> 2 

#ifndef APP_SDCARD_SPI_INSTANCE
#define APP_SDCARD_SPI_INSTANCE 0
#endif

// <o> APP_SDCARD_FREQ_INIT  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef APP_SDCARD_FREQ_INIT
#define APP_SDCARD_FREQ_INIT 67108864
#endif

// <o> APP_SDCARD_FREQ_DATA  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef APP_SDCARD_FREQ_DATA
#define APP_SDCARD_FREQ_DATA 1073741824
#endif

#endif //APP_SDCARD_ENABLED
// </e>

// <e> APP_UART_ENABLED - app_uart - UART driver
//==========================================================
#ifndef APP_UART_ENABLED
#define APP_UART_ENABLED 1
#endif
#if  APP_UART_ENABLED
// <o> APP_UART_DRIVER_INSTANCE  - UART instance used
 
// <0=> 0 

#ifndef APP_UART_DRIVER_INSTANCE
#define APP_UART_DRIVER_INSTANCE 0
#endif

#endif //APP_UART_ENABLED
// </e>

// <e> SPI_ENABLED - nrf_drv_spi - SPI/SPIM peripheral driver
//==========================================================
#ifndef SPI_ENABLED
#define SPI_ENABLED 1
#endif
#if  SPI_ENABLED
// <o> SPI_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 

#ifndef SPI_DEFAULT_CONFIG_IRQ_PRIORITY
#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY 3
#endif

// <e> SPI0_ENABLED - Enable SPI0 instance
//==========================================================
#ifndef SPI0_ENABLED
#define SPI0_ENABLED 1
#endif
#if  SPI0_ENABLED
// <q> SPI0_USE_EASY_DMA  - Use EasyDMA
 

#ifndef SPI0_USE_EASY_DMA
#define SPI0_USE_EASY_DMA 0
#endif

// <o> SPI0_DEFAULT_FREQUENCY  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef SPI0_DEFAULT_FREQUENCY
#define SPI0_DEFAULT_FREQUENCY 1073741824
#endif

#endif //SPI0_ENABLED
// </e>

// <e> SPI1_ENABLED - Enable SPI1 instance
//==========================================================
#ifndef SPI1_ENABLED
#define SPI1_ENABLED 1
#endif
#if  SPI1_ENABLED
// <q> SPI1_USE_EASY_DMA  - Use EasyDMA
 

#ifndef SPI1_USE_EASY_DMA
#define SPI1_USE_EASY_DMA 0
#endif

// <o> SPI1_DEFAULT_FREQUENCY  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef SPI1_DEFAULT_FREQUENCY
#define SPI1_DEFAULT_FREQUENCY 1073741824
#endif

#endif //SPI1_ENABLED
// </e>

// <e> SPI2_ENABLED - Enable SPI2 instance
//==========================================================
#ifndef SPI2_ENABLED
#define SPI2_ENABLED 0
#endif
#if  SPI2_ENABLED
// <q> SPI2_USE_EASY_DMA  - Use EasyDMA
 

#ifndef SPI2_USE_EASY_DMA
#define SPI2_USE_EASY_DMA 0
#endif

// <q> SPI2_DEFAULT_FREQUENCY  - Use EasyDMA
 

#ifndef SPI2_DEFAULT_FREQUENCY
#define SPI2_DEFAULT_FREQUENCY 0
#endif

#endif //SPI2_ENABLED
// </e>

// <e> SPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SPI_CONFIG_LOG_ENABLED
#define SPI_CONFIG_LOG_ENABLED 0
#endif
#if  SPI_CONFIG_LOG_ENABLED
// <o> SPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SPI_CONFIG_LOG_LEVEL
#define SPI_CONFIG_LOG_LEVEL 3
#endif

// <o> SPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_INFO_COLOR
#define SPI_CONFIG_INFO_COLOR 0
#endif

// <o> SPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_DEBUG_COLOR
#define SPI_CONFIG_DEBUG_COLOR 0
#endif

#endif //SPI_CONFIG_LOG_ENABLED
// </e>

#endif //SPI_ENABLED
// </e>



// </h> 
//==========================================================

// <<< end of configuration section >>>
#endif //SDK_CONFIG_H














//
//
//#ifndef SDK_CONFIG_H
//#define SDK_CONFIG_H
//// <<< Use Configuration Wizard in Context Menu >>>\n
//#ifdef USE_APP_CONFIG
//#include "app_config.h"
//#endif
//// <h> Application 
//
////==========================================================
//// <h> SPI_CONFIGURATION - Spi configuration
//
////==========================================================
//// <o> SPI_SCK_PIN - Pin number  <0-31> 
//
//
//#ifndef SPI_SCK_PIN
//#define SPI_SCK_PIN 9 // SCK 9 alt 13
//#endif
//
//// <o> SPI_MISO_PIN - Pin number  <0-31> 
//
//
//#ifndef SPI_MISO_PIN
//#define SPI_MISO_PIN 10 // SDO 10 alt 15
//#endif
//
//// <o> SPI_MOSI_PIN - Pin number  <0-31> 
//
//
//#ifndef SPI_MOSI_PIN
//#define SPI_MOSI_PIN 8 // SDA 8 alt 14
//#endif
//
//// <o> SPI_SS_PIN - Pin number  <0-31> 
//
//
//#ifndef SPI_SS_PIN
//#define SPI_SS_PIN 11 // SCS 11 alt 16
//#endif
//
//// <o> SPI_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef SPI_IRQ_PRIORITY
//#define SPI_IRQ_PRIORITY 3
//#endif
//
//// </h> 
////==========================================================
//
//// <h> nRF_Drivers 
//
////==========================================================
//// <e> ADC_ENABLED - nrf_drv_adc - Driver for ADC peripheral (nRF51)
////==========================================================
//#ifndef ADC_ENABLED
//#define ADC_ENABLED 1
//#endif
//#if  ADC_ENABLED
//// <o> ADC_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef ADC_CONFIG_IRQ_PRIORITY
//#define ADC_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <e> ADC_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef ADC_CONFIG_LOG_ENABLED
//#define ADC_CONFIG_LOG_ENABLED 1
//#endif
//#if  ADC_CONFIG_LOG_ENABLED
//// <o> ADC_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef ADC_CONFIG_LOG_LEVEL
//#define ADC_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> ADC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef ADC_CONFIG_INFO_COLOR
//#define ADC_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> ADC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef ADC_CONFIG_DEBUG_COLOR
//#define ADC_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //ADC_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //ADC_ENABLED
//// </e>
//
//// <e> CLOCK_ENABLED - nrf_drv_clock - CLOCK peripheral driver
////==========================================================
//#ifndef CLOCK_ENABLED
//#define CLOCK_ENABLED 1
//#endif
//#if  CLOCK_ENABLED
//// <o> CLOCK_CONFIG_XTAL_FREQ  - HF XTAL Frequency
// 
//// <0=> Default (64 MHz) 
//// <255=> Default (16 MHz) 
//// <0=> 32 MHz 
//
//#ifndef CLOCK_CONFIG_XTAL_FREQ
//#define CLOCK_CONFIG_XTAL_FREQ 255
//#endif
//
//// <o> CLOCK_CONFIG_LF_SRC  - LF Clock Source
// 
//// <0=> RC 
//// <1=> XTAL 
//// <2=> Synth 
//
//#ifndef CLOCK_CONFIG_LF_SRC
//#define CLOCK_CONFIG_LF_SRC 1
//#endif
//
//// <o> CLOCK_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef CLOCK_CONFIG_IRQ_PRIORITY
//#define CLOCK_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <e> CLOCK_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef CLOCK_CONFIG_LOG_ENABLED
//#define CLOCK_CONFIG_LOG_ENABLED 0
//#endif
//#if  CLOCK_CONFIG_LOG_ENABLED
//// <o> CLOCK_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef CLOCK_CONFIG_LOG_LEVEL
//#define CLOCK_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> CLOCK_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef CLOCK_CONFIG_INFO_COLOR
//#define CLOCK_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> CLOCK_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef CLOCK_CONFIG_DEBUG_COLOR
//#define CLOCK_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //CLOCK_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //CLOCK_ENABLED
//// </e>
//
////==========================================================
//// <e> GPIOTE_ENABLED - nrf_drv_gpiote - GPIOTE peripheral driver
////==========================================================
//#ifndef GPIOTE_ENABLED
//#define GPIOTE_ENABLED 1
//#endif
//#if  GPIOTE_ENABLED
//// <o> GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS - Number of lower power input pins 
//#ifndef GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
//#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 4
//#endif
//
//// <o> GPIOTE_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef GPIOTE_CONFIG_IRQ_PRIORITY
//#define GPIOTE_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <e> GPIOTE_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef GPIOTE_CONFIG_LOG_ENABLED
//#define GPIOTE_CONFIG_LOG_ENABLED 0
//#endif
//#if  GPIOTE_CONFIG_LOG_ENABLED
//// <o> GPIOTE_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef GPIOTE_CONFIG_LOG_LEVEL
//#define GPIOTE_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> GPIOTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef GPIOTE_CONFIG_INFO_COLOR
//#define GPIOTE_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> GPIOTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef GPIOTE_CONFIG_DEBUG_COLOR
//#define GPIOTE_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //GPIOTE_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //GPIOTE_ENABLED
//// </e>
//
//// <e> PERIPHERAL_RESOURCE_SHARING_ENABLED - nrf_drv_common - Peripheral drivers common module
////==========================================================
//#ifndef PERIPHERAL_RESOURCE_SHARING_ENABLED
//#define PERIPHERAL_RESOURCE_SHARING_ENABLED 0
//#endif
//#if  PERIPHERAL_RESOURCE_SHARING_ENABLED
//// <e> COMMON_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef COMMON_CONFIG_LOG_ENABLED
//#define COMMON_CONFIG_LOG_ENABLED 0
//#endif
//#if  COMMON_CONFIG_LOG_ENABLED
//// <o> COMMON_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef COMMON_CONFIG_LOG_LEVEL
//#define COMMON_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> COMMON_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef COMMON_CONFIG_INFO_COLOR
//#define COMMON_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> COMMON_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef COMMON_CONFIG_DEBUG_COLOR
//#define COMMON_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //COMMON_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //PERIPHERAL_RESOURCE_SHARING_ENABLED
//// </e>
//
//// <e> PPI_ENABLED - nrf_drv_ppi - PPI peripheral driver
////==========================================================
//#ifndef PPI_ENABLED
//#define PPI_ENABLED 1
//#endif
//#if  PPI_ENABLED
//// <e> PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef PPI_CONFIG_LOG_ENABLED
//#define PPI_CONFIG_LOG_ENABLED 0
//#endif
//#if  PPI_CONFIG_LOG_ENABLED
//// <o> PPI_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef PPI_CONFIG_LOG_LEVEL
//#define PPI_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> PPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef PPI_CONFIG_INFO_COLOR
//#define PPI_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> PPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef PPI_CONFIG_DEBUG_COLOR
//#define PPI_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //PPI_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //PPI_ENABLED
//// </e>
//
//// <e> SPI_ENABLED - nrf_drv_spi - SPI/SPIM peripheral driver
////==========================================================
//#ifndef SPI_ENABLED
//#define SPI_ENABLED 1
//#endif
//#if  SPI_ENABLED
//// <o> SPI_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef SPI_DEFAULT_CONFIG_IRQ_PRIORITY
//#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <e> SPI0_ENABLED - Enable SPI0 instance
////==========================================================
//#ifndef SPI0_ENABLED
//#define SPI0_ENABLED 1
//#endif
//#if  SPI0_ENABLED
//// <q> SPI0_USE_EASY_DMA  - Use EasyDMA
// 
//
//#ifndef SPI0_USE_EASY_DMA
//#define SPI0_USE_EASY_DMA 0
//#endif
//
//// <o> SPI0_DEFAULT_FREQUENCY  - SPI frequency
// 
//// <33554432=> 125 kHz 
//// <67108864=> 250 kHz 
//// <134217728=> 500 kHz 
//// <268435456=> 1 MHz 
//// <536870912=> 2 MHz 
//// <1073741824=> 4 MHz 
//// <2147483648=> 8 MHz 
//
//#ifndef SPI0_DEFAULT_FREQUENCY
//#define SPI0_DEFAULT_FREQUENCY 1073741824
//#endif
//
//#endif //SPI0_ENABLED
//// </e>
//
//// <e> SPI1_ENABLED - Enable SPI1 instance
////==========================================================
//#ifndef SPI1_ENABLED
//#define SPI1_ENABLED 0
//#endif
//#if  SPI1_ENABLED
//// <q> SPI1_USE_EASY_DMA  - Use EasyDMA
// 
//
//#ifndef SPI1_USE_EASY_DMA
//#define SPI1_USE_EASY_DMA 0
//#endif
//
//// <o> SPI1_DEFAULT_FREQUENCY  - SPI frequency
// 
//// <33554432=> 125 kHz 
//// <67108864=> 250 kHz 
//// <134217728=> 500 kHz 
//// <268435456=> 1 MHz 
//// <536870912=> 2 MHz 
//// <1073741824=> 4 MHz 
//// <2147483648=> 8 MHz 
//
//#ifndef SPI1_DEFAULT_FREQUENCY
//#define SPI1_DEFAULT_FREQUENCY 1073741824
//#endif
//
//#endif //SPI1_ENABLED
//// </e>
//
//// <e> SPI2_ENABLED - Enable SPI2 instance
////==========================================================
//#ifndef SPI2_ENABLED
//#define SPI2_ENABLED 0
//#endif
//#if  SPI2_ENABLED
//// <q> SPI2_USE_EASY_DMA  - Use EasyDMA
// 
//
//#ifndef SPI2_USE_EASY_DMA
//#define SPI2_USE_EASY_DMA 0
//#endif
//
//// <q> SPI2_DEFAULT_FREQUENCY  - Use EasyDMA
// 
//
//#ifndef SPI2_DEFAULT_FREQUENCY
//#define SPI2_DEFAULT_FREQUENCY 0
//#endif
//
//#endif //SPI2_ENABLED
//// </e>
//
//// <e> SPI_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef SPI_CONFIG_LOG_ENABLED
//#define SPI_CONFIG_LOG_ENABLED 0
//#endif
//#if  SPI_CONFIG_LOG_ENABLED
//// <o> SPI_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef SPI_CONFIG_LOG_LEVEL
//#define SPI_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> SPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef SPI_CONFIG_INFO_COLOR
//#define SPI_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> SPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef SPI_CONFIG_DEBUG_COLOR
//#define SPI_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //SPI_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //SPI_ENABLED
//// </e>
//
//// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver
////==========================================================
//#ifndef TIMER_ENABLED
//#define TIMER_ENABLED 1
//#endif
//#if  TIMER_ENABLED
//// <o> TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
// 
//// <0=> 16 MHz 
//// <1=> 8 MHz 
//// <2=> 4 MHz 
//// <3=> 2 MHz 
//// <4=> 1 MHz 
//// <5=> 500 kHz 
//// <6=> 250 kHz 
//// <7=> 125 kHz 
//// <8=> 62.5 kHz 
//// <9=> 31.25 kHz 
//
//#ifndef TIMER_DEFAULT_CONFIG_FREQUENCY
//#define TIMER_DEFAULT_CONFIG_FREQUENCY 9
//#endif
//
//// <o> TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
// 
//// <0=> Timer 
//// <1=> Counter 
//
//#ifndef TIMER_DEFAULT_CONFIG_MODE
//#define TIMER_DEFAULT_CONFIG_MODE 0
//#endif
//
//// <o> TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
// 
//// <0=> 16 bit 
//// <1=> 8 bit 
//// <2=> 24 bit 
//// <3=> 32 bit 
//
//#ifndef TIMER_DEFAULT_CONFIG_BIT_WIDTH
//#define TIMER_DEFAULT_CONFIG_BIT_WIDTH 0
//#endif
//
//// <o> TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
//#define TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <q> TIMER0_ENABLED  - Enable TIMER0 instance
// 
//
//#ifndef TIMER0_ENABLED
//#define TIMER0_ENABLED 1
//#endif
//
//// <q> TIMER1_ENABLED  - Enable TIMER1 instance
// 
//
//#ifndef TIMER1_ENABLED
//#define TIMER1_ENABLED 0
//#endif
//
//// <q> TIMER2_ENABLED  - Enable TIMER2 instance
// 
//
//#ifndef TIMER2_ENABLED
//#define TIMER2_ENABLED 0
//#endif
//
//// <q> TIMER3_ENABLED  - Enable TIMER3 instance
// 
//
//#ifndef TIMER3_ENABLED
//#define TIMER3_ENABLED 0
//#endif
//
//// <q> TIMER4_ENABLED  - Enable TIMER4 instance
// 
//
//#ifndef TIMER4_ENABLED
//#define TIMER4_ENABLED 0
//#endif
//
//// <e> TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef TIMER_CONFIG_LOG_ENABLED
//#define TIMER_CONFIG_LOG_ENABLED 0
//#endif
//#if  TIMER_CONFIG_LOG_ENABLED
//// <o> TIMER_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef TIMER_CONFIG_LOG_LEVEL
//#define TIMER_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef TIMER_CONFIG_INFO_COLOR
//#define TIMER_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef TIMER_CONFIG_DEBUG_COLOR
//#define TIMER_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //TIMER_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //TIMER_ENABLED
//// </e>
//
//// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver
////==========================================================
//#ifndef UART_ENABLED
//#define UART_ENABLED 1
//#endif
//#if  UART_ENABLED
//// <o> UART_DEFAULT_CONFIG_HWFC  - Hardware Flow Control
// 
//// <0=> Disabled 
//// <1=> Enabled 
//
//#ifndef UART_DEFAULT_CONFIG_HWFC
//#define UART_DEFAULT_CONFIG_HWFC 0
//#endif
//
//// <o> UART_DEFAULT_CONFIG_PARITY  - Parity
// 
//// <0=> Excluded 
//// <14=> Included 
//
//#ifndef UART_DEFAULT_CONFIG_PARITY
//#define UART_DEFAULT_CONFIG_PARITY 0
//#endif
//
//// <o> UART_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate
// 
//// <323584=> 1200 baud 
//// <643072=> 2400 baud 
//// <1290240=> 4800 baud 
//// <2576384=> 9600 baud 
//// <3862528=> 14400 baud 
//// <5152768=> 19200 baud 
//// <7716864=> 28800 baud 
//// <10289152=> 38400 baud 
//// <15400960=> 57600 baud 
//// <20615168=> 76800 baud 
//// <30924800=> 115200 baud 
//// <61865984=> 230400 baud 
//// <67108864=> 250000 baud 
//// <121634816=> 460800 baud 
//// <251658240=> 921600 baud 
//// <268435456=> 57600 baud 
//
//#ifndef UART_DEFAULT_CONFIG_BAUDRATE
//#define UART_DEFAULT_CONFIG_BAUDRATE 30924800
//#endif
//
//// <o> UART_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
// 
//
//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest) 
//// <1=> 1 
//// <2=> 2 
//// <3=> 3 
//
//#ifndef UART_DEFAULT_CONFIG_IRQ_PRIORITY
//#define UART_DEFAULT_CONFIG_IRQ_PRIORITY 3
//#endif
//
//// <q> UART_EASY_DMA_SUPPORT  - Driver supporting EasyDMA
// 
//
//#ifndef UART_EASY_DMA_SUPPORT
//#define UART_EASY_DMA_SUPPORT 1
//#endif
//
//// <q> UART_LEGACY_SUPPORT  - Driver supporting Legacy mode
// 
//
//#ifndef UART_LEGACY_SUPPORT
//#define UART_LEGACY_SUPPORT 1
//#endif
//
//// <e> UART0_ENABLED - Enable UART0 instance
////==========================================================
//#ifndef UART0_ENABLED
//#define UART0_ENABLED 1
//#endif
//#if  UART0_ENABLED
//// <q> UART0_CONFIG_USE_EASY_DMA  - Default setting for using EasyDMA
// 
//
//#ifndef UART0_CONFIG_USE_EASY_DMA
//#define UART0_CONFIG_USE_EASY_DMA 1
//#endif
//
//#endif //UART0_ENABLED
//// </e>
//
//// <e> UART_CONFIG_LOG_ENABLED - Enables logging in the module.
////==========================================================
//#ifndef UART_CONFIG_LOG_ENABLED
//#define UART_CONFIG_LOG_ENABLED 0
//#endif
//#if  UART_CONFIG_LOG_ENABLED
//// <o> UART_CONFIG_LOG_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef UART_CONFIG_LOG_LEVEL
//#define UART_CONFIG_LOG_LEVEL 3
//#endif
//
//// <o> UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef UART_CONFIG_INFO_COLOR
//#define UART_CONFIG_INFO_COLOR 0
//#endif
//
//// <o> UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef UART_CONFIG_DEBUG_COLOR
//#define UART_CONFIG_DEBUG_COLOR 0
//#endif
//
//#endif //UART_CONFIG_LOG_ENABLED
//// </e>
//
//#endif //UART_ENABLED
//// </e>
//
//// </h> 
////==========================================================
//
//// <h> nRF_Libraries 
//
////==========================================================
//// <e> APP_SDCARD_ENABLED - app_sdcard - SD/MMC card support using SPI
////==========================================================
//#ifndef APP_SDCARD_ENABLED
//#define APP_SDCARD_ENABLED 1
//#endif
//#if  APP_SDCARD_ENABLED
//// <o> APP_SDCARD_SPI_INSTANCE  - SPI instance used
// 
//// <0=> 0 
//// <1=> 1 
//// <2=> 2 
//
//#ifndef APP_SDCARD_SPI_INSTANCE
//#define APP_SDCARD_SPI_INSTANCE 0
//#endif
//
//// <o> APP_SDCARD_FREQ_INIT  - SPI frequency
// 
//// <33554432=> 125 kHz 
//// <67108864=> 250 kHz 
//// <134217728=> 500 kHz 
//// <268435456=> 1 MHz 
//// <536870912=> 2 MHz 
//// <1073741824=> 4 MHz 
//// <2147483648=> 8 MHz 
//
//#ifndef APP_SDCARD_FREQ_INIT
//#define APP_SDCARD_FREQ_INIT 67108864
//#endif
//
//// <o> APP_SDCARD_FREQ_DATA  - SPI frequency
// 
//// <33554432=> 125 kHz 
//// <67108864=> 250 kHz 
//// <134217728=> 500 kHz 
//// <268435456=> 1 MHz 
//// <536870912=> 2 MHz 
//// <1073741824=> 4 MHz 
//// <2147483648=> 8 MHz 
//
//#ifndef APP_SDCARD_FREQ_DATA
//#define APP_SDCARD_FREQ_DATA 1073741824
//#endif
//
//#endif //APP_SDCARD_ENABLED
//// </e>
//
//// <e> APP_TIMER_ENABLED - app_timer - Application timer functionality
////==========================================================
//#ifndef APP_TIMER_ENABLED
//#define APP_TIMER_ENABLED 1
//#endif
//#if  APP_TIMER_ENABLED
//// <q> APP_TIMER_WITH_PROFILER  - Enable app_timer profiling
// 
//
//#ifndef APP_TIMER_WITH_PROFILER
//#define APP_TIMER_WITH_PROFILER 0
//#endif
//
//// <q> APP_TIMER_KEEPS_RTC_ACTIVE  - Enable RTC always on
// 
//
//// <i> If option is enabled RTC is kept running even if there is no active timers.
//// <i> This option can be used when app_timer is used for timestamping.
//
//#ifndef APP_TIMER_KEEPS_RTC_ACTIVE
//#define APP_TIMER_KEEPS_RTC_ACTIVE 0
//#endif
//
//#endif //APP_TIMER_ENABLED
//// </e>
//
//// <q> BUTTON_ENABLED  - app_button - buttons handling module
////==========================================================
//#ifndef BUTTON_ENABLED
//#define BUTTON_ENABLED 1
//#endif
//// </e> //BUTTON_ENABLED
//
//// </h> 
////==========================================================
//
//// <h> nRF_Log 
//
////==========================================================
//// <e> NRF_LOG_ENABLED - nrf_log - Logging
////==========================================================
//#ifndef NRF_LOG_ENABLED
//#define NRF_LOG_ENABLED 1
//#endif
//#if  NRF_LOG_ENABLED
//// <e> NRF_LOG_USES_COLORS - If enabled then ANSI escape code for colors is prefixed to every string
////==========================================================
//#ifndef NRF_LOG_USES_COLORS
//#define NRF_LOG_USES_COLORS 1
//#endif
//#if  NRF_LOG_USES_COLORS
//// <o> NRF_LOG_COLOR_DEFAULT  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef NRF_LOG_COLOR_DEFAULT
//#define NRF_LOG_COLOR_DEFAULT 0
//#endif
//
//// <o> NRF_LOG_ERROR_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef NRF_LOG_ERROR_COLOR
//#define NRF_LOG_ERROR_COLOR 0
//#endif
//
//// <o> NRF_LOG_WARNING_COLOR  - ANSI escape code prefix.
// 
//// <0=> Default 
//// <1=> Black 
//// <2=> Red 
//// <3=> Green 
//// <4=> Yellow 
//// <5=> Blue 
//// <6=> Magenta 
//// <7=> Cyan 
//// <8=> White 
//
//#ifndef NRF_LOG_WARNING_COLOR
//#define NRF_LOG_WARNING_COLOR 0
//#endif
//
//#endif //NRF_LOG_USES_COLORS
//// </e>
//
//// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level
// 
//// <0=> Off 
//// <1=> Error 
//// <2=> Warning 
//// <3=> Info 
//// <4=> Debug 
//
//#ifndef NRF_LOG_DEFAULT_LEVEL
//#define NRF_LOG_DEFAULT_LEVEL 3
//#endif
//
//// <e> NRF_LOG_DEFERRED - Enable deffered logger.
//
//// <i> Log data is buffered and can be processed in idle.
////==========================================================
//#ifndef NRF_LOG_DEFERRED
//#define NRF_LOG_DEFERRED 1
//#endif
//#if  NRF_LOG_DEFERRED
//// <o> NRF_LOG_DEFERRED_BUFSIZE - Size of the buffer for logs in words. 
//// <i> Must be power of 2
//
//#ifndef NRF_LOG_DEFERRED_BUFSIZE
//#define NRF_LOG_DEFERRED_BUFSIZE 256
//#endif
//
//#endif //NRF_LOG_DEFERRED
//// </e>
//
//// <q> NRF_LOG_USES_TIMESTAMP  - Enable timestamping
// 
//
//// <i> Function for getting the timestamp is provided by the user
//
//#ifndef NRF_LOG_USES_TIMESTAMP
//#define NRF_LOG_USES_TIMESTAMP 0
//#endif
//
//#endif //NRF_LOG_ENABLED
//// </e>
//
//// <h> nrf_log_backend - Logging sink
//
////==========================================================
//// <o> NRF_LOG_BACKEND_MAX_STRING_LENGTH - Buffer for storing single output string 
//// <i> Logger backend RAM usage is determined by this value.
//
//#ifndef NRF_LOG_BACKEND_MAX_STRING_LENGTH
//#define NRF_LOG_BACKEND_MAX_STRING_LENGTH 256
//#endif
//
//// <o> NRF_LOG_TIMESTAMP_DIGITS - Number of digits for timestamp 
//// <i> If higher resolution timestamp source is used it might be needed to increase that
//
//#ifndef NRF_LOG_TIMESTAMP_DIGITS
//#define NRF_LOG_TIMESTAMP_DIGITS 8
//#endif
//
//// <e> NRF_LOG_BACKEND_SERIAL_USES_UART - If enabled data is printed over UART
////==========================================================
//#ifndef NRF_LOG_BACKEND_SERIAL_USES_UART
//#define NRF_LOG_BACKEND_SERIAL_USES_UART 0
//#endif
//#if  NRF_LOG_BACKEND_SERIAL_USES_UART
//// <o> NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE  - Default Baudrate
// 
//// <323584=> 1200 baud 
//// <643072=> 2400 baud 
//// <1290240=> 4800 baud 
//// <2576384=> 9600 baud 
//// <3862528=> 14400 baud 
//// <5152768=> 19200 baud 
//// <7716864=> 28800 baud 
//// <10289152=> 38400 baud 
//// <15400960=> 57600 baud 
//// <20615168=> 76800 baud 
//// <30924800=> 115200 baud 
//// <61865984=> 230400 baud 
//// <67108864=> 250000 baud 
//// <121634816=> 460800 baud 
//// <251658240=> 921600 baud 
//// <268435456=> 57600 baud 
//
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE
//#define NRF_LOG_BACKEND_SERIAL_UART_BAUDRATE 30924800
//#endif
//
//// <o> NRF_LOG_BACKEND_SERIAL_UART_TX_PIN - UART TX pin 
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_TX_PIN
//#define NRF_LOG_BACKEND_SERIAL_UART_TX_PIN 9
//#endif
//
//// <o> NRF_LOG_BACKEND_SERIAL_UART_RX_PIN - UART RX pin 
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_RX_PIN
//#define NRF_LOG_BACKEND_SERIAL_UART_RX_PIN 11
//#endif
//
//// <o> NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN - UART RTS pin 
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN
//#define NRF_LOG_BACKEND_SERIAL_UART_RTS_PIN 8
//#endif
//
//// <o> NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN - UART CTS pin 
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN
//#define NRF_LOG_BACKEND_SERIAL_UART_CTS_PIN 10
//#endif
//
//// <o> NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL  - Hardware Flow Control
// 
//// <0=> Disabled 
//// <1=> Enabled 
//
//#ifndef NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL
//#define NRF_LOG_BACKEND_SERIAL_UART_FLOW_CONTROL 0
//#endif
//
//// <o> NRF_LOG_BACKEND_UART_INSTANCE  - UART instance used
// 
//// <0=> 0 
//
//#ifndef NRF_LOG_BACKEND_UART_INSTANCE
//#define NRF_LOG_BACKEND_UART_INSTANCE 0
//#endif
//
//#endif //NRF_LOG_BACKEND_SERIAL_USES_UART
//// </e>
//
//// <e> NRF_LOG_BACKEND_SERIAL_USES_RTT - If enabled data is printed using RTT
////==========================================================
//#ifndef NRF_LOG_BACKEND_SERIAL_USES_RTT
//#define NRF_LOG_BACKEND_SERIAL_USES_RTT 1
//#endif
//#if  NRF_LOG_BACKEND_SERIAL_USES_RTT
//// <o> NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE - RTT output buffer size. 
//// <i> Should be equal or bigger than \ref NRF_LOG_BACKEND_MAX_STRING_LENGTH.
//// <i> This value is used in Segger RTT configuration to set the buffer size
//// <i> if it is bigger than default RTT buffer size.
//
//#ifndef NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE
//#define NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE 512
//#endif
//
//#endif //NRF_LOG_BACKEND_SERIAL_USES_RTT
//// </e>
//
//// </h> 
////==========================================================
//
//// </h> 
////==========================================================
//
//// <h> nRF_Segger_RTT 
//
////==========================================================
//// <h> segger_rtt - SEGGER RTT
//
////==========================================================
//// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_UP - Size of upstream buffer. 
//#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_UP
//#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 64
//#endif
//
//// <o> SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS - Size of upstream buffer. 
//#ifndef SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS
//#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS 2
//#endif
//
//// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN - Size of upstream buffer. 
//#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN
//#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN 16
//#endif
//
//// <o> SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS - Size of upstream buffer. 
//#ifndef SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS
//#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS 2
//#endif
//
//// </h> 
////==========================================================
//
//// </h> 
////==========================================================
//
//// <<< end of configuration section >>>
//#endif //SDK_CONFIG_H