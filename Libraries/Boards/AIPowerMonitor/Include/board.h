

#ifndef _BOARD_H
#define _BOARD_H

#include "gpio.h"
#include "ioman.h"
#include "led.h"
#include "pb.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONSOLE_UART
#define CONSOLE_UART	1   	/// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD    115200  /// Console baud rate
#endif

// Pushbutton Indices
#define SW1             0       /// Pushbutton index for SW1
#define SW2             1       /// Pushbutton index for SW2

#define LED_OFF         1       /// Inactive state of LEDs
#define LED_ON          0       /// Active state of LEDs

// Console UART configuration
extern const uart_cfg_t console_uart_cfg;
extern const sys_cfg_uart_t console_sys_cfg;
extern const gpio_cfg_t console_uart_rx;
extern const gpio_cfg_t console_uart_tx;

/**
 * @brief   Initialize the BSP and board interfaces.
 * @retval  E_NO_ERROR if everything is successful
 */
int Board_Init(void);

/**
 * @brief   Initialize or reinitialize the console. Reinitialization may be
 *          necessary if the system clock rate is changed.
 * @retval  E_NO_ERROR if everything is successful
 */
int Console_Init(void);

/**
 * @brief   Prepare the console for sleep.
 * @retval  E_NO_ERROR if ready to sleep, #E_BUSY if not ready for sleep.
 */
int Console_PrepForSleep(void);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H */
