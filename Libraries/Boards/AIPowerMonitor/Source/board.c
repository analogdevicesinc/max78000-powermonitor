
#include <stdio.h>
#include "mxc_config.h"
#include "mxc_assert.h"
#include "pwrman_regs.h"
#include "board.h"
#include "gpio.h"
#include "uart.h"

// LEDs
// Note: Power monitor LEDs use 3.3v supply so these must be open-drain.
const gpio_cfg_t led_pin[] = {
    { PORT_2, PIN_4, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL /*GPIO_PAD_OPEN_DRAIN*/ },
    { PORT_2, PIN_5, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL /*GPIO_PAD_OPEN_DRAIN*/ },
    { PORT_2, PIN_6, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL /*GPIO_PAD_OPEN_DRAIN*/ },
};

const unsigned int num_leds = (sizeof(led_pin) / sizeof(gpio_cfg_t));

// Push buttons
const gpio_cfg_t pb_pin[] = {
    { PORT_2, PIN_7, GPIO_FUNC_GPIO, GPIO_PAD_INPUT },
    { PORT_3, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_INPUT },
};

const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(gpio_cfg_t));

// Console UART configuration
const uart_cfg_t console_uart_cfg = {
    .parity = UART_PARITY_DISABLE,
    .size = UART_DATA_SIZE_8_BITS,
    .extra_stop = 0,
    .cts = 0,
    .rts = 0,
    .baud = CONSOLE_BAUD,
};

const sys_cfg_uart_t console_sys_cfg = {
    .clk_scale = CLKMAN_SCALE_AUTO,
    .io_cfg = IOMAN_UART(CONSOLE_UART, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0)
};

const gpio_cfg_t console_uart_rx = { PORT_2, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };
const gpio_cfg_t console_uart_tx = { PORT_2, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    while (1);
}

/******************************************************************************/
void reset_peripherals(void)
{
    // Deconfigure things the bootloader may have configured

    // Disable interrupts
    SysTick->CTRL = 0;
    NVIC_DisableIRQ(GPIO_P0_IRQn);
    NVIC_DisableIRQ(GPIO_P1_IRQn);
    NVIC_DisableIRQ(GPIO_P2_IRQn);
    NVIC_DisableIRQ(GPIO_P3_IRQn);
    NVIC_DisableIRQ(GPIO_P4_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART2_IRQn);
    NVIC_DisableIRQ(OWM_IRQn);
    NVIC_DisableIRQ(USB_IRQn);
    NVIC_DisableIRQ(AFE_IRQn);

    // Reset peripherals
    MXC_PWRMAN->peripheral_reset = MXC_F_PWRMAN_PERIPHERAL_RESET_GPIO  |
                                   MXC_F_PWRMAN_PERIPHERAL_RESET_UART0 |
                                   MXC_F_PWRMAN_PERIPHERAL_RESET_UART2 |
                                   MXC_F_PWRMAN_PERIPHERAL_RESET_USB   |
                                   MXC_F_PWRMAN_PERIPHERAL_RESET_ADC   |
                                   MXC_F_PWRMAN_PERIPHERAL_RESET_OWM;
    MXC_PWRMAN->peripheral_reset = 0;

    // Switch IO reference to reset state
    MXC_IOMAN->use_vddioh_0 = 0;
    MXC_IOMAN->use_vddioh_1 = 0;
}

/******************************************************************************/
int Board_Init(void)
{
    int err;

    reset_peripherals();

    if ((err = Console_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    SYS_IOMAN_UseVDDIOH(led_pin + 0);
    SYS_IOMAN_UseVDDIOH(led_pin + 1);
    SYS_IOMAN_UseVDDIOH(led_pin + 2);

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }


    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = UART_Init(MXC_UART_GET_UART(CONSOLE_UART), &console_uart_cfg, &console_sys_cfg)) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_PrepForSleep(void)
{
#if defined ( __GNUC__ )
    fflush(stdout);
#endif /* __GNUC__ */
    return UART_PrepForSleep(MXC_UART_GET_UART(CONSOLE_UART));
}
