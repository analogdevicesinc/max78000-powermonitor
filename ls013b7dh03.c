/*******************************************************************************
 * Copyright (C) 2020-2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "mxc_config.h"
#include "spim.h"
#include "ugui.h"

#define LCD_SPIM      MXC_SPIM0
#define LCD_SCS       0
#define LCD_SCLK_RATE 1100000

// bit per pixel
static uint8_t framebuf[128 * 16];

// bit per line (row)
static uint32_t line_flag[4];

static inline void mark_line_for_update(uint32_t line)
{
    line_flag[line >> 5] |= 1 << (line & 0x1F);
}

/* Clear display, framebuffer and line update flags */
int lcd_clear(void)
{
	uint8_t buf[2] = { 0x20, 0x00 };
	struct spim_req req = { 0 };

    memset(framebuf, 0xFF, sizeof(framebuf));
    memset(line_flag, 0, sizeof(line_flag));

    req.tx_data = buf;
    req.len = sizeof(buf);
    req.deass = 1;
    return SPIM_Trans(LCD_SPIM, &req);
}

int lcd_update(void)
{
    int i;
    uint32_t tmp;
    uint8_t buf[20] = { 0x80 };
    struct spim_req req = { 0 };

    for (i = 0; i < 4; i++) {
        while (line_flag[i]) {
            // first bit from right
            tmp = __builtin_ctz(line_flag[i]);

            // clear bit; before tmp becomes something else
            line_flag[i] &= ~(1 << tmp);

            // tmp becomes lcd line number
            tmp += (i * 32);

            // except lcd lines are 1's based and bit reversed
            buf[1] = __RBIT(tmp + 1) >> 24;

            // tmp becomes frambuffer offset
            tmp *= 16;
            memcpy(&buf[2], &framebuf[tmp], 16);

            // send to display
            req.tx_data = buf;
            req.len = (line_flag[i]) ? 18 : 20;
            req.deass = !line_flag[i];
            if (SPIM_Trans(LCD_SPIM, &req) < 0) {
                return -1;
            }
        }
    }

    return 0;
}

void lcd_show_bitmap(uint8_t *bitmap)
{
    memcpy(framebuf, bitmap, sizeof(framebuf));
    memset(line_flag, 0xFF, sizeof(line_flag));
    lcd_update();
}

void lcd_set_pixel(UG_S16 x, UG_S16 y, UG_COLOR c)
{
    uint32_t offs;
    uint8_t mask;
    uint8_t tmp;

    offs = (y * 16) + (x / 8);
    mask = 0x80 >> (x % 8);
    tmp = framebuf[offs];

    if (c == C_BLACK) {
        if (!(tmp & mask)) {
            return; // done if bit at required value
        }
        framebuf[offs] &= ~mask;
    } else {
        if (tmp & mask) {
            return; // done if bit at required value
        }
        framebuf[offs] |= mask;
    }

    mark_line_for_update(y);
}

int lcd_init(void)
{
    int status;

    sys_cfg_spim_t lcd_sys_cfg = {
        .clk_scale = CLKMAN_SCALE_AUTO,
        .io_cfg = IOMAN_SPIM0(1, 1, 0, 0, 0, 0, 0, 0),
    };

    spim_cfg_t lcd_spim_cfg = { 0, SPIM_SSEL0_HIGH, LCD_SCLK_RATE };

    gpio_cfg_t lcd_spi = { PORT_0, (PIN_4 | PIN_5 | PIN_7), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
    gpio_cfg_t lcd_en  = { PORT_1, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

    GPIO_OutClr(&lcd_en);
    if ((status = GPIO_Config(&lcd_en)) != E_NO_ERROR) {
        return status;
    }

    if ((status = SPIM_Init(LCD_SPIM, &lcd_spim_cfg, &lcd_sys_cfg)) != E_NO_ERROR) {
        return status;
    }

    SYS_IOMAN_UseVDDIOH(&lcd_spi);
    SYS_IOMAN_UseVDDIOH(&lcd_en);

    GPIO_OutSet(&lcd_en);

    lcd_clear();

    return 0;
}
