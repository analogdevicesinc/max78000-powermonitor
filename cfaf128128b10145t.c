/*******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/*
  Crystalfontz CFAF128128B10145T 128x128 TFT LCD Display w/Sitronix ST7735S controller wired for 9-bit SPI mode.
*/
#include <stdint.h>
#include <string.h>
#include "mxc_config.h"
#include "tmr_utils.h"
#include "lcd.h"
#include "ugui.h"

#define FB_CACHE

#define LCD_WIDTH 128
#define LCD_HEIGHT 128

#define BLU 0x06C2D118
#define BG  0x07F3F9FC
#define FG  0x04020100

#define COL_PHYS_OFFS 3
#define ROW_PHYS_OFFS 2

#define ST7735S_SWRESET  0x01
#define ST7735S_SLPOUT   0x11
#define ST7735S_DISPOFF  0x28
#define ST7735S_DISPON   0x29
#define ST7735S_CASET    0x2A
#define ST7735S_RASET    0x2B
#define ST7735S_RAMWR    0x2C
#define ST7735S_MADCTL   0x36
#define ST7735S_COLMOD   0x3A
#define ST7735S_FRMCTR1  0xB1
#define ST7735S_FRMCTR2  0xB2
#define ST7735S_FRMCTR3  0xB3
#define ST7735S_INVCTR   0xB4
#define ST7735S_PWCTR1   0xC0
#define ST7735S_PWCTR2   0xC1
#define ST7735S_PWCTR3   0xC2
#define ST7735S_PWCTR4   0xC3
#define ST7735S_PWCTR5   0xC4
#define ST7735S_VMCTR1   0xC5
#define ST7735S_GAMCTRP1 0xE0
#define ST7735S_GAMCTRN1 0xE1

#define SSO_H { MXC_SETBIT(&MXC_GPIO->out_val[PORT_0], 7); }
#define SSO_L { MXC_CLRBIT(&MXC_GPIO->out_val[PORT_0], 7); }

#define MOSI_H { MXC_SETBIT(&MXC_GPIO->out_val[PORT_0], 5); }
#define MOSI_L { MXC_CLRBIT(&MXC_GPIO->out_val[PORT_0], 5); }

#define SCK_TOG { MXC_SETBIT(&MXC_GPIO->out_val[PORT_0], 4); \
                  __NOP();                                   \
                  MXC_CLRBIT(&MXC_GPIO->out_val[PORT_0], 4); }

static const gpio_cfg_t lcd_sso = { PORT_0, PIN_7, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
static const gpio_cfg_t lcd_sck  = { PORT_0, PIN_4, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
static const gpio_cfg_t lcd_mosi  = { PORT_0, PIN_5, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

#if defined(FB_CACHE)
static uint8_t fb[LCD_WIDTH * LCD_HEIGHT / 8];
#endif

static inline void send_cmd(uint8_t cmd)
{
  MOSI_L
  SCK_TOG

  for (int i = 0; i < 8; i++) {
    if (cmd & 0x80)
      MOSI_H
    else
      MOSI_L
    SCK_TOG
    cmd <<= 1;
  }
}

static inline void send_data(uint8_t data)
{
  MOSI_H
  SCK_TOG

  for (int i = 0; i < 8; i++) {
    if (data & 0x80)
      MOSI_H
    else
      MOSI_L
    SCK_TOG
    data <<= 1;
  }
}

static inline void send_rgb(uint32_t rgb)
{
  for (int i = 0; i < 27; i++) {
    if (rgb & 0x04000000)
      MOSI_H
    else
      MOSI_L
    SCK_TOG
    rgb <<= 1;
  }
}

static inline void send_cmd_data(uint8_t cmd, void *data, uint8_t len)
{
  uint8_t *buf = (uint8_t*)data;

  SSO_L
  send_cmd(cmd);
  while (len--)
    send_data(*buf++);
  SSO_H
}

static inline void lcd_xyloc(uint8_t row, uint8_t col)
{
  static uint8_t col_data[4] = { 0, 0, 0, LCD_WIDTH + COL_PHYS_OFFS };
  static uint8_t row_data[4] = { 0, 0, 0, LCD_HEIGHT + ROW_PHYS_OFFS };

  col_data[1] = COL_PHYS_OFFS + col;
  row_data[1] = ROW_PHYS_OFFS + row;

  send_cmd_data(ST7735S_CASET, col_data, sizeof(col_data));
  send_cmd_data(ST7735S_RASET, row_data, sizeof(row_data));
}

static void panel_init(void)
{
  static const struct {
    uint8_t cmd;
    uint8_t msdelay;
    uint8_t len;
    char *data;
  } *pcfg, cfgs[] = {
    /* Send a software reset to clear regs (120ms delay mandatory) */
    { ST7735S_SWRESET, 120,  0, NULL },
    /* Wake up panel (120ms delay mandatory) */
    { ST7735S_SLPOUT,  120,  0, NULL },
    /* Frame rate control, panel-specific */
    { ST7735S_FRMCTR1,   0,  3, "\x02\x35\x36" },
    { ST7735S_FRMCTR2,   0,  3, "\x02\x35\x36" },
    { ST7735S_FRMCTR3,   0,  6, "\x02\x35\x36\x02\x35\x36" },
    /* Display inversion control */
    { ST7735S_INVCTR,    0,  1, "\x07" },
    /* Power control 1; GVDD = 4.70V, AVDD = 2.5uA */
    { ST7735S_PWCTR1,    0,  2, "\x02\x02" },
    /* Power control 2; VGH = 14.70V, VGL = -7.35V */
    { ST7735S_PWCTR2,    0,  1, "\xC5" },
    /* Power control 3 (full-color); Opamp Bias = "Large", DC Booster Frequency = BCLK / 1 */
    { ST7735S_PWCTR3,    0,  2, "\x0D\x00" },
    /* Power control 4 (8-color); Opamp Bias = "Large", DC Booster Frequency = BCLK / 4 */
    { ST7735S_PWCTR4,    0,  2, "\x8D\x1A" },
    /* Power control 5 (partial-color); Opamp Bias = "Large", DC Booster Frequency = BCLK / 8 */
    { ST7735S_PWCTR5,    0,  2, "\x8D\xEE" },
    /* VCOM Control 1; VCOMH = +4.525V, VCOML = -0.575V */
    { ST7735S_VMCTR1,    0,  2, "\x51\x4D" },
    /* Gamma correction (negative) */
    { ST7735S_GAMCTRP1,  0, 16, "\x0A\x1C\x0C\x14\x33\x2B\x24\x28\x27\x25\x2C\x39\x00\x05\x03\x0D" },
    /* Gamma correction (positive) */
    { ST7735S_GAMCTRN1,  0, 16, "\x0A\x1C\x0C\x14\x33\x2B\x24\x28\x27\x25\x2D\x3A\x00\x05\x03\x0D" },
    /* Color format; 18-bit/pixel */
    { ST7735S_COLMOD,    0,  1, "\x06" },
    /* Memory Data Access Control; Swap and reverse row and column address order for panel orientation */
    { ST7735S_MADCTL,    0,  1, "\xE0" },
  };

  int ncfgs = sizeof(cfgs) / sizeof(*cfgs);

  /* Delay after POR or RESET */
  TMR_Delay(MXC_TMR0, MSEC(500));

  for (pcfg = cfgs; ncfgs; ncfgs--, pcfg++) {
    send_cmd_data(pcfg->cmd, pcfg->data, pcfg->len);
    if (pcfg->msdelay)
      TMR_Delay(MXC_TMR0, MSEC(pcfg->msdelay));
  }
}

void lcd_set_pixel(UG_S16 x, UG_S16 y, UG_COLOR c)
{
#if defined(FB_CACHE)
  uint32_t offs;
  uint8_t mask;
  uint8_t tmp;

  offs = (y * 16) + (x / 8);
  mask = 0x01 << (x % 8);
  tmp = fb[offs];

  if (c) {
    if (tmp & mask)
      return; /* bit value is already what's needed so no need for update */
    fb[offs] |= mask;
  } else {
    if (!(tmp & mask))
      return; /* bit value is already what's needed so no need for update */
    fb[offs] &= ~mask;
  }
#endif

  lcd_xyloc(x, y);

  SSO_L
  send_cmd(ST7735S_RAMWR);
  send_rgb(c ? BG : FG);
  SSO_H
}

int lcd_clear(void)
{
  /* Turn display off during full screen update to hide tearing */
  send_cmd_data(ST7735S_DISPOFF, NULL, 0);

  for (int y = 0; y < LCD_HEIGHT; y++) {
    lcd_xyloc(y, 0);

    SSO_L
    send_cmd(ST7735S_RAMWR);
    for (int x = 0; x < LCD_WIDTH; x++)
      send_rgb(BG);
    SSO_H
  }

  send_cmd_data(ST7735S_DISPON, NULL, 0);

#if defined(FB_CACHE)
  memset(fb, BG ? 0xFF : 0x00, sizeof(fb));
#endif
  return 0;
}

int lcd_update(void)
{
  return 0;
}

void lcd_show_bitmap(const uint8_t *bitmap)
{
  uint8_t tmp;

  /* Turn display off during full screen update to hide tearing */
  send_cmd_data(ST7735S_DISPOFF, NULL, 0);

  for (int y = 0; y < LCD_HEIGHT; y++) {
    lcd_xyloc(y, 0);

    SSO_L
    send_cmd(ST7735S_RAMWR);
    for (int x = 0; x < (LCD_WIDTH / 8); x++) {
      tmp = *bitmap++;
      for (int p = 0; p < 8; p++) {
        send_rgb((tmp & 1) ? BLU : BG);
        tmp >>= 1;
      }
    }
    SSO_H
  }

  send_cmd_data(ST7735S_DISPON, NULL, 0);
}

int lcd_init(void)
{
  GPIO_OutSet(&lcd_sso);
  SYS_IOMAN_UseVDDIOH(&lcd_sso);
  GPIO_Config(&lcd_sso);

  GPIO_OutClr(&lcd_sck);
  SYS_IOMAN_UseVDDIOH(&lcd_sck);
  GPIO_Config(&lcd_sck);

  GPIO_OutClr(&lcd_mosi);
  SYS_IOMAN_UseVDDIOH(&lcd_mosi);
  GPIO_Config(&lcd_mosi);

  panel_init();

  lcd_clear();

  return 0;
}
