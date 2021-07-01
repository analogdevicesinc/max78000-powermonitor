/*******************************************************************************
 * Copyright (C) 2020 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdlib.h>
#include "mxc_config.h"
#include "i2cm.h"
#include "tmr_utils.h"
#include "max34417.h"

#ifdef DUMP_RESPONSE
#include <stdio.h>
#endif

#define MAX34417_I2CM       MXC_I2CM0
#define MAX34417_I2CM_IDX   0
#define MAX34417_I2CM_SPEED I2CM_SPEED_400KHZ
#define MAX34417_I2C_ADDR   0x10

#define UPDATE_CMD     0x00
#define CONTROL_CMD    0x01
#define ACC_COUNT_CMD  0x02
#define PWR_ACC_1_CMD  0x03
#define PWR_ACC_2_CMD  0x04
#define PWR_ACC_3_CMD  0x05
#define PWR_ACC_4_CMD  0x06
#define BULK_PWR_CMD   0x10
#define BULK_VOLT_CMD  0x11
#define DEV_REV_ID_CMD 0x0F

#define CNTRL_MODE    0x80
#define CNTRL_CAM     0x40
#define CNTRL_SMM     0x20
#define CNTRL_PARK_EN 0x10
#define CNTRL_PARK1   0x08
#define CNTRL_PARK0   0x04
#define CNTRL_SLOW    0x02
#define CNTRL_OVF     0x01

static uint32_t convert_14to16( const uint8_t * p14 )
{
    uint32_t v16;
    uint8_t * p16 = (uint8_t*)&v16;
    uint8_t i;
    for (i=0;i<2;i++)
    {
        p16[1-i] = p14[i];
    }
    return v16 >> 2;
}

static uint32_t convert_24to32( const uint8_t * p24 )
{
    uint32_t v32;
    uint8_t * p32 = (uint8_t*)&v32;
    uint8_t i;
    for (i=0;i<3;i++)
    {
        p32[2-i] = p24[i];
    }
    p32[i] = 0;
    return v32;
}

static uint64_t convert_56to64( const uint8_t * p56 )
{
    uint64_t v64;
    uint8_t * p64 = (uint8_t*)&v64;
    uint8_t i;
    for (i=0;i<7;i++)
    {
        p64[6-i] = p56[i];
    }
    p64[i] = 0;
    return v64;
}

int init_max34417(void)
{
    int status;
    uint8_t buf[2];
    sys_cfg_i2cm_t i2cm_sys_cfg;
    ioman_cfg_t io_cfg = IOMAN_I2CM(MAX34417_I2CM_IDX, 1, 0);

    i2cm_sys_cfg.clk_scale = CLKMAN_SCALE_DIV_1;
    i2cm_sys_cfg.io_cfg = io_cfg;
    I2CM_Init(MAX34417_I2CM, &i2cm_sys_cfg, MAX34417_I2CM_SPEED);

    buf[0] = DEV_REV_ID_CMD;
    status = I2CM_Read(MAX34417_I2CM, MAX34417_I2C_ADDR, &buf[0], 1, &buf[1], 1);
    if (status != 1) {
        return status;
    }

    buf[0] = CONTROL_CMD;
    buf[1] = CNTRL_MODE;
    status = I2CM_Write(MAX34417_I2CM, MAX34417_I2C_ADDR, NULL, 0, buf, 2);
    if (status != 2) {
        return status;
    }

    max34417_update();
    TMR_Delay(MXC_TMR0, MSEC(1));

    buf[0] = CONTROL_CMD;
    buf[1] = CNTRL_CAM | CNTRL_MODE;
    status = I2CM_Write(MAX34417_I2CM, MAX34417_I2C_ADDR, NULL, 0, buf, 2);
    if (status != 2) {
        return status;
    }

    max34417_update();
    TMR_Delay(MXC_TMR0, MSEC(1));

    return 0;
}

int max34417_update(void)
{
    int status;
    uint8_t cmd;

    cmd = UPDATE_CMD;
    status = I2CM_Write(MAX34417_I2CM, MAX34417_I2C_ADDR, NULL, 0, &cmd, 1);
    if (status != 1) {
        return status;
    }

    return 0;
}

uint32_t max34417_acc_count( void )
{
    uint8_t cmd;
    static uint8_t buf[4];

    cmd = ACC_COUNT_CMD;
    if (I2CM_Read(MAX34417_I2CM, MAX34417_I2C_ADDR, &cmd, 1, buf, 4) != 4) {
        return 1;
    }
    return convert_24to32( &buf[1] );
}

bool max34417_bulk_voltage( double voltage[4] )
{
    uint8_t cmd;
    uint16_t adc;
    uint8_t i;
    static uint8_t buf[9];
    static const double adc2v = 24.0 / (16384.0 - 1.0);
    cmd = BULK_VOLT_CMD;
    if (I2CM_Read(MAX34417_I2CM, MAX34417_I2C_ADDR, &cmd, 1, buf, 9) != 9)
    {
        return false;
    }
    for (i=0;i<4;i++)
    {
        adc = convert_14to16( &buf[1+i*2] );
        voltage[i] = (double)adc * adc2v;
    }
    return true;
}

bool max34417_bulk_energy( double energy_raw[4] )
{
    uint8_t i;
    uint8_t cmd;
    static uint8_t buf[29];

    cmd = BULK_PWR_CMD;
    if (I2CM_Read(MAX34417_I2CM, MAX34417_I2C_ADDR, &cmd, 1, buf, 29) != 29) {
        return false;
    }

    for (i=0;i<4;i++)
    {
        energy_raw[i] = convert_56to64( &buf[1+i*7] );
        //printf( "%g\n", energy_raw[i] );
    }
    return true;
}

bool max34417_bulk_power( double power_raw[4] )
{
    uint8_t i;
    uint32_t acc_count = max34417_acc_count();
    if( !max34417_bulk_energy( power_raw ) )
        return false;
    for (i=0;i<4;i++)
    {
        power_raw[i] = power_raw[i] / (double)acc_count;
        //printf( "%g\n", power_raw[i] );
    }
    return true;
}


