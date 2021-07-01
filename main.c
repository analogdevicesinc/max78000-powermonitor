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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "mxc_config.h"
#include "led.h"
#include "tmr_utils.h"
#include "max34417.h"
#include "lcd.h"
#include "ugui.h"
#include "logo.xbm"
#include "Led.h"
#include "tmr.h"

#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "cdc_acm.h"
#include "descriptors.h"
#include "pwrman_regs.h"
#include <ctype.h>

#define POLL_PERIOD_MS      1
#define UPDATE_PERIOD_MS    500/POLL_PERIOD_MS

#define LEFT_COL  1
#define TOP_ROW   2
#define ROW_SPACE 20
#define ROW(n) (TOP_ROW + (ROW_SPACE * (n)))

#define AI_DEV "PMON 1.4"

typedef enum {STATE_V, STATE_IP, STATE_AP, STATE_TRIG_CNN, STATE_TRIG_SYSTEM } state_t;

static const gpio_cfg_t gpio_cfg_trig =
{
    PORT_0, PIN_2 | PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_INPUT,
};

typedef enum
{
    trig_state_host_reset,
    trig_state_host_idle,
    trig_state_measure_idle,
    trig_state_measure_idle_done,
    trig_state_measure_active,
    trig_state_measure_active_done,
    trig_state_unchanged
}
trig_state_t;

#define EVENT_ENUM_COMP     MAXUSB_NUM_EVENTS
#define EVENT_REMOTE_WAKE   (EVENT_ENUM_COMP + 1)

volatile unsigned int event_flags;

static const acm_cfg_t acm_cfg = {
    1,                  /* EP OUT */
    MXC_USB_MAX_PACKET, /* OUT max packet size */
    2,                  /* EP IN */
    MXC_USB_MAX_PACKET, /* IN max packet size */
    3,                  /* EP Notify */
    MXC_USB_MAX_PACKET, /* Notify max packet size */
};

static UG_GUI gui;
static char txt_buf[16];

static void print_right_justified( double v, uint8_t row )
{
    sprintf( txt_buf, "%.2f", v );
    int l = strlen( txt_buf );
    UG_PutString( gui.x_dim - 2 - l * gui.font.char_width, ROW( row + 2 ), txt_buf );
}

static void print_centered( const char * txt, uint8_t row )
{
    int l = strlen( txt );
    UG_PutString( (gui.x_dim - l * gui.font.char_width) / 2, ROW( row ), (char*)txt );
}

void show_volts( bool redraw )
{
    double voltage[4];
    int i;

    if( redraw )
    {
        lcd_clear();
        UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
        print_centered( "SUPPLY V", 1 );
        UG_PutString( LEFT_COL, ROW( 2 ), "DDB" );
        UG_PutString( LEFT_COL, ROW( 3 ), "CA" );
        UG_PutString( LEFT_COL, ROW( 4 ), "CB" );
        UG_PutString( LEFT_COL, ROW( 5 ), "RA" );
    }

    if( !max34417_bulk_voltage( voltage ) )
    {
        // need to update lcd with error msg
        printf( "show_volts() failed to read power monitor\n" );
        return;
    }
    for( i = 0; i < 4; i++ )
    {
        if( acm_present() )
        {
            sprintf( txt_buf, "%g", voltage[i] );
            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
            if( i != 3 )
            {
                acm_write( (uint8_t*)",", 1 );
            }
            else
            {
                acm_write( (uint8_t*)"\r\n", 2 );
            }
        }
        print_right_justified( voltage[i], i );
    }
}

static void compensated_power( double power[4] )
{
    uint8_t i;
    static const double scale_and_comp = 2.24525201657553590785e-5; //2.247483130902358e-05; // converts raw power figure to mW
    max34417_bulk_power( power );
    for( i = 0; i < 4; i++ )
    {
        power[i] *= scale_and_comp; // scale and comp to mW
    }
}

void show_avg_power( bool redraw )
{
    double power[4];
    uint8_t i;

    if( redraw )
    {
        lcd_clear();
        UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
        print_centered( "AVG PWR mW", 1 );
        UG_PutString( LEFT_COL, ROW( 2 ), "DDB" );
        UG_PutString( LEFT_COL, ROW( 3 ), "CA" );
        UG_PutString( LEFT_COL, ROW( 4 ), "CB" );
        UG_PutString( LEFT_COL, ROW( 5 ), "RA" );

        //  memset(acc, 0, sizeof(acc));
        //  cnt = 0;
    }

    compensated_power( power );
    for( i = 0; i < 4; i++ )
    {
        if( acm_present() )
        {
            sprintf( txt_buf, "%g", power[i] / 1000.0 );
            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
            if( i != 3 )
            {
                acm_write( (uint8_t*)",", 1 );
            }
            else
            {
                acm_write( (uint8_t*)"\r\n", 2 );
            }
        }
        print_right_justified( power[i], i );
    }
}


void show_inst_current( bool redraw )
{
    double voltage[4], power[4];
    uint8_t i;

    if( redraw )
    {
        lcd_clear();
        UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
        print_centered( "CURRENT mA", 1 );
        UG_PutString( LEFT_COL, ROW( 2 ), "DDB" );
        UG_PutString( LEFT_COL, ROW( 3 ), "CA" );
        UG_PutString( LEFT_COL, ROW( 4 ), "CB" );
        UG_PutString( LEFT_COL, ROW( 5 ), "RA" );
    }
    if( !max34417_bulk_voltage( voltage ) )
    {
        // need to update lcd with error msg
        printf( "show_volts() failed to read power monitor\n" );
        return;
    }
    compensated_power( power );
    for( i = 0; i < 4; i++ )
    {
        double current;
        if( voltage[i] )
            current = power[i] / voltage[i];
        else
            current = 0;
        if( acm_present() )
        {
            sprintf( txt_buf, "%g", current / 1000.0 );
            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
            if( i != 3 )
            {
                acm_write( (uint8_t*)",", 1 );
            }
            else
            {
                acm_write( (uint8_t*)"\r\n", 2 );
            }
        }
        print_right_justified( current, i );
    }
}

static void enter_system_power_mode( void )
{
    lcd_clear();
    UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
    print_centered( "SYSTEM", 2 );
    print_centered( "POWER", 3 );
    print_centered( "MODE", 4 );
    lcd_update();
}

static void enter_cnn_power_mode( void )
{
    lcd_clear();
    UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
    print_centered( "CNN", 2 );
    print_centered( "POWER", 3 );
    print_centered( "MODE", 4 );
    lcd_update();
}

trig_state_t trig_state( void )
{
    trig_state_t ts;
    static trig_state_t last_trig_state = trig_state_host_reset;

    uint32_t gpio = GPIO_InGet( &gpio_cfg_trig );
    if( !gpio )
    {
        last_trig_state = ts = trig_state_host_reset;
    }
    else if( gpio == PIN_3 )
    {

        if( last_trig_state == trig_state_measure_idle )
        {
            ts = trig_state_unchanged;
        }
        else
        {
            last_trig_state = ts = trig_state_measure_idle;
        }
    }
    else if( gpio == PIN_2 )
    {
        if( last_trig_state == trig_state_host_reset )
            return trig_state_host_reset;
        if( last_trig_state == trig_state_measure_active )
        {
            ts = trig_state_unchanged;
        }
        else
        {
            last_trig_state = ts = trig_state_measure_active;
        }
    }
    else if( last_trig_state == trig_state_measure_active )
    {
        if( last_trig_state == trig_state_measure_active_done )
        {
            ts = trig_state_unchanged;
        }
        else
        {
            last_trig_state = ts = trig_state_measure_active_done;
        }

    }
    else if( last_trig_state == trig_state_measure_idle )
    {
        if( last_trig_state == trig_state_measure_idle_done )
        {
            ts = trig_state_unchanged;
        }
        else
        {
            last_trig_state = ts = trig_state_measure_idle_done;
        }
    }
    else
        last_trig_state = ts = trig_state_host_idle;
    return ts;
}

static int setconfig_callback( usb_setup_pkt * sud, void * cbdata )
{
    /* Confirm the configuration value */
    if( sud->wValue == config_descriptor.config_descriptor.bConfigurationValue )
    {
        MXC_SETBIT( &event_flags, EVENT_ENUM_COMP );
        return acm_configure( &acm_cfg ); /* Configure the device class */
    }
    else if( sud->wValue == 0 )
    {
        return acm_deconfigure();
    }

    return -1;
}

static int setfeature_callback( usb_setup_pkt * sud, void * cbdata )
{
    return 0;
}

static int clrfeature_callback( usb_setup_pkt * sud, void * cbdata )
{
    return 0;
}

static int event_callback( maxusb_event_t evt, void * data );

/******************************************************************************/
static void usb_app_sleep( void )
{
    usb_sleep();
    MXC_PWRMAN->pwr_rst_ctrl &= ~MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
    if( MXC_USB->dev_cn & MXC_F_USB_DEV_CN_CONNECT )
    {
        usb_event_clear( MAXUSB_EVENT_DPACT );
        usb_event_enable( MAXUSB_EVENT_DPACT, event_callback, NULL );
    }
    else
    {
        usb_event_disable( MAXUSB_EVENT_DPACT );
    }
}

static void usb_app_wakeup( void )
{
    usb_event_disable( MAXUSB_EVENT_DPACT );
    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
    usb_wakeup();
}

static int configure_uart( void )
{
    return 0;
}

static volatile int usb_read_complete;

static int usb_read_callback( void )
{
    usb_read_complete = 1;
    return 0;
}

static int event_callback( maxusb_event_t evt, void * data )
{
    /* Set event flag */
    MXC_SETBIT( &event_flags, evt );

    switch (evt)
    {
        case MAXUSB_EVENT_NOVBUS:
            usb_event_disable( MAXUSB_EVENT_BRST );
            usb_event_disable( MAXUSB_EVENT_SUSP );
            usb_event_disable( MAXUSB_EVENT_DPACT );
            usb_disconnect();
            enum_clearconfig();
            acm_deconfigure();
            usb_app_sleep();
            break;
        case MAXUSB_EVENT_VBUS:
            usb_event_clear( MAXUSB_EVENT_BRST );
            usb_event_enable( MAXUSB_EVENT_BRST, event_callback, NULL );
            usb_event_clear( MAXUSB_EVENT_SUSP );
            usb_event_enable( MAXUSB_EVENT_SUSP, event_callback, NULL );
            usb_connect();
            usb_app_sleep();
            break;
        case MAXUSB_EVENT_BRST:
            usb_app_wakeup();
            enum_clearconfig();
            acm_deconfigure();
            break;
        case MAXUSB_EVENT_SUSP:
            usb_app_sleep();
            break;
        case MAXUSB_EVENT_DPACT:
            usb_app_wakeup();
            break;
        default:
            break;
    }

    return 0;
}

static void init_usb( void )
{
    /* Initialize state */
    event_flags = 0;
    /* Enable the USB clock and power */
    SYS_USB_Enable( 1 );

    /* Initialize the usb module */
    if( usb_init( NULL ) != 0 )
    {
        printf( "usb_init() failed\n" );
        while( 1 );
    }

    /* Initialize the enumeration module */
    if( enum_init() != 0 )
    {
        printf( "enum_init() failed\n" );
        while( 1 );
    }

    /* Register enumeration data */
    enum_register_descriptor( ENUM_DESC_DEVICE, (uint8_t*)&device_descriptor, 0 );
    enum_register_descriptor( ENUM_DESC_CONFIG, (uint8_t*)&config_descriptor, 0 );
    enum_register_descriptor( ENUM_DESC_STRING, lang_id_desc, 0 );
    enum_register_descriptor( ENUM_DESC_STRING, mfg_id_desc, 1 );
    enum_register_descriptor( ENUM_DESC_STRING, prod_id_desc, 2 );

    /* Handle configuration */
    enum_register_callback( ENUM_SETCONFIG, setconfig_callback, NULL );

    /* Handle feature set/clear */
    enum_register_callback( ENUM_SETFEATURE, setfeature_callback, NULL );
    enum_register_callback( ENUM_CLRFEATURE, clrfeature_callback, NULL );

    /* Initialize the class driver */
    if( acm_init() != 0 )
    {
        printf( "acm_init() failed\n" );
        while( 1 );
    }

    /* Register callbacks */
    usb_event_enable( MAXUSB_EVENT_NOVBUS, event_callback, NULL );
    usb_event_enable( MAXUSB_EVENT_VBUS, event_callback, NULL );
    acm_register_callback( ACM_CB_SET_LINE_CODING, configure_uart );
    acm_register_callback( ACM_CB_READ_READY, usb_read_callback );
    usb_read_complete = 0;

    if( configure_uart() != 0 )
    {
        printf( "configure_uart() failed\n" );
        while( 1 );
    }

    /* Start with USB in low power mode */
    usb_app_sleep();
    NVIC_EnableIRQ( USB_IRQn );
}

static void show_power_review_page( double active_power, double idle_power, double time )
{
    UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
    sprintf( txt_buf, "E %5.0f uJ", (active_power - idle_power) * time * 1000.0 );
    UG_PutString( LEFT_COL, ROW( 2 ), txt_buf );
    sprintf( txt_buf, "T %5.1f ms", time * 1000.0 );
    UG_PutString( LEFT_COL, ROW( 3 ), txt_buf );
    sprintf( txt_buf, "I %5.2f mW", idle_power );
    UG_PutString( LEFT_COL, ROW( 4 ), txt_buf );
    sprintf( txt_buf, "A %5.2f mW", active_power );
    UG_PutString( LEFT_COL, ROW( 5 ), txt_buf );
    lcd_update();

}
int main( void )
{
    static uint8_t power_review_mode = 0;
    static uint8_t power_review_page = 0;
    static uint8_t active_phase = 0;
    static uint8_t power_review_update = 1;

    state_t state, prev_state;
    int count;
    uint32_t left, prev_left;
    uint32_t right, prev_right;

    static double active_power[3][4], idle_power[4];
    static double time[3];

    SYS_IOMAN_UseVDDIOH( &gpio_cfg_trig );
    GPIO_Config( &gpio_cfg_trig );

    init_usb();

    init_max34417();

    lcd_init();

    UG_Init( &gui, lcd_set_pixel, 128, 128 );
    UG_FontSelect( &FONT_12X16 );
    UG_FontSetHSpace( 0 );
    UG_SetBackcolor( C_WHITE );
    UG_SetForecolor( C_BLACK );

    count = 0;
    right = prev_right = -1;
    left = prev_left = -1;

    state = STATE_AP;
    prev_state = ~STATE_AP;

    lcd_show_bitmap( logo_bits );
    max34417_update();
    TMR_Delay( MXC_TMR0, MSEC( 1000 ) );

    TMR_Init( MXC_TMR1, TMR_PRESCALE_DIV_2_0, NULL );
    static const tmr32_cfg_t tmr_cfg_con =
    {
        .mode = TMR32_MODE_CONTINUOUS
    };
    TMR32_Config( MXC_TMR1, &tmr_cfg_con );
    TMR32_Start( MXC_TMR1 );

    for(;;)
    {
        // 4Hz screen update, if count in ms.
        trig_state_t ts = trig_state();
        if( state != STATE_TRIG_CNN && state != STATE_TRIG_SYSTEM )
        {
            TMR_Delay( MXC_TMR0, MSEC( POLL_PERIOD_MS ) );
            if( count++ >= UPDATE_PERIOD_MS )
            {
                max34417_update();
                count = 0;
                switch (state)
                {
                    case STATE_V:
                        show_volts( state != prev_state );
                        break;
                    case STATE_IP:
                        show_inst_current( state != prev_state );
                        break;
                    case STATE_AP:
                        show_avg_power( state != prev_state );
                        break;
                    default:
                        break;
                }
                prev_state = state;
                lcd_update();
            }
        }
        else if( state == STATE_TRIG_CNN )
        {
            // PM in power accumulation mode
            if( ts == trig_state_measure_idle )
            {
                max34417_update();
                lcd_clear();
                UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
                print_centered( "MEASURING", 2 );
                print_centered( "IDLE", 3 );
                print_centered( "POWER", 4 );
                lcd_update();
                active_phase = 0;
                power_review_update = 0;
            }
            else if( ts == trig_state_measure_active )
            {
                max34417_update();
                TMR32_SetCount( MXC_TMR1, 0 );
                lcd_clear();
                UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
                switch (active_phase)
                {
                    case 0:
                    {
                        print_centered( "KERNEL", 2 );
                        print_centered( "LOADING", 3 );
                        break;
                    }
                    case 1:
                    {
                        print_centered( "INPUT", 2 );
                        print_centered( "LOADING", 3 );
                        break;
                    }
                    case 2:
                    {
                        print_centered( "INPUT AND", 2 );
                        print_centered( "INFERENCE", 3 );
                        break;
                    }
                }
                print_centered( "POWER", 4 );
                lcd_update();
            }
            else if( ts == trig_state_measure_idle_done )
            {
                max34417_update();
                TMR_Delay( MXC_TMR0, MSEC( 1 ) );
                compensated_power( idle_power );
            }
            else if( ts == trig_state_measure_active_done )
            {
                time[active_phase] = (double)TMR32_GetCount( MXC_TMR1 ) / ((double)SystemCoreClock * 100.0);
                max34417_update();
                TMR_Delay( MXC_TMR0, MSEC( 1 ) );
                compensated_power( active_power[active_phase] );
                active_phase++;
                if( active_phase > 2 )
                {
                    power_review_page = 0;
                    power_review_mode = 1;
                    lcd_clear();

                    lcd_update();
                    active_phase = 0;
                    power_review_update = 1;
                    if( acm_present() )
                    {
                        for( uint8_t i = 0; i < 3; i++ )
                        {
                            sprintf( txt_buf, "%g,", (active_power[i][1] - idle_power[1]) * time[i] / 1000.0 );
                            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                            sprintf( txt_buf, "%g,", time[i] );
                            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                            sprintf( txt_buf, "%g,", idle_power[1] / 1000.0 );
                            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                            if( i == 2 )
                                sprintf( txt_buf, "%g\r\n", active_power[i][1] / 1000.0 );
                            else
                                sprintf( txt_buf, "%g,", active_power[i][1] / 1000.0 );
                            acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                        }
                    }
                }
            }
            if( power_review_mode && power_review_update )
            {
                lcd_clear();
                switch (power_review_page)
                {
                    case 0:
                    {
                        print_centered( "KERNELS", 1 );
                        break;
                    }
                    case 1:
                    {
                        print_centered( "INPUT", 1 );
                        break;
                    }
                    case 2:
                    {
                        print_centered( "INPUT+INF", 1 );
                        break;
                    }
                }
                show_power_review_page( active_power[power_review_page][1], idle_power[1], time[power_review_page] );
                power_review_update = 0;
            }
        }
        else if( state == STATE_TRIG_SYSTEM )
        {
            if( ts == trig_state_measure_idle )
            {
                max34417_update();
                TMR32_SetCount( MXC_TMR1, 0 );
                lcd_clear();
                UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
                print_centered( "MEASURING", 2 );
                print_centered( "SYSTEM", 3 );
                print_centered( "POWER", 4 );
                lcd_update();
            }
            else if( ts == trig_state_measure_idle_done )
            {
                uint32_t t = TMR32_GetCount( MXC_TMR1 );
                max34417_update();
                TMR_Delay( MXC_TMR0, MSEC( 1 ) );
                compensated_power( idle_power );
                double time =  (double)t / (double)SystemCoreClock;
                lcd_clear();
                UG_PutString( LEFT_COL + 13, ROW( 0 ), AI_DEV );
                print_centered( "SYSTEM", 1 );
                sprintf( txt_buf, "  %5.0f uJ", idle_power[1] * time * 1000.0 );
                UG_PutString( LEFT_COL, ROW( 2 ), txt_buf );
                sprintf( txt_buf, "  %5.1f ms", time * 1000.0 );
                UG_PutString( LEFT_COL, ROW( 3 ), txt_buf );
                sprintf( txt_buf, "  %5.2f mW", idle_power[1] );
                UG_PutString( LEFT_COL, ROW( 4 ), txt_buf );
                lcd_update();
                if( acm_present() )
                {
                    sprintf( txt_buf, "%g,", idle_power[1] * time / 1000.0 );
                    acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                    sprintf( txt_buf, "%g,", time );
                    acm_write( (uint8_t*)txt_buf, strlen( txt_buf ) );
                    sprintf( txt_buf, "%g\r\n", idle_power[1] / 1000.0);
                    acm_write( (uint8_t*)txt_buf, strlen( txt_buf )  );
                }
            }
        }
        left <<= 1;
        left |= GPIO_InGet( &pb_pin[0] ) ? 1 : 0;

        right <<= 1;
        right |= GPIO_InGet( &pb_pin[1] ) ? 1 : 0;

        if( (left == 0) && (prev_left != 0) )
        {
            prev_left = 0;
            switch (state)
            {
                case STATE_V:
                {
                    state = STATE_TRIG_SYSTEM;
                    enter_system_power_mode();
                    break;
                }
                case STATE_TRIG_CNN:
                {
                    state = STATE_AP;
                    break;
                }
                case STATE_TRIG_SYSTEM:
                {
                    state = STATE_TRIG_CNN;
                    enter_cnn_power_mode();
                    break;
                }
                case STATE_AP:
                {
                    state = STATE_IP;
                    break;
                }
                case STATE_IP:
                {
                    state = STATE_V;
                    break;
                }
            }
            count = UPDATE_PERIOD_MS;
            power_review_update = power_review_mode = 0;
        }
        else if( (left == -1) && (prev_left != -1) )
        {
            prev_left = -1;
        }
        if( (right == 0) && (prev_right != 0) )
        {
            prev_right = 0;
            switch (state)
            {
                case STATE_V:
                    state = STATE_IP;
                    break;
                case STATE_IP:
                    state = STATE_AP;
                    break;
                case STATE_AP:
                {
                    prev_state = state = STATE_TRIG_CNN;
                    enter_cnn_power_mode();
                    break;
                }
                case STATE_TRIG_CNN:
                {
                    if( !power_review_mode )
                    {
                        state = STATE_TRIG_SYSTEM;
                        enter_system_power_mode();
                        break;
                    }
                    power_review_page++;
                    if( power_review_page > 2 )
                    {
                        power_review_page = 0;
                    }
                    power_review_update = 1;
                    break;
                }
                case STATE_TRIG_SYSTEM:
                {
                    prev_state = STATE_TRIG_SYSTEM;
                    state = STATE_V;
                    break;
                }
            }
            count = UPDATE_PERIOD_MS;
        }
        else if( (right == -1) && (prev_right != -1) )
        {
            prev_right = -1;
        }
        {
            uint8_t c;

            if( acm_canread() && acm_read( &c, 1 ) == 1 )
            {
                switch (tolower( c ))
                {
                    case 'v':
                    {
                        state = STATE_V;
                        break;
                    }
                    case 'i':
                    {
                        state = STATE_IP;
                        break;
                    }
                    case 'w':
                    {
                        state = STATE_AP;
                        break;
                    }
                    case 'c':
                    case 't':
                    {
                        state = STATE_TRIG_CNN;
                        enter_cnn_power_mode();
                        prev_state = state = STATE_TRIG_CNN;
                        break;
                    }
                    case 's':
                    {
                        enter_system_power_mode();
                        state = STATE_TRIG_SYSTEM;
                        break;
                    }
                    default:
                        break;
                }
            }
        }

        if( event_flags )
        {
            /* Display events */
            if( MXC_GETBIT( &event_flags, MAXUSB_EVENT_NOVBUS ) )
            {
                MXC_CLRBIT( &event_flags, MAXUSB_EVENT_NOVBUS );
            }
            else if( MXC_GETBIT( &event_flags, MAXUSB_EVENT_VBUS ) )
            {
                MXC_CLRBIT( &event_flags, MAXUSB_EVENT_VBUS );
            }
            else if( MXC_GETBIT( &event_flags, MAXUSB_EVENT_BRST ) )
            {
                MXC_CLRBIT( &event_flags, MAXUSB_EVENT_BRST );
            }
            else if( MXC_GETBIT( &event_flags, MAXUSB_EVENT_SUSP ) )
            {
                MXC_CLRBIT( &event_flags, MAXUSB_EVENT_SUSP );
            }
            else if( MXC_GETBIT( &event_flags, MAXUSB_EVENT_DPACT ) )
            {
                MXC_CLRBIT( &event_flags, MAXUSB_EVENT_DPACT );
            }
            else if( MXC_GETBIT( &event_flags, EVENT_ENUM_COMP ) )
            {
                MXC_CLRBIT( &event_flags, EVENT_ENUM_COMP );
            }
            else if( MXC_GETBIT( &event_flags, EVENT_REMOTE_WAKE ) )
            {
                MXC_CLRBIT( &event_flags, EVENT_REMOTE_WAKE );
            }
        }
    }
}

void USB_IRQHandler( void )
{
    usb_event_handler();
}

