/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_twi_mngr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_gpiote.h"
#include "app_button.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_twi_sensor.h"
#include "nrf_delay.h"

#include "tcs34725.h"

#include <stdio.h>

#define TWI_INSTANCE_ID 0

#define MAX_PENDING_TRANSACTIONS 20

//TWI PIN
#define TCS34725_SDA_PIN 28
#define TCS34725_SCL_PIN 29
#define TCS34725_INT_PIN 31
#define TCS34725_LED_PIN 30

//Macro that simplifies defining a TWI transaction manager instance.
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

//Macro creating common twi sensor instance.
NRF_TWI_SENSOR_DEF(sensor_instance, &m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS);

//Macro that creates sensor instance.
TCS34725_INSTANCE_DEF(tcs34725_instance, &sensor_instance, TCS34725_ADDR);

APP_TIMER_DEF(m_timer);

tcs34725_color_data_t color_str;


void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void twi_config(void)
{
    uint32_t err_code;
    
    nrf_drv_twi_config_t const config={
      .scl=TCS34725_SCL_PIN,
      .sda=TCS34725_SDA_PIN,
      .frequency=NRF_DRV_TWI_FREQ_400K,
      .interrupt_priority=APP_IRQ_PRIORITY_MID,
      };
    
    err_code=nrf_twi_mngr_init(&m_nrf_twi_mngr,&config);
    APP_ERROR_CHECK(err_code);
}

static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


ret_code_t tcs34725_read_reg(tcs34725_instance_t const * p_instance,
                              tcs34725_reg_data_t *      p_reg_data,
                              tcs34725_data_callback_t   user_cb
                             )
{
    return nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                                   p_instance->sensor_addr,
                                   p_reg_data->reg_addr|0x80,
                                   (nrf_twi_sensor_reg_cb_t) user_cb,
                                   (uint8_t *) p_reg_data,
                                   TCS34725_REGISTER_SIZE);
}


ret_code_t tcs34725_write_reg(tcs34725_instance_t const * p_instance,
                              tcs34725_reg_data_t *       p_reg_data)
{
    return nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                                    p_instance->sensor_addr,
                                    p_reg_data->reg_addr|0x80,
                                    (uint8_t *)&p_reg_data->reg_data,
                                    TCS34725_REGISTER_SIZE);
}




ret_code_t tcs34725_set_init(tcs34725_instance_t const * p_instance,
                              tcs34725_config_enable_t *  config)
{
    ret_code_t err_code;
    tcs34725_reg_data_t enable_reg;

    enable_reg.reg_data=(config->rgbc_interrupt << TCS34725_INT_POS)|
                        (config->wait_enable << TCS34725_WAIT_POS)|
                        (config->rgbc_enable << TCS34725_RGBC_ENABLE_POS)|
                        (config->power_on << TCS34725_POWER_ON_POS);

    NRF_LOG_INFO("init config : %X", enable_reg.reg_data);
    
    enable_reg.reg_addr=TCS34725_REG_ENABLE|0x80;

    tcs34725_write_reg(&tcs34725_instance, &enable_reg);

//    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
//                                    p_instance->sensor_addr,
//                                    TCS34725_REG_ENABLE,
//                                    (uint8_t *)&configuration,
//                                    TCS34725_REGISTER_BYTES);
    
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("TCS34725 Init wrtie failed");
//        return err_code;
    }

    return err_code;
}

ret_code_t tcs34725_init(tcs34725_instance_t const * p_instance)
{
    //int enable, wait enable, rgbc enable, power on/off
    ret_code_t err_code;
    tcs34725_reg_data_t enable_reg;

    tcs34725_config_enable_t init_config;
    init_config.rgbc_interrupt=false;   //bit4
    init_config.wait_enable=true;       //bit3
    init_config.rgbc_enable=true;       //bit1
    init_config.power_on=true;          //bit0

//    err_code=tcs34725_set_init(p_instance, &init_config);
    enable_reg.reg_addr=TCS34725_REG_ENABLE;
    enable_reg.reg_data=(init_config.rgbc_interrupt << TCS34725_INT_POS)|
                        (init_config.wait_enable << TCS34725_WAIT_POS)|
                        (init_config.rgbc_enable << TCS34725_RGBC_ENABLE_POS)|
                        (init_config.power_on << TCS34725_POWER_ON_POS);

    err_code=tcs34725_write_reg(&tcs34725_instance, &enable_reg);
    return err_code;
}

ret_code_t tcs34725_set_timing(tcs34725_instance_t const * p_instance,
                                uint16_t atime)
{
    ret_code_t err_code;
    if((atime==0)||(256 < atime))
    {
        err_code=NRF_ERROR_INVALID_DATA;
        return err_code;
    }

    tcs34725_reg_data_t timing_str;
    timing_str.reg_addr=TCS34725_REG_TIMING;
    timing_str.reg_data=(256-atime);
    
    err_code=tcs34725_write_reg(p_instance,&timing_str);
    return err_code;
}

ret_code_t tcs34725_set_wait_time(tcs34725_instance_t const * p_instance,
                                  uint8_t wait_val)
{
    ret_code_t err_code;
    if((wait_val==0)||(256 < wait_val))
    {
        err_code=NRF_ERROR_INVALID_DATA;
        return err_code;
    }

    tcs34725_reg_data_t wait_time_str;
    wait_time_str.reg_addr=TCS34725_REG_WAIT_TIME;
    wait_time_str.reg_data=wait_val;
    
    err_code=tcs34725_write_reg(p_instance,&wait_time_str);
    return err_code;
}


ret_code_t tcs34725_set_persistence(tcs34725_instance_t const * p_instance,
                                    tcs34725_persistence_t out_of_range_val)
{
    ret_code_t err_code;
    tcs34725_reg_data_t persistence;
    persistence.reg_addr=TCS34725_REG_PERSISTENCE;
    persistence.reg_data=out_of_range_val;
    err_code=tcs34725_write_reg(p_instance, &persistence);
    return err_code;
}

ret_code_t tcs34725_set_wait_long(tcs34725_instance_t const * p_instance,
                                   tcs34725_wait_long_t wait_long_val)
{
    ret_code_t err_code;
    tcs34725_reg_data_t wait_long;
    wait_long.reg_addr=TCS34725_REG_WAIT_TIME;
    wait_long.reg_data=wait_long_val << TCS34725_WAIT_LONG_POS;
    err_code=tcs34725_write_reg(p_instance, &wait_long);
    return err_code;
}

ret_code_t tcs34725_set_gain(tcs34725_instance_t const * p_instance,
                                 tcs34725_gain_t gain_val)
{
    ret_code_t err_code;
    tcs34725_reg_data_t gain;
    gain.reg_addr=TCS34725_REG_CONTROL;
    gain.reg_data=gain_val;
    err_code=tcs34725_write_reg(p_instance, &gain);
    return err_code;
}

ret_code_t tcs34725_read_rgbc(tcs34725_instance_t const * p_instance,
                               tcs34725_color_data_t *     rgbc_str,
                               tcs34725_rgbc_callback_t    user_cb)
{
    ret_code_t err_code;
    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           (TCS34725_REG_CLEAR|0xA0),
                           (nrf_twi_sensor_reg_cb_t) user_cb,
                           (uint8_t *) rgbc_str,
                           TCS34725_RGBC_BYTES);
    if(err_code!=NRF_SUCCESS)
    {
        NRF_LOG_INFO("Read RGBC Failed");
    }
    return err_code;
}



void tcs34725_read_reg_cb(ret_code_t result, tcs34725_reg_data_t * p_raw_data)
{
    if(result!=NRF_SUCCESS)
    {
        NRF_LOG_INFO("TCS34725 register read fail");
        return;
    }
    p_raw_data->reg_addr&=0x1F;

    switch(p_raw_data->reg_addr)
    {
        case TCS34725_REG_ENABLE :
            NRF_LOG_INFO("Enable register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_TIMING :
            NRF_LOG_INFO("Timing register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_WAIT_TIME :
            NRF_LOG_INFO("Wait time register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_PERSISTENCE :
            NRF_LOG_INFO("Persistence register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_CONFIG :
            NRF_LOG_INFO("Configuration register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_CONTROL :
            NRF_LOG_INFO("Control register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_ID :
            NRF_LOG_INFO("ID register : %X",p_raw_data->reg_data);
            break;
        case TCS34725_REG_STATUS :
            NRF_LOG_INFO("Status register : %X",p_raw_data->reg_data);
            break;
        default :
            break;
    }
}

void tcs34725_rgbc_print(tcs34725_color_data_t * color_str)
{
    double max_value;
    float red,blue,green;
    uint16_t c_red,c_green,c_blue;
    uint32_t sum;

    if(color_str->blue <= color_str->red)
    {
        if(color_str->red <= color_str->green)
        {
            max_value=color_str->green;
        }
        else
        {
            max_value=color_str->red;
        }
    }
    else if(color_str->red <= color_str->blue)
    {
        if(color_str->blue <= color_str->green)
        {
            max_value=color_str->green;
        }
        else
        {
            max_value=color_str->blue;
        }
    }
    if(color_str->red <= color_str->green)
    {
        if(color_str->green <= color_str->blue)
        {
            max_value=color_str->blue;
        }
        else
        {
            max_value=color_str->green;
        }
    }

    sum=color_str->red+color_str->green+color_str->blue;

    c_red=(int)((double)color_str->red/sum*255);
    c_green=(int)((double)color_str->green/sum*255);
    c_blue=(int)((double)color_str->blue/sum*255);

//    NRF_LOG_INFO("Original");
    NRF_LOG_INFO("Clear : %d",color_str->clear);
//    NRF_LOG_INFO("Red   : %d",c_red);
//    NRF_LOG_INFO("Green : %d",c_green);
//    NRF_LOG_INFO("Blue  : %d",c_blue);

    c_red=(int)((double)color_str->red/color_str->clear*255);
    c_green=(int)((double)color_str->green/color_str->clear*255);
    c_blue=(int)((double)color_str->blue/color_str->clear*255);

//    NRF_LOG_INFO("Clear");
//    NRF_LOG_INFO("Red   : %d",c_red);
//    NRF_LOG_INFO("Green : %d",c_green);
//    NRF_LOG_INFO("Blue  : %d",c_blue);
//    NRF_LOG_INFO("Clear");
    NRF_LOG_INFO("Red   : %d",c_red);
    NRF_LOG_INFO("Green : %d",c_green);
    NRF_LOG_INFO("Blue  : %d",c_blue);

    red=(color_str->red/max_value);
    green=(color_str->green/max_value);
    blue=(color_str->blue/max_value);

    color_str->red=(int)(red*255);
    color_str->green=(int)(green*255);
    color_str->blue=(int)(blue*255);
    
//    NRF_LOG_INFO("Max");
//    NRF_LOG_INFO("Red   : %d",color_str->red);
//    NRF_LOG_INFO("Green : %d",color_str->green);
//    NRF_LOG_INFO("Blue  : %d",color_str->blue);
}

void tcs34725_rgbc_callback(ret_code_t result, tcs34725_color_data_t * p_raw_data)
{
    if(result!=NRF_SUCCESS)
    {
        NRF_LOG_INFO("tcs reg callback failed");
        return;
    }
    tcs34725_rgbc_print(p_raw_data);
}


static void tcs34725_calibration()
{
    
}

ret_code_t tcs34725_set_threshold(tcs34725_instance_t const * p_instance,
                                  tcs34725_threshold_lh_t threshold_low_high,
                                  uint16_t threshold_val)
{
    ret_code_t err_code;
    tcs34725_threshold_data_t threshold_str;
    threshold_str.threshold_data=threshold_val;

    if(threshold_low_high==TCS34725_THRESHOLD_LOW)
    {
        threshold_str.reg_addr=TCS34725_REG_THRESHOLD_LOW_L;
    }
    else if(threshold_low_high==TCS34725_THRESHOLD_HIGH)
    {
        threshold_str.reg_addr=TCS34725_REG_THRESHOLD_HIGH_L;
    }
    else
    {
        err_code=NRF_ERROR_INVALID_ADDR;
        return err_code;
    }

    NRF_LOG_INFO("Set threshold value : %d",threshold_str.threshold_data);
    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                                      p_instance->sensor_addr,
                                      threshold_str.reg_addr|0xA0,
                                      (uint8_t *)&threshold_str,
                                      TCS34725_THRESHOLD_BYTES);
}

void tcs34725_read_thr_cb(ret_code_t result, tcs34725_threshold_data_t * p_reg_data)
{
    if(result!=NRF_SUCCESS)
    {
        NRF_LOG_INFO("Reading threshold regiseter is failed");
        return;
    }
    if(p_reg_data->reg_addr==TCS34725_REG_THRESHOLD_LOW_L)
    {
        NRF_LOG_INFO("Threshold Low value : %d",p_reg_data->threshold_data);
    }
    else
    {
        NRF_LOG_INFO("Threshold High value : %d",p_reg_data->threshold_data);
    }
}


ret_code_t tcs34725_read_threshold(tcs34725_instance_t const * p_instance, 
                                   tcs34725_threshold_lh_t thr_low_high,
                                   tcs34725_threshold_callback_t user_cb)
{
    ret_code_t err_code;
    static tcs34725_threshold_data_t thr_data_str;

    if(thr_low_high==TCS34725_THRESHOLD_LOW)
    {
        thr_data_str.reg_addr=TCS34725_REG_THRESHOLD_LOW_L;
    }
    else if(thr_low_high==TCS34725_THRESHOLD_HIGH)
    {
        thr_data_str.reg_addr=TCS34725_REG_THRESHOLD_HIGH_L;
    }
    else
    {
        err_code=NRF_ERROR_INVALID_ADDR;
        return err_code;
    }

    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                                     p_instance->sensor_addr,
                                     thr_data_str.reg_addr|0xA0,
                                     (nrf_twi_sensor_reg_cb_t) user_cb,
                                     (uint8_t *) &thr_data_str,
                                     TCS34725_THRESHOLD_BYTES);
    return err_code;
}


ret_code_t tcs34725_int_clear(tcs34725_instance_t const * p_instance);

void timer_handler(void * p_context)
{
    ret_code_t err_code;

    tcs34725_read_rgbc(&tcs34725_instance, &color_str, tcs34725_rgbc_callback);
    tcs34725_int_clear(&tcs34725_instance);
    APP_ERROR_CHECK(err_code);
}

void timer_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);
}

ret_code_t tcs34725_set_interrupt(tcs34725_instance_t const * p_instance,
                                  tcs34725_int_enable_t int_enable);
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    if((pin_no==BSP_BUTTON_0)&&(button_action==APP_BUTTON_PUSH))
    {
        NRF_LOG_INFO("TCS34725 LED toggle");
        nrf_gpio_pin_toggle(TCS34725_LED_PIN);
    }
    if((pin_no==BSP_BUTTON_1)&&(button_action==APP_BUTTON_PUSH))
    {
        NRF_LOG_INFO("TCS34725 Interrupt enable");
        tcs34725_set_interrupt(&tcs34725_instance,TCS34725_INTERRUPT_ENABLE);
    }
    if((pin_no==BSP_BUTTON_2)&&(button_action==APP_BUTTON_PUSH))
    {
        NRF_LOG_INFO("TCS34725 Interrupt disable")
        tcs34725_set_interrupt(&tcs34725_instance,TCS34725_INTERRUPT_DISABLE);
    }

//    switch(button_action)
//    {
//        case APP_BUTTON_PUSH :
//            nrf_gpio_pin_toggle(TCS34725_LED_PIN);
//            break;
//        case APP_BUTTON_RELEASE :
//            break;
//    }
    
}

static void buttons_init(void)
{
    ret_code_t err_code;

    nrf_gpio_cfg_output(TCS34725_LED_PIN);
    nrf_gpio_pin_write(TCS34725_LED_PIN,1);

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BSP_BUTTON_0, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {BSP_BUTTON_1, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {BSP_BUTTON_2, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
}

void tcs34725_read_all_config(tcs34725_instance_t const * p_instance, tcs34725_data_callback_t user_cb)
{
    NRF_LOG_INFO("read all config");
    tcs34725_reg_data_t enable,timing,waittime,persistence,config,control,id,status;

    enable.reg_addr=TCS34725_REG_ENABLE;
    tcs34725_read_reg(p_instance, &enable, user_cb);

    timing.reg_addr=TCS34725_REG_TIMING;
    tcs34725_read_reg(p_instance, &timing, user_cb);

    waittime.reg_addr=TCS34725_REG_WAIT_TIME;
    tcs34725_read_reg(p_instance, &waittime, user_cb);

    persistence.reg_addr=TCS34725_REG_PERSISTENCE;
    tcs34725_read_reg(p_instance, &persistence, user_cb);

    config.reg_addr=TCS34725_REG_CONFIG;
    tcs34725_read_reg(p_instance, &config, user_cb);

    control.reg_addr=TCS34725_REG_CONTROL;
    tcs34725_read_reg(p_instance, &control, user_cb);

    id.reg_addr=TCS34725_REG_ID;
    tcs34725_read_reg(p_instance, &id, user_cb);

    status.reg_addr=TCS34725_REG_STATUS;
    tcs34725_read_reg(p_instance, &status, user_cb);
}


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(pin==TCS34725_INT_PIN)
    {
        NRF_LOG_INFO("TCS34725 RGBC Interrupt occured");
        tcs34725_int_clear(&tcs34725_instance);
    }
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(TCS34725_INT_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TCS34725_INT_PIN, true);
}

ret_code_t tcs34725_int_clear(tcs34725_instance_t const * p_instance)
{
    ret_code_t err_code;
    uint8_t interrupt_cmd=0x66|0x80;    //special function+cmd 1
    err_code=nrf_twi_sensor_write(p_instance->p_sensor_data, p_instance->sensor_addr, &interrupt_cmd, 
                                  TCS34725_REGISTER_SIZE, true);
    return err_code;
}


ret_code_t tcs34725_set_interrupt(tcs34725_instance_t const * p_instance,
                                  tcs34725_int_enable_t int_enable)
{
    ret_code_t err_code;
    tcs34725_reg_data_t enable_reg_str;
    enable_reg_str.reg_addr=TCS34725_REG_ENABLE;
    tcs34725_read_reg(p_instance,&enable_reg_str,NULL);
    
    do
    {
        nrf_delay_us(10);
    }while(nrf_twi_mngr_is_idle(&m_nrf_twi_mngr)!=true);

    if(int_enable==TCS34725_INTERRUPT_ENABLE)
    {
        enable_reg_str.reg_data=(enable_reg_str.reg_data|(TCS34725_INT_MASK));
    }
    else if(int_enable==TCS34725_INTERRUPT_DISABLE)
    {
        enable_reg_str.reg_data=(enable_reg_str.reg_data&~(TCS34725_INT_MASK));
    }
    else
    {
        err_code=NRF_ERROR_INVALID_PARAM;
        return err_code;
    }
    err_code=tcs34725_write_reg(p_instance, &enable_reg_str);
    return err_code;
}

int main(void)
{
    ret_code_t err_code;

    log_init();
    bsp_board_init(BSP_INIT_LEDS);

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_RAW_INFO("\r\nTWI master example started. \r\n");
    NRF_LOG_FLUSH();

    gpio_init();
    buttons_init();
    twi_config();
    err_code=nrf_twi_sensor_init(&sensor_instance);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_FLUSH();
    
    tcs34725_init(&tcs34725_instance);
    tcs34725_set_timing(&tcs34725_instance, 130);  //73 1~256 Max RGBC Count = Input Value ¡¿ 1024 up to a maximum of 65535
    tcs34725_set_wait_time(&tcs34725_instance, 255);    //1~256
    tcs34725_set_persistence(&tcs34725_instance, TCS34725_OUT_OF_RANGE_10);
    tcs34725_set_gain(&tcs34725_instance, TCS34725_GAIN_x60);
//    tcs34725_set_wait_long(&tcs34725_instance, TCS34725_WAIT_LONG_ENABLE);
    NRF_LOG_FLUSH();

    tcs34725_read_all_config(&tcs34725_instance, tcs34725_read_reg_cb);

    tcs34725_set_threshold(&tcs34725_instance, TCS34725_THRESHOLD_LOW, 10000);
    tcs34725_set_threshold(&tcs34725_instance, TCS34725_THRESHOLD_HIGH, 65535);

    tcs34725_read_threshold(&tcs34725_instance, TCS34725_THRESHOLD_LOW, tcs34725_read_thr_cb);
    nrf_delay_ms(10);
    tcs34725_read_threshold(&tcs34725_instance, TCS34725_THRESHOLD_HIGH, tcs34725_read_thr_cb);
    NRF_LOG_FLUSH();

    timer_init();

//    nrf_gpio_cfg_output(30);
//    nrf_gpio_pin_write(30,0); //led off

    while (true)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}

/** @} */
