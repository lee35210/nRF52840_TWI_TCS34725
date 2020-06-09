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
    free(p_raw_data);
}

void tcs34725_rgbc_callback(ret_code_t result, tcs34725_color_data_t * p_raw_data)
{
    if(result!=NRF_SUCCESS)
    {
        NRF_LOG_INFO("tcs rgbc callback failed");
        return;
    }
    tcs34725_rgbc_print(p_raw_data);
    free(p_raw_data);
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
    free(p_reg_data);
}

void timer_handler(void * p_context)
{
    ret_code_t err_code;

    tcs34725_color_data_t *color_str=(tcs34725_color_data_t*)malloc(sizeof(tcs34725_color_data_t));

    //RGBC read
    tcs34725_read_rgbc(&tcs34725_instance, color_str, tcs34725_rgbc_callback);
    APP_ERROR_CHECK(err_code);
}

void timer_init(void)
{
    ret_code_t err_code;

    //타이머 생성, 반복 모드, 호출될 핸들러
    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    //APP_TIMER_TICKS()에 입력된 시간마다 핸들러 호출(           ms 단위)
    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);
}


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


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(pin==TCS34725_INT_PIN)
    {
        ret_code_t err_code;
        NRF_LOG_INFO("TCS34725 RGBC Interrupt occured");
        err_code=tcs34725_int_clear(&tcs34725_instance);
        APP_ERROR_CHECK(err_code);
        if(err_code==NRF_SUCCESS)
        {
            NRF_LOG_INFO("TCS34725 Clear channel interrupt clear");
        }

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

void tcs34725_start()
{
    ret_code_t err_code;

    err_code=tcs34725_init(&tcs34725_instance);
    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_timing(&tcs34725_instance, 120);  //1~256
    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_wait_time(&tcs34725_instance, 255);   //1~256
    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_persistence(&tcs34725_instance, TCS34725_OUT_OF_RANGE_3);
    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_gain(&tcs34725_instance, TCS34725_GAIN_x60);
    APP_ERROR_CHECK(err_code);

//    err_code=tcs34725_set_wait_long(&tcs34725_instance, TCS34725_WAIT_LONG_ENABLE);
//    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_threshold(&tcs34725_instance, TCS34725_THRESHOLD_LOW, 10000);
    APP_ERROR_CHECK(err_code);

    err_code=tcs34725_set_threshold(&tcs34725_instance, TCS34725_THRESHOLD_HIGH, 65535);
    APP_ERROR_CHECK(err_code);
}

void tcs34725_read_config()
{
    ret_code_t err_code;

    tcs34725_reg_data_t *enable=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *timing=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *wait_time=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *persistence=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *config=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *control=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *id=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    tcs34725_reg_data_t *status=(tcs34725_reg_data_t*)malloc(sizeof(tcs34725_reg_data_t));
    
    tcs34725_threshold_data_t *threshold_low=(tcs34725_threshold_data_t*)malloc(sizeof(tcs34725_threshold_data_t));
    tcs34725_threshold_data_t *threshold_high=(tcs34725_threshold_data_t*)malloc(sizeof(tcs34725_threshold_data_t));


    enable->reg_addr=TCS34725_REG_ENABLE;
    err_code=tcs34725_read_reg(&tcs34725_instance, enable, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);

    timing->reg_addr=TCS34725_REG_TIMING;
    err_code=tcs34725_read_reg(&tcs34725_instance, timing, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);

    wait_time->reg_addr=TCS34725_REG_WAIT_TIME;
    err_code=tcs34725_read_reg(&tcs34725_instance, wait_time, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);
            
    persistence->reg_addr=TCS34725_REG_PERSISTENCE;
    err_code=tcs34725_read_reg(&tcs34725_instance, persistence, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);
    
    config->reg_addr=TCS34725_REG_CONFIG;
    err_code=tcs34725_read_reg(&tcs34725_instance, config, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);
    
    control->reg_addr=TCS34725_REG_CONTROL;
    err_code=tcs34725_read_reg(&tcs34725_instance, control, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);

    id->reg_addr=TCS34725_REG_ID;
    err_code=tcs34725_read_reg(&tcs34725_instance, id, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);

    status->reg_addr=TCS34725_REG_STATUS;
    err_code=tcs34725_read_reg(&tcs34725_instance, status, tcs34725_read_reg_cb);
    APP_ERROR_CHECK(err_code);

    threshold_low->reg_addr=TCS34725_REG_THRESHOLD_LOW_L;
    err_code=tcs34725_read_threshold(&tcs34725_instance, threshold_low, tcs34725_read_thr_cb);
    APP_ERROR_CHECK(err_code);

    threshold_high->reg_addr=TCS34725_REG_THRESHOLD_HIGH_L;
    err_code=tcs34725_read_threshold(&tcs34725_instance, threshold_high, tcs34725_read_thr_cb);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t err_code;

    log_init();
    bsp_board_init(BSP_INIT_LEDS);
    lfclk_config();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_RAW_INFO("\r\nTWI master example started. \r\n");
    NRF_LOG_FLUSH();
    
    //TCS34725 Interrupt 핀 체크
    gpio_init();

    //LED 제어 및 인터럽트 활성화, 비활성화
    buttons_init();

    //TWI 초기화 
    twi_config();
    err_code=nrf_twi_sensor_init(&sensor_instance);
    APP_ERROR_CHECK(err_code);
    
    //TCS34725 설정값들 전송
    tcs34725_start();

    //TCS34725 현재 레지스터 설정값들 수신 후 출력
    tcs34725_read_config();
    
    //타이머 시작
    timer_init();

    while (true)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}

/** @} */
