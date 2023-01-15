/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <string.h>

#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_uart.h"

#include "nrf.h"
#include "nrf_drv_twi.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

//custom-included files
#include "bm1383aglv.h"
#include "mpu9250.h"
#include "sf_uart.h"
 
//custom-defined settings
#define STORE_SIZE              1000
#define UART_TX_BUF_SIZE        64                          /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        64                          /**< UART RX buffer size. */
#define SOFT_UART_RX_BUF_SIZE   256                         /**< SOFT UART RX buffer size. */

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

/*self-defined variable*/
volatile bool flag_print_data   = false; 
volatile bool flag_collect_data = false;
volatile bool flag_collect_gps  = false;

float baro_data[STORE_SIZE] = {0.0};
float gyro_data[STORE_SIZE] = {0.0};
uint16_t baro_count  = 0;
uint16_t gyro_count  = 0;
uint16_t total_count = 0;

uint8_t rx_data[UART_RX_BUF_SIZE] = {0};

//GPS data
uint8_t sf_uart_rxcount = 0;
char sf_uart_rxbuffer[SOFT_UART_RX_BUF_SIZE] = {0};

char* GPGGA_pos;
char  latitude[16];
char  longitude[16];


/**
 * @brief UART handler.
 */
void uart0_handler(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        //APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

//uart for 4G module
void uart_init(void){ 
    uint32_t err_code;
    const app_uart_comm_params_t comm_params_0 =
    {
          NEAT_BROAD_RX0_PIN,
          NEAT_BROAD_TX0_PIN,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          #if defined (UART_PRESENT)
                    NRF_UART_BAUDRATE_115200
          #else
                    NRF_UARTE_BAUDRATE_115200
          #endif
    };

    APP_UART_FIFO_INIT(&comm_params_0,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart0_handler,
                         APP_IRQ_PRIORITY_MID,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief TWI initialization.
 */
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = NEAT_BROAD_SCL_PIN,
       .sda                = NEAT_BROAD_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void twi_scan_device(nrf_drv_twi_t const* m_twi){
    
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;
    
    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            printf("TWI device detected at address 0x%x.\r\n", address);
        }
    }

    if (!detected_device)
    {
        printf("No device was found.\r\n");
    }
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //Voltage High to Low
    if (action == NRF_GPIOTE_POLARITY_HITOLO)
    {
        switch (pin)
        {   
            case FELIX_01_SWITCH:
                flag_print_data   = true;
                break;
            case FELIX_02_SWITCH:
                flag_collect_data = true;
                break;
            case FELIX_03_SWITCH:
                flag_collect_gps  = true;
                break;
            default:
                break;
        }
    }
    
}


void gpio_init(void)
{
    nrf_drv_gpiote_init();
    
    nrf_drv_gpiote_in_config_t in_config  = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);  // Set High-to-Low event
    in_config.pull = NRF_GPIO_PIN_PULLUP;   
                                                
    nrf_drv_gpiote_in_init(FELIX_01_SWITCH, &in_config, in_pin_handler);            
    nrf_drv_gpiote_in_init(FELIX_02_SWITCH, &in_config, in_pin_handler);
    nrf_drv_gpiote_in_init(FELIX_03_SWITCH, &in_config, in_pin_handler); 
    nrf_drv_gpiote_in_event_enable(FELIX_01_SWITCH, true);
    nrf_drv_gpiote_in_event_enable(FELIX_02_SWITCH, true);
    nrf_drv_gpiote_in_event_enable(FELIX_03_SWITCH, true);
}

void print_baro_data(){
    uint32_t index = 0;
    while(index < baro_count){
        printf("%.2f\r\n",baro_data[index++]);
        nrf_delay_ms(100);
    }
    baro_count = 0;
    return;
}

void print_gyro_data(){
    uint32_t index = 0;
    while(index < gyro_count){
        printf("%.3f\r\n",gyro_data[index++]);
        nrf_delay_ms(100);
    }
    gyro_count = 0;
    flag_print_data = false;
    return;
}

static void uart_send_command(uint8_t* tx_data, uint8_t length){
    for (uint32_t i = 0; i < length+1; i++)
    {
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);
        nrf_delay_ms(10);
    }
}


static void uart_send_check(void){
    uint8_t  strstr_flag = 1;
    uint8_t* tx_data = (uint8_t*)("AT\r\n");
    uint8_t  cmd_len = strlen(tx_data);

    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);   
    }
    while(strstr(rx_data, "OK") == NULL);
    nrf_delay_ms(1000);


    tx_data = (uint8_t*)("AT+QICSGP=1,1,\"Internet\",\"\",\"\",1\r\n");
    memset(rx_data, '\0', UART_RX_BUF_SIZE);
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);
    }
    while(strstr(rx_data, "OK") == NULL);
    nrf_delay_ms(1000);
    

    tx_data = (uint8_t*)("AT+QIDEACT=1\r\n");
    memset(rx_data, '\0', UART_RX_BUF_SIZE);
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);   
    }
    while(strstr(rx_data, "OK") == NULL);
    nrf_delay_ms(1000);

    tx_data = (uint8_t*)("AT+QIACT=1\r\n");
    memset(rx_data, '\0', UART_RX_BUF_SIZE);
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);   
    }
    while(strstr(rx_data, "OK") == NULL);
    nrf_delay_ms(1000);

    tx_data = (uint8_t*)("AT+QIOPEN=1,0,\"TCP\",\"35.78.125.115\",8900,0,0\r\n");
    memset(rx_data, '\0', UART_RX_BUF_SIZE);
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);   
    }
    while(strstr(rx_data, "+QIOPEN:") == NULL);
    nrf_delay_ms(2000);

}

static void sendto_server(float baro_data){
    uint8_t buffer[8];
    uint8_t* tx_data;
    uint8_t  cmd_len;
    
    tx_data = (uint8_t*)("AT+QISEND=0,7\r\n");
    memset(rx_data, '\0', UART_RX_BUF_SIZE);
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
    for(uint8_t i = 0; i < UART_RX_BUF_SIZE ; i++){
        app_uart_get(rx_data+i);
        nrf_delay_ms(10);   
    }
    while(strstr(rx_data, ">") == NULL);
    nrf_delay_ms(1000);

    //translate float to string
    sprintf(buffer, "%.2f", baro_data);
    tx_data = buffer;
    cmd_len = strlen(tx_data);
    for(uint8_t i = 0; i < cmd_len; i++){
        app_uart_put(*tx_data++);
        nrf_delay_ms(10);
    }
  
    //0x1A means translation is finished
    uint8_t end = 0x1A;
    app_uart_put(end);
    nrf_delay_ms(1000);
} 

void sf_uart_event_handler(uint8_t rx)
{
    if(rx != '\n')
        sf_uart_rxbuffer[sf_uart_rxcount++] = rx;
    else
        sf_uart_rxcount = 0;
    
}

static void gps_buffer_init(){
    memset(latitude, 0, sizeof(latitude));
    memset(longitude, 0, sizeof(longitude));
}

static void gps_collect_data(){
    
    strncpy(latitude, GPGGA_pos+19, 12);
    latitude[11] = '\0';
    printf("%s\r\n", latitude);
    nrf_delay_ms(500);

    strncpy(longitude, GPGGA_pos+31, 13);
    longitude[12] = '\0';
    printf("%s\r\n", longitude);
    nrf_delay_ms(500);

    memset(sf_uart_rxbuffer, 0, SOFT_UART_RX_BUF_SIZE);

}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint8_t err_code;
    float pressure = 0.0; 
    float gyro = 0.0; 
 
    uart_init();
    twi_init();
    gpio_init();
  
    mpu9250_gyro_init(&m_twi);
    bm1383_init(&m_twi, BM1383AGLV_DEVICE_ADDRESS);
    sf_uart_init(APP_SF_UART1, APP_SF_UART_BAUDRATE_9600, sf_uart_event_handler);
    
    gps_buffer_init();
    twi_scan_device(&m_twi);
    //uart_send_check();
    
    while (true)
    {
    
        if(flag_collect_data){
            get_baro_value(&m_twi, &pressure);
            baro_data[baro_count] = pressure;
            baro_count++;
            
            mpu9250_gyro_yaxis(&m_twi, &gyro);
            gyro_data[gyro_count++] = gyro;
            
            if (baro_count == STORE_SIZE){
                flag_collect_data = false;
                //sendto_server(pressure);
            }
        }
    
        if(total_count%200 == 0){
            sf_uart_rxEnable(1);
            while((GPGGA_pos = strstr(sf_uart_rxbuffer, "GPGGA")) == NULL);
            nrf_delay_ms(500); 
            gps_collect_data();
            sf_uart_rxDisable(1);
        }

        if(flag_print_data){
            print_baro_data();
            print_gyro_data();
            flag_print_data = false;  
        }

        total_count++;
        nrf_delay_ms(100);
    }
    
}

/** @} */
