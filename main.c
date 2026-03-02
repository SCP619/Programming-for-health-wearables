/**
 * Copyright (c) 2009 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrfx_twi.h"
#include "boards.h"
#include "lsm6dso_reg.h"
#include "lsm6dso_platform.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "app_error.h"
#include "arm_math.h"
#include "fdacoefs.h"
#include "tmwtypes.h"

/**
 * Constants and global variables
 */
#define TWI_INSTANCE_ID 0
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);
stmdev_ctx_t dev_ctx;
uint8_t whoami;

static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
nrfx_err_t err_code;

#define NUM_TAPS 50
arm_fir_instance_q15 fir_lpf;
static q15_t firStateQ15[NUM_TAPS + 1];
static float z_filtered_mg; 
static float z_raw_mg;

void twi_init(void)
{
    nrfx_twi_config_t twi_config = NRFX_TWI_DEFAULT_CONFIG;

    twi_config.scl = 27;
    twi_config.sda = 26;
    twi_config.frequency = NRF_TWI_FREQ_400K;
    
    err_code = nrfx_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    if (err_code != NRFX_SUCCESS)
    {
        while(1);
    }
    
    nrfx_twi_enable(&m_twi);
}

/**
 * Function for configuring the sensor
 */
void lsm6dso_setup(void)
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay_ms;
    dev_ctx.handle = (void *)&m_twi;

    lsm6dso_device_id_get(&dev_ctx, &whoami);

    if (whoami != LSM6DSO_ID)
    {
        while (1);
    }

    lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set accelerometer ODR to 104 Hz (as per PDF requirement)
    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz);
    
    // Set accelerometer full scale to ±2g
    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
}

q15_t fir_process_sample(q15_t in)
{
    q15_t out;
    arm_fir_q15(&fir_lpf, &in, &out, 1);
    return out;
}

void sensor_data_polling(void)
{
    uint8_t reg;

    lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg)
    {
        // Read raw acceleration data
        lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

        // Convert raw data to mg for display
        for (uint8_t i = 0; i < 3; i++)
        {
            acceleration_mg[i] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[i]);
        }
        
        // Store raw Z-axis value in mg
        z_raw_mg = acceleration_mg[2];
        
        // Process RAW sensor data through FIR filter
        // data_raw_acceleration is int16_t, which is compatible with q15_t
        q15_t z_raw_q15 = (q15_t)data_raw_acceleration[2];
        q15_t z_filtered_q15 = fir_process_sample(z_raw_q15);
        
        // Convert filtered q15_t value to mg
        z_filtered_mg = lsm6dso_from_fs2_to_mg(z_filtered_q15);

        // Print raw and filtered values (tab-separated for serial plotter)
        printf("%.2f\t%.2f\r\n", z_raw_mg, z_filtered_mg);
    }
}

void uart_error_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no = 8,
        .tx_pin_no = 6,
        .rts_pin_no = RTS_PIN_NUMBER,
        .cts_pin_no = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
        .baud_rate = NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * Function for initializing the FIR filter instance
 */
void dsp_init(void)
{
    // Cast int16_T* to q15_t* (they're the same type)
    // Need to cast away const
    arm_fir_init_q15(&fir_lpf, 
                     NUM_TAPS, 
                     (q15_t *)B,  // Cast const int16_T* to q15_t*
                     firStateQ15, 
                     1);
    
    //printf("Filter init: B[0]=%d, B[1]=%d, B[2]=%d\r\n", B[0], B[1], B[2]);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uart_init();
    //printf("Starting initialization...\r\n");
    
    twi_init();
    //printf("TWI initialized\r\n");
    
    lsm6dso_setup();
    //printf("Sensor initialized. WHO_AM_I = 0x%02X\r\n", whoami);
    
    dsp_init();
    //printf("FIR filter initialized\r\n");
    
    //printf("Starting data acquisition...\r\n");

    while (1)
    {
        sensor_data_polling();
        platform_delay_ms(NULL, 10);
    }
}