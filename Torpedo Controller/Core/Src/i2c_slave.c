/*
 * i2c.slave.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Juang
 */

#include "i2c_slave.h"
#include "main.h"

static uint8_t i2c_rx[2]; // Only need 2 bytes now
static volatile uint8_t have_new_cmd = 0;

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;

extern unsigned char tim1Degrees;
extern unsigned char direction;
extern float tim1PWM;
extern float pulseWidth;

static inline void i2c_listen(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void process_data(void)
{
    if (!have_new_cmd)
        return;

    have_new_cmd = 0;

    // First byte = angle, second byte = direction
    tim1Degrees = i2c_rx[0];
    direction   = i2c_rx[1];

    // Clamp the angle to 0–180
    if (tim1Degrees > 180)
        tim1Degrees = 180;

    // Compute pulse width in microseconds (500–2500us range)
    pulseWidth = 500 + ((tim1Degrees * 2000) / 180);
    tim1PWM = pulseWidth / 100; // Assuming timer tick = 100us

    // Start PWM and set compare
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tim1PWM);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Expect exactly 2 bytes from master
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx, 2, I2C_LAST_FRAME);
    } else {
        // Nothing to send back; just re-listen
        uint8_t dummy_tx[1] = {0};
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, dummy_tx, 1, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    have_new_cmd = 1;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_listen(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_listen(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    (void)HAL_I2C_GetError(hi2c);
    i2c_listen(hi2c);
}
