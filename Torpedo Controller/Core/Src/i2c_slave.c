/*
 * i2c.slave.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Juang
 */

#include "i2c_slave.h"
#include "main.h"
#include <string.h>

static uint8_t i2c_rx[10];
static uint8_t i2c_tx[10];
static volatile uint8_t rx_count = 0;
static volatile uint8_t tx_len = 0;
static volatile uint8_t have_new_cmd = 0;

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch2;

extern unsigned char tim1Degrees;
extern unsigned char direction;
extern float tim1PWM;
extern float pulseWidth;

#define RxSIZE 10

uint8_t RxData[RxSIZE];
uint8_t rxcount = 0;
uint8_t I2C_REGISTERS[10] = {0,0,0,0,0,0,0,0,0,0};

int is_first_recvd = 0;
int countAddr = 0;
int countrxcplt = 0;
int counterror = 0;

static inline void i2c_listen(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void process_data (void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

	int angle = I2C_REGISTERS[0];
	if (angle < 0) angle = 0;
	if (angle > 180) angle = 180;

	direction = I2C_REGISTERS[1];

	pulseWidth = 500 + ((angle * 2000)/180); // Calculate pulse width in microseconds

	tim1PWM = pulseWidth / 100; // Convert to timer ticks (100 µs per tick if period = 100us)

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tim1PWM);

	tim1Degrees = angle;

	if (have_new_cmd)
		{
			__disable_irq();
			have_new_cmd = 0;
			// Save incoming data into register array
		    for (int i = 0; i < rxcount; i++)
		    {
		    	I2C_REGISTERS[i] = RxData[i];
		    }
		    __enable_irq();
		    process_data();
		}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Master will WRITE to us: first byte is register/len etc. Receive header (1 byte)
        rx_count = 0;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[0], 1, I2C_FIRST_AND_NEXT_FRAME);
    } else {
        // Master will READ from us: prepare a small status/register window
        // Example: return 3 bytes [reg0, reg1, reg2]; adapt as you need
        i2c_tx[0] = I2C_REGISTERS[0];
        i2c_tx[1] = I2C_REGISTERS[1];
        i2c_tx[2] = I2C_REGISTERS[2];
        tx_len = 3;

        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx, tx_len, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (rx_count == 0)
    {
        // We just received the header/length at i2c_rx[0]; now receive the payload
        uint8_t payload = i2c_rx[0];
        if (payload > sizeof(i2c_rx)-1) payload = sizeof(i2c_rx)-1; // bound
        rx_count = 1 + payload;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[1], payload, I2C_LAST_FRAME);
    } else {
    	// Copy data to RxData and update count
        memcpy(RxData, i2c_rx, rx_count);
        rxcount = rx_count;
    	// Full message received: set a flag for main loop to process
        have_new_cmd = 1;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Done responding; go back to listening
    i2c_listen(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Re-enter listen mode right away
    i2c_listen(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // DO NOT heavy-handle in IRQ; just recover listen
    (void)HAL_I2C_GetError(hi2c);
    i2c_listen(hi2c);
}
