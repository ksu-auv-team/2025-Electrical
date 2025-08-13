/*
 * i2c_slave.c
 *
 *  Created on: Aug 12, 2025
 *      Author: Juang
 */

#include "i2c_slave.h"
#include "main.h"

static uint8_t i2c_rx[10];
static uint8_t i2c_tx[10];
static volatile uint8_t rx_len = 0;
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
	HAL_Delay(0);
	if (have_new_cmd)
	{
	    have_new_cmd = 0;
	}
	int startREG = RxData[0];  // get the register address
	int numREG = rxcount-1;  // Get the number of registers
	int endREG = startREG + numREG -1;  // calculate the end register
	if (endREG>2)  // There are a total of 10 registers (0-9)
	{
		Error_Handler();
	}

	int indx = 1;  // set the indx to 1 in order to start reading from RxData[1]

	for (int i=0; i<numREG; i++)
	{
		I2C_REGISTERS[startREG++] = RxData[indx++]; // Read the data from RxData and save it in the I2C_REGISTERS
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		pulseWidth = 500 + ((tim1Degrees * 2000) / 180); // pulse width in us
		tim1PWM = pulseWidth / 100; // 1 timer period = 100us
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tim1PWM);

		if (direction)
			tim1Degrees += 1;
		if (!direction)
			tim1Degrees -= 1;

		if (tim1Degrees >= 180)
			direction = 0;
		if (tim1Degrees <= 0)
			direction = 1;
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Master will WRITE to us: first byte is register/len etc. Receive header (1 byte)
        rx_len = 0;
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
    if (rx_len == 0) {
        // We just received the header/length at i2c_rx[0]; now receive the payload
        uint8_t payload = i2c_rx[0];
        if (payload > sizeof(i2c_rx)-1) payload = sizeof(i2c_rx)-1; // bound
        rx_len = 1 + payload;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[1], payload, I2C_LAST_FRAME);
    } else {
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
