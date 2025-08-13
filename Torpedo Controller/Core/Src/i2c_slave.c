/*
 * i2c.slave.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Juang
 */

#include "i2c_slave.h"
#include "main.h"

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

void process_data (void)
{

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
		HAL_Delay(28);
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) // Puts to listen mode to look for req from master
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		if (is_first_recvd == 0)
		{
			rxcount = 0;
			countAddr++;
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
			HAL_Delay(28);
		}
	}

	else
	{
		Error_Handler();
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (is_first_recvd == 0)
	{
		rxcount ++;
		is_first_recvd = 1;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, RxData[0], I2C_LAST_FRAME);
	}
	else
	{
		rxcount = rxcount+RxData[0];
		is_first_recvd = 0;
		process_data();
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	counterror++;
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == 4) //AF error
	{
		process_data();
	}
	HAL_I2C_EnableListen_IT(hi2c);
}

