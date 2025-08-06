/*
 * i2c.slave.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Juang
 */

#include "i2c_slave.h"

#include "main.h"

uint8_t I2C_REGISTERS[7] = {0,0,0,0,0,0,0};

extern I2C_HandleTypeDef hi2c2;

#define RxSIZE 2
uint8_t RxData[RxSIZE];
uint8_t rxcount = 0;

int countAddr = 0;
int countrxcplt = 0;
int counterror = 0;

void process_data (void)
{

	int startREG = RxData[0];  // get the register address
	int numREG = rxcount-1;  // Get the number of registers
	int endREG = startREG + numREG -1;  // calculate the end register
	if (endREG>9)  // There are a total of 10 registers (0-9)
	{
		Error_Handler();
	}

	int indx = 1;  // set the indx to 1 in order to start reading from RxData[1]
	for (int i=0; i<numREG; i++)
	{
		I2C_REGISTERS[startREG++] = RxData[indx++];  // Read the data from RxData and save it in the I2C_REGISTERS
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
		rxcount = 0;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
	}

	else
	{
		Error_Handler();
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rxcount++;
	if (rxcount < RxSIZE)
	{
		if (rxcount == RxSIZE-1)
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
		}
	}

	if (rxcount == RxSIZE)
	{
		process_data();
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	counterror++;
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == 4)  // AF error
	{
		process_data();
	}
	HAL_I2C_EnableListen_IT(hi2c);
}

