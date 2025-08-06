#include "main.h"
#include "i2c_slave.h"

uint8_t I2C_REGISTERS[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t datatosend[6] = {1,2,3,4,5,6};

extern I2C_HandleTypeDef hi2c2;

#define RxSIZE 2
uint8_t RxData[RxSIZE];
uint8_t rxcount = 0;
uint8_t txcount = 0;
uint8_t startPosition = 0;

int is_first_recvd = 0;
int countAddr = 0;
int countrxcplt = 0;
int counterror = 0;
int count = 0;

void process_data (void)
{
	int startREG = RxData[0];
	int numREG = rxcount - 1;
	int endREG = startREG + numREG-1;
	if (endREG > 9) Error_Handler();
	
	int indx = 1;
	for (int i = 0; i < numREG; i++)
	{
		I2C_REGISTERS[startREG++] = RxData[indx++];
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Set PA5 HIGH
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Set PA5 LOW
	HAL_Delay(100);
	
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)
	{
		if (is_first_recvd == 0)
		{
			rxcount = 0;
			countAddr++;
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
		}
	}
	else
	{
		txcount = 0;
		startPosition = RxData[0];
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txcount, 1, I2C_NEXT_FRAME);
	}
}

void HAl_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (is_first_recvd == 0)
	{
		rxcount++;
		is_first_recvd = 1;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, RxData[0], I2C_LAST_FRAME);
	}
	else
	{
		rxcount = rxcount+RxData[0];
		is_first_recvd=0;
		process_data();
	}
}

void HAL_I2C_SlaveTxCpltCallBack(I2C_HandleTypeDef *hi2c)
{
	txcount++;
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txcount, 1, I2C_FIRST_FRAME);
}

void HAl_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	counterror++;
	uint8_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == 4)
	{
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
		if (txcount == 0)
		{
			process_data();
		}
		else
		{
			txcount = txcount - 1;
		}
	}
	HAL_I2C_EnableListen_IT(hi2c);
}