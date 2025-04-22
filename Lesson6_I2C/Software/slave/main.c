#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define SCL_RESET_0			GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SCL_SET_1				GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define SCL_GEN_CLK(x)	do	{	Delay_us(x); \
															SCL_SET_1;	\
															Delay_us(x);	\
															SCL_RESET_0;	\
															Delay_us(x);	\
														}	while(0)
#define SDA_RESET_0			GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define SDA_SET_1				GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define SDA_READ				GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)	
#define RW_READ					1
#define RW_WRITE				0
#define	E_OK						1
#define E_NOT_OK				0
#define MY_ADDR					0x60

void SystemClock_Config();
void GPIO_Config();
void Delay_us(uint32_t	timeout);

uint8_t I2C_SlaveCheckAddress(uint8_t numBit);
uint8_t I2C_SlaveSend8BitData(uint8_t data);
uint8_t I2C_SlaveRecieve8BitData(uint8_t *data);

uint32_t	Timingdelay;
uint8_t rwBit;
														
int main ()
{
	SystemClock_Config();
	SysTick_Config(SystemCoreClock/1000000);
	GPIO_Config();
	SDA_SET_1;
	SCL_SET_1;
	Delay_us(100);
	while (1)
	{
		
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Config()
{
	/*
		PB6	->	SCL
		PB7	->	SDA*/
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SysTick_Handler(void)
{
	Timingdelay--;
}

void Delay_us(uint32_t	timeout)
{
	Timingdelay = timeout;
	while (Timingdelay != 0);
}

uint8_t I2C_SlaveCheckAddress(uint8_t numBit)
{
	uint8_t retVal = E_OK;
	uint8_t address[2] = {0};
	uint16_t conv_addr = 0;
	
	if (numBit == 7)
	{
		retVal = I2C_SlaveRecieve8BitData(address);
		conv_addr = (address[0] & 0xFE) >> 1;
		rwBit = address[0] &0x01;
	}
	else if (numBit == 10)
	{
		retVal = I2C_SlaveRecieve8BitData(address);
		uint8_t ff_addr = (address[0] & 0xFE) >> 1;
		uint8_t sf_addr = address[1];
		conv_addr = ((uint16_t)(ff_addr << 7)) | (sf_addr);
		rwBit = address[0] & 0x01;
	}
	else
	{
		retVal = E_NOT_OK;
	}
	if (retVal == E_OK)
	{
		if ((conv_addr == MY_ADDR ) || (conv_addr == (MY_ADDR | 0x7BFF)))
			SDA_RESET_0;
		else
			SDA_SET_1;
		Delay_us(1);			
	}
	return retVal;
}

uint8_t I2C_SlaveSend8BitData(uint8_t	data)
{
	uint8_t retVal;
	for (uint8_t i=0; i<7; i++)
	{
		if ((data & 0x80) == SET)
			SDA_SET_1;
		else
			SDA_RESET_0;
		data <<=1;
		SCL_GEN_CLK(2);
	}
	SDA_SET_1;
	Delay_us(1);
	SCL_SET_1;
	Delay_us(1);
	if (SDA_READ == SET)
		retVal = E_NOT_OK;
	else
		retVal = E_OK;
	Delay_us(1);
	SCL_RESET_0;
	Delay_us(2);
	return retVal;
}

uint8_t I2C_SlaveRecieve8BitData(uint8_t *data)
{
	uint8_t status = E_OK;
	
	for (uint8_t i=0; i<7; i++)
	{
		if (SDA_READ == SET)
		{
			SCL_SET_1;
			Delay_us(1);
			if (SDA_READ == SET)
				*data |= (1 << (7-i));
			SCL_RESET_0;
			Delay_us(1);
		}
		else
		{
			SCL_SET_1;
			Delay_us(1);
			if (SDA_READ == SET)
				*data |= (0 << (7-i));
			SCL_RESET_0;
			Delay_us(1);
		}
	}
	SDA_SET_1;
	Delay_us(1);
	SCL_SET_1;
	Delay_us(1);
	if (SDA_READ == SET)
		status = E_NOT_OK;
	else
		status = E_OK;
	Delay_us(1);
	SCL_RESET_0;
	Delay_us(2);
	return status;
}