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

void SystemClock_Config();
void GPIO_Config();
void Delay_us(uint32_t	timeout);
void I2C_MasterGenStartCond();
void I2C_MasterGenStopCond();
uint8_t I2C_MasterSendAddress(uint16_t	Address, uint8_t numBit, uint8_t	rwBit);
uint8_t I2C_MasterSend8BitData(uint8_t data);
uint8_t I2C_MasterRecieve8BitData(uint8_t *data);

uint32_t	Timingdelay;

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

void I2C_MasterGenStartCond()
{
	SDA_SET_1;
	SCL_SET_1;
	Delay_us(5);
	SDA_RESET_0;
	Delay_us(1);
	SCL_RESET_0;
	Delay_us(1);
}

void I2C_MasterGenStopCond()
{
	SDA_RESET_0;
	Delay_us(1);
	SCL_SET_1;
	Delay_us(1);
	SDA_SET_1;
	Delay_us(1);
}

uint8_t I2C_MasterSendAddress(uint16_t	Address, uint8_t numBit, uint8_t	rwBit)
{
	uint16_t padding = 0x38u; //111000
	uint8_t retVal = E_OK;
	if (numBit == 7)
	{
		for (uint8_t i=0; i<6; i++)
		{
			if ((Address & 0x40) == SET)
				SDA_SET_1;
			else
				SDA_RESET_0;
			Address <<= 1;
			SCL_GEN_CLK(2);
		}
		if (rwBit == RW_READ)
			SDA_SET_1;
		else
			SDA_RESET_0;
		SCL_GEN_CLK(1);	
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
	}
	else if (numBit == 10)
	{
		uint8_t addr_FirstFrame = padding | (Address >> 8);
		for (uint8_t i=0; i<6; i++)
		{
			if ((addr_FirstFrame & 0x40) == SET)
				SDA_SET_1;
			else
				SDA_RESET_0;
			addr_FirstFrame <<= 1;
			SCL_GEN_CLK(2);		
		}
		if (rwBit == RW_READ)
			SDA_SET_1;
		else
			SDA_RESET_0;
		SCL_GEN_CLK(2);
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
		if (retVal == E_NOT_OK)
			return retVal;
		else
		{
			I2C_MasterSend8BitData((uint8_t)(Address & 0xFF));
		}
	}
	return retVal;
}

uint8_t I2C_MasterSend8BitData(uint8_t	data)
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

uint8_t I2C_MasterRecieve8BitData(uint8_t *data)
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