#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "stm32f10x_spi.h"              // Device:StdPeriph Drivers:SPI


void SystemClock_Config();
void GPIO_Config();
void SPI_Config();
void TIM_Config();
void Delay_us(uint32_t timeouut);

uint8_t	data = 0x36;
uint8_t	rec_data = 0;

int main ()
{
	SystemClock_Config();
	GPIO_Config();
	TIM_Config();
	SPI_Config();

	while(1)
	{
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4));
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, data);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		rec_data = SPI_I2S_ReceiveData(SPI1);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config()
{
	/* 
	PA4	->	CS
	PA6	->	MISO
	PA7	->	MOSI
	PA5	->	CLK
	*/
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SPI_Config()
{
	SPI_InitTypeDef	SPI_InitStruct = {0};
	
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);
}

void TIM_Config()
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct = {0};
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
}

void Delay_us(uint32_t timeout)
{
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < timeout);
}