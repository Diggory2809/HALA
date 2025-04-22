#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define SPI_CS_SET_1			GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define SPI_CS_RESET_0		GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define SPI_MOSI_SET_1		GPIO_SetBits(GPIOA, GPIO_Pin_2)
#define SPI_MOSI_RESET_0	GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define SPI_CLK_SET_1			GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define SPI_CLK_RESET_0		GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define SPI_CLK_READ			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)
#define SPI_MISO_READ			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)

void SystemClock_Config();
void GPIO_Config();
void TIM_Config();
void Delay_us(uint32_t timeouut);
void SpiSw_MasterWrite8BitData(uint8_t	data);
uint8_t SpiSw_MasterRead8BitData();

uint8_t	data = 0x32u;
uint8_t	rec_data = 0;

int main ()
{
	SystemClock_Config();
	GPIO_Config();
	TIM_Config();

	SPI_CS_SET_1;
	SPI_MOSI_RESET_0;
	SPI_CLK_RESET_0;
	Delay_us(10000);
	while(1)
	{
		SpiSw_MasterWrite8BitData(data);
		Delay_us(10000);
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config()
{
	/* 
	PA0	->	CS
	PA1	->	MISO
	PA2	->	MOSI
	PA3	->	CLK
	*/
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TIM_Config()
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct = {0};
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM2, ENABLE);
}

void Delay_us(uint32_t timeout)
{
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < timeout);
}

void SpiSw_MasterWrite8BitData(uint8_t	data)
{
	/* Gen Start condition */
	SPI_CS_RESET_0;
	Delay_us(1000);				
	/* Transfer data */
	for (uint8_t i = 0; i<8; i++)
	{
			if (((data & 0x80u)>>7) == SET)
				SPI_MOSI_SET_1;
			else
				SPI_MOSI_RESET_0;		
			data <<= 1;

			Delay_us(1000);
			SPI_CLK_SET_1;
			Delay_us(2000);
			SPI_CLK_RESET_0;
			Delay_us(1000);	
	}
	SPI_CS_SET_1;
	Delay_us(1000);
}

uint8_t SpiSw_MasterRead8BitData()
{
	uint8_t data = 0;
	for (uint8_t i =0; i<8; i++)
	{
		if (SPI_MISO_READ == SET)
		{
			SPI_CLK_SET_1;
			Delay_us(1000);
			if (SPI_MISO_READ == SET)
				data |= (1 << (8-i));
			Delay_us(1000);
			SPI_CLK_RESET_0;
			Delay_us(2000);
		}

		else
		{
			SPI_CLK_SET_1;
			Delay_us(1000);
			if (SPI_MISO_READ == SET)
				data |= (0 << (8-i));
			Delay_us(1000);
			SPI_CLK_RESET_0;
			Delay_us(2000);
		}
	}
	return data;

}