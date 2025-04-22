#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define SPI_CS_SET_1			GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define SPI_CS_RESET_0		GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define SPI_CS_READ				GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define SPI_MOSI_SET_1		GPIO_SetBits(GPIOA, GPIO_Pin_2)
#define SPI_MOSI_RESET_0	GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define SPI_MOSI_READ			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)
#define SPI_CLK_SET_1			GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define SPI_CLK_RESET_0		GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define SPI_CLK_READ			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)
#define SPI_MISO_SET_1		GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define SPI_MISO_RESET_0	GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define SPI_MISO_READ			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)

void SystemClock_Config();
void GPIO_Config();
void TIM_Config();
void Delay_us(uint32_t timeouut);

uint8_t	data = 0x36;
uint8_t	rec_data = 0;

int main ()
{
	SystemClock_Config();
	GPIO_Config();
	TIM_Config();

	while(1)
	{
		/* Waiting Start condition */
		while (SPI_CS_READ);			
		/* Transfer data */
		for (uint8_t i = 0; i<8; i++)
		{
			while (!SPI_CLK_READ);
				if (SPI_MOSI_READ == SET)
					rec_data |= (1 << (8-i));
				else
					rec_data |= (0 << (8-i));
				if ((data & 0x80) == SET)
					SPI_MISO_SET_1;
				else
					SPI_MISO_RESET_0;		
				data <<= 1;	
				Delay_us(1);
		}
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
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	
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
}

void Delay_us(uint32_t timeout)
{
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < timeout);
}