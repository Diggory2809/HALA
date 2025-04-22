#include "stm32f10x.h"                  // Device header
#include "stm32f10x_adc.h"              // Device:StdPeriph Drivers:ADC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "stm32f10x_dma.h"              // Device:StdPeriph Drivers:DMA

#define map(x)	x*16

void RCC_ClockConfiguration();
void GPIO_Configuration();
void NVIC_Configuration();
void ADC_Configuration();
void DMA_Configuration();
void TIM_Configuration();
void Delay_us(uint32_t timeout);

__IO uint16_t ADC_ConvertedValue = 0;

int main ()
{
	RCC_ClockConfiguration();
	GPIO_Configuration();
	NVIC_Configuration();
	DMA_Configuration();
	ADC_Configuration();
	TIM_Configuration();
	
	while (1)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		Delay_us(1000);
	}
	return 0;
}

void RCC_ClockConfiguration()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GPIO_Configuration()
{
	/*	ADC1 -> PA1
			TIM2_PWM1 -> PA0*/
	GPIO_InitTypeDef	GPIO_InitStruct ={0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void ADC_Configuration()
{
	ADC_InitTypeDef	ADC_InitStruct;
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}

void TIM_Configuration()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	TIM_OCInitTypeDef TIM_OCInitStruct = {0};									  
	

	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;											  
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM3, ENABLE);	
}

void DMA_Configuration()
{
	DMA_InitTypeDef	DMA_InitStruct = {0};
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_BufferSize =1;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	DMA_SetCurrDataCounter(DMA1_Channel1, 1);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);	
}

void NVIC_Configuration()
{
	NVIC_InitTypeDef	NVIC_InitStruct = {0};
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void Delay_us (uint32_t	timeout)
{
	TIM_SetCounter(TIM3, 0);
	while (TIM_GetCounter(TIM3) < timeout);
}

void DMA1_Channel1_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		TIM2->CCR1 = map(ADC_ConvertedValue);
		DMA_Cmd(DMA1_Channel1, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Channel1, 1);
		DMA_Cmd(DMA1_Channel1, ENABLE);
	}
}