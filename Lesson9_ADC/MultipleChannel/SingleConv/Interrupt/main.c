#include "stm32f10x.h"                  // Device header
#include "stm32f10x_adc.h"              // Device:StdPeriph Drivers:ADC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define map(x)	x*16

void SystemClock_Config();
void GPIO_Config();
void NVIC_Config();
void ADC_Config();
void TIM_Config();
void Delay_us(uint32_t timeout);

int main ()
{
	SystemClock_Config();
	GPIO_Config();
	NVIC_Config();
	ADC_Config();
	TIM_Config();
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (1)
	{
		
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Config()
{
	/*	PA1	->	ADC1_Ch1				PA15	->	PWM1
			PA2	->	ADC1_Ch2				PB3		->	PWM2
			PA3	->	ADC1_Ch3				PB10	->	PWM3
			PA4	->	ADC1_Ch4				PB11	->	PWM4*/
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void NVIC_Config()
{
	NVIC_InitTypeDef	NVIC_InitStruct = {0};
	
	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}

void ADC_Config()
{
	ADC_InitTypeDef	ADC_InitStruct = {0};
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_NbrOfChannel = 4;
	
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_ITConfig(ADC1, ADC_IT_EOC);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}

void TIM_Config()
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct = {0};
	TIM_OCInitTypeDef	TIM_OCInitStruct = {0};
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM3, ENABLE);
}

void Delay_us(uint32_t timeout)
{
	TIM_SetCounter(TIM3, 0);
	while (TIM_GetCounter(TIM3) < timeout);
}

void ADC1_2_IRQHandler()
{
	
}