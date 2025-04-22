#ifndef	_MAIN_H
#define _MAIN_H

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "stm32f10x_usart.h"            // Device:StdPeriph Drivers:USART
#include "stm32f10x_dma.h"              // Device:StdPeriph Drivers:DMA

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void SystemClock_Config();
void GPIO_Config();
void NVIC_Config();
void DMA_USART2_Config();
void USART2_Config();
void Delay_ms(uint32_t	timeout);


#endif