#ifndef	_MAIN_H
#define	_MAIN_H

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_spi.h"              // Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include <stdio.h>

void SystemClock_Config();
void GPIO_Config();
void SPI2_Config();
void Delay_ms(uint32_t	timeout);

#endif