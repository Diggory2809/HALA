#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_spi.h"              // Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define BNO085_SPI_CHANNEL		SPI2

#define BNO085_SPI_SCLK_PIN		GPIO_Pin_13
#define BNO085_SPI_SCLK_PORT	GPIOB
#define BNO085_SPI_SCLK_CLK		RCC_APB2Periph_GPIOB

#define BNO085_SPI_MISO_PIN		GPIO_Pin_14
#define BNO085_SPI_MISO_PORT	GPIOB
#define BNO085_SPI_MISO_CLK		RCC_APB2Periph_GPIOB

#define BNO085_SPI_MOSI_PIN		GPIO_Pin_15
#define BNO085_SPI_MOSI_PORT	GPIOB
#define BNO085_SPI_MOSI_CLK		RCC_APB2Periph_GPIOB

#define BNO085_SPI_CS_PIN		GPIO_Pin_12
#define BNO085_SPI_CS_PORT		GPIOB
#define BNO085_SPI_CS_CLK		RCC_APB2Periph_GPIOB

#define BNO085_P1_P2_PIN		GPIO_Pin_10
#define BNO085_P1_P2_PORT	GPIOA
#define BNO085_P1_P2_CLK		RCC_APB2Periph_GPIOA

#define BNO085_RST_PIN			GPIO_Pin_9
#define BNO085_RST_PORT			GPIOA
#define BNO085_RST_CLK			RCC_APB2Periph_GPIOA

#define BNO085_INT_PIN			GPIO_Pin_8
#define BNO085_INT_PORT			GPIOA
#define BNO085_INT_CLK			RCC_APB2Periph_GPIOA

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(BNO085)		GPIO_ResetBits(BNO085_SPI_CS_PORT, BNO085_SPI_CS_PIN)
#define CHIP_DESELECT(BNO085)	GPIO_SetBits(BNO085_SPI_CS_PORT, BNO085_SPI_CS_PIN)

#define WAKE_HIGH()				GPIO_ResetBits(BNO085_P1_P2_PORT, BNO085_P1_P2_PIN)
#define WAKE_LOW()				GPIO_SetBits(BNO085_P1_P2_PORT, BNO085_P1_P2_PIN)

#define RESET_HIGH()			GPIO_ResetBits(BNO085_RST_PORT, BNO085_RST_PIN)
#define RESET_LOW()				GPIO_SetBits(BNO085_RST_PORT, BNO085_RST_PIN)

uint32_t	Timingdelay;
int count = 0;
float f = 1.234;
float q[4];
float quatRadianAccuracy;

void SystemClock_Config();
void GPIO_Config();
void SPI2_Config();
void Delay_ms(uint32_t	timeout);

unsigned char SPI2_SendByte(unsigned char data)
{
	while(SPI_I2S_GetFlagStatus(BNO085_SPI_CHANNEL, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(BNO085_SPI_CHANNEL, data);
	
	while(SPI_I2S_GetFlagStatus(BNO085_SPI_CHANNEL, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(BNO085_SPI_CHANNEL);
}

int main ()
{
	SystemClock_Config();
	SysTick_Config(SystemCoreClock/1000);
	GPIO_Config();
	SPI2_Config();
	

	Delay_ms(100);
	
	while (1)
	{
		uint8_t data = SPI2_SendByte(0x32);
		Delay_ms(100);
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
}

void GPIO_Config()
{
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	
	/**SPI2 GPIO Configuration
	PB13   ------> SPI2_SCK
	PB14   ------> SPI2_MISO
	PB15   ------> SPI2_MOSI
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	/**BNO085 GPIO Control Configuration
	 * PB12 ------> BNO085_CS (output)
	 * PA10  ------> BNO085_P1/P2 (output)
	 * PA9  ------> BNO085_RST (output)
	 * PA8  ------> BNO085_INT (input)
	 */
	/**/
	GPIO_ResetBits(BNO085_RST_PORT, BNO085_RST_PIN);
	GPIO_ResetBits(BNO085_SPI_CS_PORT, BNO085_SPI_CS_PIN);
	GPIO_ResetBits(BNO085_P1_P2_PORT, BNO085_P1_P2_PIN);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_SPI_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_SPI_CS_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_RST_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_RST_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_P1_P2_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_P1_P2_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_INT_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_INT_PORT, &GPIO_InitStruct);
}

void SPI2_Config()
{
	SPI_InitTypeDef	SPI_InitStruct = {0};
	
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_CRCPolynomial = 10;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI2, &SPI_InitStruct);
	SPI_Cmd(SPI2, ENABLE);
}

void SysTick_Handler(void)
{
	Timingdelay--;
}

void Delay_ms(uint32_t	timeout)
{
	Timingdelay = timeout;
	while (Timingdelay != 0);
}