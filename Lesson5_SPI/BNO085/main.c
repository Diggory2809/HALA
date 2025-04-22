#include "main.h"
#include "BNO085.h"
#include "Quaternion.h"

uint32_t	Timingdelay;
int count = 0;
float f = 1.234;
float q[4];
float quatRadianAccuracy;
int main ()
{
	SystemClock_Config();
	SysTick_Config(SystemCoreClock/1000);
	GPIO_Config();
	SPI2_Config();
	
	BNO085_Initialization();
  BNO085_enableRotationVector(2500);
	Delay_ms(100);
	
	while (1)
	{
	  if(BNO085_dataAvailable() == 1)
	  {

		  q[0] = BNO085_getQuatI();
		  q[1] = BNO085_getQuatJ();
		  q[2] = BNO085_getQuatK();
		  q[3] = BNO085_getQuatReal();
		  quatRadianAccuracy = BNO085_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);

		  //printf("%d,%d,%d\n", (int)(BNO085_Roll*100), (int)(BNO085_Pitch*100), (int)(BNO085_Yaw*100));
	  }
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