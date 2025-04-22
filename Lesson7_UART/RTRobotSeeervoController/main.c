#include "main.h"
#include "RTRobotServoController.h"
#include "lwrb.h"

volatile uint32_t TimingDelay;

int main ()
{	
	SystemClock_Config();
	GPIO_Config();
	SysTick_Config(SystemCoreClock/1000);
	DMA_USART2_Config();
	USART2_Config();
	
	RTRobot_Init();
	usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n");
  usart_send_string("Start sending data to STM32\r\n");
	while (1)
	{
		
	}
	return 0;
}

void SystemClock_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GPIO_Config()
{
	GPIO_InitTypeDef	GPIO_InitStruct = {0};
	/**USART2 GPIO Configuration
	PA2   ------> USART2_TX
	PA3   ------> USART2_RX
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void DMA_USART2_Config()
{
	DMA_DeInit(DMA1_Channel6);
	DMA_InitTypeDef	DMA_InitStruct = {0};
	
		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStruct.DMA_BufferSize = ARRAY_LEN(usart_rx_dma_buffer);
		DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
		DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(USART2->DR));
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)usart_rx_dma_buffer;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		DMA_Init(DMA1_Channel6, &DMA_InitStruct);

    /* Enable HT & TC interrupts */
    DMA_ITConfig(DMA1_Channel6, DMA_IT_HT, ENABLE);
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(DMA1_Channel6, ENABLE);
}

void USART2_Config()
{
	USART_InitTypeDef	USART_InitStruct = {0};
	
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	
	
	USART_Cmd(USART2, ENABLE);
}

void SysTick_Handler(void)
{
	if (TimingDelay > 0)
	{
		TimingDelay --;
	}
}

void Delay_ms(uint32_t	timeout)
{
	TimingDelay = timeout;
	while (TimingDelay != 0);
}

void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTz Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}

void DMA1_Channel6_IRQHandler(void) 
{
    /* Check half-transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_HT6) && DMA_GetITStatus(DMA1_IT_HT6))
	{
		DMA_ClearFlag(DMA1_FLAG_HT6);						/* Clear half-transfer complete flag */
    usart_rx_check();                       /* Check for data to process */
  }

    /* Check transfer-complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC6) && DMA_GetITStatus(DMA1_IT_TC6))
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);						/* Clear transfer complete flag */
		usart_rx_check();												/* Check for data to process */
	}
    /* Implement other events when needed */
}

/**
 * \brief           USART1 global interrupt handler
 */
void USART1_IRQHandler(void) 
{
    /* Check for IDLE line interrupt */
	if (USART_GetITStatus(USART2, USART_IT_IDLE) && USART_GetFlagStatus(USART2, USART_FLAG_IDLE))
	{
		USART_ClearFlag(USART2, USART_FLAG_IDLE);		/* Clear IDLE line flag */
		usart_rx_check();  													/* Check for data to process */
	}
    /* Implement other events when needed */
}