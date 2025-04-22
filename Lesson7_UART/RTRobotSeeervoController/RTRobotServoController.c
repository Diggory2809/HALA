#include "RTRobotServoController.h"

uint8_t usart_rx_dma_buffer[64];
volatile size_t usart_tx_dma_current_len;

void RTRobot_Init()
{
    USART_InitTypeDef USART_InitStruct = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
		DMA_InitTypeDef	DMA_InitStruct = {0};
		NVIC_InitTypeDef	NVIC_InitStruct = {0};

    /* Peripheral clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /*
     * USART1 GPIO Configuration
     *
     * PA2   ------> USART2_TX
     * PA3   ------> USART2_RX
     */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		
		GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1_RX Init */
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

    /* DMA1_Channel6_IRQn interrupt configuration */
		NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel6_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStruct);

    /* USART configuration */
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);		

    /* USART interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStruct);

    /* Enable USART and DMA */
    USART_Cmd(USART2, ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);
}

void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - DMA_GetCurrDataCounter(DMA1_Channel6);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}


void usart_process_data(const void* data, size_t len) {
    const uint8_t* d = data;
    
    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
    
    for (; len > 0; --len, ++d) {
			USART_SendData(USART2, *d);
			while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    }
    while (!USART_GetFlagStatus(USART2, USART_FLAG_TC));
}

void usart_send_string(const char* str) {
    usart_process_data(str, strlen(str));
}
