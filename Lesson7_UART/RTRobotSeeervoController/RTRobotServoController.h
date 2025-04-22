#ifndef _RTROBOTSERVOCONTROLLER_H
#define _RTROBOTSERVOCONTROLLER_H

#include "main.h"
#include "lwrb.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

extern uint8_t usart_rx_dma_buffer[64];

void RTRobot_Init();
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);

#endif