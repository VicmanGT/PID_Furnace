#ifndef usart_H
#define usart_H
#include "timers.c"
#include <stdint.h>
void USART_Init(void);
void USART_SendChar(char c);
void parseReceivedData(char* data);
void USART1_IRQHandler(void);
void SendUsart(int value);
#endif