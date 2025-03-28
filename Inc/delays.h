#ifndef delays_H
#define delays_H
#include <stdint.h>
#include "delays.c"
void delay_us(uint32_t us);
void delay_ms(uint32_t milliseconds);
#endif