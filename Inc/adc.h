#ifndef adc_H
#define adc_H
#include "adc.c"
#include <stdint.h>
void ADC_Init(void);
void ADC_COMP_IRQHandler(void);
#endif
