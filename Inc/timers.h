#ifndef timers_H
#define timers_H
#include "timers.c"
#include <stdint.h>
void TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);
void TIM17_Init(void);
void TIM14_IRQHandler(void);
void TIM14_Init(void);
void TIM6_Init(void);
void TIM6_DAC_IRQHandler (void);
void TIM1_CC_IRQHandler(void);
void TIM15_IRQHandler(void);
void TIM3_IRQHandler(void);
#endif