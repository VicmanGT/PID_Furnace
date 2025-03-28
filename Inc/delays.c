#include "stm32f0xx.h"
void delay_ms(uint32_t milliseconds)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM clock
    TIM2->PSC = 8000000 / 1000 - 1; // Prescaler for 1ms tick
    TIM2->ARR = milliseconds; // Auto-reload value
    TIM2->CNT = 0; // Restart counter
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
    while (!(TIM2->SR & TIM_SR_UIF)); // Wait for update interrupt flag
    TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    TIM2->CR1 &= ~TIM_CR1_CEN; // Disable counter
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; // Disable TIM clock
}

void delay_us(uint32_t us) {
    // Configure TIM2 for microsecond delay
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM clock
    TIM2->PSC = 8 - 1; // Prescaler for 1us tick
    TIM2->ARR = us; // Auto-reload value
    TIM2->CNT = 0; // Restart counter
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
    while (!(TIM2->SR & TIM_SR_UIF)); // Wait for update interrupt flag
    TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    TIM2->CR1 &= ~TIM_CR1_CEN; // Disable counter
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; // Disable TIM clock
}