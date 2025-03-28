#include "stm32f0xx.h"
void FanConveyorCtrlInit(void){
	GPIOC->MODER |=(1<<16);
	GPIOC->MODER |=(1<<14);
	GPIOC->MODER |=(1<<12);
	GPIOC->MODER |=(1<<20);
}

void TurnOnHeat(int t){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 clock
    TIM3->PSC = 8 - 1; // Prescaler for 1us tick
    TIM3->ARR = t; // Auto-reload value
    TIM3->CNT = 0; // Restart counter
    TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_SetPriority(TIM3_IRQn, 0);  // Set priority for TIM3 interrupt
    NVIC_EnableIRQ(TIM3_IRQn);       // Enable TIM3 interrupt in NVIC
    TIM3->CR1 |= TIM_CR1_CEN; // Enable counter
    //GPIOC->ODR &= ~ (1<<8); // Set PC8 high
}
void TurnOnConveyor(int t)
{
    //**** Inicializa Timer7  *******
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // Enable TIM3 clock
    TIM15->PSC = 8 - 1; // Prescaler for 1us tick
    TIM15->ARR = t;
    TIM15->CNT = 0; // Restart counter
    TIM15->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_SetPriority(TIM15_IRQn, 0);  // Set priority for TIM3 interrupt
    NVIC_EnableIRQ(TIM15_IRQn);       // Enable TIM3 interrupt in NVIC
    TIM15->CR1 |= TIM_CR1_CEN; // Enable counter
    //GPIOC->ODR &= ~ (1<<8); // Set PC8 high
}
{
	fan_state++;
	if (fan_state < t) GPIOC->ODR |= (1<<8);
	else GPIOC->ODR &= ~ (1<<8);
	if (fan_state > 50){
		fan_state = 0;
	}

}