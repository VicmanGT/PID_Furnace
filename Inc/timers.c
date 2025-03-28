#include "stm32f0xx.h"
void TIM3_IRQHandler(void){
	if (TIM3->SR & TIM_SR_UIF) // Check if update interrupt flag is set
	  {
		GPIOC->ODR |= (1<<7);
		//GPIOC->ODR |= (1<<7);
		TIM3->SR &= ~TIM_SR_UIF;
	    TIM3->CR1 &= ~TIM_CR1_CEN; // Disable counter
	    RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; // Disable TIM clock
	    NVIC_DisableIRQ(TIM3_IRQn);
	    GPIOC->ODR &= ~ (1<<7);
	    //GPIOC->ODR &= ~ (1<<7);
	  }
}

void TIM15_IRQHandler(void){
	if (TIM15->SR & TIM_SR_UIF) // Check if update interrupt flag is set
	  {
		GPIOC->ODR |= (1<<10);
		//GPIOC->ODR |= (1<<7);
		TIM15->SR &= ~TIM_SR_UIF;
	    TIM15->CR1 &= ~TIM_CR1_CEN; // Disable counter
	    RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN; // Disable TIM clock
	    NVIC_DisableIRQ(TIM15_IRQn);
	    GPIOC->ODR &= ~ (1<<10);
	    //delay_us(50);

	  }
}

void TIM1_CC_IRQHandler(void){
	if (TIM1->SR & TIM_SR_UIF) // Check if update interrupt flag is set
	  {
		GPIOC->ODR |= (1<<10);
		//GPIOC->ODR |= (1<<7);
		TIM1->SR &= ~TIM_SR_UIF;
	    TIM1->CR1 &= ~TIM_CR1_CEN; // Disable counter
	    RCC->APB2ENR &= ~RCC_APB1ENR_TIM2EN; // Disable TIM clock
	    NVIC_DisableIRQ(TIM1_CC_IRQn);
	    GPIOC->ODR &= ~ (1<<10);
	    //GPIOC->ODR &= ~ (1<<7);
	  }
}

void TIM6_DAC_IRQHandler (void)
{
   if (TIM6->SR & TIM_SR_UIF) // Check if update interrupt flag is set
   {
       TIM6->SR &= ~TIM_SR_UIF;
       ADC1->CR |= ADC_CR_ADSTART;
   }
   // Send internal_temperature via USART
}

void TIM6_Init(void)
{
    //**** Inicializa Timer6  *******
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  // Enable TIM6 clock
    TIM6->PSC = 8000 - 1;               // Set prescaler
    TIM6->ARR = 100;                     // Set auto-reload value to 999 (1 second interval)
    TIM6->CR1 |= TIM_CR1_CEN;            // Enable TIM6 counter
    TIM6->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);  // Set priority for TIM6 interrupt
    NVIC_EnableIRQ(TIM6_DAC_IRQn);       // Enable TIM6 interrupt in NVIC
}

void TIM14_Init(void)
{
    //**** Inicializa Timer6  *******
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  // Enable TIM6 clock
    TIM14->PSC = 8000 - 1;               // Set prescaler
    TIM14->ARR = 3000;                     // Set auto-reload value to 999 (1 second interval)
    TIM14->CR1 |= TIM_CR1_CEN;            // Enable TIM6 counter
    TIM14->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    NVIC_SetPriority(TIM14_IRQn, 0);  // Set priority for TIM6 interrupt
    NVIC_EnableIRQ(TIM14_IRQn);       // Enable TIM6 interrupt in NVIC
}

void TIM14_IRQHandler(void){
	TIM14->SR = 0;  // Clear status register
    DHT_SetPinOutputInput();
    char buffer[32];
    sprintf(buffer, "H:%d T:%d\n", (int)external_humidity, (int)external_temperature);
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_SendChar(buffer[i]);
    }
}


void TIM17_Init(void)
{
    //**** Inicializa Timer6  *******
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;  // Enable TIM6 clock
    TIM17->PSC = 8000 - 1;               // Set prescaler
    TIM17->ARR = 5001;                     // Set auto-reload value to 999 (1 second interval)
    TIM17->CR1 |= TIM_CR1_CEN;            // Enable TIM6 counter
    TIM17->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    NVIC_SetPriority(TIM17_IRQn, 0);  // Set priority for TIM6 interrupt
    NVIC_EnableIRQ(TIM17_IRQn);       // Enable TIM6 interrupt in NVIC
}

void TIM17_IRQHandler(void)
{
    if (TIM17->SR & TIM_SR_UIF) // Check if update interrupt flag is set
    {
        TIM17->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
        if (mode == 1)
        conveyor_state=!conveyor_state;

    }
}

void TIM16_IRQHandler(void)
{
    if (TIM16->SR & TIM_SR_UIF) // Check if update interrupt flag is set
    {
        TIM16->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

        if (on_off != on_off_new)
        {
            on_off = on_off_new;
            if (on_off == 0)
            {
                TurnOffSystem();
            }
            else
            {
                TurnOnSystem();
            }
        }
    }
}

