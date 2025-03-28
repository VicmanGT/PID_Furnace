#include "stm32f0xx.h"
void TurnOnSystem(void)
{
        // Enabling TIM6
        TIM6->CR1 |= TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM6_DAC_IRQn);

        // Enabling TIM17
        TIM17->CR1 |= TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM17_IRQn);

        // Enabling TIM15
        TIM15->CR1 |=TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM15_IRQn);

        // Enabling TIM14
        TIM14->CR1 |=TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM14_IRQn);

        // Enabling ADC
        ADC1->CR |=ADC_CR_ADEN;
        NVIC_EnableIRQ(ADC1_COMP_IRQn);

        // Enable USART1
        //USART1->CR1 &= ~USART_CR1_UE;
        //NVIC_DisableIRQ(USART1_IRQn);

        // Enable EXT Interrupt
        EXTI->IMR |= EXTI_IMR_MR0;
        NVIC_EnableIRQ(EXTI0_1_IRQn);

}



void TurnOffSystem(void)
{
        // Disable TIM6
        TIM6->CR1 &= ~TIM_CR1_CEN;
        NVIC_DisableIRQ(TIM6_DAC_IRQn);

        // Disable TIM15
        TIM17->CR1 &= ~TIM_CR1_CEN;
        NVIC_DisableIRQ(TIM17_IRQn);

        // Disable TIM15
        TIM15->CR1 &= ~TIM_CR1_CEN;
        NVIC_DisableIRQ(TIM15_IRQn);

        // Disable TIM14
        TIM14->CR1 &= ~TIM_CR1_CEN;
        NVIC_DisableIRQ(TIM14_IRQn);

        // Disable ADC
        ADC1->CR &= ~ADC_CR_ADEN;
        NVIC_DisableIRQ(ADC1_COMP_IRQn);

        // Disable USART1
        //USART1->CR1 &= ~USART_CR1_UE;
        //NVIC_DisableIRQ(USART1_IRQn);

        // Disable EXT Interrupt
        EXTI->IMR &= ~ EXTI_IMR_MR0;
        NVIC_DisableIRQ(EXTI0_1_IRQn);

        //RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;

        // Set GPIO pins to a safe state
        GPIOC->ODR &= ~(1<<10); // Turn off fan
        GPIOC->ODR &= ~(1<<7); // Turn off conveyor
        GPIOC->ODR &= ~(1<<8); // Turn off heater or other component
}