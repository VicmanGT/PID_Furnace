#include "stm32f0xx.h"

void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //(1<<9); ADC clock enable
	RCC->CR2 |= RCC_CR2_HSI14ON; //1<<0; HSI clock enable
	while((RCC->CR2&RCC_CR2_HSI14RDY)==0); //RCC->CR2&0x2 wait until clock is stable
	//Calibration
	ADC1->CR &= ~ADC_CR_ADEN; //DIsable ADC pg. 265
	ADC1->CR |= ADC_CR_ADCAL; //(1<<31); Enable Calibration
	while((ADC1->CR&ADC_CR_ADCAL)!=0); //Wait until calibration is done
	/////
	ADC1->CR |= ADC_CR_ADEN; //ADC enable
	while((ADC1->ISR&ADC_ISR_ADRDY)==0); //Wait until ADC is ready
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; //1<<15 pg. 267 ADC is off when not converting
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

    // Enable ADC interrupt
    ADC1->IER |= ADC_IER_EOCIE; // Enable end of conversion interrupt
    NVIC_SetPriority(ADC1_COMP_IRQn, 1); // Set priority for ADC interrupt
    NVIC_EnableIRQ(ADC1_COMP_IRQn); // Enable ADC interrupt in NVIC
}

void ADC_COMP_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_EOC) // Check if end of conversion flag is set
    {
        unsigned int vic = ADC1->DR;
        internal_temperature = (((long) vic - 200) * 8392) / 100000 - 112;
        ADC1->ISR |= ADC_ISR_EOC; // Clear end of conversion flag
        //ADC1->CR |= ADC_CR_ADSTART; // Restart ADC conversion
    }
    // Send internal_temperature via USART
    char buffer[16];
    sprintf(buffer, "I:%lu\n", internal_temperature);
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_SendChar(buffer[i]);
    }
}