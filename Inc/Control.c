#include "stm32f0xx.h"
void PID(void){
	//e = external_humidity /10 - internal_temperature; //change for internal_temperature_dashboard
	e = internal_temperature_dashboard - internal_temperature;
	ei = ei + e * 0.01;
	ed = (e - e1)/0.01;
	e1 = e;
	int m = P * (e + 0*ei + 0*ed); //integral and derivate gains are 0 @ the moment

	t -=m;
	//t -=7800;
	if (t>pid_upper_limit){
		t = pid_upper_limit;
	}
	else if (t<pid_lower_limit)
	{
		t = pid_lower_limit;
	}
}

void ZeroCrossingInit(void)
{
	RCC->AHBENR |=(1<<19);		//Habilita reloj PORTC
	RCC->AHBENR |=(1<<17);		//Habilita reloj PORTA
	GPIOC->MODER |=(1<<18);

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
    GPIOA->MODER &= ~(1<<0);    //PA0 as digital input
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0); // Clear EXTI0 configuration
    SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PA); // Select PA0 for EXTI0

    EXTI->IMR |= EXTI_IMR_MR0; // Unmask EXTI0
    EXTI->FTSR |= (EXTI_FTSR_TR0); // Trigger on falling edge

    NVIC_SetPriority(EXTI0_1_IRQn, 1); // Highest priority
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

