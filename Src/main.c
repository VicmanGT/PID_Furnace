#include "stm32f0xx.h"
#include "stdio.h"
#include <stdint.h>

volatile long internal_temperature;
volatile int internal_temperature_dashboard;
volatile int fan_speed;
volatile int conveyor_speed;
volatile int on_off;
volatile int on_off_new;
volatile int usart_in;
volatile int e;
volatile int e1;
volatile int ei;
volatile int ed;
volatile int t;
volatile int t_f;
volatile int t_c;
volatile float external_temperature;
volatile float external_humidity;
volatile int sum;
volatile uint8_t received_bytes[5] = {0};
int P = 1;
int I;
int D;
int temp_t;
int pid_upper_limit = 7500;
int pid_lower_limit = 200;
int fan_state;
int mode;
int conveyor_state;


#define MAX_TIMEOUT 10000

void USART_Init(void);
void USART_SendChar(char c);
void USART_IRQHandler(void);

void ADC_Init(void);
void ADC_COMP_IRQHandler (void);

void TIM6_Init(void);
void TIM6_DAC_IRQHandler (void);

void EXTI0_1_IRQHandler(void);
void ZeroCrossing(void);

void delay_ms(uint32_t);
void delay_us(uint32_t);






void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) // Check if EXTI0 interrupt is pending
    {
        EXTI->PR = EXTI_PR_PR0; // Clear the interrupt flag by writing 1

        PID();
        TurnOnHeat(t);

        t_f = fan_speed / 2;
        FanStateManager(t_f);

        if (conveyor_speed != 0){
        t_c = 7500 - 7300 * conveyor_speed / 100;
        if (conveyor_state == 1){
        	TurnOnConveyor(t_c);
        }
        }



    }
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

void TurnOnSystem(void)
{
        // Disable TIM6
        TIM6->CR1 |= TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM6_DAC_IRQn);

        // Disable TIM17
        TIM17->CR1 |= TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM17_IRQn);

        // Disable TIM15
        TIM15->CR1 |=TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM15_IRQn);

        // Disable TIM14
        TIM14->CR1 |=TIM_CR1_CEN;
        NVIC_EnableIRQ(TIM14_IRQn);

        // Disable ADC
        ADC1->CR |=ADC_CR_ADEN;
        NVIC_EnableIRQ(ADC1_COMP_IRQn);

        // Disable USART1
        //USART1->CR1 &= ~USART_CR1_UE;
        //NVIC_DisableIRQ(USART1_IRQn);

        // Disable EXT Interrupt
        EXTI->IMR |= EXTI_IMR_MR0;
        NVIC_EnableIRQ(EXTI0_1_IRQn);

}

void InitSystem(void){
	USART_Init();
    TIM6_Init();
    ADC_Init();
    ZeroCrossingInit();
    DHT_Init();
    TIM14_Init();
    //ConveyorON();
    FanConveyorCtrlInit();
    TIM16_Init();
    TIM17_Init();
}




int main(void)
{
	InitSystem();
    TIM16_Init();

    while (1)
    {

    }
}

