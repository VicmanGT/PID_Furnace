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

int main(void)
{
	InitSystem();
    TIM16_Init();

    while (1)
    {

    }
}
