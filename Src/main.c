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



uint8_t ReadByte(void) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
    	uint32_t timeout=0;
        // Wait for the start of the bit
        while (!(GPIOA->IDR & GPIO_IDR_2)) { // Wait while pin is low
            if (timeout++ > MAX_TIMEOUT) {
                // Timeout occurred
                return -1; // Return error or partial result
            }
        }
        // Measure high time
        delay_us(40);
        if ((GPIOA->IDR & GPIO_IDR_2)) { // if pin high
            result |= (1 << (7 - i)); // Bit is '1'
            // Reset timeout counter
            timeout = 0;

            // Wait for the pin to go low
            while (GPIOA->IDR & GPIO_IDR_2) { // Wait while pin is high
                if (timeout++ > MAX_TIMEOUT) {
                    // Timeout occurred
                    return -1; // Return error or partial result
                }}
        // Wait for the end of the bit

    }

}
    return result;
}

void DHT_SetPinOutputInput(void){
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; // Set to 01 for pull-up
	//GPIOA->MODER |= ~ (1<<4);
	GPIOA->MODER |= (1<<4);  // Configure pin PA2 for output
	GPIOC->ODR &= ~(1<<2); // Output low
	delay_us(1000);
	GPIOA->MODER &= ~(1<<4);  // Configure pin PA2 for input
	//DHT_Response();
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);  //disenable pull up resistor
	delay_us(200);
	//NVIC_EnableIRQ(EXTI2_3_IRQn);

    for (int i = 0; i < 5; i++) {
    	received_bytes[i] = ReadByte();
    	if (received_bytes[i] == -1){
    		external_humidity = -555;
    		external_temperature = -555;
    		return;

    	}

    }
    int check_sum = received_bytes[0] + received_bytes[1] + received_bytes[2] + received_bytes[3];
    if( check_sum != received_bytes[4])
    {
    	//working = 1;
    }
    else{
    	//working = -1;
    }
    external_humidity = (((received_bytes[0] << 8) | received_bytes[1]) > 1000) ? external_humidity: (received_bytes[0] << 8) | received_bytes[1];
	external_temperature = (((received_bytes[2] << 8) | received_bytes[3]) > 1000) ? external_temperature: (received_bytes[2] << 8) | received_bytes[3];
	sum = received_bytes[4];
}


void SendUsart(int value){
    char buffer[10];
    sprintf(buffer, "%lu\n", value);
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_SendChar(buffer[i]);
    }
}

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








void DHT_Init(void){
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA2 as input
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);

    // Enable pull-up resistor on PA2
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; // Set to 01 for pull-up

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTI line for PA2
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI2);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;

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

