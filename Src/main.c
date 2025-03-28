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

void FanConveyorCtrlInit(void){
	GPIOC->MODER |=(1<<16);
	GPIOC->MODER |=(1<<14);
	GPIOC->MODER |=(1<<12);
	GPIOC->MODER |=(1<<20);
}

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

void FanStateManager(int t){
	fan_state++;
	if (fan_state < t) GPIOC->ODR |= (1<<8);
	else GPIOC->ODR &= ~ (1<<8);
	if (fan_state > 50){
		fan_state = 0;
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

void TIM6_DAC_IRQHandler (void)
{
   if (TIM6->SR & TIM_SR_UIF) // Check if update interrupt flag is set
   {
       TIM6->SR &= ~TIM_SR_UIF;
       ADC1->CR |= ADC_CR_ADSTART;
   }
   // Send internal_temperature via USART
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

void TIM16_Init(void)
{
    //**** Inicializa Timer6  *******
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;  // Enable TIM6 clock
    TIM16->PSC = 8000 - 1;               // Set prescaler
    TIM16->ARR = 1001;                     // Set auto-reload value to 999 (1 second interval)
    TIM16->CR1 |= TIM_CR1_CEN;            // Enable TIM6 counter
    TIM16->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    NVIC_SetPriority(TIM16_IRQn, 0);  // Set priority for TIM6 interrupt
    NVIC_EnableIRQ(TIM16_IRQn);       // Enable TIM6 interrupt in NVIC
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


int main(void)
{
	InitSystem();
    TIM16_Init();

    while (1)
    {

    }
}

void USART_Init(void)
{
    // Enable the clock for GPIOA and USART1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure PA9 as USART1_TX (AF1)
    GPIOA->MODER |= GPIO_MODER_MODER9_1;
    GPIOA->AFR[1] |= (1 << (1 * 4)); // AF1 for PA9

    // Configure PA10 as USART1_RX (AF1)
    GPIOA->MODER |= GPIO_MODER_MODER10_1;
    GPIOA->AFR[1] |= (1 << (2 * 4)); // AF1 for PA10

    // Configure USART1
    USART1->BRR = 8000000 / 9600; // Assuming 8 MHz clock and 9600 baud rate
    USART1->CR1 |= USART_CR1_TE; // Enable transmitter
    USART1->CR1 |= USART_CR1_RE; // Enable receiver
    //USART1->CR1 |= (1<<6); // Enable Local Intr
    USART1->CR1 |= USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE; // Enable USART

    // Enable USART1 interrupt
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART_SendChar(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE)); // Wait until transmit data register is empty
    USART1->TDR = c; // Transmit the character
}

void parseReceivedData(char* data) {
    int conveyor = 0;
    int fan = 0;
    int temp_control = 0;
    int onoff = 0;
    int mode_ = 0;

    // Parse the string and extract the values
    if (sscanf(data, "C:%d F:%d T:%d O:%d M:%d", &conveyor, &fan, &temp_control, &onoff, &mode_) == 5) {

        on_off_new = onoff;
        if (mode_ != mode){
        	mode = mode_;
        	if (mode == 0){
                TIM17->CR1 &= ~TIM_CR1_CEN;
                NVIC_DisableIRQ(TIM17_IRQn);

        	}else{
        		TIM17->CR1 |= TIM_CR1_CEN;
        		NVIC_EnableIRQ(TIM17_IRQn);
        	}
        }

        if (mode == 0){
           conveyor_speed = conveyor;
           fan_speed = fan;
           internal_temperature_dashboard = temp_control;
           conveyor_state = 1;
        }
    } else {
        // Handle parsing error if needed
    }
}

void USART1_IRQHandler(void)
{
    if (USART1->ISR & USART_ISR_RXNE) // Check if data is received
    {
        static char rx_buffer[64];
        static uint8_t rx_index = 0;
        char received_char = USART1->RDR; // Read received character

        if (received_char == '\n') {
            rx_buffer[rx_index] = '\0'; // Null-terminate the string
            parseReceivedData(rx_buffer); // Parse the received data
            rx_index = 0; // Reset the buffer index
        } else {
            rx_buffer[rx_index++] = received_char;
            if (rx_index >= sizeof(rx_buffer) - 1) {
                rx_index = 0; // Prevent buffer overflow
            }
        }
        USART1->ICR |= USART_ICR_ORECF; // Clear overrun error flag
    }
}
