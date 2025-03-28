#include "stm32f0xx.h"
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

void SendUsart(int value){
    char buffer[10];
    sprintf(buffer, "%lu\n", value);
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_SendChar(buffer[i]);
    }
}
