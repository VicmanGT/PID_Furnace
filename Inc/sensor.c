#include "stm32f0xx.h"

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