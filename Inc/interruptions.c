#include "stm32f0xx.h"
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