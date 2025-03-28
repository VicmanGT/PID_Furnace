#ifndef peripherals_H
#define peripherals_H
#include <stdint.h>
void FanConveyorCtrlInit(void);
void TurnOnHeat(int t);
void TurnOnConveyor(int t);
void FanStateManager(int t);
#endif