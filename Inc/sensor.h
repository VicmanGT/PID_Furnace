#ifndef sensor_H
#define sensor_H
#include "sensor.c"
#include <stdint.h>
void DHT_SetPinOutputInput(void);
void DHT_Init(void);
uint8_t ReadByte(void);
#endif