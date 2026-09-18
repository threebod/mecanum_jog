#ifndef TRANSFER_CAN_H
#define TRANSFER_CAN_H
#include "stm32f4xx.h"
typedef unsigned char bool;
#define true 1
#define false 0
void can_SendCmd(volatile uint8_t *cmd,uint8_t len);
#endif
