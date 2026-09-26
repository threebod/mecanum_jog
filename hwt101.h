#ifndef HWT101_H
#define HWT101_H

#include <stdint.h>

extern volatile float imuYaw;
extern volatile uint32_t imuStamp;
extern volatile uint8_t imuValid;

void hwt101Init(uint32_t baud);

#endif
