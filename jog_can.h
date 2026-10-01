#ifndef JOG_CAN_H
#define JOG_CAN_H

#include <stdint.h>

typedef struct {
    uint32_t extId;
    uint8_t dlc;
    uint8_t data[8];
} JogCanReply;

/* Nonblocking, one requested S_FLAG reply per auxiliary motor (5/6).
 * Call reset before a new pose and expect only after synchronous start. */
void jogCanResetAuxStatus(void);
void jogCanExpectAuxStatus(uint8_t motorId);
uint8_t jogCanTakeAuxStatus(uint8_t motorId, uint8_t *status);

/* 0: interrupted, 1: reply received, 2: timed out. */
uint8_t jogCanReadStatus(uint8_t motorId, const volatile uint8_t *stopRequested,
                         JogCanReply *reply);

#endif
