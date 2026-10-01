#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
#include <stdbool.h>
#define __IO volatile
typedef struct { uint32_t StdId, ExtId; uint8_t IDE, RTR, DLC, Data[8]; } CanTxMsg;
typedef CanTxMsg CanRxMsg;
typedef struct { CanRxMsg CAN_RxMsg; CanTxMsg CAN_TxMsg; bool rxFrameFlag; } CAN_t;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
uint8_t CAN_Transmit(int bus, CanTxMsg *tx);
uint8_t CAN_TransmitStatus(int bus, uint8_t mailbox);
void CAN_CancelTransmit(int bus, uint8_t mailbox);
void CAN_Receive(int bus, int fifo, CanRxMsg *rx);
void can_SendCmd(volatile uint8_t *cmd, uint8_t len);
#endif
