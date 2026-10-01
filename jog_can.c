#include "can.h"
#include "delay.h"
#include "jog_can.h"

/* Local transport: do not change the competition project's shared CAN driver. */
__IO CAN_t can = {0};
volatile uint8_t jogCanFault;
static volatile uint8_t auxStatusExpected[2], auxStatusReady[2], auxStatus[2];

void jogCanResetAuxStatus(void)
{
    __disable_irq();
    auxStatusExpected[0] = auxStatusExpected[1] = 0U;
    auxStatusReady[0] = auxStatusReady[1] = 0U;
    __enable_irq();
}

void jogCanExpectAuxStatus(uint8_t motorId)
{
    if (motorId < 5U || motorId > 6U) return;
    __disable_irq();
    /* A reply may have arrived since the caller last checked the cache.
     * Only a new pose/reset or consuming the reply may clear its ready bit. */
    auxStatusExpected[motorId - 5U] = 1U;
    __enable_irq();
}

uint8_t jogCanTakeAuxStatus(uint8_t motorId, uint8_t *status)
{
    uint8_t ready;
    if (motorId < 5U || motorId > 6U) return 0U;
    __disable_irq();
    ready = auxStatusReady[motorId - 5U];
    if (ready) *status = auxStatus[motorId - 5U];
    auxStatusReady[motorId - 5U] = 0U;
    __enable_irq();
    return ready;
}

void CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&can.CAN_RxMsg);
    can.rxFrameFlag = true;
    if (can.CAN_RxMsg.IDE == CAN_Id_Extended &&
        can.CAN_RxMsg.RTR == CAN_RTR_Data && can.CAN_RxMsg.DLC == 3U &&
        can.CAN_RxMsg.Data[0] == 0x3AU && can.CAN_RxMsg.Data[2] == 0x6BU &&
        (can.CAN_RxMsg.ExtId == 0x500U || can.CAN_RxMsg.ExtId == 0x600U)) {
        uint8_t index = (uint8_t)((can.CAN_RxMsg.ExtId >> 8) - 5U);
        if (auxStatusExpected[index]) {
            auxStatus[index] = can.CAN_RxMsg.Data[1];
            auxStatusReady[index] = 1U;
            auxStatusExpected[index] = 0U;
        }
    }
}

uint8_t can_GetRxFlag(void) { return 0U; }

void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    CanTxMsg tx;
    uint8_t offset = 2U, packet = 0U, mailbox, i;
    uint16_t wait;
    if (len < 3U || len > 16U) { jogCanFault = 1U; return; }
    while (offset < len) {
        tx.StdId = 0U;
        tx.ExtId = ((uint32_t)cmd[0] << 8) | packet++;
        tx.IDE = CAN_Id_Extended;
        tx.RTR = CAN_RTR_Data;
        tx.Data[0] = cmd[1];
        tx.DLC = 1U;
        for (i = 1U; i < 8U && offset < len; ++i) {
            tx.Data[i] = cmd[offset++];
            ++tx.DLC;
        }
        mailbox = CAN_Transmit(CAN1, &tx);
        if (mailbox == CAN_TxStatus_NoMailBox) {
            jogCanFault = 1U;
            return;
        }
        for (wait = 0U; wait < 50U; ++wait) {
            if (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok) break;
            delay_us(100U);
        }
        if (wait == 50U) {
            CAN_CancelTransmit(CAN1, mailbox);
            jogCanFault = 1U;
            return;
        }
    }
}
