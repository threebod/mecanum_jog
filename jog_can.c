#include "can.h"
#include "delay.h"

/* Local transport: do not change the competition project's shared CAN driver. */
__IO CAN_t can = {0};
volatile uint8_t jogCanFault;

void CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&can.CAN_RxMsg);
    can.rxFrameFlag = true;
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
