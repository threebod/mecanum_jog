#include <assert.h>
#include <stdio.h>
#include "stubs/can.h"
extern volatile uint8_t jogCanFault;
static unsigned sent, cancelled, waits;
static uint8_t failMode;
static CanTxMsg packets[4];
uint8_t CAN_Transmit(int bus, CanTxMsg *tx)
{
    (void)bus;
    packets[sent++] = *tx;
    return failMode == 1 ? CAN_TxStatus_NoMailBox : 0;
}
uint8_t CAN_TransmitStatus(int bus, uint8_t mailbox)
{ (void)bus; (void)mailbox; return failMode == 2 ? 0 : CAN_TxStatus_Ok; }
void CAN_CancelTransmit(int bus, uint8_t mailbox)
{ (void)bus; (void)mailbox; ++cancelled; }
void CAN_Receive(int bus, int fifo, CanRxMsg *rx)
{ (void)bus; (void)fifo; (void)rx; }
void delay_us(uint32_t us) { assert(us == 100); ++waits; }
int main(void)
{
    uint8_t cmd[13] = {2, 0xFD, 0, 0, 30, 50, 0, 0, 4, 6, 0, 1, 0x6B};
    can_SendCmd(cmd, 13);
    assert(!jogCanFault && sent == 2);
    assert(packets[0].ExtId == 0x200 && packets[1].ExtId == 0x201);
    assert(packets[0].DLC == 8 && packets[1].DLC == 5);
    assert(packets[0].Data[0] == 0xFD && packets[1].Data[4] == 0x6B);
    sent = 0; failMode = 1;
    can_SendCmd(cmd, 13);
    assert(jogCanFault && sent == 1 && waits == 0);
    sent = 0; jogCanFault = 0; failMode = 2;
    can_SendCmd(cmd, 13);
    assert(jogCanFault && sent == 1 && waits == 50 && cancelled == 1);
    sent = 0; jogCanFault = 0;
    can_SendCmd(cmd, 0);
    assert(jogCanFault && sent == 0);
    puts("PASS: CAN segmentation, mailbox exhaustion, timeout cancellation");
    return 0;
}
