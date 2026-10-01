#include <assert.h>
#include <stdio.h>
#include "stubs/can.h"
#include "../jog_can.h"
void CAN1_RX0_IRQHandler(void);
extern volatile uint8_t jogCanFault;
static unsigned sent, cancelled, waits;
static uint8_t failMode;
static CanTxMsg packets[4];
static CanRxMsg received;
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
{ (void)bus; (void)fifo; *rx = received; }
void delay_us(uint32_t us) { assert(us == 100); ++waits; }
static void reply(uint8_t id, uint8_t status)
{
    received.ExtId = (uint32_t)id << 8;
    received.IDE = CAN_Id_Extended;
    received.RTR = CAN_RTR_Data;
    received.DLC = 3U;
    received.Data[0] = 0x3AU;
    received.Data[1] = status;
    received.Data[2] = 0x6BU;
    CAN1_RX0_IRQHandler();
}
int main(void)
{
    uint8_t cmd[13] = {2, 0xFD, 0, 0, 30, 50, 0, 0, 4, 6, 0, 1, 0x6B};
    uint8_t status;
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
    jogCanResetAuxStatus();
    reply(5U, 3U);
    assert(!jogCanTakeAuxStatus(5U, &status)); /* Unrequested/old frame. */
    jogCanExpectAuxStatus(5U);
    jogCanExpectAuxStatus(6U);
    reply(5U, 3U);
    reply(6U, 1U);
    reply(2U, 3U); /* Other replies cannot overwrite either auxiliary cache. */
    assert(jogCanTakeAuxStatus(5U, &status) && status == 3U);
    assert(jogCanTakeAuxStatus(6U, &status) && status == 1U);
    assert(!jogCanTakeAuxStatus(5U, &status));
    /* A reply can arrive after the main loop checked the cache but before
     * it prepares the next poll. Preparing must not discard that reply. */
    jogCanExpectAuxStatus(6U);
    reply(6U, 3U);
    jogCanExpectAuxStatus(6U);
    assert(jogCanTakeAuxStatus(6U, &status) && status == 3U);
    jogCanExpectAuxStatus(5U);
    received.ExtId = 0x500U;
    received.Data[2] = 0U;
    CAN1_RX0_IRQHandler();
    assert(!jogCanTakeAuxStatus(5U, &status));
    received.Data[2] = 0x6BU;
    received.Data[0] = 0xFDU;
    CAN1_RX0_IRQHandler();
    assert(!jogCanTakeAuxStatus(5U, &status));
    received.Data[0] = 0x3AU;
    received.DLC = 2U;
    CAN1_RX0_IRQHandler();
    assert(!jogCanTakeAuxStatus(5U, &status));
    received.DLC = 3U;
    received.ExtId = 0x501U;
    CAN1_RX0_IRQHandler();
    assert(!jogCanTakeAuxStatus(5U, &status));
    received.ExtId = 0x500U;
    received.IDE = 0U;
    CAN1_RX0_IRQHandler();
    assert(!jogCanTakeAuxStatus(5U, &status));
    reply(5U, 3U);
    jogCanResetAuxStatus();
    assert(!jogCanTakeAuxStatus(5U, &status));
    reply(5U, 3U);
    assert(!jogCanTakeAuxStatus(5U, &status));
    puts("PASS: independent requested auxiliary status, reset and malformed frames");
    return 0;
}
