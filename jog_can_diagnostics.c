#include "can.h"
#include "delay.h"
#include "Emm_V5.h"
#include "jog_can.h"

#define CAN_CHECK_TIMEOUT_MS 300U

extern volatile uint8_t jogCanFault;

uint8_t jogCanReadStatus(uint8_t motorId, const volatile uint8_t *stopRequested,
                         JogCanReply *reply)
{
    uint32_t elapsed;
    uint8_t i;

    __disable_irq();
    can.rxFrameFlag = false;
    __enable_irq();
    Emm_V5_Read_Sys_Params(motorId, S_FLAG);

    for (elapsed = 0U; elapsed < CAN_CHECK_TIMEOUT_MS; ++elapsed) {
        if (*stopRequested || jogCanFault) return 0U;
        if (can.rxFrameFlag) {
            __disable_irq();
            reply->extId = can.CAN_RxMsg.ExtId;
            reply->dlc = can.CAN_RxMsg.DLC;
            if (reply->dlc > 8U) reply->dlc = 8U;
            for (i = 0U; i < reply->dlc; ++i) {
                reply->data[i] = can.CAN_RxMsg.Data[i];
            }
            can.rxFrameFlag = false;
            __enable_irq();
            if (((reply->extId >> 8) & 0xFFU) == motorId && reply->dlc >= 3U &&
                reply->data[0] == 0x3AU && reply->data[reply->dlc - 1U] == 0x6BU) {
                return 1U;
            }
        }
        delay_ms(1U);
    }
    return 2U;
}
