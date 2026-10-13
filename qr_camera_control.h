/* Shared K230: request QR only after the chassis has stopped at the QR station. */
#ifndef MECANUM_QR_CAMERA_CONTROL_H
#define MECANUM_QR_CAMERA_CONTROL_H
static void serviceQrCamera(void)
{
    static uint16_t token=1;
    static uint8_t previousEnabled=255;
    static uint32_t sentAt;
    uint8_t atStation=(uint8_t)((avoidance.active && avoidance.stage==0 && avoidance.phase==AV_DWELL) ||
        (!avoidance.active && !navRunning && routeActive && routeWaiting && routeIndex==2));
    uint8_t enabled;char line[64];unsigned i,n;
    if(atStation!=qrAtStation) {
        qrAtStation=atStation;qrResultAtStation=0;
        if(++token==0) ++token;
        previousEnabled=255;
    }
    enabled=(uint8_t)(atStation && !qrResultAtStation && (!missionActive || !missionTaskValid));
    if(enabled==previousEnabled && (!enabled || clockMs-sentAt<200)) return;
    n=(unsigned)sprintf(line,"QRQ:%u:%u",(unsigned)token,(unsigned)enabled);
    n+=(unsigned)sprintf(line+n,":%04X\n",(unsigned)av_crc16(line,n));
    for(i=0;i<n;++i) {
        while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET) {}
        USART_SendData(USART3,(uint8_t)line[i]);
    }
    previousEnabled=enabled;sentAt=clockMs;
}
#endif
