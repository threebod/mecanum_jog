#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include "../avoidance_run.h"
#include "../route_plan.h"
static AvRun avoidance;
static volatile AvObservation avoidanceRxPacket;
static volatile uint8_t avoidanceRxReady;
static uint32_t clockMs,imuStamp;
static uint8_t imuValid=1,armed,routeActive,routeWaiting,routeAuto,navRunning,navInitialized,fullRouteRunning,motionMode,mechanismActionActive;
static int8_t yawSign=1,routeHeading;
static float imuYaw,routeX,routeY,routeBaseYaw,routeYaw,routeDriveRpm,routeTurnRpm,routeCorrection;
static float routeForwardScale=1,routeLateralScale=.9f;
static uint16_t routeRpm,routeTurnRpmLimit=45,routeLateralRpmLimit=150;
static uint8_t routeSentValid;
static uint8_t qrAtStation,qrResultAtStation,missionActive,missionTaskValid,routeIndex;
static uint8_t servoChannelMoving[3];
static RouteHeadingPid routeHeadingPid;
static RouteTurnPid routeTurnPid;
static RouteHeadingPidGains routeHeadingGains={2,.25f,.12f};
static int16_t sentForward,sentRight,sentTurn;
static char hostLine[512],requestLine[64];static unsigned hostLength,requestLength,stops,stateLines,pathLines;
static uint16_t requestToken;static uint8_t requestRegion,requestConfirm;
static uint16_t qrRequestToken;static uint8_t qrRequestEnabled;
static unsigned qrEnableRequests,qrDisableRequests;
#define USART3 3
#define USART_FLAG_TXE 1
#define RESET 0
static uint8_t motionInterrupted(void){return 0;}
static uint8_t visionSessionActive(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static void stopServoMotion(void){}
static void stopDriveMotors(void){sentForward=sentRight=sentTurn=0;++stops;}
static void stopAllMotors(void){stopDriveMotors();routeActive=0;armed=0;}
static void setAllMotorsEnabled(bool enabled){(void)enabled;}
static float headingError(float target,float actual){float e=target-actual;while(e>180)e-=360;while(e<-180)e+=360;return e;}
static int16_t navFieldYawCdeg(void){return (int16_t)((90+imuYaw)*100);}
static void serialSendString(const char *text){
    if(strcmp(text,"\r\n")==0){hostLine[hostLength]=0;if(strncmp(hostLine,"AV STATE",8)==0)++stateLines;if(strncmp(hostLine,"AV PATH",7)==0)++pathLines;hostLength=0;}
    else {unsigned n=(unsigned)strlen(text);assert(hostLength+n<sizeof(hostLine));memcpy(hostLine+hostLength,text,n);hostLength+=n;}
}
static void serialSendUint(uint16_t n){char b[20];sprintf(b,"%u",n);serialSendString(b);}
static void serialSendUint32(uint32_t n){char b[20];sprintf(b,"%lu",(unsigned long)n);serialSendString(b);}
static void serialSendInt(int16_t n){char b[20];sprintf(b,"%d",n);serialSendString(b);}
static uint8_t parsePair(const char *p,uint16_t *a,uint16_t *b){unsigned x,y;char extra;if(sscanf(p,"%u %u %c",&x,&y,&extra)!=2 || x>65535 || y>65535)return 0;*a=(uint16_t)x;*b=(uint16_t)y;return 1;}
static int USART_GetFlagStatus(int port,int flag){(void)port;(void)flag;return 1;}
static void USART_SendData(int port,uint8_t byte){
    (void)port;
    if(byte=='\n') {
        unsigned token,region,confirm;requestLine[requestLength]=0;
        if(strncmp(requestLine,"AVQ:",4)==0) {
            assert(sscanf(requestLine,"AVQ:%u:%u:%u",&token,&region,&confirm)==3);
            requestToken=(uint16_t)token;requestRegion=(uint8_t)region;requestConfirm=(uint8_t)confirm;
        } else {
            unsigned checksum;const char *last=strrchr(requestLine,':');
            assert(sscanf(requestLine,"QRQ:%u:%u:%X",&token,&region,&checksum)==3 && last);
            assert(checksum==av_crc16(requestLine,(unsigned)(last-requestLine)));
            qrRequestToken=(uint16_t)token;qrRequestEnabled=(uint8_t)region;
            if(region)++qrEnableRequests;else ++qrDisableRequests;
        }
        requestLength=0;
    } else {assert(requestLength<sizeof(requestLine)-1);requestLine[requestLength++]=(char)byte;}
}
static void sendRouteSpeeds(int16_t f,int16_t r,int16_t t){sentForward=f;sentRight=r;sentTurn=t;assert(abs(r)<=150);}
#include "../avoidance_firmware.h"
#include "../qr_camera_control.h"

static void run_laps(unsigned zone,unsigned rpm,uint8_t addObstacle)
{
    char command[40];unsigned step,confirmFrames=0,qrWaitTicks=0;uint16_t lastToken=0;uint8_t added=0;
    sprintf(command,"avoid start %u %u",zone,rpm);armed=1;imuStamp=clockMs;
    assert(processAvoidanceCommand(command) && avoidance.active);
    for(step=0;step<30000 && avoidance.active;++step) {
        clockMs+=20;imuYaw+=sentTurn*.02f*1.5f;imuStamp=clockMs;
        if(avoidance.phase==AV_DWELL && avoidance.stage==0 && ++qrWaitTicks>=20) qrResultAtStation=1;
        if(step%5==0 && requestToken) {
            AvObservation packet={0};packet.token=requestToken;packet.region=requestRegion;
            if(addObstacle && !added && requestRegion==3) {
                packet.seen=1;packet.u=320;packet.bottom=407;packet.pixels=3000;
                if(lastToken!=requestToken){confirmFrames=0;lastToken=requestToken;}
                if(requestConfirm && ++confirmFrames>=3){packet.stable=1;added=1;}
            }
            avoidanceRxPacket=packet;avoidanceRxReady=1;
        }
        serviceAvoidance();
        serviceQrCamera();
        if(qrRequestEnabled) assert(avoidance.active && avoidance.phase==AV_DWELL && avoidance.stage==0 && avoidance.heading==3);
        if(avoidance.phase==AV_DWELL && avoidance.stage==0 && !qrResultAtStation) assert(!sentForward && !sentRight && !sentTurn);
    }
    if(avoidance.phase!=AV_DONE)fprintf(stderr,"runtime zone=%u rpm=%u phase=%s reason=%s stage=%s xy=%.1f,%.1f\n",zone,rpm,av_phase_name(avoidance.phase),avoidance.reason,av_target_name(avoidance.stage),avoidance.x,avoidance.y);
    assert(avoidance.phase==AV_DONE && !routeActive && !sentForward && !sentRight && !sentTurn);
    assert(qrWaitTicks>=20);
    assert(!qrRequestEnabled);
    if(addObstacle)assert(added && avoidance.blocked==(1U<<2) && avoidance.replans==1);
}
int main(void)
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
#endif
    run_laps(1,30,1);
    run_laps(2,250,0); /* remembered R3 must be avoided on a subsequent start too */
    assert(avoidance.blocked==(1U<<2));
    armed=1;assert(processAvoidanceCommand("avoid start 1 9") && !avoidance.active);
    assert(processAvoidanceCommand("avoid start 1 251") && !avoidance.active);
    assert(processAvoidanceCommand("avoid start 1 10") && avoidance.active);
    processAvoidanceCommand("avoid clear");assert(avoidance.blocked==(1U<<2));
    clockMs+=1500;imuStamp=clockMs;avoidanceTick=clockMs-20;serviceAvoidance();
    assert(!avoidance.active && avoidance.phase==AV_FAULT && !routeActive);
    processAvoidanceCommand("avoid stop");assert(avoidance.blocked==(1U<<2));
    armed=1;imuStamp=clockMs;
    processAvoidanceCommand("avoid start 1 30");assert(avoidance.active);
    clockMs+=20;imuStamp=clockMs;
    avoidanceRxPacket=(AvObservation){avoidance.token,avoidance.region,0,0,0,0,0};avoidanceRxReady=1;
    serviceAvoidance();assert(avoidance.camera && avoidance.active);
    clockMs+=501;imuStamp=clockMs;avoidanceTick=clockMs-20;
    serviceAvoidance();assert(!avoidance.active && !strcmp(avoidance.reason,"CAMERA_LOST") && avoidance.blocked==(1U<<2));
    armed=1;processAvoidanceCommand("avoid start 1 30");assert(avoidance.active);
    routeActive=0;serviceAvoidance(); /* existing emergency/watchdog cancellation */
    assert(!avoidance.active && !strcmp(avoidance.reason,"CANCELLED") && avoidance.blocked==(1U<<2));
    processAvoidanceCommand("avoid clear");assert(!avoidance.blocked);
    routeActive=routeWaiting=missionActive=1;routeIndex=1;missionTaskValid=0;
    serviceQrCamera();assert(!qrRequestEnabled);
    routeIndex=2;serviceQrCamera();assert(qrRequestEnabled);
    {
        uint16_t token=qrRequestToken;clockMs+=200;serviceQrCamera();
        assert(qrRequestEnabled && token==qrRequestToken);
    }
    missionTaskValid=1;serviceQrCamera();assert(!qrRequestEnabled);
    routeActive=missionActive=0;serviceQrCamera();assert(!qrAtStation);
    assert(qrEnableRequests>=3 && qrDisableRequests>=3);
    assert(stateLines>10 && pathLines>10 && stops>10);
    puts("PASS: actual firmware bridge two laps, live obstacle/replan, 250 RPM, timeout, stop and clear");
    return 0;
}
