$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
$active = $source.Substring($source.LastIndexOf('static uint8_t visionSessionActive(void)'))
$active = $active.Substring(0, $active.IndexOf('static const char *visionStateName('))
$service = $source.Substring($source.LastIndexOf('static void serviceRawPickRoute(void)'))
$service = $service.Substring(0, $service.IndexOf('static void missionReport('))
$colors = [regex]::Match($source, 'static const uint8_t rawPickColors[^;]+;').Value
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../vision_config.h"
#include "../raw_pick_action.h"
static VisionSession visionSession;
static VisionPacket visionRxPacket,lastRequest;
static uint8_t visionRxReady,visionPickActive,rawPickAtStation,rawPickItemIndex;
static uint8_t rawPickMissingReported,routeWaiting,missionActive,routeIndex;
enum {MISSION_IDLE, MISSION_TEMP_ALIGN};
static uint8_t missionPhase;
static uint32_t clockMs,imuStamp;
static uint8_t imuValid=1,mechanismActionActive;
static MechanismState mechanismState;
static MechanismAction rawPickRouteActions[RAW_PICK_ACTION_COUNT];
static const VisionCalibration visionPickCalibration={
    1,{0,0,320,240},160,120,{0,0,0,0}};
static unsigned moves,picks;
static char logText[16384];
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static void serialSendString(const char *s) {strcat(logText,s);}
static void serialSendUint(uint16_t v) {char b[16];sprintf(b,"%u",v);strcat(logText,b);}
static void serialSendUint32(uint32_t v) {(void)v;}
static void serialSendInt(int16_t v) {(void)v;}
static void serialSendFloat1(float v) {(void)v;}
static uint8_t motionInterrupted(void) {return 0;}
static void printVisionState(void) {}
static void stopDriveMotors(void) {}
static void failVision(const char *reason) {
    fprintf(stderr,"unexpected vision failure: %s\n",reason);assert(0);
}
static void startMechanismPose(const MechanismPose *p) {
    mechanismState.target=*p;mechanismState.running=1;
}
static void visionUartSend(const VisionPacket *p) {lastRequest=*p;}
static uint8_t startVisionMotion(float f,float r,uint16_t rpm,uint8_t automatic) {
    assert(rawPickItemIndex==0 && rpm==15 && automatic && (f!=0 || r!=0));
    ++moves;return 1;
}
uint8_t mechanismActionStart(const MechanismInitialState *initial,
                             const MechanismAction *actions,uint16_t count) {
    assert(initial==&rawPickInitial && count==RAW_PICK_ACTION_COUNT);
    assert(actions[0].value==rawPickItemIndex+1);
    assert(visionSession.state==VISION_STATE_ALIGNED);
    ++picks;mechanismActionActive=1;return 1;
}
'@
$suffix = @'
static void observe(uint8_t valid,int16_t u) {
    visionRxPacket=lastRequest;
    visionRxPacket.type=VISION_MESSAGE_RESULT;
    visionRxPacket.flags=valid ? VISION_FLAG_VALID|VISION_FLAG_STABLE : 0;
    visionRxPacket.value[0]=u;visionRxPacket.value[1]=120;
    visionRxPacket.value[2]=valid ? 90 : 0;
    visionRxReady=1;serviceVision();
}
int main(void) {
    unsigned round,item,miss;
    static const uint8_t expected[2][3]={{4,2,6},{5,1,3}};
    visionMaterialCalibration.calibrated=1;
    for(round=0;round<3;++round) {
        unsigned batchMoves=moves,batchPicks=picks;
        missionActive=round!=2;routeIndex=round==1 ? 9 : 4;
        rawPickAtStation=routeWaiting=1;rawPickItemIndex=0;
        vision_session_init(&visionSession);visionPickActive=0;logText[0]=0;
        for(item=0;item<3;++item) {
            unsigned before=moves;
            mechanismState.valid=1;mechanismState.running=0;
            mechanismState.current.horizontalDmm=0;
            mechanismState.current.liftDmm=0;mechanismState.current.turretDdeg=2700;
            serviceRawPickRoute();assert(visionSession.state==VISION_STATE_PREP);
            mechanismState.current=mechanismState.target;mechanismState.running=0;
            serviceVision();assert(visionSession.state==VISION_STATE_REQUEST);
            for(miss=0;miss<5;++miss) {
                serviceVision();assert(lastRequest.selector==expected[round==1][item]);
                observe(0,260);assert(visionSession.state==VISION_STATE_REQUEST);
                assert(moves==before && !mechanismActionActive);
            }
            serviceVision();observe(1,180);
            if(item==0) {
                assert(visionSession.state==VISION_STATE_MOVE && moves==before+1);
                vision_session_move_complete(&visionSession,clockMs);
                clockMs+=300;imuStamp=clockMs;serviceVision();observe(1,160);
            } else {
                assert(visionSession.state==VISION_STATE_REQUEST && moves==before);
            }
            assert(visionSession.state==VISION_STATE_REQUEST);
            serviceVision();observe(1,item==0 ? 160 : 260);
            assert(visionSession.state==VISION_STATE_PICK && mechanismActionActive);
            assert(moves==before+(item==0));
            mechanismActionActive=0;serviceVision();
            assert(rawPickItemIndex==item+1 && visionSession.state==VISION_STATE_PICK_DONE);
        }
        assert(moves==batchMoves+1 && picks==batchPicks+3);
        assert(strstr(logText,round==1 ? "DONE color=3 slot=3" : "DONE color=6 slot=3"));
    }
    puts("PASS: actual raw pickup, one alignment per round, color-only later slots, both color plans and completion logs");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_raw_pick_runtime.c'
$exe = Join-Path $root 'Objects/test_raw_pick_runtime.exe'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $colors + "`n" + $active + "`n" + $service + "`n" + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o $exe
if ($LASTEXITCODE -ne 0) {throw 'Raw pickup runtime compile failed'}
& $exe
if ($LASTEXITCODE -ne 0) {throw 'Raw pickup runtime test failed'}
