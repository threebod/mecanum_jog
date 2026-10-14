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
$commands = $source.Substring($source.LastIndexOf('static uint8_t processVisionCommand('))
$commands = $commands.Substring(0, $commands.IndexOf('static uint8_t processTaskCommand('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../vision_config.h"
#include "../raw_pick_action.h"
#include "../task_code.h"
static MissionTask missionTask;
static VisionSession visionSession;
static VisionPacket visionRxPacket,lastRequest;
static uint8_t visionRxReady,visionPickActive,rawPickAtStation,rawPickItemIndex;
static uint8_t rawPickMissingReported,routeWaiting,missionActive,routeIndex;
static uint8_t rawPickCurrentColor,rawPickCurrentSlot,rawPickCollectedMask;
enum {MISSION_IDLE, MISSION_TEMP_ALIGN};
static uint8_t missionPhase;
static uint32_t clockMs,imuStamp;
static uint8_t imuValid=1,mechanismActionActive;
static uint8_t armed,routeActive,motionMode,servoChannelMoving[3];
static MechanismState mechanismState;
static MechanismAction rawPickRouteActions[RAW_PICK_ACTION_COUNT];
static const VisionCalibration visionPickCalibration={
    1,{0,0,320,240},160,120,{0,0,0,0}};
static unsigned moves,picks;
static float lastForward,lastRight;
static MechanismPose pickFinalPose;
static MechanismAction pickedActions[RAW_PICK_ACTION_COUNT];
static const char *lastFault;
static char logText[16384];
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static void serialSendString(const char *s) {strcat(logText,s);}
static void serialSendUint(uint16_t v) {char b[16];sprintf(b,"%u",v);strcat(logText,b);}
static void serialSendUint32(uint32_t v) {(void)v;}
static void serialSendInt(int16_t v) {char b[16];sprintf(b,"%d",v);strcat(logText,b);}
static void serialSendFloat1(float v) {(void)v;}
static uint8_t motionInterrupted(void) {return 0;}
static void printVisionState(void) {}
static void stopDriveMotors(void) {}
static void stopServoMotion(void) {}
static void stopAllMotors(void) {motionMode=routeActive=armed=0;}
static void invalidateNavigation(const char *reason) {(void)reason;}
static void failVision(const char *reason) {
    lastFault=reason;visionSession.state=VISION_STATE_FAILED;visionPickActive=0;
}
static void startMechanismPose(const MechanismPose *p) {
    mechanismState.target=*p;mechanismState.running=1;
}
static void visionUartSend(const VisionPacket *p) {lastRequest=*p;}
static uint8_t startVisionMotion(float f,float r,uint16_t rpm,uint8_t automatic) {
    if(!automatic) {
        assert(visionSession.sampleOnly && rpm==15);
        lastForward=f;lastRight=r;++moves;motionMode=4;return 1;
    }
    assert(rawPickItemIndex<3 && rpm==15 && automatic && (f!=0 || r!=0));
    assert((rawPickItemIndex==0 && visionSession.selector==0) ||
           visionSession.selector==rawPickCurrentColor);
    lastForward=f;lastRight=r;++moves;return 1;
}
uint8_t mechanismActionStart(const MechanismInitialState *initial,
                             const MechanismAction *actions,uint16_t count) {
    assert(initial==&rawPickInitial && count==RAW_PICK_ACTION_COUNT);
    memcpy(pickedActions,actions,sizeof(pickedActions));
    assert(actions[0].value==rawPickCurrentSlot);
    assert(rawPickCurrentSlot==rawPickItemIndex+1);
    assert(visionSession.selector==rawPickCurrentColor && visionSession.pickupMode==2);
    assert(visionSession.observedColor==rawPickCurrentColor);
    assert(!(rawPickCollectedMask & (1U << (rawPickCurrentColor-1))));
    assert(visionSession.state==VISION_STATE_ALIGNED);
    assert(!mechanismState.running && mechanismState.current.liftDmm==500);
    assert(actions[2].pose.liftDmm==500 && actions[3].type==MECHANISM_ACTION_GRIPPER);
    pickFinalPose=actions[RAW_PICK_ACTION_COUNT-1].pose;
    ++picks;mechanismActionActive=1;return 1;
}
'@
$suffix = @'
static void observeAt(uint8_t valid,int16_t u,int16_t v,uint8_t color) {
    visionRxPacket=lastRequest;
    visionRxPacket.type=VISION_MESSAGE_RESULT;
    visionRxPacket.flags=valid ? VISION_FLAG_VALID|VISION_FLAG_STABLE : 0;
    visionRxPacket.value[0]=u;visionRxPacket.value[1]=v;
    visionRxPacket.value[2]=0; /* Quality is diagnostic only, even for a valid target. */
    visionRxPacket.value[5]=color;
    visionRxReady=1;serviceVision();
}
static void observe(uint8_t valid,int16_t u,uint8_t color) {
    observeAt(valid,u,120,color);
}
static void prepare(void) {
    mechanismState.valid=1;mechanismState.running=0;
    mechanismState.current.horizontalDmm=0;
    mechanismState.current.liftDmm=0;
    mechanismState.current.turretDdeg=rawPickItemIndex==0 ? 650 : 2720;
    serviceRawPickRoute();assert(visionSession.state==VISION_STATE_PREP);
    assert(mechanismState.target.turretDdeg==2720);
    serviceVision();assert(visionSession.state==VISION_STATE_PREP);
    mechanismState.current=mechanismState.target;mechanismState.running=0;
    serviceVision();assert(visionSession.state==VISION_STATE_REQUEST);
    assert(mechanismState.running && mechanismState.current.liftDmm==0);
    assert(mechanismState.target.horizontalDmm==-770 && mechanismState.target.liftDmm==500);
    serviceVision();assert(visionSession.state==VISION_STATE_WAIT && mechanismState.running);
    observe(0,160,0); /* The camera runs before the lift completes. */
    assert(visionSession.state==VISION_STATE_REQUEST && mechanismState.running);
}
static void finishLowering(void) {
    mechanismState.current=mechanismState.target;mechanismState.running=0;
}
int main(void) {
    unsigned round,item,miss,permutation;
    static const uint8_t orders[6][3]={{4,2,6},{4,6,2},{2,4,6},{2,6,4},{6,4,2},{6,2,4}};
    visionMaterialCalibration.calibrated=1;
    assert(missionTaskParse("156+123+516+231",&missionTask));
    for(round=0;round<3;++round) for(permutation=0;permutation<6;++permutation) {
        unsigned batchMoves=moves,batchPicks=picks;
        missionActive=round!=2;routeIndex=round==1 ? 11 : 4;
        rawPickAtStation=routeWaiting=1;rawPickItemIndex=rawPickCollectedMask=0;
        rawPickCurrentColor=rawPickCurrentSlot=0;
        vision_session_init(&visionSession);visionPickActive=0;logText[0]=0;lastFault=0;
        for(item=0;item<3;++item) {
            unsigned before=moves;
            unsigned alignmentMoves=permutation==0 ? 0 : 1;
            uint8_t color=missionActive ? missionTask.colors[round==1][item] : rawPickColors[0][item];
            uint8_t collected=rawPickCollectedMask;
            prepare();
            finishLowering();
            assert(visionSession.pickupMode==(item==0 ? 1 : 2));
            assert(visionCalibration()==&visionMaterialCalibration);
            if(item==0) {
                uint8_t alignColor=(uint8_t)(permutation==4 ? 6 : permutation+1); /* Black is excluded from chassis calibration. */
                VisionPacket alignmentResult;
                assert(visionSession.selector==0 && visionSession.materialMask==47);
                for(miss=0;miss<60;++miss) {
                    clockMs+=1000;imuStamp=clockMs;serviceVision();
                    assert(lastRequest.selector==0 && lastRequest.value[6]==47);
                    observe(0,260,0);
                    assert(visionSession.state==VISION_STATE_REQUEST && !lastFault);
                    assert(moves==before && !mechanismActionActive);
                }
                serviceVision();observe(1,260,0); /* Unknown colors cannot align. */
                assert(visionSession.state==VISION_STATE_REQUEST && moves==before);
                serviceVision();observe(1,260,7);
                assert(visionSession.state==VISION_STATE_REQUEST && moves==before);
                serviceVision();observe(1,260,5);
                assert(visionSession.state==VISION_STATE_REQUEST && moves==before);
                serviceVision();observe(1,alignmentMoves ? 260 : 160,alignColor);
                if(alignmentMoves) {
                    assert(visionSession.state==VISION_STATE_MOVE && moves==before+1);
                    assert(lastForward < -63.99f && lastForward > -64.01f && lastRight==0);
                    vision_session_move_complete(&visionSession,clockMs);
                    clockMs+=300;imuStamp=clockMs;serviceVision();observe(1,191,alignColor);
                    assert(visionSession.state==VISION_STATE_MOVE && moves==before+2);
                    assert(visionSession.iteration==2 && !mechanismActionActive);
                    assert(lastForward < -19.83f && lastForward > -19.85f);
                    vision_session_move_complete(&visionSession,clockMs);
                    clockMs+=300;imuStamp=clockMs;serviceVision();observe(1,174,alignColor);
                }
                assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
                alignmentResult=visionRxPacket;
                /* Even alignment on the first task color needs a fresh pickup confirmation. */
                assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
                assert(visionSession.selector==color && visionSession.pickupMode==2);
                assert(rawPickCurrentColor==color && rawPickCurrentSlot==1);
                assert(rawPickCollectedMask==0 && picks==batchPicks);
                serviceVision();
                assert(lastRequest.selector==color && lastRequest.value[6]==(int16_t)(1U << (color-1)));
                visionRxPacket=alignmentResult;visionRxReady=1;serviceVision();
                assert(visionSession.state==VISION_STATE_WAIT && !mechanismActionActive);
                observe(0,260,0);
            }
            assert(visionSession.pickupMode==2 && visionCalibration()==&visionMaterialCalibration);
            before=moves;
            for(miss=0;miss<60;++miss) {
                clockMs+=1000;imuStamp=clockMs;serviceVision();
                assert(lastRequest.selector==color && lastRequest.value[6]==(int16_t)(1U << (color-1)));
                observe(0,260,0);
                assert(visionSession.state==VISION_STATE_REQUEST && !lastFault);
                assert(moves==before && !mechanismActionActive);
            }
            /* Every other color must wait, including supported colors appearing early. */
            for(miss=0;miss<8;++miss) {
                uint8_t rejected=(uint8_t)miss;
                if(rejected==color) continue;
                serviceVision();observe(1,260,rejected);
                assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
                assert(moves==before && rawPickCollectedMask==collected);
            }
            for(miss=0;miss<3;++miss) {
                uint8_t early=orders[permutation][miss];
                if(early==color) continue;
                serviceVision();observe(1,260,early);
                assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
                assert(moves==before);
            }
            serviceVision();
            visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
            --visionRxPacket.token;visionRxReady=1;serviceVision();
            assert(visionSession.state==VISION_STATE_WAIT && !mechanismActionActive);
            observe(1,191,color);
            assert(visionSession.state==VISION_STATE_MOVE && moves==before+1 && !mechanismActionActive);
            assert(lastForward < -19.83f && lastForward > -19.85f && lastRight==0);
            vision_session_move_complete(&visionSession,clockMs);
            clockMs+=300;imuStamp=clockMs;serviceVision();observe(1,174,color);
            assert(visionSession.state==VISION_STATE_PICK && mechanismActionActive);
            assert(rawPickCurrentColor==color && rawPickCurrentSlot==item+1);
            assert(rawPickCollectedMask==collected); /* Mark only after action completion. */
            assert(moves==before+1);
            mechanismState.current=pickFinalPose;
            mechanismActionActive=0;serviceVision();
            if(item==2) {
                assert(!mechanismState.running && mechanismState.current.turretDdeg==650);
                assert(mechanismState.current.horizontalDmm==0 && mechanismState.current.liftDmm==0);
                assert(moves==before+1);
            }
            assert(rawPickItemIndex==item+1 && visionSession.state==VISION_STATE_PICK_DONE);
            assert(rawPickCollectedMask==(collected | (1U << (color-1))));
            assert(!lastFault);
        }
        assert(rawPickCollectedMask==(missionActive ? 49 : 42) &&
               moves==batchMoves+3+2*(permutation!=0) && picks==batchPicks+3);
        assert(strstr(logText,"DONE color=6 slot=3"));
    }
    /* Only the expected yellow can pick, after the lift finishes and a fresh result arrives. */
    rawPickItemIndex=1;rawPickCollectedMask=8;visionPickActive=0;
    vision_session_init(&visionSession);prepare();
    /* A fast camera must not start pickup while the lift is still moving. */
    serviceVision();observe(1,160,2);
    serviceVision();observe(1,160,2);
    assert(visionSession.state==VISION_STATE_REQUEST && mechanismState.running &&
           !mechanismActionActive && !lastFault);
    finishLowering();
    serviceVision();observe(1,160,6);
    assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
    serviceVision();observe(0,160,2);
    assert(visionSession.state==VISION_STATE_REQUEST && !mechanismActionActive);
    serviceVision();observe(1,160,2);
    assert(visionSession.state==VISION_STATE_PICK && rawPickCurrentSlot==2);
    mechanismActionActive=0;serviceVision();
    /* Three unanswered camera requests must fail even though material waiting is unlimited. */
    rawPickItemIndex=0;rawPickCollectedMask=0;visionPickActive=0;
    vision_session_init(&visionSession);prepare();
    for(miss=0;miss<3;++miss) {
        serviceVision();clockMs+=1800;imuStamp=clockMs;serviceVision();
    }
    assert(lastFault && strcmp(lastFault,"CAMERA_TIMEOUT")==0);
    {
        VisionPacket oldRingResult;
        unsigned beforeMoves=moves,beforePicks=picks;
        assert(missionTaskParse("156+123+516+231",&missionTask));
        missionActive=1;missionPhase=MISSION_TEMP_ALIGN;routeIndex=16;
        rawPickAtStation=0;visionPickActive=0;mechanismActionActive=0;
        mechanismState.valid=1;mechanismState.running=0;
        visionRingCalibration[1]=visionMaterialCalibration;
        vision_session_init(&visionSession);
        assert(vision_session_start(&visionSession,VISION_MODE_RING,5,2,
                                   &visionRingCalibration[1],clockMs));
        visionSession.requestFlags=VISION_FLAG_RING_NO_DIGIT;
        visionSession.waitForTarget=1;
        serviceVision();
        assert(lastRequest.mode==VISION_MODE_RING && lastRequest.selector==5 &&
               lastRequest.flags==VISION_FLAG_RING_NO_DIGIT);
        oldRingResult=lastRequest;oldRingResult.type=VISION_MESSAGE_RESULT;
        observe(0,0,0);
        assert(visionSession.state==VISION_STATE_REQUEST && visionSession.mode==VISION_MODE_MATERIAL);
        assert(moves==beforeMoves && picks==beforePicks);
        serviceVision();
        assert(lastRequest.mode==VISION_MODE_MATERIAL && lastRequest.selector==5 &&
               lastRequest.value[6]==16 && lastRequest.flags==0 &&
               lastRequest.token>oldRingResult.token);
        visionRxPacket=oldRingResult;visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_WAIT && moves==beforeMoves);
        observe(1,160,4);
        assert(visionSession.state==VISION_STATE_REQUEST && moves==beforeMoves);
        serviceVision();observe(1,160,5);
        assert(visionSession.state==VISION_STATE_REQUEST && moves==beforeMoves);
        serviceVision();observe(1,160,5);
        assert(visionSession.state==VISION_STATE_ALIGNED && moves==beforeMoves && picks==beforePicks);
    }
    {
        unsigned beforeMoves=moves,beforePicks=picks;
        VisionPacket oldSample;
        rawPickAtStation=visionPickActive=mechanismActionActive=0;
        mechanismState.running=0;motionMode=routeActive=0;
        vision_session_init(&visionSession);
        visionRingCalibration[1].calibrated=0;
        assert(processVisionCommand("vision sample ring 2 0"));
        assert(visionSession.state==VISION_STATE_IDLE);
        assert(processVisionCommand("vision sample ring 1 7"));
        assert(visionSession.state==VISION_STATE_IDLE);
        servoChannelMoving[0]=1;
        assert(processVisionCommand("vision sample ring 2 7") && visionSession.state==VISION_STATE_IDLE);
        servoChannelMoving[0]=0;
        assert(processVisionCommand("vision sample ring 2 7") && !armed && visionSession.sampleOnly);
        serviceVision();oldSample=lastRequest;
        visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
        visionRxPacket.flags=VISION_FLAG_VALID;
        visionRxPacket.value[2]=80;visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_REQUEST && moves==beforeMoves && picks==beforePicks);
        serviceVision();assert(lastRequest.token>oldSample.token);
        logText[0]=0;
        visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
        visionRxPacket.flags=VISION_FLAG_VALID|VISION_FLAG_STABLE;
        visionRxPacket.value[0]=200;visionRxPacket.value[1]=140;visionRxPacket.value[2]=90;
        visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_IDLE && moves==beforeMoves && picks==beforePicks);
        assert(strstr(logText,"VISION MEASURE id=7 ring=2 token=") && strstr(logText," du=40 dv=20 quality=90"));
        armed=1;
        assert(processVisionCommand("vision jog 20 0 15") && motionMode==4 && !armed);
        assert(moves==beforeMoves+1 && lastForward==20 && lastRight==0);
        motionMode=0;vision_session_pause(&visionSession);
        assert(processVisionCommand("vision sample ring 2 8"));
        serviceVision();assert(lastRequest.token>oldSample.token);
        oldSample.type=VISION_MESSAGE_RESULT;oldSample.flags=VISION_FLAG_VALID|VISION_FLAG_STABLE;
        oldSample.value[2]=90;visionRxPacket=oldSample;visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_WAIT && moves==beforeMoves+1);
        visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
        visionRxPacket.flags=VISION_FLAG_VALID|VISION_FLAG_STABLE;visionRxPacket.value[2]=90;
        visionRxReady=1;serviceVision();assert(visionSession.state==VISION_STATE_IDLE);
        armed=1;
        assert(processVisionCommand("vision scale ring 2 500 1000") && !armed);
        assert(visionRingCalibration[1].calibrated && visionRingCalibration[1].matrix[0]==-0.5f &&
               visionRingCalibration[1].matrix[3]==-1.0f);
        armed=1;
        assert(processVisionCommand("vision align ring 2") && !visionSession.sampleOnly);
    }
    {
        unsigned beforeMoves=moves,beforePicks=picks;
        rawPickAtStation=visionPickActive=mechanismActionActive=0;
        mechanismState.running=0;motionMode=routeActive=0;
        vision_session_init(&visionSession);
        visionMaterialCalibration.calibrated=0;
        assert(processVisionCommand("vision sample material 0 9") && !visionSession.sampleOnly);
        assert(processVisionCommand("vision sample material 4 0") && !visionSession.sampleOnly);
        assert(processVisionCommand("vision sample material 7 9") && !visionSession.sampleOnly);
        imuValid=0;
        assert(processVisionCommand("vision sample material 4 9") && !visionSession.sampleOnly);
        imuValid=1;imuStamp=clockMs;
        assert(processVisionCommand("vision sample material 4 9") && visionSession.sampleOnly);
        serviceVision();
        assert(lastRequest.mode==VISION_MODE_MATERIAL && lastRequest.selector==4 && lastRequest.target==0);
        visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
        visionRxPacket.flags=VISION_FLAG_VALID|VISION_FLAG_STABLE;
        visionRxPacket.value[0]=180;visionRxPacket.value[1]=140;
        visionRxPacket.value[2]=0;visionRxPacket.value[5]=2;
        visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_REQUEST);
        serviceVision();
        visionRxPacket=lastRequest;visionRxPacket.type=VISION_MESSAGE_RESULT;
        visionRxPacket.flags=VISION_FLAG_VALID|VISION_FLAG_STABLE;
        visionRxPacket.value[0]=180;visionRxPacket.value[1]=140;
        visionRxPacket.value[2]=0;visionRxPacket.value[5]=4;
        logText[0]=0;visionRxReady=1;serviceVision();
        assert(visionSession.state==VISION_STATE_IDLE && moves==beforeMoves && picks==beforePicks);
        assert(strstr(logText,"VISION MEASURE id=9 material=4 token=") && strstr(logText," quality=0"));
        armed=1;
        assert(processVisionCommand("vision scale material 500 1000") && !armed);
        assert(visionMaterialCalibration.matrix[0]==-0.5f && visionMaterialCalibration.matrix[3]==-1.0f);
    }
    {
        unsigned index;
        const int16_t vertical[3]={35,-35,10};
        const int16_t expected[3]={-436,36,-267};
        missionActive=0;routeIndex=4;rawPickAtStation=routeWaiting=1;
        visionMaterialCalibration.matrix[3]=-0.673f;
        for(index=0;index<3;++index) {
            unsigned beforeMoves=moves,beforePicks=picks;
            rawPickItemIndex=1;rawPickCollectedMask=8;
            visionPickActive=mechanismActionActive=0;lastFault=0;
            vision_session_init(&visionSession);prepare();finishLowering();
            serviceVision();observeAt(1,174,(int16_t)(120+vertical[index]),2);
            assert(moves==beforeMoves && picks==beforePicks+1 && !lastFault);
            assert(pickedActions[2].pose.horizontalDmm==expected[index]);
            assert(pickedActions[4].pose.horizontalDmm==expected[index]);
            assert(pickedActions[5].pose.horizontalDmm==expected[index]);
            assert(pickedActions[6].pose.horizontalDmm==-500 && pickedActions[7].pose.liftDmm==250);
            assert(pickedActions[0].value==2 && pickedActions[10].value==3);
            assert(rawPickActions[2].pose.horizontalDmm==-200); /* No global or previous-item drift. */
        }
        {
            unsigned beforeMoves=moves,beforePicks=picks;
            rawPickItemIndex=1;rawPickCollectedMask=8;
            visionPickActive=mechanismActionActive=0;lastFault=0;
            vision_session_init(&visionSession);prepare();finishLowering();
            serviceVision();observeAt(1,175,155,2);
            assert(moves==beforeMoves+1 && lastRight==0 && picks==beforePicks);
            vision_session_move_complete(&visionSession,clockMs);
            clockMs+=300;imuStamp=clockMs;serviceVision();
            observeAt(1,160,85,2); /* Use the fresh Y after chassis correction. */
            assert(picks==beforePicks+1 && pickedActions[2].pose.horizontalDmm==36);
        }
        for(index=0;index<2;++index) {
            unsigned beforePicks=picks;
            rawPickItemIndex=1;rawPickCollectedMask=8;
            visionPickActive=mechanismActionActive=0;lastFault=0;
            visionMaterialCalibration.matrix[3]=-2.0f;
            vision_session_init(&visionSession);prepare();finishLowering();
            serviceVision();observeAt(1,160,index==0 ? 239 : 1,2);
            assert(picks==beforePicks && lastFault && strcmp(lastFault,"PICK_HORIZONTAL_RANGE")==0);
        }
        {
            unsigned beforeMoves=moves,beforePicks=picks;
            rawPickItemIndex=rawPickCollectedMask=0;
            visionPickActive=mechanismActionActive=0;lastFault=0;
            visionMaterialCalibration.matrix[3]=-0.673f;
            vision_session_init(&visionSession);prepare();finishLowering();
            serviceVision();observeAt(1,160,170,2); /* Any-color alignment must not fix Y on the chassis. */
            assert(moves==beforeMoves && picks==beforePicks && visionSession.selector==4);
            serviceVision();observeAt(1,160,85,4);
            assert(picks==beforePicks+1 && pickedActions[2].pose.horizontalDmm==36);
            assert(pickedActions[0].value==1 && !lastFault); /* Use this item's Y, not the prior color's. */
        }
    }
    puts("PASS: pickup runtime, X-only chassis correction, Y grasp compensation and horizontal limits");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_raw_pick_runtime.c'
$exe = Join-Path $root 'Objects/test_raw_pick_runtime.exe'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $colors + "`n" + $active + "`n" + $service + "`n" + $commands + "`n" + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o $exe
if ($LASTEXITCODE -ne 0) {throw 'Raw pickup runtime compile failed'}
& $exe
if ($LASTEXITCODE -ne 0) {throw 'Raw pickup runtime test failed'}
