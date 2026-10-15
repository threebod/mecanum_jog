$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
# Compile the actual route state machine with a simulated clock / IMU / motors.
$service = $source.Substring($source.LastIndexOf('static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn)'))
$service = $service.Substring(0, $service.IndexOf('static void serviceRawPickRoute('))
$mission = $source.Substring($source.IndexOf('static void missionReport('))
$mission = $mission.Substring(0, $mission.IndexOf('static uint8_t processVisionCommand('))
$parser = $source.Substring($source.IndexOf('static uint8_t parseUint('))
$parser = $parser.Substring(0, $parser.IndexOf('static CommandResult parseServoCommand('))
$pair = $source.Substring($source.IndexOf('static uint8_t parsePair('))
$pair = $pair.Substring(0, $pair.IndexOf('static void printMechanismPose('))
$motion = $source.Substring($source.LastIndexOf('static void serviceMotion(void)'))
$motion = $motion.Substring(0, $motion.IndexOf('static uint8_t parsePair('))
$calibration = $source.Substring($source.IndexOf('static const VisionCalibration *visionCalibration(void)'))
$calibration = $calibration.Substring(0, $calibration.IndexOf('static const char *visionStateName('))
$taskParser = $source.Substring($source.IndexOf('static uint8_t processTaskCommand('))
$taskParser = $taskParser.Substring(0, $taskParser.IndexOf('static void processCommand('))
$buttonInit = $source.Substring($source.IndexOf('static void missionButtonInit('))
$buttonInit = $buttonInit.Substring(0, $buttonInit.IndexOf('static void serialInit('))
$buttonDefines = ([regex]::Matches($source, '#define MISSION_BUTTON_[^\r\n]+') | ForEach-Object { $_.Value }) -join "`n"
$qrService = $source.Substring($source.IndexOf('static void serviceQrScan('))
$qrService = $qrService.Substring(0, $qrService.IndexOf('void UART4_IRQHandler('))
$watchdog = $source.Substring($source.IndexOf('static void serviceHostWatchdog('))
$watchdog = $watchdog.Substring(0, $watchdog.IndexOf('static void processServoCommand('))
$displayReset = $source.Substring($source.IndexOf('static void tjcDisplayResetCounts('))
$displayReset = $displayReset.Substring(0, $displayReset.IndexOf('void USART3_IRQHandler('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../route_plan.h"
#include "../map_navigation.h"
#include "../straight_control.h"
#include "../mission_actions.h"
#include "../raw_pick_action.h"
#include "../task_code.h"
#include "../qr_scan.h"
static MissionTask missionTask;
static uint8_t missionTaskValid;
static uint16_t missionGrabCount,missionPlaceCount;
static unsigned countUpdates;
static char missionQrCode[16];
static uint8_t missionQrValid;
static uint8_t missionOffline;
static uint8_t qrRxReady;
static uint8_t qrAtStation, qrResultAtStation;
static char qrRxPayload[QR_SCAN_PAYLOAD_MAX+1];
static uint8_t hostHeartbeatActive;
static uint32_t hostHeartbeatStamp;
#define HOST_HEARTBEAT_TIMEOUT_MS 1000U
static uint8_t missionButtonReady,missionButtonRawHigh,missionButtonStableHigh,missionButtonLongSent;
static uint16_t missionButtonRpm;
static uint32_t missionButtonChangedAt,missionButtonPressedAt;
static uint8_t buttonPinHigh=1;
static unsigned motorEnableCalls;
#define GPIOC 1
#define GPIO_Pin_9 0x0200U
#define GPIO_Mode_IN 0
#define GPIO_PuPd_UP 1
#define RCC_AHB1Periph_GPIOC 1
#define ENABLE 1
typedef struct {unsigned GPIO_Pin,GPIO_Mode,GPIO_PuPd;} GPIO_InitTypeDef;
static void RCC_AHB1PeriphClockCmd(unsigned port,unsigned enabled) {
    assert(port==RCC_AHB1Periph_GPIOC && enabled==ENABLE);
}
static void GPIO_StructInit(GPIO_InitTypeDef *gpio) {memset(gpio,0,sizeof(*gpio));}
static void GPIO_Init(unsigned port,const GPIO_InitTypeDef *gpio) {
    assert(port==GPIOC && gpio->GPIO_Pin==GPIO_Pin_9 &&
           gpio->GPIO_Mode==GPIO_Mode_IN && gpio->GPIO_PuPd==GPIO_PuPd_UP);
}
static uint8_t GPIO_ReadInputDataBit(unsigned port,unsigned pin) {
    assert(port==GPIOC && pin==GPIO_Pin_9);return buttonPinHigh;
}
static uint8_t routeActive,routeWaiting,routeIndex,routeStartZone,routeStep;
static uint8_t routeAuto,routeRotating,routeOnlyTurn,routeTurnInBand,fullRouteRunning;
static uint8_t routeStationAlignment;
static uint8_t rawPickRouteActive,rawPickAtStation,rawPickItemIndex;
static uint8_t rawPickMissingReported;
static uint8_t rawPickCurrentColor,rawPickCurrentSlot,rawPickCollectedMask;
static const uint8_t rawPickColors[2][3]={{4,2,6},{4,2,6}};
typedef enum {
    MISSION_IDLE, MISSION_QR_WAIT, MISSION_RAW_PICK,
    MISSION_RING_ALIGN, MISSION_TEMP_ALIGN, MISSION_COARSE_PLACE, MISSION_COARSE_PICK,
    MISSION_TEMP_PLACE, MISSION_RECENTER, MISSION_RAW_EXIT
} MissionPhase;
static uint8_t missionActive,missionActionStarted;
static MissionPhase missionPhase;
static uint32_t missionDeadline;
static float missionOffsetForward,missionOffsetRight;
static MechanismAction missionWorkActions[MISSION_PLACE_COUNT];
static MechanismAction missionPickBackActions[MISSION_PICK_BACK_COUNT];
static const MechanismPose missionRingObservePose={-770,0,2720,1000,200,90,50,2200};
static uint16_t mechanismActionIndex,mechanismActionCount;
static unsigned missionPlaceRuns,missionPickRuns,missionStorageRuns,missionRingAligns,missionMaterialAligns;
static unsigned missionRawItems, missionRawExitMoves;
static float lastVisionForward,lastVisionRight;
static uint8_t visionPickActive;
static uint8_t motionMode,armed,imuValid=1,emergencyStop,jogCanFault;
static MechanismState mechanismState;
static uint8_t mechanismActionActive;
static uint8_t servoChannelMoving[3];
static uint8_t servoChannelEnabled[3]={1,1,1};
static uint32_t currentAngleMdeg[3]={90000,26000,270000};
static uint8_t auxMoveMotorId;
static int8_t routeHeading,routeNextHeading,yawSign=1;
static float routeX,routeY,routeYaw,routeBaseYaw,imuYaw,routeLegProgress;
static float routeDriveRpm,routeTurnRpm,routeCorrection;
static int16_t routeRawLeftCorrection;
static uint32_t headingPidLastReport;
static RouteHeadingPid routeHeadingPid;
static RouteTurnPid routeTurnPid;
static RouteHeadingPidGains routeHeadingGains={
    ROUTE_HEADING_KP,ROUTE_HEADING_KI,ROUTE_HEADING_KD};
static uint16_t routeRpm=ROUTE_RPM;
static float routeLateralScale=ROUTE_LATERAL_SCALE;
static float routeForwardScale=ROUTE_FORWARD_SCALE;
static uint16_t routeTurnRpmLimit=(uint16_t)ROUTE_TURN_RPM;
static uint16_t routeLateralRpmLimit=ROUTE_LATERAL_RPM_MAX;
static uint8_t routeSentValid;
static int16_t routeSent[4], staged[4];
static const uint8_t motorDirections[1][5]={{1,1,0,0,1}};
static uint8_t motorInvert[5];
static uint16_t motorTrim[5]={1000,1000,1000,1000,1000};
static unsigned batches, writes;
static unsigned statusReads;
static uint8_t straightLateral;
static uint8_t straightUseRoutePid;
static float straightSpeedState,straightTurnState,targetYaw;
static char relativeMoveDirection;
static uint16_t relativeMoveDistance;
static float relativeMoveProgress;
static unsigned moveDoneReports;
static unsigned stopCalls;
static uint32_t motionStart,motionDuration,lastControl;
static uint16_t straightRpm;
static int16_t straightDirection;
static int16_t routeForward,routeRight,commandTurn;
static uint32_t routeTick,routeLegStart,routeSettle,routeTurnStart,routeTurnStable,clockMs,imuStamp;
static unsigned turnTicks;
static uint8_t navInitialized,navRunning,navCurrentNode,navPathCount;
static NavPoint navPath[NAV_PATH_CAPACITY];
static uint32_t navLastReport,routeLastReport;
static unsigned navPosReports,navDoneReports,navInvalidReports;
static unsigned pidTraceReports;
static unsigned routePosReports,routeStageReports,routeDoneReports,routeInvalidReports;
enum {VISION_STATE_IDLE, VISION_STATE_REQUEST, VISION_STATE_ALIGNED,
      VISION_STATE_PICK, VISION_STATE_PICK_DONE, VISION_STATE_WAIT};
#define VISION_MODE_RING 2U
#define VISION_MODE_MATERIAL 1U
typedef struct {uint8_t state,waitForTarget,mode,target,selector,requestFlags;} VisionSession;
#define VISION_FLAG_RING_NO_DIGIT 0x04U
static VisionSession visionSession;
typedef struct {uint8_t calibrated;} VisionCalibration;
static VisionCalibration visionMaterialCalibration={1};
static VisionCalibration visionPickCalibration={1};
static VisionCalibration visionRingCalibration[3]={{1},{1},{1}};
static uint8_t vision_calibration_valid(const VisionCalibration *calibration) {
    return calibration->calibrated;
}
static uint8_t visionSessionActive(void) {return 0;}
static void vision_session_init(VisionSession *session) {session->state=0;}
static uint8_t vision_session_start(VisionSession *session,uint8_t mode,
        uint8_t selector,uint8_t target,const VisionCalibration *cal,uint32_t now) {
    (void)mode;(void)selector;(void)target;(void)now;
    if(!cal->calibrated) return 0;
    if(mode==VISION_MODE_RING) {
        assert(target==2 && cal==&visionRingCalibration[1]);
        assert(mechanismState.current.horizontalDmm==-770);
        assert(missionPhase!=MISSION_TEMP_ALIGN || routeIndex==9 || routeIndex==16);
        if(missionPhase==MISSION_TEMP_ALIGN && routeIndex==16) {
            unsigned item=0;
            while(missionTask.rings[0][item]!=2) ++item;
            assert(selector==missionTask.colors[0][item]);
        } else assert(selector==0);
    } else {
        unsigned item=0;
        while(missionTask.rings[0][item]!=2) ++item;
        assert(mode==VISION_MODE_MATERIAL && selector==missionTask.colors[0][item] && target==2);
        assert(missionPhase==MISSION_TEMP_ALIGN && routeIndex==16);
        assert(cal==&visionRingCalibration[1]);
        assert(mechanismState.current.horizontalDmm==-770);
        ++missionMaterialAligns;
    }
    if(mode==VISION_MODE_RING) ++missionRingAligns;
    session->state=VISION_STATE_REQUEST;
    session->mode=mode;session->target=target;session->selector=selector;session->requestFlags=0;
    return 1;
}
static uint8_t visionMotionAutomatic;
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define AUX_MOVE_STATUS_POLL_MS 50U
#define S_FLAG 13
typedef struct {uint32_t ExtId;uint8_t DLC;uint8_t Data[8];} TestCanRxMsg;
static struct {TestCanRxMsg CAN_RxMsg;bool rxFrameFlag;} can;
static void serialSendString(const char *s) {
    if(strcmp(s,"MOVE DONE dir=")==0) ++moveDoneReports;
    if(strcmp(s,"NAV POS x=")==0) ++navPosReports;
    if(strcmp(s,"PID TRACE target_cdeg=")==0) ++pidTraceReports;
    if(strcmp(s,"NAV DONE x=")==0) ++navDoneReports;
    if(strncmp(s,"NAV INVALID",11)==0) ++navInvalidReports;
    if(strcmp(s,"ROUTE DIST index=")==0) ++routePosReports;
    if(strcmp(s,"ROUTE STAGE index=")==0) ++routeStageReports;
    if(strcmp(s,"ROUTE DONE mode=DISTANCE\r\n")==0) ++routeDoneReports;
    if(strncmp(s,"ROUTE INVALID",13)==0) ++routeInvalidReports;
}
static void serialSendChar(char c) {(void)c;}
static void serialSendUint(uint16_t x) {(void)x;}
static void tjcDisplayQr(const char *s) {(void)s;}
static void tjcDisplayCounts(void) {++countUpdates;}
static void tjcDisplayResetCounts(void);
static void serialSendInt(int16_t x) {(void)x;}
static void printRawPickState(const char *state,uint8_t color,uint8_t slot);
static void printRouteStatus(void) {}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAllMotorsEnabled(bool x) {if(x) ++motorEnableCalls;}
static void stopDriveMotors(void) {routeForward=routeRight=commandTurn=0;}
static void failVision(const char *reason) {(void)reason;motionMode=0;}
static void printVisionState(void) {}
static void stopServoMotion(void) {}
static void startMechanismPose(const MechanismPose *pose) {
    mechanismState.target=*pose;mechanismState.running=1;
    mechanismState.deadlineMs=clockMs+120;
}
uint8_t mechanismActionStart(const MechanismInitialState *initial,
        const MechanismAction *actions,uint16_t count) {
    (void)initial;(void)actions;
    if(actions==missionWorkActions && missionPhase==MISSION_COARSE_PLACE) {
        assert(initial==&missionCoarseInitial && count==MISSION_COARSE_PLACE_COUNT);
        static const uint16_t angle[3]={3065,2657,2320};
        static const int16_t horizontal[3]={55,-240,435};
        for(unsigned item=0;item<3;++item) {
            unsigned platform=item==0 ? 0 : item==1 ? 10 : 18;
            unsigned release=item==0 ? 8 : item==1 ? 16 : 26;
            assert(actions[platform].value==item+1);
            assert(actions[release].pose.turretDdeg==angle[missionTask.rings[routeIndex==14][item]-1]);
            assert(actions[release].pose.horizontalDmm==horizontal[missionTask.rings[routeIndex==14][item]-1]);
        }
        ++missionPlaceRuns;
    } else if(actions==missionPickBackActions) {
        assert(initial==&missionCoarseInitial && count==MISSION_PICK_BACK_COUNT);
        unsigned ring=missionTask.rings[routeIndex==14][2]-1;
        static const int16_t ringH[3]={55,-240,435};
        static const uint16_t ringT[3]={3065,2657,2320};
        assert(mechanismState.current.horizontalDmm==ringH[ring] &&
               mechanismState.current.liftDmm==1300 &&
               mechanismState.current.turretDdeg==ringT[ring]);
        assert(missionPlaceRuns==missionPickRuns+1); /* All three placed before recovery. */
        assert(actions[0].type==MECHANISM_ACTION_POSE && actions[0].pose.liftDmm==0);
        assert(actions[0].pose.horizontalDmm==(routeIndex==14 ? -470 : -450) &&
               actions[0].pose.turretDdeg==1320);
        for(unsigned i=0;i<3;++i) {
            unsigned base=1+i*11,retract=base+7;
            ring=missionTask.rings[routeIndex==14][i]-1;
            assert(actions[base].value==i+1);
            assert(actions[base+1].type==MECHANISM_ACTION_GRIPPER && actions[base+1].value==1);
            assert(actions[base+2].pose.liftDmm==0);
            assert(actions[base+3].pose.horizontalDmm==ringH[ring] &&
                   actions[base+3].pose.turretDdeg==ringT[ring] && actions[base+3].pose.liftDmm==1300);
            assert(actions[base+4].type==MECHANISM_ACTION_GRIPPER && actions[base+4].value==0);
            assert(actions[base+6].pose.horizontalDmm==ringH[ring] && actions[base+6].pose.turretDdeg==1320);
            assert(actions[retract].pose.horizontalDmm==(routeIndex==14 ? -470 : -450));
            assert(actions[retract+1].pose.horizontalDmm==(routeIndex==14 ? -470 : -450));
            assert(actions[retract+1].pose.liftDmm==300);
            assert(actions[retract+2].type==MECHANISM_ACTION_GRIPPER && actions[retract+2].value==1);
            if(i<2) assert(actions[retract+3].pose.horizontalDmm==(routeIndex==14 ? -470 : -450));
        }
        ++missionPickRuns;
    }
    else if(actions==missionWorkActions) {
        assert(initial==&missionInitial && count==MISSION_PLACE_COUNT);
        assert(visionSession.state==VISION_STATE_ALIGNED);
        static const uint16_t angle[3]={3065,2657,2320};
        static const int16_t horizontal[3]={55,-240,435};
        for(unsigned item=0;item<3;++item) {
            unsigned release=item==0 ? 8 : item==1 ? 16 : 26;
            unsigned platform=item==0 ? 0 : item==1 ? 10 : 18;
            unsigned batch=routeIndex==16;
            unsigned baseSlot=0;
            while(missionTask.colors[0][baseSlot]!=missionTask.colors[batch][item]) ++baseSlot;
            assert(actions[platform].value==item+1);
            assert(actions[release].pose.turretDdeg==angle[missionTask.rings[0][baseSlot]-1]);
            assert(actions[release].pose.horizontalDmm==horizontal[missionTask.rings[0][baseSlot]-1]);
        }
        assert(missionRingAligns+missionMaterialAligns==2 ||
               missionRingAligns+missionMaterialAligns==4);
        ++missionStorageRuns;
    }
    mechanismActionActive=1;mechanismActionIndex=0;mechanismActionCount=count;
    return 1;
}
static uint8_t startVisionMotion(float forward,float right,uint16_t rpm,uint8_t automatic) {
    (void)rpm;(void)automatic;
    if((routeIndex==4 || routeIndex==11) && missionPhase==MISSION_RECENTER) {
        assert(rpm==15 && !automatic);
        ++missionRawExitMoves;
    }
    lastVisionForward=forward;lastVisionRight=right;
    missionOffsetForward+=forward;missionOffsetRight+=right;
    motionMode=4;return 1;
}
static void vision_session_move_complete(VisionSession *session,uint32_t now) {
    (void)session;(void)now;
}
static void vision_session_pause(VisionSession *session) {(void)session;}
static void invalidateNavigation(const char *reason) {
    (void)reason;serialSendString("NAV INVALID reason=");
    navInitialized=navRunning=navPathCount=0;
}
static void stopAllMotors(void) {
    ++stopCalls;
    routeActive=routeWaiting=routeRotating=routeOnlyTurn=routeTurnInBand=0;
    routeStationAlignment=0;
    fullRouteRunning=0;
    missionButtonReady=0;
    missionOffline=0;
    missionQrValid=0;
    missionActive=missionActionStarted=0;missionPhase=MISSION_IDLE;
    rawPickRouteActive=rawPickAtStation=rawPickItemIndex=rawPickMissingReported=0;
    rawPickCurrentColor=rawPickCurrentSlot=rawPickCollectedMask=0;
    routeForward=routeRight=commandTurn=0; armed=0;
    navInitialized=navRunning=navPathCount=0;
    routeDriveRpm=routeTurnRpm=routeCorrection=0;routeSentValid=0;
    routeHeadingPidReset(&routeHeadingPid);
    motionMode=0;straightUseRoutePid=0;straightSpeedState=straightTurnState=0;
}
static void Emm_V5_Vel_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t acc,bool sync) {
    (void)acc;assert(sync);++writes;
    staged[id-1]=(dir==motorDirections[0][id])?(int16_t)rpm:-(int16_t)rpm;
}
static void Emm_V5_Synchronous_motion(uint8_t id) {
    assert(id==0);++batches;
    commandTurn=(int16_t)((staged[0]-staged[1]-staged[2]+staged[3])/4);
    if(commandTurn) ++turnTicks;
}
static void Emm_V5_Read_Sys_Params(uint8_t id,int parameter) {
    assert(id==auxMoveMotorId && parameter==S_FLAG);++statusReads;
}
static void missionBeginStation(uint8_t nextIndex);
'@
$suffix = @'
static void receiveQr(const char *code) {
    strcpy(qrRxPayload,code);qrRxReady=1;serviceQrScan();
}
static void checkOfflineMission(void) {
    stopAllMotors();imuStamp=clockMs;imuValid=1;emergencyStop=jogCanFault=0;
    mechanismState.valid=0;mechanismState.running=mechanismActionActive=0;
    visionMaterialCalibration.calibrated=visionRingCalibration[1].calibrated=0;
    memset(servoChannelMoving,0,sizeof(servoChannelMoving));
    servoChannelEnabled[0]=servoChannelEnabled[1]=1;
    currentAngleMdeg[0]=35000;currentAngleMdeg[1]=26000;
    buttonPinHigh=1;missionButtonInit();prepareOfflineMission();
    assert(missionButtonReady && missionButtonRpm==ROUTE_RPM);
    assert(mechanismState.valid && mechanismState.current.horizontalDmm==0 &&
           mechanismState.current.liftDmm==0 && mechanismState.current.turretDdeg==650);
    assert(visionMaterialCalibration.calibrated && visionRingCalibration[1].calibrated);
    hostHeartbeatActive=1;hostHeartbeatStamp=clockMs-1001;
    serviceHostWatchdog();assert(missionButtonReady);
    missionTaskValid=1;startButtonMission(1);
    assert(routeActive && missionActive && missionOffline && !missionTaskValid);
    assert(routeStartZone==1 && routeRpm==ROUTE_RPM);
    serviceHostWatchdog();assert(routeActive && missionOffline);
    receiveQr("156+113+516+231");assert(!missionTaskValid && !missionQrValid);
    routeWaiting=1;routeIndex=2;missionBeginStation(2);
    assert(!missionTaskValid);
    clockMs+=2000;imuStamp=clockMs;serviceMission();
    assert(routeWaiting && missionPhase==MISSION_QR_WAIT);
    qrAtStation=1;qrResultAtStation=0;
    receiveQr("156+113+516+231");assert(!missionTaskValid);
    assert(!qrResultAtStation);
    serviceMission();assert(routeWaiting);
    receiveQr("516+231+156+312");assert(qrResultAtStation);serviceMission();assert(missionTaskValid);
    qrAtStation=0;
    assert(missionTask.colors[0][0]==5 && missionTask.rings[1][0]==3);
    MissionTask scanned=missionTask;
    receiveQr("156+123+516+231");
    assert(memcmp(&scanned,&missionTask,sizeof(scanned))==0);
    serviceMission();assert(!routeWaiting && missionPhase==MISSION_IDLE);
    stopAllMotors();assert(!missionOffline && !missionButtonReady && !missionQrValid);
    serviceHostWatchdog();assert(!hostHeartbeatActive);
    imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route mission 1 40") && routeActive && !missionOffline);
    hostHeartbeatActive=1;hostHeartbeatStamp=clockMs-1001;
    serviceHostWatchdog();assert(!routeActive && !missionOffline);
    hostHeartbeatActive=0;
    /* Single-shot K230 can send before the button press, then never send again. */
    stopAllMotors();imuStamp=clockMs;prepareOfflineMission();
    receiveQr("156+123+516+231");assert(missionQrValid);
    startButtonMission(1);assert(routeActive && !missionTaskValid && missionQrValid);
    routeWaiting=1;routeIndex=2;missionBeginStation(2);serviceMission();
    assert(missionTaskValid && !missionQrValid && routeWaiting);
    clockMs+=999;imuStamp=clockMs;serviceMission();assert(routeWaiting);
    clockMs+=1;imuStamp=clockMs;serviceMission();assert(!routeWaiting);
    assert(missionTask.colors[0][0]==1 && missionTask.rings[1][0]==2);
    stopAllMotors();imuStamp=clockMs;prepareOfflineMission();startButtonMission(1);
    routeWaiting=1;routeIndex=2;missionBeginStation(2);
    clockMs+=2000;imuStamp=clockMs;serviceMission();
    assert(routeWaiting && !missionTaskValid && !missionQrValid);
    stopAllMotors();
    puts("PASS: offline boot, QR freshness/validation/latching, wait, and host watchdog isolation");
}
static void buttonTick(uint32_t elapsed,uint8_t high) {
    clockMs+=elapsed;imuStamp=clockMs;buttonPinHigh=high;serviceMissionButton();
}
static void prepareButtonTest(uint8_t high,uint16_t rpm) {
    char command[32];
    stopAllMotors();emergencyStop=jogCanFault=0;imuValid=1;imuStamp=clockMs;
    mechanismState.valid=1;mechanismState.running=mechanismActionActive=0;
    mechanismState.current.horizontalDmm=0;mechanismState.current.liftDmm=0;
    mechanismState.current.turretDdeg=650;
    memset(servoChannelMoving,0,sizeof(servoChannelMoving));
    servoChannelEnabled[0]=servoChannelEnabled[1]=1;
    currentAngleMdeg[0]=90000;currentAngleMdeg[1]=26000;
    vision_session_init(&visionSession);
    visionMaterialCalibration.calibrated=visionRingCalibration[1].calibrated=1;
    missionTaskValid=1;buttonPinHigh=high;missionButtonInit();
    unsigned enables=motorEnableCalls;
    armed=1;sprintf(command,"route button %u",rpm);
    assert(processRouteCommand(command) && missionButtonReady && !armed);
    assert(!routeActive && !missionActive && motorEnableCalls==enables);
}
static void checkMissionButton(void) {
    unsigned enables;
    prepareButtonTest(1,90);enables=motorEnableCalls;
    /* Bounce must not produce a press; short press starts only after release. */
    buttonTick(0,0);buttonTick(10,1);buttonTick(30,1);
    assert(!routeActive && missionButtonReady && motorEnableCalls==enables);
    buttonTick(0,0);buttonTick(29,0);assert(!routeActive);
    buttonTick(1,0);buttonTick(100,0);assert(!routeActive);
    buttonTick(0,1);buttonTick(29,1);assert(!routeActive);
    buttonTick(1,1);
    assert(routeActive && missionActive && routeStartZone==1 && routeRpm==90);
    assert(!missionButtonReady && !armed && motorEnableCalls==enables+1);
    /* Presses while running are ignored, with no queued restart. */
    buttonTick(0,0);buttonTick(1000,0);buttonTick(2000,0);
    assert(motorEnableCalls==enables+1);
    prepareButtonTest(1,250);enables=motorEnableCalls;
    buttonTick(0,0);buttonTick(999,0);assert(!routeActive);
    buttonTick(1,0);
    assert(routeActive && missionActive && routeStartZone==2 && routeRpm==250);
    buttonTick(2000,0);buttonTick(0,1);buttonTick(30,1);
    assert(motorEnableCalls==enables+1); /* Long release must not start zone 1. */
    /* Held during boot stays inert until released and pressed again. */
    prepareButtonTest(0,40);buttonTick(5000,0);assert(!routeActive && missionButtonReady);
    buttonTick(0,1);buttonTick(30,1);assert(!routeActive);
    buttonTick(0,0);buttonTick(30,0);buttonTick(0,1);buttonTick(30,1);
    assert(routeActive && routeStartZone==1 && routeRpm==40);
    /* Start-time checks must reject changed state, and stop cancels readiness. */
    prepareButtonTest(1,90);mechanismState.valid=0;
    startButtonMission(1);assert(!routeActive && !armed && !missionButtonReady);
    prepareButtonTest(1,90);visionMaterialCalibration.calibrated=0;
    startButtonMission(2);assert(!routeActive && !armed);
    prepareButtonTest(1,90);missionTaskValid=0;
    startButtonMission(1);assert(routeActive && missionOffline && !missionTaskValid && !armed);
    prepareButtonTest(1,90);imuStamp=clockMs-251;
    startButtonMission(1);assert(!routeActive && !armed);
    prepareButtonTest(1,90);enables=motorEnableCalls;jogCanFault=1;
    startButtonMission(1);assert(!routeActive && motorEnableCalls==enables);
    prepareButtonTest(1,90);enables=motorEnableCalls;emergencyStop=1;
    startButtonMission(1);assert(!routeActive && motorEnableCalls==enables);
    prepareButtonTest(1,90);enables=motorEnableCalls;stopAllMotors();
    startButtonMission(1);assert(!routeActive && motorEnableCalls==enables);
    /* Unsigned elapsed-time arithmetic must work across the millisecond wrap. */
    clockMs=0xFFFFFFF0U;prepareButtonTest(1,120);
    buttonTick(0,0);buttonTick(1000,0);
    assert(routeActive && routeStartZone==2 && routeRpm==120);
    stopAllMotors();
    puts("PASS: PC9 pull-up, debounce, short/long zone selection, no repeat, startup guards and timer wrap");
}
static float temporaryApproachTravel[2];
static float temporaryLeftTravel[2];
static float coarseApproachTravel[2];
static float rawReturnTravel;
static float secondRawRightTravel;
static unsigned rawReturnTurnTicks;
static float firstRawApproachTravel;
static float firstRawUpTravel;
static unsigned firstRawTurnTicks;
static unsigned firstRawDepartureTurnTicks;
static float homeVerticalTravel,homeHorizontalTravel,homeTailVerticalTravel,homeTailRightTravel;
static float coarseLeftTravel[2],coarseRightTravel[2];
static void tick(void) {
    /* Ideal plant: (L+W)/2 = 215mm, wheel diameter100mm. */
    imuYaw += commandTurn * 1.3955f * 0.020f * yawSign;
    while(imuYaw>180) imuYaw-=360;
    while(imuYaw<-180) imuYaw+=360;
    uint8_t distanceRoute = !navRunning && routeActive && !routeOnlyTurn;
    uint8_t previousIndex = routeIndex;
    float previousProgress = routeLegProgress;
    float staleX = -9000.0f - clockMs % 1000;
    float staleY = 9000.0f + clockMs % 1000;
    if(distanceRoute) {
        static const int16_t distances[2][ROUTE_COUNT] = {
            {-200,-1050,830,-900,-830,-1720,860,-860,850,880,850,-830,-1720,860,-860,850,880,1700,220,150},
            {-150,1050,825,-900,-830,-1720,860,-860,850,890,850,-830,-1720,860,-860,850,-850,1700,-200,150}
        };
        static const uint8_t axes[ROUTE_COUNT] = {1,0,0,1,1,0,1,1,0,0,1,1,0,1,1,0,0,1,0,1};
        RouteLeg leg=activeDistanceLeg();
        int16_t length=distances[routeStartZone-1][routeIndex];
        if(routeIndex==4 || routeIndex==11) length+=routeRawLeftCorrection;
        assert(leg.upMm==(axes[routeIndex] ? 0 : length));
        assert(leg.rightMm==(axes[routeIndex] ? length : 0));
        /* Corrupt global coordinates on every tick: distance routes must ignore them. */
        routeX=staleX;routeY=staleY;
        if(routeRotating) {
            assert(routeForward==0 && routeRight==0);
            if(routeIndex==2) {
                if(routeNextHeading==0) {
                    assert(routeHeading==-1 && routeLegProgress==0);
                    ++firstRawDepartureTurnTicks;
                } else {
                    assert(routeNextHeading==1 && routeHeading==0);
                    assert(routeLegProgress>=routeAbs(length)-1.5f);
                    ++firstRawTurnTicks;
                }
            }
            if(routeIndex==9) {assert(routeNextHeading==1);++rawReturnTurnTicks;}
            if(routeIndex==7 || routeIndex==14) assert(routeNextHeading==2);
            if(routeIndex==16) assert(routeNextHeading==-1);
        } else {
            int16_t forward,right;
            float travel=routeAbs(routeEstimateScaled(axes[routeIndex] ? routeRight : routeForward,20,
                ((axes[routeIndex] && (routeHeading==0 || routeHeading==2)) ||
                 (!axes[routeIndex] && (routeHeading==1 || routeHeading==-1))) ?
                routeLateralScale : routeForwardScale));
            assert(axes[routeIndex] ? routeForward==0 : routeRight==0);
            assert((axes[routeIndex] ? routeRight : routeForward)*length>=0);
            routeBody(routeForward,routeRight,routeHeading,&forward,&right);
            if(routeIndex==4 || routeIndex==11) {
                if(travel>0) assert(routeHeading==1 && forward>0 && right==0);
                coarseLeftTravel[routeIndex==11]+=travel;
            }
            if(routeIndex==6 || routeIndex==13) {
                if(travel>0) assert(routeHeading==-1 && forward>0 && right==0);
                coarseRightTravel[routeIndex==13]+=travel;
            }
            if(routeIndex==5 || routeIndex==12) {
                if(travel>0) assert(routeHeading==2 && forward>0 && right==0);
                coarseApproachTravel[routeIndex==12]+=travel;
            }
            if(routeIndex==7 || routeIndex==14) {
                if(travel>0) assert(routeHeading==-1);
                temporaryLeftTravel[routeIndex==14]+=travel;
            }
            if(routeIndex==8 || routeIndex==15) {
                assert(routeHeading==2);
                if(travel>0) assert(forward<0 && right==0);
                temporaryApproachTravel[routeIndex==15]+=travel;
            }
            if(routeIndex==2) {
                if(travel>0) assert(routeHeading==0 && forward>0 && right==0);
                firstRawUpTravel+=travel;
            }
            if(routeIndex==3) {assert(routeHeading==1);firstRawApproachTravel+=travel;}
            if(routeIndex==9) {
                if(travel>0) assert(routeHeading==2);
                rawReturnTravel+=travel;
            }
            if(routeIndex==10) {
                assert(routeHeading==1 && forward<=0 && right==0);
                secondRawRightTravel+=travel;
            }
            if(routeIndex==16) {
                if(travel>0) assert(routeHeading==2);
                homeVerticalTravel+=travel;
            }
            if(routeIndex>=17) {
                assert(routeHeading==-1);
                if(travel>0) {
                    if(routeIndex==18) assert(forward==0 && right*length<0);
                    else assert(forward>0 && right==0);
                }
                if(routeIndex==17) homeHorizontalTravel+=travel;
                else if(routeIndex==18) homeTailVerticalTravel+=travel;
                else homeTailRightTravel+=travel;
            }
        }
    }
    clockMs+=20; imuStamp=clockMs; serviceRoute();
    if(distanceRoute) {
        assert(routeX==staleX && routeY==staleY);
        if(routeIndex!=previousIndex) assert(routeLegProgress==0);
        else if(routeWaiting) assert(routeLegProgress==previousProgress);
    }
    if(!navRunning && routeWaiting && (routeIndex==7 || routeIndex==14))
        assert(routeHeading==-1);
    if(missionActive && missionPhase==MISSION_IDLE && mechanismState.running &&
       (int32_t)(clockMs-mechanismState.deadlineMs)>=0) {
        mechanismState.current=mechanismState.target;
        mechanismState.running=0;
    }
    if(missionActive && !routeWaiting &&
       (routeIndex==7 || routeIndex==8 || routeIndex==14 || routeIndex==15)) {
        assert(mechanismState.current.horizontalDmm==(routeIndex>=14 ? -470 : -450));
        assert(mechanismState.current.turretDdeg==1320);
        if(mechanismState.running) {
            assert(mechanismState.target.horizontalDmm==mechanismState.current.horizontalDmm);
            assert(mechanismState.target.liftDmm==0 && mechanismState.target.turretDdeg==1320);
        } else assert(mechanismState.current.liftDmm==0);
    }
    if(missionActive && routeWaiting) {
        if((missionPhase==MISSION_RING_ALIGN || missionPhase==MISSION_TEMP_ALIGN) && missionActionStarted==1 &&
           mechanismState.running) {
            mechanismState.current=mechanismState.target;
            mechanismState.running=0;
        }
        if(missionPhase==MISSION_RAW_PICK && rawPickItemIndex<3 &&
           clockMs%200==0) {
            if(rawPickItemIndex==0)
                assert(rawPickCollectedMask==0 && rawPickCurrentColor==0 && rawPickCurrentSlot==0);
            if((routeIndex==4 || routeIndex==11) && rawPickItemIndex==0) {
                missionOffsetForward=45;missionOffsetRight=-30;
            }
            ++rawPickItemIndex;++missionRawItems;
            mechanismState.current.turretDdeg=rawPickItemIndex==3 ? 650 : 2720;
        }
        if((missionPhase==MISSION_RING_ALIGN || missionPhase==MISSION_TEMP_ALIGN) && missionActionStarted==2) {
            assert(visionSession.waitForTarget);
            assert(visionSession.requestFlags==((missionPhase==MISSION_TEMP_ALIGN && routeIndex==16) ?
                                               VISION_FLAG_RING_NO_DIGIT : 0));
            assert(visionCalibration()==&visionRingCalibration[1]);
            missionOffsetForward=45;missionOffsetRight=-30;
            visionSession.state=VISION_STATE_ALIGNED;
        }
        if(mechanismActionActive) {
            mechanismState.current.horizontalDmm=missionPhase==MISSION_COARSE_PLACE ? missionWorkActions[26].pose.horizontalDmm :
                missionPhase==MISSION_COARSE_PICK ? (routeIndex==14 ? -470 : -450) : 0;
            mechanismState.current.liftDmm=missionPhase==MISSION_COARSE_PLACE ? 1300 :
                missionPhase==MISSION_COARSE_PICK ? 300 : 0;
            mechanismState.current.turretDdeg=missionPhase==MISSION_COARSE_PLACE ? missionWorkActions[26].pose.turretDdeg :
                missionPhase==MISSION_COARSE_PICK ? 1320 : 2720;
            mechanismActionActive=0;mechanismActionIndex=mechanismActionCount;
        }
        if(motionMode==4) motionMode=0;
        serviceMission();
        if(missionPhase==MISSION_COARSE_PLACE || missionPhase==MISSION_TEMP_PLACE)
            assert(missionOffsetForward==0 && missionOffsetRight==0 && motionMode==0);
    }
}
static void checkBodyLateralLimits(void) {
    static const int8_t headings[] = {0, 1, 2, -1};
    unsigned h, axis, slow, n;
    int16_t forward, right, peak;
    uint16_t expected;
    routeLateralRpmLimit=60;
    for(h=0;h<4;++h) for(axis=0;axis<2;++axis) for(slow=0;slow<2;++slow) {
        stopAllMotors();yawSign=1;routeBaseYaw=0;
        routeHeading=routeNextHeading=headings[h];
        imuYaw=routeYaw=headings[h]*90.0f;
        routeX=routeY=1200;routeIndex=0;
        navRunning=navInitialized=routeActive=routeAuto=1;
        navPathCount=1;
        navPath[0].x=axis==0 ? 400 : 1200;
        navPath[0].y=axis==1 ? 400 : 1200;
        navPath[0].arrivalHeading=NAV_HEADING_KEEP;
        routeRpm=slow ? 40 : 200;
        routeTick=routeLegStart=routeSettle=clockMs;
        peak=0;
        expected=slow ? 40 :
            ((headings[h]==0 || headings[h]==2) ?
                (axis==0 ? 60 : 200) : (axis==1 ? 60 : 200));
        for(n=0;n<1000 && routeActive;++n) {
            tick();
            routeBody(routeForward,routeRight,routeHeading,&forward,&right);
            assert(routeAbs(right)<=routeLateralRpmLimit);
            assert(routeAbs(forward)<=routeRpm);
            if(routeAbs(forward)>peak) peak=(int16_t)routeAbs(forward);
            if(routeAbs(right)>peak) peak=(int16_t)routeAbs(right);
        }
        assert(!routeActive && routeX==navPath[0].x && routeY==navPath[0].y);
        if(expected<=60) assert(peak==expected);
        else assert(peak>60);
    }
    stopAllMotors();routeLateralRpmLimit=ROUTE_LATERAL_RPM_MAX;
    puts("PASS: body lateral limit at all four headings, both map axes, low requested RPM preserved");
}
static void checkStationHeadingAlignment(void) {
    unsigned zone,station,n;
    int sign;
    for(zone=1;zone<=2;++zone) for(sign=-1;sign<=1;sign+=2)
        for(station=8;station<=15;station+=7) {
            char command[32];
            stopAllMotors();imuValid=1;imuYaw=zone==1 ? -1.0f : 179.0f;
            imuStamp=clockMs;yawSign=(int8_t)sign;armed=1;
            sprintf(command,"route auto %u 60",zone);
            assert(processRouteCommand(command) && routeActive);
            float baseYaw=routeBaseYaw;
            routeIndex=(uint8_t)station;routeLegProgress=850;
            routeHeading=routeNextHeading=2;
            routeYaw=baseYaw+180.0f*yawSign;
            imuYaw=headingError(routeYaw-3.0f,0.0f);
            assert(missionTaskParse("156+123+516+231",&missionTask));
            missionActive=1;missionPhase=MISSION_IDLE;
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            /* Same direction still needs alignment before station work. */
            assert(routeRotating && !routeWaiting && routeIndex==station);
            assert(missionPhase==MISSION_IDLE && routeBaseYaw==baseYaw);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(commandTurn*yawSign>0 && !routeForward && !routeRight);
            imuYaw=headingError(routeYaw-0.5f,0.0f);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(routeRotating && commandTurn==0);
            /* Reusing one IMU sample for 200ms must not finish alignment. */
            for(n=0;n<10;++n) {clockMs+=20;serviceRoute();}
            assert(routeRotating && !routeWaiting && routeIndex==station);
            /* Leaving the 1deg band restarts correction and stability timing. */
            imuYaw=headingError(routeYaw-1.2f,0.0f);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(routeRotating && commandTurn!=0 && !routeWaiting);
            imuYaw=headingError(routeYaw-0.5f,0.0f);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            for(n=0;n<9;++n) {clockMs+=20;imuStamp=clockMs;serviceRoute();}
            assert(routeRotating && !routeWaiting);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(!routeRotating && routeIndex==station);
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(routeWaiting && routeIndex==station+1 &&
                   missionPhase==MISSION_TEMP_ALIGN && !missionActionStarted);
            assert(routeBaseYaw==baseYaw &&
                   routeAbs(headingError(routeYaw,imuYaw))<=1.0f);
        }
    stopAllMotors();yawSign=1;imuYaw=0;imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route auto 1 60") && routeActive);
    routeIndex=8;routeLegProgress=850;routeHeading=routeNextHeading=2;
    routeYaw=180;imuYaw=177;
    clockMs+=20;imuStamp=clockMs;serviceRoute();assert(routeRotating);
    clockMs+=20;imuStamp=clockMs-251;serviceRoute();
    assert(!routeActive && !routeWaiting && !routeStationAlignment);
    /* HOME keeps the previous target heading rather than treating KEEP as 360deg. */
    imuYaw=0;imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route auto 1 60") && routeActive);
    routeIndex=19;routeLegProgress=150;routeHeading=routeNextHeading=-1;
    routeYaw=imuYaw=-90;
    clockMs+=20;imuStamp=clockMs;serviceRoute();
    assert(routeRotating && routeNextHeading==-1 && routeYaw==-90);
    for(n=0;n<15 && routeActive;++n) {clockMs+=20;imuStamp=clockMs;serviceRoute();}
    assert(!routeActive && routeYaw==-90);
    stopAllMotors();
    puts("PASS: both temporary stations align before vision, fresh samples and 1deg stability required");
}
static void checkRecoveryDeparture(void) {
    for(unsigned batch=0;batch<2;++batch) {
        int16_t trayH=batch ? -470 : -450;
        stopAllMotors();
        mechanismStateInitialize(&mechanismState,&missionCoarseInitial.pose);
        mechanismState.current.horizontalDmm=trayH;
        mechanismState.current.liftDmm=300;
        mechanismState.current.turretDdeg=1320;
        missionActive=routeActive=routeWaiting=1;
        missionPhase=MISSION_COARSE_PICK;missionActionStarted=1;
        routeIndex=batch ? 14 : 7;
        routeAuto=fullRouteRunning=1;routeStartZone=1;
        routeHeading=routeNextHeading=-1;routeBaseYaw=0;
        routeYaw=imuYaw=-90;imuStamp=clockMs;
        routeLegProgress=0;routeRpm=40;
        mechanismActionCount=MISSION_PICK_BACK_COUNT;
        mechanismActionIndex=mechanismActionCount-1;
        mechanismActionActive=1;
        /* The last open command must finish before either chassis or lift starts. */
        serviceMission();
        assert(routeWaiting && !mechanismState.running && missionPhase==MISSION_COARSE_PICK);
        mechanismActionIndex=mechanismActionCount;mechanismActionActive=0;
        serviceMission();
        assert(!routeWaiting && missionPhase==MISSION_IDLE && mechanismState.running);
        assert(mechanismState.current.horizontalDmm==trayH && mechanismState.current.liftDmm==300);
        assert(mechanismState.target.horizontalDmm==trayH && mechanismState.target.liftDmm==0 &&
               mechanismState.target.turretDdeg==1320);
        for(unsigned tick=0;tick<4;++tick) {
            clockMs+=20;imuStamp=clockMs;serviceRoute();
            assert(routeActive && !routeWaiting && mechanismState.running);
        }
        assert(routeForward!=0 || routeRight!=0);
        /* Arrival waits for a slow background lift, then unfolds from the retained pose. */
        routeIndex=batch ? 16 : 9;routeWaiting=1;
        missionPhase=MISSION_TEMP_ALIGN;missionActionStarted=0;
        serviceMission();
        assert(missionActive && !missionActionStarted && mechanismState.target.liftDmm==0);
        mechanismState.current=mechanismState.target;mechanismState.running=0;
        serviceMission();
        assert(missionActive && missionActionStarted==1 && mechanismState.running);
        assert(mechanismState.target.horizontalDmm==-770 && mechanismState.target.liftDmm==0 &&
               mechanismState.target.turretDdeg==2720);
        mechanismState.current=mechanismState.target;mechanismState.running=0;
        stopAllMotors();
    }
    imuYaw=0;imuStamp=clockMs;
    puts("PASS: both recoveries depart after final release while lifting only, hold tray/132deg, unfold at storage");
}
int main(void) {
    assert(processTaskCommand("task 156+123+516+231") && missionTaskValid);
    MissionTask savedTask=missionTask;
    assert(processTaskCommand("task 156+113+516+231"));
    assert(memcmp(&savedTask,&missionTask,sizeof(missionTask))==0);
    routeActive=1;
    assert(processTaskCommand("task 426+231+426+123"));
    assert(memcmp(&savedTask,&missionTask,sizeof(missionTask))==0);
    routeActive=0;
    /* A second batch without matching first-layer colors cannot start stacking. */
    assert(missionTaskParse("123+321+456+312",&missionTask));
    routeActive=missionActive=routeWaiting=1;routeIndex=16;
    missionBeginStation(16);
    assert(!routeActive && !missionActive && !mechanismActionActive && !mechanismState.running);
    missionTask=savedTask;missionTaskValid=1;

    static const unsigned targetNode[] = {
        NAV_START_1,NAV_GATE_1,NAV_UPPER_RIGHT,NAV_QR,NAV_START_2,
        NAV_GATE_2,NAV_RAW,NAV_CENTER,NAV_COARSE,NAV_TEMP
    };
    unsigned n,start,target; int sign;
    int16_t previous,peak;
    char cmd[32];
    assert(processRouteCommand("pid get"));
    assert(processRouteCommand("pid set 235 40 18") &&
           routeAbs(routeHeadingGains.kp-2.35f)<0.001f &&
           routeAbs(routeHeadingGains.ki-0.40f)<0.001f &&
           routeAbs(routeHeadingGains.kd-0.18f)<0.001f);
    routeActive=1;
    assert(processRouteCommand("pid set 300 50 20") &&
           routeAbs(routeHeadingGains.kp-2.35f)<0.001f);
    routeActive=0;
    assert(processRouteCommand("pid set 1000 500 500") &&
           routeAbs(routeHeadingGains.kp-10.0f)<0.001f &&
           routeAbs(routeHeadingGains.ki-5.0f)<0.001f &&
           routeAbs(routeHeadingGains.kd-5.0f)<0.001f);
    assert(processRouteCommand("pid set 1001 25 12") &&
           routeAbs(routeHeadingGains.kp-10.0f)<0.001f);
    assert(processRouteCommand("pid set 1000 501 12") &&
           routeAbs(routeHeadingGains.ki-5.0f)<0.001f);
    assert(processRouteCommand("pid set 1000 25 501") &&
           routeAbs(routeHeadingGains.kd-5.0f)<0.001f);
    assert(processRouteCommand("pid set 0 0 0") &&
           routeHeadingGains.kp==0 && routeHeadingGains.ki==0 &&
           routeHeadingGains.kd==0);
    assert(processRouteCommand("pid set 200 25 12") &&
           routeAbs(routeHeadingGains.kp-ROUTE_HEADING_KP)<0.001f);
    assert(processRouteCommand("route scale 9250") &&
           routeAbs(routeLateralScale-0.925f)<0.0001f);
    routeActive=1;assert(processRouteCommand("route scale 8000") &&
                         routeAbs(routeLateralScale-0.925f)<0.0001f);
    routeActive=0;assert(processRouteCommand("route scale 4999") &&
                         routeAbs(routeLateralScale-0.925f)<0.0001f);
    assert(processRouteCommand("route tune 10200 9000 45") &&
           routeAbs(routeForwardScale-1.02f)<0.0001f &&
           routeAbs(routeLateralScale-0.9f)<0.0001f &&
           routeTurnRpmLimit==45 && routeLateralRpmLimit==ROUTE_LATERAL_RPM_MAX);
    assert(processRouteCommand("route tune 10200 9000 50 80") &&
           routeTurnRpmLimit==50 && routeLateralRpmLimit==80);
    assert(processRouteCommand("route tune 10200 9000 50 251") &&
           routeLateralRpmLimit==80);
    assert(processRouteCommand("route tune 10200 9000 45 60") &&
           routeTurnRpmLimit==45 && routeLateralRpmLimit==60);
    routeActive=1;assert(processRouteCommand("route tune 9000 9000 60") &&
                         routeAbs(routeForwardScale-1.02f)<0.0001f &&
                         routeTurnRpmLimit==45 && routeLateralRpmLimit==60);
    routeActive=0;
    assert(processRouteCommand("route tune 10200 9000 250 250") &&
           routeTurnRpmLimit==250 && routeLateralRpmLimit==250);
    assert(processRouteCommand("route tune 10200 9000 230 150") &&
           routeTurnRpmLimit==230 && routeLateralRpmLimit==150);
    checkBodyLateralLimits();batches=writes=0;
    sendRouteSpeeds(20,0,0);assert(batches==1 && writes==4);
    sendRouteSpeeds(20,0,0);assert(batches==1 && writes==4);
    sendRouteSpeeds(0,20,0);assert(batches==2 && writes==8);
    assert(staged[0]==-20 && staged[1]==20 &&
           staged[2]==-20 && staged[3]==20);
    sendRouteSpeeds(0,0,0);assert(batches==3 && writes==12);
    sendRouteSpeeds(0,0,0);assert(batches==3 && writes==12);
    for(sign=-1;sign<=1;sign+=2) for(start=1;start<=2;++start) {
        stopAllMotors(); yawSign=(int8_t)sign; imuYaw=179; turnTicks=0; armed=1;
        routePosReports=routeStageReports=routeDoneReports=0;
        sprintf(cmd,"route auto %u",start);
        assert(processRouteCommand(cmd) && routeActive && routeAuto);
        assert(routeRpm==ROUTE_RPM);
        for(n=0;n<30000 && routeActive;++n) {tick();assert(!routeWaiting);}
        assert(!routeActive && routeIndex==ROUTE_COUNT-1 && routeHeading==-1);
        assert(routeHeading==-1);
        assert(turnTicks>100);
        assert(routePosReports>0 && routeStageReports==8 && routeDoneReports==1);
    }
    for(start=1;start<=2;++start) {
        unsigned qrStarted=0;
        temporaryApproachTravel[0]=temporaryApproachTravel[1]=0;
        temporaryLeftTravel[0]=temporaryLeftTravel[1]=0;
        coarseApproachTravel[0]=coarseApproachTravel[1]=0;
        rawReturnTravel=secondRawRightTravel=0;
        coarseLeftTravel[0]=coarseLeftTravel[1]=coarseRightTravel[0]=coarseRightTravel[1]=0;
        rawReturnTurnTicks=0;
        firstRawApproachTravel=firstRawUpTravel=0;
        firstRawTurnTicks=firstRawDepartureTurnTicks=0;
        homeVerticalTravel=homeHorizontalTravel=homeTailVerticalTravel=homeTailRightTravel=0;
        missionPlaceRuns=missionPickRuns=missionStorageRuns=0;
        missionRingAligns=missionMaterialAligns=missionRawItems=missionRawExitMoves=0;
        stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
        mechanismState.valid=1;
        mechanismState.current.horizontalDmm=0;
        mechanismState.current.liftDmm=0;
        mechanismState.current.turretDdeg=start==1 ? 650 : 2720;
        missionTaskValid=0;
        missionGrabCount=7;missionPlaceCount=8;
        visionRingCalibration[1].calibrated=0;
        assert(processRouteCommand("route mission 1") && !routeActive);
        assert(missionGrabCount==7 && missionPlaceCount==8);
        visionRingCalibration[1].calibrated=1;
        currentAngleMdeg[0]=45000;
        assert(processRouteCommand("route mission 1") && !routeActive);
        assert(missionGrabCount==7 && missionPlaceCount==8);
        currentAngleMdeg[0]=start==1 ? 35000 : 90000;
        sprintf(cmd,"route mission %u 40",start);
        assert(processRouteCommand(cmd) && routeActive && missionActive);
        assert(missionGrabCount==0 && missionPlaceCount==0 && countUpdates>0);
        missionGrabCount=4;missionPlaceCount=5;
        for(n=0;n<30000 && routeActive;++n) {
            if(start==1 && routeIndex<4)
                assert(mechanismState.current.turretDdeg==650);
            tick();
            if(missionPhase==MISSION_QR_WAIT && !qrStarted) qrStarted=clockMs;
            if(missionPhase==MISSION_QR_WAIT && !missionTaskValid)
                receiveQr(start==1 ? "156+123+516+231" : "123+321+123+312");
            if(qrStarted && clockMs-qrStarted<980U)
                assert(routeWaiting && routeIndex==2);
        }
        assert(!routeActive && routeDoneReports>0 &&
               routeIndex==ROUTE_COUNT-1 && routeHeading==-1);
        assert(missionGrabCount==4 && missionPlaceCount==5);
        assert(qrStarted && missionPlaceRuns==2 && missionPickRuns==2 &&
               missionStorageRuns==2 && missionRingAligns==4 && missionMaterialAligns==0 &&
               missionRawItems==6 && missionRawExitMoves==2);
        assert(temporaryLeftTravel[0]>858 && temporaryLeftTravel[0]<865);
        assert(temporaryLeftTravel[1]>858 && temporaryLeftTravel[1]<865);
        assert(temporaryApproachTravel[0]>848 && temporaryApproachTravel[0]<855);
        assert(temporaryApproachTravel[1]>848 && temporaryApproachTravel[1]<855);
        assert(coarseLeftTravel[0]>783 && coarseLeftTravel[0]<790);
        assert(coarseLeftTravel[1]>783 && coarseLeftTravel[1]<790);
        assert(coarseRightTravel[0]>858 && coarseRightTravel[0]<865);
        assert(coarseRightTravel[1]>858 && coarseRightTravel[1]<865);
        assert(coarseApproachTravel[0]>1718 && coarseApproachTravel[0]<1725);
        assert(coarseApproachTravel[1]>1718 && coarseApproachTravel[1]<1725);
        assert(rawReturnTravel>(start==1 ? 878 : 888) &&
               rawReturnTravel<(start==1 ? 885 : 895) && rawReturnTurnTicks>0);
        assert(secondRawRightTravel>848 && secondRawRightTravel<855);
        assert(firstRawApproachTravel>898 && firstRawApproachTravel<905);
        assert(firstRawUpTravel>(start==1 ? 828 : 823) &&
               firstRawUpTravel<(start==1 ? 835 : 830));
        assert(firstRawTurnTicks>0 && firstRawDepartureTurnTicks>0);
        assert(homeVerticalTravel>(start==1 ? 878 : 848) &&
               homeVerticalTravel<(start==1 ? 885 : 855));
        assert(homeHorizontalTravel>1698 && homeHorizontalTravel<1705);
        assert(homeTailVerticalTravel>(start==1 ? 218 : 198) &&
               homeTailVerticalTravel<(start==1 ? 225 : 205));
        assert(homeTailRightTravel>148 && homeTailRightTravel<155);
    }
    missionBuildStoragePlace(missionWorkActions,2);
    assert(missionWorkActions[8].pose.liftDmm==650 &&
           missionWorkActions[16].pose.liftDmm==650 &&
           missionWorkActions[26].pose.liftDmm==650);
    stopAllMotors();imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route mission 1 40") && missionActive);
    routeWaiting=1;routeIndex=11;missionBeginStation(11);
    visionSession.state=VISION_STATE_PICK;visionPickActive=1;
    mechanismActionActive=1;
    clockMs+=60000;imuStamp=clockMs;
    serviceMission();
    assert(missionActive && routeActive && mechanismActionActive);
    clockMs+=10000;imuStamp=clockMs;serviceMission();
    assert(missionActive && routeActive); /* slow pickup is not an absent material */
    rawPickItemIndex=1;visionPickActive=mechanismActionActive=0;
    visionSession.state=VISION_STATE_PICK_DONE;serviceMission();
    assert(missionActive && routeActive);
    visionSession.state=VISION_STATE_WAIT;
    clockMs+=60000;imuStamp=clockMs;serviceMission();
    assert(missionActive && routeActive); /* Missing material has no time limit. */
    rawPickItemIndex=3;visionSession.state=VISION_STATE_PICK_DONE;
    serviceMission();assert(missionPhase==MISSION_RECENTER);
    stopAllMotors();imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route mission 1 40") && missionActive);
    for(start=1;start<=2;++start) {
        routeWaiting=1;routeIndex=start==1 ? 4 : 11;
        routeHeading=1;
        missionPhase=MISSION_RAW_PICK;rawPickItemIndex=3;
        visionPickActive=mechanismActionActive=0;
        missionOffsetForward=45;missionOffsetRight=-30;
        mechanismState.current.horizontalDmm=0;
        mechanismState.current.liftDmm=0;
        mechanismState.current.turretDdeg=650;
        mechanismState.running=1;
        serviceMission();assert(missionPhase==MISSION_RECENTER &&
                                missionOffsetForward==45 && missionOffsetRight==-30);
        serviceMission();assert(routeWaiting && motionMode==0);
        mechanismState.running=0;
        n=missionRawExitMoves;
        serviceMission();
        assert(routeWaiting && motionMode==4 && missionPhase==MISSION_RAW_EXIT);
        assert(lastVisionForward==0 && lastVisionRight==30);
        assert(missionOffsetForward==45 && missionOffsetRight==0);
        assert(activeDistanceLeg().rightMm==-785);
        serviceMission();assert(routeWaiting && missionRawExitMoves==n+1);
        motionMode=0;serviceMission();
        assert(!routeWaiting && missionActive && motionMode==0);
        assert(missionRawExitMoves==n+1);
        assert(missionOffsetForward==0 && missionOffsetRight==0);
    }
    /* Both pickups restore map Y and merge signed map X compensation into left travel. */
    {
        static const float horizontalOffsets[]={-70,0,45};
        static const float verticalOffsets[]={-30,0,65};
        unsigned x,y,zone,round,base;
        for(zone=1;zone<=2;++zone) for(round=0;round<2;++round) for(x=0;x<3;++x) for(y=0;y<3;++y) {
            stopAllMotors();armed=1;imuStamp=clockMs;
            mechanismState.valid=1;mechanismState.running=0;mechanismActionActive=0;
            mechanismState.current.horizontalDmm=0;mechanismState.current.liftDmm=0;
            mechanismState.current.turretDdeg=650;
            sprintf(cmd,"route mission %u 40",zone);
            assert(processRouteCommand(cmd) && routeRawLeftCorrection==0);
            base=round ? 11 : 4;
            routeWaiting=1;routeIndex=(uint8_t)base;routeHeading=1;
            missionPhase=MISSION_RECENTER;
            mechanismState.valid=1;mechanismState.running=0;
            mechanismState.current.horizontalDmm=0;
            mechanismState.current.liftDmm=0;
            mechanismState.current.turretDdeg=650;
            missionOffsetForward=horizontalOffsets[x];
            missionOffsetRight=verticalOffsets[y];
            n=missionRawExitMoves;
            serviceMission();
            assert(routeRawLeftCorrection==horizontalOffsets[x]);
            assert(activeDistanceLeg().rightMm==-830+horizontalOffsets[x]);
            if(verticalOffsets[y]!=0) {
                assert(routeWaiting && motionMode==4 && missionPhase==MISSION_RAW_EXIT);
                assert(lastVisionForward==0 && lastVisionRight==-verticalOffsets[y]);
                assert(missionOffsetForward==horizontalOffsets[x] && missionOffsetRight==0);
                motionMode=0;serviceMission();
            }
            assert(!routeWaiting && missionActive && motionMode==0);
            assert(missionRawExitMoves==n+(verticalOffsets[y]!=0));
            coarseLeftTravel[0]=coarseLeftTravel[1]=coarseRightTravel[0]=coarseRightTravel[1]=0;
            routeYaw=routeBaseYaw+90*yawSign;imuYaw=headingError(routeYaw,0);
            for(n=0;n<10000 && routeIndex==base;++n) tick();
            assert(routeIndex==base+1 && routeHeading==2 && !routeWaiting);
            assert(coarseLeftTravel[round]>828-horizontalOffsets[x] &&
                   coarseLeftTravel[round]<835-horizontalOffsets[x]);
            for(n=0;n<10000 && routeIndex==base+1;++n) tick();
            assert(routeIndex==base+2 && routeHeading==-1 && !routeWaiting);
            for(n=0;n<10000 && routeIndex==base+2;++n) tick();
            assert(routeIndex==base+3 && routeHeading==-1 && routeWaiting);
            assert(missionPhase==MISSION_RING_ALIGN && missionPlaceRuns==2);
            mechanismState.running=0;
            assert(coarseRightTravel[round]>858 && coarseRightTravel[round]<865);
        }
    }
    stopAllMotors();armed=1;imuStamp=clockMs;
    assert(processRouteCommand("route mission 1 40") && missionActive);
    routeWaiting=1;missionPhase=MISSION_COARSE_PLACE;
    missionActionStarted=1;mechanismActionActive=0;
    mechanismActionCount=MISSION_PLACE_COUNT;mechanismActionIndex=0;
    n=routeInvalidReports;serviceMission();
    assert(!missionActive && !routeActive && routeInvalidReports==n+1);
    stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
    mechanismState.valid=1;
    mechanismState.current.turretDdeg=2720;
    visionMaterialCalibration.calibrated=0;
    assert(processRouteCommand("route rawpick 1") && !routeActive && armed);
    visionMaterialCalibration.calibrated=1;
    assert(processRouteCommand("route rawpick 1") && rawPickRouteActive);
    for(n=0;n<10000 && !routeWaiting;++n) tick();
    assert(routeWaiting && rawPickAtStation && routeIndex==4 &&
           routeLegProgress==0 && routeHeading==1);
    n=batches;
    for(start=0;start<150;++start) tick();
    assert(routeWaiting && batches==n && routeLegProgress==0);
    assert(processRouteCommand("route next") && routeWaiting);
    rawPickItemIndex=3;
    assert(processRouteCommand("route next") && !routeWaiting && !rawPickRouteActive);
    stopAllMotors();
    stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route step 1"));
    for(n=0;n<25;++n) tick();
    for(n=0;n<10;++n) {
        imuYaw=routeYaw+8;clockMs+=20;imuStamp=clockMs;serviceRoute();
    }
    assert(routeCorrection<0);
    for(n=0;n<7;++n) {
        imuYaw=routeYaw-8;clockMs+=20;imuStamp=clockMs;serviceRoute();
    }
    assert(routeCorrection>0); /* reverse promptly when yaw crosses target */
    stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
    assert(!routeHeadingPid.initialized && routeHeadingPid.integral==0);
    assert(processRouteCommand("route step 1"));
    peak=0;
    for(n=0;n<1000 && !routeWaiting;++n) {
        tick();
        assert(routeAbs(routeRight)<=routeLateralRpmLimit);
        if(routeAbs(routeRight)>peak) peak=(int16_t)routeAbs(routeRight);
    }
    assert(routeWaiting && routeIndex==1 && peak>0);
    assert(processRouteCommand("route next") && !routeWaiting);
    stopAllMotors(); tick(); assert(!routeActive); /* cancel cannot resume */
    assert(processRouteCommand("route next") && !routeActive);
    armed=1; processRouteCommand("turn L 90");
    for(n=0;n<1000 && routeActive;++n) tick();
    assert(!routeActive && routeAbs(headingError(routeYaw,imuYaw))<=2);
    armed=1; processRouteCommand("turn R 180");
    for(n=0;n<1000 && routeActive;++n) tick();
    assert(!routeActive && routeAbs(headingError(routeYaw,imuYaw))<=2);
    armed=1; processRouteCommand("turn R 90");
    for(n=0;n<1000 && routeActive;++n) {
        clockMs+=20;imuStamp=clockMs;serviceRoute(); /* stuck motor */
    }
    assert(!routeActive && routeAbs(headingError(routeYaw,imuYaw))>2);
    armed=1;processRouteCommand("route auto 1");
    clockMs+=300;serviceRoute();assert(!routeActive); /* stale IMU */
    pidTraceReports=0;
    for(sign=-1;sign<=1;sign+=2) for(start=1;start<=2;++start)
        for(target=0;target<sizeof(targetNode)/sizeof(targetNode[0]);++target) {
            NavPoint destination=navPoint((uint8_t)targetNode[target]);
            stopAllMotors();imuYaw=179;imuStamp=clockMs;imuValid=1;yawSign=(int8_t)sign;
            sprintf(cmd,"nav init %u",start);
            assert(processNavCommand(cmd) && navInitialized &&
                   navCurrentNode==navStartNode((uint8_t)start));
            armed=1;navPosReports=navDoneReports=0;
            sprintf(cmd,"nav goto %d %d 250",destination.x,destination.y);
            assert(processNavCommand(cmd) && navRunning);
            assert(routeRpm==250);
            for(n=0;n<12000 && navRunning;++n) tick();
            assert(navInitialized && !navRunning && navCurrentNode==targetNode[target]);
            assert(routeX==destination.x && routeY==destination.y);
            assert(routeHeading==(destination.arrivalHeading==NAV_HEADING_KEEP ?
                                  NAV_HEADING_UP : destination.arrivalHeading));
            assert(navPosReports>0 && navDoneReports==1);
            armed=1;assert(processNavCommand(cmd) && navRunning); /* repeat target */
            for(n=0;n<100 && navRunning;++n) tick();
            assert(navInitialized && !navRunning && navDoneReports==2);
        }
    assert(pidTraceReports>0); /* translating navigation emits heading samples */
    stopAllMotors();imuYaw=0;imuStamp=clockMs;imuValid=1;yawSign=1;
    assert(processNavCommand("nav init 1") && navInitialized);
    armed=1;assert(processNavCommand("nav goto 400 1200") && navRunning);
    armed=1;assert(processNavCommand("nav goto 1200 1200") && navRunning); /* busy reject */
    navInvalidReports=0;clockMs+=20;imuStamp=clockMs-300;serviceRoute();
    assert(!navInitialized && !navRunning && navInvalidReports==1);
    assert(processNavCommand("nav goto 1200 1200") && !navRunning);
    stopAllMotors();imuYaw=0;imuStamp=clockMs;imuValid=1;
    assert(processNavCommand("nav init 1") && navInitialized);
    armed=1;assert(processNavCommand("nav goto 2100 2250") && navRunning);
    imuYaw=routeYaw+21;clockMs+=20;imuStamp=clockMs;serviceRoute();
    assert(!navInitialized && !navRunning && navInvalidReports==2);
    stopAllMotors();imuStamp=clockMs;imuValid=1;
    assert(processNavCommand("nav init 1") && navInitialized);
    armed=1;assert(processNavCommand("nav goto 300 300 120") && navRunning);
    for(n=0;n<12000 && navRunning;++n) tick();
    assert(navInitialized && !navRunning && routeX==300 && routeY==300 &&
           navCurrentNode==NAV_INVALID_NODE);
    armed=1;assert(processNavCommand("nav goto 1800 300 120") && navRunning);
    for(n=0;n<12000 && navRunning;++n) tick();
    assert(navInitialized && !navRunning && routeX==1800 && routeY==300);
    armed=1;imuStamp=clockMs;assert(processRouteCommand("route auto 1 250") && routeRpm==250);
    stopAllMotors();armed=1;imuStamp=clockMs;
    assert(processRouteCommand("route auto 1 251") && !routeActive);
    for(sign=-1;sign<=1;sign+=2) {
        stopAllMotors(); straightLateral=1;straightDirection=(int16_t)sign;
        straightRpm=60;motionDuration=2000;motionStart=lastControl=clockMs;
        targetYaw=imuYaw;motionMode=2;previous=0;peak=0;
        for(n=0;n<110 && motionMode;++n) {
            clockMs+=20;imuStamp=clockMs;serviceMotion();
            if(motionMode) {
                assert(staged[0]==staged[2] && staged[1]==staged[3]);
                assert(staged[0]==-staged[1]);
                assert(routeAbs(staged[1]-previous)<=3); /* <=120RPM/s plus rounding */
                previous=staged[1];if(routeAbs(previous)>peak)peak=(int16_t)routeAbs(previous);
            }
        }
        assert(!motionMode && peak==60);
    }
    stopAllMotors();straightUseRoutePid=straightLateral=1;straightDirection=-1;
    straightRpm=20;motionDuration=2000;motionStart=clockMs;lastControl=clockMs-20;
    targetYaw=0;imuYaw=3;motionMode=2;pidTraceReports=0;
    for(n=0;n<50;++n) {clockMs+=20;imuStamp=clockMs;serviceMotion();}
    assert(motionMode==2 && routeHeadingPid.initialized &&
           commandTurn<0 && pidTraceReports>0);
    stopAllMotors();assert(!straightUseRoutePid && !routeHeadingPid.initialized);
    n=statusReads;motionMode=3;auxMoveMotorId=5;motionStart=lastControl=clockMs;motionDuration=1000;
    clockMs+=50;serviceMotion();assert(motionMode==3 && statusReads==n+1);
    can.CAN_RxMsg.ExtId=5U<<8;can.CAN_RxMsg.DLC=3;
    can.CAN_RxMsg.Data[0]=0x3A;can.CAN_RxMsg.Data[1]=0x02;can.CAN_RxMsg.Data[2]=0x6B;
    can.rxFrameFlag=true;serviceMotion();assert(motionMode==0);
    /* No repeated stop/restart as measurement oscillates around entry boundary. */
    armed=1;processRouteCommand("turn L 90");
    imuYaw=routeYaw-1.9f;clockMs+=20;imuStamp=clockMs;serviceRoute();
    unsigned previousBatches=batches;
    for(n=0;n<12 && routeActive;++n) {
        imuYaw=routeYaw-(n%2?1.9f:2.1f);clockMs+=20;imuStamp=clockMs;serviceRoute();
    }
    assert(!routeActive && batches==previousBatches);
    for(sign=-1;sign<=1;sign+=2) for(start=0;start<2;++start) {
        float physicalYaw=179.0f, physicalTurn=0.0f, overshoot=0.0f;
        stopAllMotors();imuYaw=physicalYaw;imuStamp=clockMs;yawSign=(int8_t)sign;armed=1;
        assert(processRouteCommand(start ? "turn L 90" : "turn R 90"));
        for(n=0;n<1200 && routeActive;++n) {
            float progress;
            physicalTurn+=(commandTurn-physicalTurn)*20.0f/(180.0f+20.0f);
            physicalYaw+=physicalTurn*1.3955f*0.020f*yawSign;
            clockMs+=20;
            if(n%3==0) {imuYaw=physicalYaw;imuStamp=clockMs;}
            progress=(physicalYaw-179.0f)*(start?1.0f:-1.0f)*yawSign;
            if(progress-90.0f>overshoot) overshoot=progress-90.0f;
            serviceRoute();
        }
        assert(!routeActive && overshoot<3.5f);
        assert(routeAbs(headingError(routeYaw,physicalYaw))<3.5f);
    }
    for(start=0;start<4;++start) for(sign=0;sign<2;++sign) {
        static const char ways[]="WSAD";
        unsigned before=moveDoneReports;
        stopAllMotors();armed=1;imuYaw=sign ? 37.0f : -145.0f;imuStamp=clockMs;
        routeLateralScale=0.9f;routeForwardScale=1.02f;routeLateralRpmLimit=60;
        sprintf(cmd,"move %c 500 200",ways[start]);
        mechanismState.running=mechanismActionActive=0;
        mechanismState.valid=1;
        target=stopCalls;
        assert(processRouteCommand(cmd) && motionMode==5 && !armed);
        assert(straightRpm==(start>=2 ? 60 : 200));
        assert(targetYaw==imuYaw);
        for(n=0;n<10000 && motionMode;++n) {
            clockMs+=20;imuStamp=clockMs;serviceMotion();
            assert(routeAbs(straightLateral ? routeRight : routeForward)<=straightRpm);
            if(motionMode) assert(start>=2 ? routeForward==0 : routeRight==0);
        }
        assert(!motionMode && moveDoneReports==before+1 && mechanismState.valid);
        assert(stopCalls==target); /* Successful movement must not invalidate the mechanism. */
        assert(relativeMoveProgress>=499.5f && relativeMoveProgress<510);
    }
    stopAllMotors();armed=1;imuStamp=clockMs;
    assert(processRouteCommand("move W 0 30") && !motionMode);
    assert(processRouteCommand("move W 2401 30") && !motionMode);
    mechanismState.running=1;
    assert(processRouteCommand("move W 100 30") && !motionMode);
    mechanismState.running=0;
    assert(processRouteCommand("move W 1 30") && motionMode==5);
    for(n=0;n<500 && motionMode;++n) {clockMs+=20;imuStamp=clockMs;serviceMotion();}
    assert(!motionMode);
    armed=1;imuStamp=clockMs;mechanismState.valid=1;navInitialized=1;
    target=stopCalls;processRouteCommand("turn L 90");
    for(n=0;n<2000 && routeActive;++n) tick();
    assert(!routeActive && stopCalls==target && mechanismState.valid && !navInitialized);
    armed=1;imuStamp=clockMs;processRouteCommand("move D 2400 30");
    assert(motionMode==5);
    clockMs+=300;serviceMotion();assert(!motionMode && !armed);
    checkStationHeadingAlignment();
    checkMissionButton();
    checkOfflineMission();
    checkRecoveryDeparture();
    puts("PASS: routes, relative moves, optimized motion and navigation runtime checks");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_route_runtime.c'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $displayReset + "`n" + $buttonDefines + "`n" + $buttonInit + "`n" + $parser + "`n" + $pair + "`n" + $service + "`n" + $calibration + "`n" + $mission + "`n" + $motion + "`n" + $qrService + "`n" + $watchdog + "`n" + $taskParser + "`n" + $suffix)
& 'E:\Qt\Tools\mingw1310_64\bin\gcc.exe' -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o (Join-Path $root 'Objects/test_route_runtime.exe')
$compileExit = $LASTEXITCODE
if ($compileExit -ne 0) {throw "Runtime test compile failed: exit $compileExit"}
& (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test failed'}
