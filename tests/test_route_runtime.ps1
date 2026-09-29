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
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../route_plan.h"
#include "../map_navigation.h"
#include "../straight_control.h"
#include "../mission_actions.h"
static uint8_t routeActive,routeWaiting,routeIndex,routeStartZone,routeStep;
static uint8_t routeAuto,routeRotating,routeOnlyTurn,routeTurnInBand,fullRouteRunning;
static uint8_t rawPickRouteActive,rawPickAtStation,rawPickItemIndex;
static uint8_t rawPickMissingReported;
typedef enum {
    MISSION_IDLE, MISSION_QR_WAIT, MISSION_RAW_PICK,
    MISSION_RING_ALIGN, MISSION_TEMP_ALIGN, MISSION_COARSE_PLACE, MISSION_COARSE_PICK,
    MISSION_TEMP_PLACE, MISSION_RECENTER
} MissionPhase;
static uint8_t missionActive,missionActionStarted,missionObservedItem;
static MissionPhase missionPhase;
static uint32_t missionDeadline;
static float missionOffsetForward,missionOffsetRight;
static MechanismAction missionWorkActions[MISSION_PLACE_COUNT];
static MechanismAction missionPickBackActions[MISSION_PICK_BACK_COUNT];
static const MechanismPose missionRingObservePose={-500,0,2700,30,50,90,50,1200};
static uint16_t mechanismActionIndex,mechanismActionCount;
static unsigned missionPlaceRuns,missionPickRuns,missionStorageRuns,missionRingAligns;
static unsigned missionRawItems;
static uint8_t visionPickActive;
static uint8_t motionMode,armed,imuValid=1,emergencyStop,jogCanFault;
static MechanismState mechanismState;
static uint8_t mechanismActionActive;
static uint8_t servoChannelMoving[3];
static uint8_t servoChannelEnabled[3]={1,1,1};
static uint32_t currentAngleMdeg[3]={70000,26000,270000};
static uint8_t auxMoveMotorId;
static int8_t routeHeading,routeNextHeading,yawSign=1;
static float routeX,routeY,routeYaw,routeBaseYaw,imuYaw;
static float routeDriveRpm,routeTurnRpm,routeCorrection;
static uint32_t headingPidLastReport;
static RouteHeadingPid routeHeadingPid;
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
enum {VISION_STATE_IDLE, VISION_STATE_REQUEST, VISION_STATE_ALIGNED};
#define VISION_MODE_RING 2U
typedef struct {uint8_t state;} VisionSession;
static VisionSession visionSession;
typedef struct {uint8_t calibrated;} VisionCalibration;
static VisionCalibration visionMaterialCalibration={1};
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
        assert(mechanismState.current.horizontalDmm==-500);
    }
    if(mode==VISION_MODE_RING) ++missionRingAligns;
    session->state=VISION_STATE_REQUEST;
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
    if(strcmp(s,"NAV POS x=")==0) ++navPosReports;
    if(strcmp(s,"PID TRACE target_cdeg=")==0) ++pidTraceReports;
    if(strcmp(s,"NAV DONE x=")==0) ++navDoneReports;
    if(strncmp(s,"NAV INVALID",11)==0) ++navInvalidReports;
    if(strcmp(s,"ROUTE POS x=")==0) ++routePosReports;
    if(strcmp(s,"ROUTE STAGE index=")==0) ++routeStageReports;
    if(strcmp(s,"ROUTE DONE x=")==0) ++routeDoneReports;
    if(strncmp(s,"ROUTE INVALID",13)==0) ++routeInvalidReports;
}
static void serialSendChar(char c) {(void)c;}
static void serialSendUint(uint16_t x) {(void)x;}
static void serialSendInt(int16_t x) {(void)x;}
static void printRouteStatus(void) {}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAllMotorsEnabled(bool x) {(void)x;}
static void stopDriveMotors(void) {routeForward=routeRight=commandTurn=0;}
static void failVision(const char *reason) {(void)reason;motionMode=0;}
static void printVisionState(void) {}
static void stopServoMotion(void) {}
static void startMechanismPose(const MechanismPose *pose) {
    mechanismState.target=*pose;mechanismState.running=1;
}
uint8_t mechanismActionStart(const MechanismInitialState *initial,
        const MechanismAction *actions,uint16_t count) {
    (void)initial;(void)actions;
    if(actions==missionPlaceActions) ++missionPlaceRuns;
    else if(actions==missionPickBackActions) ++missionPickRuns;
    else if(actions==missionWorkActions) {
        assert(visionSession.state==VISION_STATE_ALIGNED);
        assert(missionRingAligns==2 || missionRingAligns==4);
        ++missionStorageRuns;
    }
    mechanismActionActive=1;mechanismActionIndex=0;mechanismActionCount=count;
    return 1;
}
static uint8_t startVisionMotion(float forward,float right,uint16_t rpm,uint8_t automatic) {
    (void)rpm;(void)automatic;
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
    routeActive=routeWaiting=routeRotating=routeOnlyTurn=routeTurnInBand=0;
    fullRouteRunning=0;
    missionActive=missionActionStarted=0;missionPhase=MISSION_IDLE;
    rawPickRouteActive=rawPickAtStation=rawPickItemIndex=rawPickMissingReported=0;
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
static void tick(void) {
    /* Ideal plant: (L+W)/2 = 215mm, wheel diameter100mm. */
    imuYaw += commandTurn * 1.3955f * 0.020f * yawSign;
    while(imuYaw>180) imuYaw-=360;
    while(imuYaw<-180) imuYaw+=360;
    clockMs+=20; imuStamp=clockMs; serviceRoute();
    if(missionActive && routeWaiting) {
        if((missionPhase==MISSION_RING_ALIGN || missionPhase==MISSION_TEMP_ALIGN) && missionActionStarted==1 &&
           mechanismState.running) {
            mechanismState.current=mechanismState.target;
            mechanismState.running=0;
        }
        if(missionPhase==MISSION_RAW_PICK && rawPickItemIndex<3 &&
           clockMs%200==0) {++rawPickItemIndex;++missionRawItems;}
        if((missionPhase==MISSION_RING_ALIGN || missionPhase==MISSION_TEMP_ALIGN) && missionActionStarted==2)
            visionSession.state=VISION_STATE_ALIGNED;
        if(mechanismActionActive) {
            mechanismState.current.horizontalDmm=0;
            mechanismState.current.liftDmm=0;
            mechanismState.current.turretDdeg=2700;
            mechanismActionActive=0;mechanismActionIndex=mechanismActionCount;
        }
        if(motionMode==4) motionMode=0;
        serviceMission();
    }
}
int main(void) {
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
           routeTurnRpmLimit==45 && routeLateralRpmLimit==60);
    assert(processRouteCommand("route tune 10200 9000 50 80") &&
           routeTurnRpmLimit==50 && routeLateralRpmLimit==80);
    assert(processRouteCommand("route tune 10200 9000 50 121") &&
           routeLateralRpmLimit==80);
    assert(processRouteCommand("route tune 10200 9000 45 60") &&
           routeTurnRpmLimit==45 && routeLateralRpmLimit==60);
    routeActive=1;assert(processRouteCommand("route tune 9000 9000 60") &&
                         routeAbs(routeForwardScale-1.02f)<0.0001f &&
                         routeTurnRpmLimit==45 && routeLateralRpmLimit==60);
    routeActive=0;
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
        assert(!routeActive && routeIndex==ROUTE_COUNT-1 && routeHeading==0);
        assert(routeX==2250 && routeY==(start==1?2250:150));
        assert(turnTicks>100);
        assert(routePosReports>0 && routeStageReports==8 && routeDoneReports==1);
    }
    for(start=1;start<=2;++start) {
        unsigned qrStarted=0;
        missionPlaceRuns=missionPickRuns=missionStorageRuns=0;
        missionRingAligns=missionRawItems=0;
        stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
        mechanismState.valid=1;
        mechanismState.current.horizontalDmm=0;
        mechanismState.current.liftDmm=0;
        mechanismState.current.turretDdeg=2700;
        visionRingCalibration[1].calibrated=0;
        assert(processRouteCommand("route mission 1") && !routeActive);
        visionRingCalibration[1].calibrated=1;
        currentAngleMdeg[0]=35000;
        assert(processRouteCommand("route mission 1") && !routeActive);
        currentAngleMdeg[0]=70000;
        sprintf(cmd,"route mission %u 40",start);
        assert(processRouteCommand(cmd) && routeActive && missionActive);
        for(n=0;n<30000 && routeActive;++n) {
            tick();
            if(missionPhase==MISSION_QR_WAIT && !qrStarted) qrStarted=clockMs;
            if(qrStarted && clockMs-qrStarted<980U)
                assert(routeWaiting && routeIndex==2);
        }
        assert(!routeActive && routeDoneReports>0 &&
               routeX==2250 && routeY==(start==1?2250:150));
        assert(qrStarted && missionPlaceRuns==2 && missionPickRuns==2 &&
               missionStorageRuns==2 && missionRingAligns==4 &&
               missionRawItems==6);
    }
    missionBuildStoragePlace(missionWorkActions,2);
    assert(missionWorkActions[9].pose.liftDmm==880 &&
           missionWorkActions[18].pose.liftDmm==800 &&
           missionWorkActions[30].pose.liftDmm==850);
    stopAllMotors();imuStamp=clockMs;armed=1;
    assert(processRouteCommand("route mission 1 40") && missionActive);
    routeWaiting=1;missionPhase=MISSION_RECENTER;
    missionOffsetForward=45;missionOffsetRight=-30;
    serviceMission();assert(motionMode==4 && missionOffsetForward==25 &&
                            missionOffsetRight==-10);
    motionMode=0;serviceMission();assert(motionMode==4 &&
                                        missionOffsetForward==5 &&
                                        missionOffsetRight==0);
    motionMode=0;serviceMission();assert(motionMode==4 &&
                                        missionOffsetForward==0);
    motionMode=0;serviceMission();assert(!routeWaiting && missionActive);
    routeWaiting=1;missionPhase=MISSION_COARSE_PLACE;
    missionActionStarted=1;mechanismActionActive=0;
    mechanismActionCount=MISSION_PLACE_COUNT;mechanismActionIndex=0;
    n=routeInvalidReports;serviceMission();
    assert(!missionActive && !routeActive && routeInvalidReports==n+1);
    stopAllMotors();imuYaw=0;imuStamp=clockMs;armed=1;
    mechanismState.valid=1;
    mechanismState.current.turretDdeg=2700;
    visionMaterialCalibration.calibrated=0;
    assert(processRouteCommand("route rawpick 1") && !routeActive && armed);
    visionMaterialCalibration.calibrated=1;
    assert(processRouteCommand("route rawpick 1") && rawPickRouteActive);
    for(n=0;n<10000 && !routeWaiting;++n) tick();
    assert(routeWaiting && rawPickAtStation && routeIndex==4 &&
           routeX==1200 && routeY==2080 && routeHeading==1);
    n=batches;
    for(start=0;start<150;++start) tick();
    assert(routeWaiting && batches==n && routeX==1200 && routeY==2080);
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
            sprintf(cmd,"nav goto %d %d 120",destination.x,destination.y);
            assert(processNavCommand(cmd) && navRunning);
            assert(routeRpm==120);
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
    armed=1;imuStamp=clockMs;assert(processRouteCommand("route auto 1 120") && routeRpm==120);
    stopAllMotors();armed=1;imuStamp=clockMs;
    assert(processRouteCommand("route auto 1 121") && !routeActive);
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
    previous=(int16_t)batches;
    for(n=0;n<12 && routeActive;++n) {
        imuYaw=routeYaw-(n%2?1.9f:2.1f);clockMs+=20;imuStamp=clockMs;serviceRoute();
    }
    assert(!routeActive && batches==(unsigned)previous);
    puts("PASS: routes, optimized motion and navigation runtime checks");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_route_runtime.c'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $parser + "`n" + $pair + "`n" + $service + "`n" + $mission + "`n" + $motion + "`n" + $suffix)
& 'E:\Qt\Tools\mingw1310_64\bin\gcc.exe' -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o (Join-Path $root 'Objects/test_route_runtime.exe')
$compileExit = $LASTEXITCODE
if ($compileExit -ne 0) {throw "Runtime test compile failed: exit $compileExit"}
& (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test failed'}
