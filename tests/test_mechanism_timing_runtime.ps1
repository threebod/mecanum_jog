$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
$start = $source.Substring($source.IndexOf('static void startMechanismPose('))
$start = $start.Substring(0, $start.IndexOf('static uint8_t processMechanismCommand('))
$service = $source.Substring($source.IndexOf('static void serviceMechanism(void)'))
$service = $service.Substring(0, $service.IndexOf('uint8_t mechanismActionStart('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include "../mechanism_action.h"
#include "../tests/stubs/can.h"
extern volatile uint8_t jogCanFault;
static MechanismState mechanismState;
static uint32_t clockMs;
static uint8_t servoChannelMoving[3], emergencyStop;
static uint8_t gripperMayHoldMaterial = 1U;
#define SERVO_EMPTY_TURRET_SPEED_DPS10 2500U
#define SERVO_UPDATE_HZ 50U
static unsigned queries, dones, stops, servoStops, syncs;
static unsigned commandMs,lastPositionMs,positionWrites;
static uint8_t horizontalDirection;
static uint16_t horizontalRpm, liftRpm;
static uint8_t liftAccel;
static uint32_t horizontalPulses;
static uint8_t failedSync;
uint8_t CAN_Transmit(int bus, CanTxMsg *tx) {(void)bus;(void)tx;return 0;}
uint8_t CAN_TransmitStatus(int bus, uint8_t mailbox)
{(void)bus;(void)mailbox;return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(int bus, uint8_t mailbox) {(void)bus;(void)mailbox;}
void CAN_Receive(int bus, int fifo, CanRxMsg *rx) {(void)bus;(void)fifo;(void)rx;}
void delay_us(uint32_t us) {(void)us;}
void delay_ms(uint32_t ms) {commandMs+=ms;}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAuxMotorsEnabled(bool enabled) {assert(enabled);}
static void stopAuxMotors(void) {++stops;}
static void stopServoMotion(void) {servoChannelMoving[2]=0;++servoStops;}
static void stopAllMotors(void) {mechanismStateInvalidate(&mechanismState);++stops;}
static void serialSendString(const char *s) {(void)s;}
static void printMechanismPose(const char *s, const MechanismPose *pose)
{(void)pose;if(strcmp(s,"MECH DONE")==0) ++dones;}
static void Emm_V5_Pos_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t accel,
                              uint32_t pulses,bool absolute,bool sync)
{
    assert(id==5U || id==6U);(void)dir;(void)rpm;(void)accel;
    assert(pulses && !absolute && sync);
    if(id==5U) {liftRpm=rpm;liftAccel=accel;}
    if(id==6U) {horizontalDirection=dir;horizontalRpm=rpm;horizontalPulses=pulses;}
    if(positionWrites) assert(commandMs-lastPositionMs>=2U);
    lastPositionMs=commandMs;++positionWrites;
}
static uint8_t servoSetTargetMdeg(uint8_t channel,uint32_t angle,uint16_t speed)
{assert(channel==4U);(void)speed;servoChannelMoving[2]=angle!=mechanismState.start.turretDdeg*100U;
 return servoChannelMoving[2];}
static void Emm_V5_Synchronous_motion(uint8_t id)
{
    assert(id==0U);
    if(positionWrites) assert(commandMs-lastPositionMs>=2U);
    ++syncs;if(failedSync) jogCanFault=1U;
}
static void Emm_V5_Read_Sys_Params(uint8_t id,uint8_t flag) {(void)id;(void)flag;++queries;}
'@
$suffix = @'
static void reset(void)
{
    const MechanismPose initial={0,0,2720,80,50,2000,240,1200};
    mechanismStateInitialize(&mechanismState,&initial);
    servoChannelMoving[2]=emergencyStop=jogCanFault=failedSync=0;
    gripperMayHoldMaterial=1U;
    queries=dones=stops=servoStops=syncs=0;
    commandMs=lastPositionMs=positionWrites=0;
    liftRpm=liftAccel=0;
    clockMs=100U;
}
int main(void)
{
    MechanismPose target;
    reset();target=mechanismState.current;target.horizontalDmm=-500;
    startMechanismPose(&target);serviceMechanism();
    assert(syncs==1 && mechanismState.running && !queries);
    clockMs=mechanismState.deadlineMs-1U;serviceMechanism();
    assert(mechanismState.running && !dones && !queries);
    ++clockMs;serviceMechanism();
    assert(!mechanismState.running && dones==1 && mechanismState.current.horizontalDmm==-500);
    assert(clockMs==mechanismState.startMs+mechanismState.durationMs);
    puts("PASS: horizontal estimate completes at deadline without status queries");

    reset();target=mechanismState.current;target.horizontalDmm=-300;
    target.horizontalRpm=500U;target.horizontalAccel=200U;
    startMechanismPose(&target);
    assert(mechanismState.durationMs==451U && mechanismState.deadlineMs==551U);
    clockMs=501U;serviceMechanism();assert(mechanismState.running && !dones && !stops);
    clockMs=550U;serviceMechanism();assert(mechanismState.running && !dones && !stops);
    ++clockMs;serviceMechanism();assert(dones==1 && !mechanismState.running);
    reset();target=mechanismState.current;target.horizontalDmm=300;
    target.horizontalRpm=500U;target.horizontalAccel=200U;
    startMechanismPose(&target);assert(mechanismState.durationMs==451U);
    puts("PASS: both horizontal directions include 50 ms margin, no stop at the old deadline");

    reset();mechanismState.current.turretDdeg=650U;
    target=mechanismState.current;target.horizontalDmm=-500;
    target.horizontalRpm=500U;target.horizontalAccel=200U;target.turretDdeg=2720U;
    startMechanismPose(&target);
    assert(positionWrites==1 && syncs==1 && commandMs>=2U);
    assert(horizontalDirection==1U && horizontalRpm==500U && horizontalPulses==1273U);
    puts("PASS: arrival preparation sends horizontal retract and delays before synchronous start");

    reset();target=mechanismState.current;target.liftDmm=1300U;
    startMechanismPose(&target);
    assert(mechanismState.durationMs==790U && mechanismState.deadlineMs==890U);
    clockMs=889U;serviceMechanism();assert(!dones && mechanismState.running);
    ++clockMs;serviceMechanism();assert(dones==1 && !queries);
    assert(liftRpm==2000U && liftAccel==240U);
    puts("PASS: lift sends 2000 RPM / acc 240 and completes on estimate, no additional 500 ms");

    reset();target=mechanismState.current;target.horizontalDmm=-200;target.liftDmm=300;
    target.turretDdeg=1320U;
    startMechanismPose(&target);
    assert(mechanismState.durationMs==mechanismMoveDurationMs(&mechanismState.start,&target));
    clockMs=mechanismState.deadlineMs;serviceMechanism();
    assert(mechanismState.running && !dones);
    servoChannelMoving[2]=0U;serviceMechanism();
    assert(dones==1 && mechanismState.valid && !queries);
    reset();target=mechanismState.current;
    startMechanismPose(&target);serviceMechanism();assert(dones==1 && !queries);
    puts("PASS: mixed pose waits for turret trajectory, unchanged pose completes immediately");
    reset();gripperMayHoldMaterial=0U;target=mechanismState.current;
    target.turretDdeg=900U;startMechanismPose(&target);
    assert(mechanismState.durationMs==740U);
    clockMs=mechanismState.deadlineMs;serviceMechanism();
    assert(mechanismState.running);
    servoChannelMoving[2]=0U;serviceMechanism();assert(dones==1);
    reset();gripperMayHoldMaterial=0U;target=mechanismState.current;
    target.horizontalDmm=-500;target.turretDdeg=900U;
    startMechanismPose(&target);
    assert(mechanismState.durationMs==mechanismMotorDurationMs(
        mechanismHorizontalPulses(-500),target.horizontalRpm,target.horizontalAccel)+50U);
    puts("PASS: empty turret deadline uses 250 dps and still waits for the slower stepper axis");

    reset();target=mechanismState.current;target.liftDmm=300U;
    startMechanismPose(&target);emergencyStop=1U;serviceMechanism();
    assert(!mechanismState.valid && !dones && servoStops==1 && stops);
    reset();startMechanismPose(&target);jogCanFault=1U;serviceMechanism();
    assert(!mechanismState.valid && !dones && servoStops==1 && stops);
    reset();failedSync=1U;startMechanismPose(&target);
    assert(!mechanismState.valid && !dones && servoStops==1 && stops);
    puts("PASS: emergency stop and CAN faults still stop and invalidate pose");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_mechanism_timing_runtime.c'
$exe = Join-Path $root 'Objects/test_mechanism_timing_runtime.exe'
Set-Content -Path $generated -Value ($prefix + $start + $service + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function -I (Join-Path $PSScriptRoot 'stubs') $generated (Join-Path $root 'jog_can.c') -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
