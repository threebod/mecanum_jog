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
static unsigned queries, dones, stops, servoStops, syncs;
static uint8_t failedSync;
uint8_t CAN_Transmit(int bus, CanTxMsg *tx) {(void)bus;(void)tx;return 0;}
uint8_t CAN_TransmitStatus(int bus, uint8_t mailbox)
{(void)bus;(void)mailbox;return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(int bus, uint8_t mailbox) {(void)bus;(void)mailbox;}
void CAN_Receive(int bus, int fifo, CanRxMsg *rx) {(void)bus;(void)fifo;(void)rx;}
void delay_us(uint32_t us) {(void)us;}
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
{assert(id==5U || id==6U);(void)dir;(void)rpm;(void)accel;assert(pulses && !absolute && sync);}
static uint8_t servoSetTargetMdeg(uint8_t channel,uint32_t angle,uint16_t speed)
{assert(channel==4U);(void)speed;servoChannelMoving[2]=angle!=mechanismState.start.turretDdeg*100U;
 return servoChannelMoving[2];}
static void Emm_V5_Synchronous_motion(uint8_t id)
{assert(id==0U);++syncs;if(failedSync) jogCanFault=1U;}
static void Emm_V5_Read_Sys_Params(uint8_t id,uint8_t flag) {(void)id;(void)flag;++queries;}
'@
$suffix = @'
static void reset(void)
{
    const MechanismPose initial={0,0,2700,80,50,1000,200,1200};
    mechanismStateInitialize(&mechanismState,&initial);
    servoChannelMoving[2]=emergencyStop=jogCanFault=failedSync=0;
    queries=dones=stops=servoStops=syncs=0;
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
    puts("PASS: horizontal estimate completes at deadline without status queries or extra settle");

    reset();target=mechanismState.current;target.liftDmm=1300U;
    startMechanismPose(&target);
    assert(mechanismState.durationMs==1478U && mechanismState.deadlineMs==1578U);
    clockMs=1577U;serviceMechanism();assert(!dones && mechanismState.running);
    ++clockMs;serviceMechanism();assert(dones==1 && !queries);
    puts("PASS: lift completes on estimate, no additional 500 ms");

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
