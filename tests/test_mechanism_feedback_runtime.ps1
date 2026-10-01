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
#include "../jog_can.h"
#include "../tests/stubs/can.h"
#define AUX_MOVE_STATUS_POLL_MS 50U
#define S_FLAG 13U
extern volatile uint8_t jogCanFault;
void CAN1_RX0_IRQHandler(void);
static MechanismState mechanismState;
static uint32_t clockMs, mechanismLastStatusPoll;
static uint8_t mechanismStatusSeen, mechanismLastStatus[2];
static uint8_t servoChannelMoving[3], emergencyStop;
static CanRxMsg received;
static unsigned queries[2], poses[2], syncs, dones, stops, servoStops;
static unsigned failedQueries, failedSync;
static uint8_t replyDuringReport;
static char logText[8192];
static void reply(uint8_t id,uint8_t status);
uint8_t CAN_Transmit(int bus, CanTxMsg *tx) {(void)bus;(void)tx;return 0;}
uint8_t CAN_TransmitStatus(int bus, uint8_t mailbox)
{(void)bus;(void)mailbox;return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(int bus, uint8_t mailbox) {(void)bus;(void)mailbox;}
void CAN_Receive(int bus, int fifo, CanRxMsg *rx) {(void)bus;(void)fifo;*rx=received;}
void delay_us(uint32_t us) {(void)us;}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAuxMotorsEnabled(bool enabled) {assert(enabled);}
static void stopAuxMotors(void) {++stops;}
static void stopServoMotion(void) {servoChannelMoving[2]=0;++servoStops;}
static void stopAllMotors(void)
{mechanismStateInvalidate(&mechanismState);jogCanResetAuxStatus();++stops;}
static void serialSendString(const char *s) {strcat(logText,s);}
static void serialSendUint(uint16_t value)
{char text[8];sprintf(text,"%u",value);strcat(logText,text);}
static void serialSendHex8(uint8_t value)
{char text[3];sprintf(text,"%02X",value);strcat(logText,text);}
static void printMechanismPose(const char *s, const MechanismPose *pose)
{(void)pose;if(strcmp(s,"MECH DONE")==0) ++dones;
 if(replyDuringReport && strcmp(s,"MECH POS")==0) {reply(6U,3U);replyDuringReport=0U;}}
static void Emm_V5_Pos_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t accel,
                              uint32_t pulses,bool absolute,bool sync)
{(void)dir;(void)rpm;(void)accel;assert(pulses && !absolute && sync);++poses[id-5U];}
static uint8_t servoSetTargetMdeg(uint8_t channel,uint32_t angle,uint16_t speed)
{assert(channel==4U);(void)speed;servoChannelMoving[2]=angle!=mechanismState.start.turretDdeg*100U;
 return servoChannelMoving[2];}
static void Emm_V5_Synchronous_motion(uint8_t id)
{assert(id==0U);++syncs;if(failedSync) jogCanFault=1U;}
static void Emm_V5_Read_Sys_Params(uint8_t id,uint8_t flag)
{assert(id==5U || id==6U);assert(flag==S_FLAG);++queries[id-5U];
 if(failedQueries) jogCanFault=1U;}
'@
$suffix = @'
static void reply(uint8_t id,uint8_t status)
{
    received.ExtId=(uint32_t)id<<8;received.IDE=CAN_Id_Extended;received.RTR=CAN_RTR_Data;
    received.DLC=3U;received.Data[0]=0x3AU;received.Data[1]=status;received.Data[2]=0x6BU;
    CAN1_RX0_IRQHandler();
}
static void reset(void)
{
    const MechanismPose initial={0,0,2700,80,50,1000,200,1200};
    mechanismStateInitialize(&mechanismState,&initial);
    jogCanResetAuxStatus();servoChannelMoving[2]=emergencyStop=jogCanFault=0;
    queries[0]=queries[1]=poses[0]=poses[1]=syncs=dones=stops=servoStops=failedQueries=failedSync=0;
    replyDuringReport=0U;logText[0]='\0';
    clockMs=100U;
}
int main(void)
{
    MechanismPose target={-200,300,1320,80,50,1000,200,1200};
    reset();
    jogCanExpectAuxStatus(5U);reply(5U,3U); /* Previous pose's cached completion. */
    startMechanismPose(&target);
    reply(6U,3U); /* Not yet queried: must not complete the new pose. */
    serviceMechanism();
    assert(syncs==1 && poses[0]==1 && poses[1]==1 && queries[0]==1 && queries[1]==1);
    assert(mechanismState.pendingMotors==3U && mechanismState.running);
    reply(5U,3U);reply(6U,1U);
    serviceMechanism();
    assert(mechanismState.pendingMotors==2U && mechanismState.running && !dones);
    assert(strstr(logText,"MECH MOTOR id=6 status=0x01\r\n"));
    clockMs+=50U;serviceMechanism();
    assert(queries[0]==1 && queries[1]==2);
    reply(6U,3U);serviceMechanism();
    assert(mechanismState.pendingMotors==0U && mechanismState.running); /* Turret not done. */
    assert(strstr(logText,"MECH MOTOR id=6 status=0x03\r\n"));
    servoChannelMoving[2]=0U;
    serviceMechanism();
    assert(!mechanismState.running && dones==1 && clockMs==150U);
    assert(mechanismState.current.horizontalDmm==-200 && mechanismState.current.liftDmm==300U);
    puts("PASS: dual axis feedback + turret completion advances without fixed settle");

    reset();target=mechanismState.current;target.liftDmm=300U;
    startMechanismPose(&target);serviceMechanism();
    assert(queries[0]==1 && queries[1]==0 && poses[0]==1 && poses[1]==0);
    reply(5U,3U);serviceMechanism();assert(dones==1);
    reset();target=mechanismState.current;target.horizontalDmm=-200;
    startMechanismPose(&target);serviceMechanism();
    assert(queries[0]==0 && queries[1]==1 && poses[0]==0 && poses[1]==1);
    reply(6U,3U);serviceMechanism();assert(dones==1);
    reset();target=mechanismState.current;
    startMechanismPose(&target);serviceMechanism();
    assert(!queries[0] && !queries[1] && dones==1);
    reset();target=mechanismState.current;target.turretDdeg=1320U;
    startMechanismPose(&target);serviceMechanism();
    assert(!queries[0] && !queries[1] && mechanismState.running);
    servoChannelMoving[2]=0;serviceMechanism();assert(dones==1);
    puts("PASS: single axis, zero movement and turret-only poses");

    reset();target=mechanismState.current;target.horizontalDmm=-200;
    startMechanismPose(&target);serviceMechanism();
    clockMs+=200U;replyDuringReport=1U;
    serviceMechanism(); /* Reply arrives after cache read, before next query. */
    assert(mechanismState.running);
    serviceMechanism();assert(dones==1 && !mechanismState.running);
    puts("PASS: an arrival reply between cache read and next poll is preserved");

    reset();target.liftDmm=300U;
    startMechanismPose(&target);serviceMechanism();
    clockMs=mechanismState.startMs+mechanismState.durationMs;
    serviceMechanism();assert(mechanismState.running && !dones);
    assert(mechanismState.current.liftDmm<=target.liftDmm); /* Estimate stays bounded. */
    clockMs=mechanismState.deadlineMs;serviceMechanism();
    assert(!mechanismState.valid && !mechanismState.running && !dones && servoStops==1 && stops);
    assert(strstr(logText,"MECH TIMEOUT motor=5 received=0\r\n"));
    assert(strstr(logText,"MECH TIMEOUT motor=6 received=0\r\n"));
    reply(5U,3U);serviceMechanism();assert(!dones);
    reset();target=mechanismState.current;target.horizontalDmm=-200;
    startMechanismPose(&target);serviceMechanism();reply(6U,1U);serviceMechanism();
    clockMs=mechanismState.deadlineMs;serviceMechanism();
    assert(!dones && strstr(logText,"MECH TIMEOUT motor=6 received=1 status=0x01\r\n"));
    reset();startMechanismPose(&target);serviceMechanism();
    reply(5U,3U);emergencyStop=1U;serviceMechanism();
    assert(!mechanismState.valid && servoStops==1 && !dones);
    reset();startMechanismPose(&target);failedQueries=1U;serviceMechanism();
    assert(!mechanismState.valid && jogCanFault && servoStops==1 && !dones);
    reset();failedSync=1U;startMechanismPose(&target);
    assert(!mechanismState.valid && jogCanFault && servoStops==1 && !dones);
    assert(!queries[0] && !queries[1]);
    puts("PASS: no reply times out, emergency stop and CAN fault invalidate pose");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_mechanism_feedback_runtime.c'
$exe = Join-Path $root 'Objects/test_mechanism_feedback_runtime.exe'
Set-Content -Path $generated -Value ($prefix + $start + $service + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function -I (Join-Path $PSScriptRoot 'stubs') $generated (Join-Path $root 'jog_can.c') -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
