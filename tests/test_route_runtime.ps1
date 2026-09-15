$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
# Compile the actual route state machine with a simulated clock / IMU / motors.
$service = $source.Substring($source.LastIndexOf('static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn)'))
$service = $service.Substring(0, $service.IndexOf('static void processCommand('))
$parser = $source.Substring($source.IndexOf('static uint8_t parseUint('))
$parser = $parser.Substring(0, $parser.IndexOf('static CommandResult parseServoCommand('))
$motion = $source.Substring($source.LastIndexOf('static void serviceMotion(void)'))
$motion = $motion.Substring(0, $motion.IndexOf('static uint8_t parsePair('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../route_plan.h"
#include "../straight_control.h"
static uint8_t routeActive,routeWaiting,routeIndex,routeStartZone,routeStep;
static uint8_t routeAuto,routeRotating,routeOnlyTurn,routeTurnInBand;
static uint8_t motionMode,armed,imuValid=1,emergencyStop,jogCanFault;
static int8_t routeHeading,routeNextHeading,yawSign=1;
static float routeX,routeY,routeYaw,routeBaseYaw,imuYaw;
static float routeDriveRpm,routeTurnRpm,routeCorrection;
static uint8_t routeSentValid;
static int16_t routeSent[4], staged[4];
static const uint8_t motorDirections[1][5]={{1,1,0,0,1}};
static uint8_t motorInvert[5];
static uint16_t motorTrim[5]={1000,1000,1000,1000,1000};
static unsigned batches, writes;
static uint8_t straightLateral;
static float straightSpeedState,straightTurnState,targetYaw;
static uint32_t motionStart,motionDuration,lastControl;
static uint16_t straightRpm;
static int16_t straightDirection;
static int16_t routeForward,routeRight,commandTurn;
static uint32_t routeTick,routeLegStart,routeSettle,routeTurnStart,routeTurnStable,clockMs,imuStamp;
static unsigned turnTicks;
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static void serialSendString(const char *s) {(void)s;}
static void serialSendChar(char c) {(void)c;}
static void serialSendUint(uint16_t x) {(void)x;}
static void printRouteStatus(void) {}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAllMotorsEnabled(bool x) {(void)x;}
static void stopAllMotors(void) {
    routeActive=routeWaiting=routeRotating=routeOnlyTurn=routeTurnInBand=0;
    routeForward=routeRight=commandTurn=0; armed=0;
    routeDriveRpm=routeTurnRpm=routeCorrection=0;routeSentValid=0;
    motionMode=0;straightSpeedState=straightTurnState=0;
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
'@
$suffix = @'
static void tick(void) {
    /* Ideal plant: (L+W)/2 = 215mm, wheel diameter100mm. */
    imuYaw += commandTurn * 1.3955f * 0.020f * yawSign;
    while(imuYaw>180) imuYaw-=360;
    while(imuYaw<-180) imuYaw+=360;
    clockMs+=20; imuStamp=clockMs; serviceRoute();
}
int main(void) {
    unsigned n,start; int sign;
    int16_t previous,peak;
    char cmd[24];
    sendRouteSpeeds(20,0,0);assert(batches==1 && writes==4);
    sendRouteSpeeds(20,0,0);assert(batches==1 && writes==4);
    sendRouteSpeeds(0,20,0);assert(batches==2 && writes==8);
    sendRouteSpeeds(0,0,0);assert(batches==3 && writes==12);
    sendRouteSpeeds(0,0,0);assert(batches==3 && writes==12);
    for(sign=-1;sign<=1;sign+=2) for(start=1;start<=2;++start) {
        stopAllMotors(); yawSign=(int8_t)sign; imuYaw=179; turnTicks=0; armed=1;
        sprintf(cmd,"route auto %u",start);
        assert(processRouteCommand(cmd) && routeActive && routeAuto);
        for(n=0;n<30000 && routeActive;++n) {tick();assert(!routeWaiting);}
        assert(!routeActive && routeIndex==ROUTE_COUNT-1 && routeHeading==0);
        assert(routeX==2250 && routeY==(start==1?2250:150));
        assert(turnTicks>100);
    }
    armed=1; assert(processRouteCommand("route step 1"));
    for(n=0;n<1000 && !routeWaiting;++n) tick();
    assert(routeWaiting && routeIndex==1);
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
    /* No repeated stop/restart as measurement oscillates around entry boundary. */
    armed=1;processRouteCommand("turn L 90");
    imuYaw=routeYaw-1.9f;clockMs+=20;imuStamp=clockMs;serviceRoute();
    previous=(int16_t)batches;
    for(n=0;n<12 && routeActive;++n) {
        imuYaw=routeYaw-(n%2?1.9f:2.1f);clockMs+=20;imuStamp=clockMs;serviceRoute();
    }
    assert(!routeActive && batches==(unsigned)previous);
    puts("PASS: auto both starts/signs, yaw wrap, step waits, cancel, turns, timeout, stale IMU");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_route_runtime.c'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $parser + "`n" + $service + "`n" + $motion + "`n" + $suffix)
& 'C:\Users\11967\Downloads\mingw\bin\gcc.exe' -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test compile failed'}
& (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test failed'}
