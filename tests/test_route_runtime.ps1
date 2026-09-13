$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
# Compile the actual route state machine with a simulated clock / IMU / motors.
$service = $source.Substring($source.IndexOf('static void serviceRoute(void)'))
$service = $service.Substring(0, $service.IndexOf('static void processCommand('))
$parser = $source.Substring($source.IndexOf('static uint8_t parseUint('))
$parser = $parser.Substring(0, $parser.IndexOf('static CommandResult parseServoCommand('))
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
}
static void sendRouteSpeeds(int16_t f,int16_t r,int16_t t) {
    (void)f;(void)r;commandTurn=t;
    if(t) ++turnTicks;
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
    char cmd[24];
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
    puts("PASS: auto both starts/signs, yaw wrap, step waits, cancel, turns, timeout, stale IMU");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_route_runtime.c'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $parser + "`n" + $service + "`n" + $suffix)
& 'C:\Users\11967\Downloads\mingw\bin\gcc.exe' -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test compile failed'}
& (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test failed'}
