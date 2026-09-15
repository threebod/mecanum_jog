$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
# Compile the actual route state machine with a simulated clock / IMU / motors.
$service = $source.Substring($source.IndexOf('static int16_t navFieldYawCdeg(void)'))
$service = $service.Substring(0, $service.IndexOf('static void processCommand('))
$parser = $source.Substring($source.IndexOf('static uint8_t parseUint('))
$parser = $parser.Substring(0, $parser.IndexOf('static CommandResult parseServoCommand('))
$pair = $source.Substring($source.IndexOf('static uint8_t parsePair('))
$pair = $pair.Substring(0, $pair.IndexOf('static void startLine('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../route_plan.h"
#include "../map_navigation.h"
#include "../straight_control.h"
static uint8_t routeActive,routeWaiting,routeIndex,routeStartZone,routeStep;
static uint8_t routeAuto,routeRotating,routeOnlyTurn,routeTurnInBand;
static uint8_t motionMode,armed,imuValid=1,emergencyStop,jogCanFault;
static int8_t routeHeading,routeNextHeading,yawSign=1;
static float routeX,routeY,routeYaw,routeBaseYaw,imuYaw;
static int16_t routeForward,routeRight,commandTurn;
static uint32_t routeTick,routeLegStart,routeSettle,routeTurnStart,routeTurnStable,clockMs,imuStamp;
static unsigned turnTicks;
static uint8_t navInitialized,navRunning,navCurrentNode,navPath[NAV_NODE_COUNT],navPathCount;
static uint32_t navLastReport;
static unsigned navPosReports,navDoneReports,navInvalidReports;
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static void serialSendString(const char *s) {
    if(strcmp(s,"NAV POS x=")==0) ++navPosReports;
    if(strcmp(s,"NAV DONE x=")==0) ++navDoneReports;
    if(strncmp(s,"NAV INVALID",11)==0) ++navInvalidReports;
}
static void serialSendChar(char c) {(void)c;}
static void serialSendUint(uint16_t x) {(void)x;}
static void serialSendInt(int16_t x) {(void)x;}
static void printRouteStatus(void) {}
static uint8_t motionInterrupted(void) {return emergencyStop || jogCanFault;}
static void setAllMotorsEnabled(bool x) {(void)x;}
static void stopDriveMotors(void) {routeForward=routeRight=commandTurn=0;}
static void invalidateNavigation(const char *reason) {
    (void)reason;serialSendString("NAV INVALID reason=");
    navInitialized=navRunning=navPathCount=0;
}
static void stopAllMotors(void) {
    routeActive=routeWaiting=routeRotating=routeOnlyTurn=routeTurnInBand=0;
    routeForward=routeRight=commandTurn=0; armed=0;
    navInitialized=navRunning=navPathCount=0;
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
    static const unsigned targetNode[] = {
        NAV_START_1,NAV_GATE_1,NAV_UPPER_RIGHT,NAV_QR,NAV_START_2,
        NAV_GATE_2,NAV_RAW,NAV_CENTER,NAV_COARSE,NAV_TEMP
    };
    unsigned n,start,target; int sign;
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
    for(sign=-1;sign<=1;sign+=2) for(start=1;start<=2;++start)
        for(target=0;target<sizeof(targetNode)/sizeof(targetNode[0]);++target) {
            NavPoint destination=navPoint((uint8_t)targetNode[target]);
            stopAllMotors();imuYaw=179;imuStamp=clockMs;imuValid=1;yawSign=(int8_t)sign;
            sprintf(cmd,"nav init %u",start);
            assert(processNavCommand(cmd) && navInitialized &&
                   navCurrentNode==navStartNode((uint8_t)start));
            armed=1;navPosReports=navDoneReports=0;
            sprintf(cmd,"nav goto %d %d",destination.x,destination.y);
            assert(processNavCommand(cmd) && navRunning);
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
    stopAllMotors();imuYaw=0;imuStamp=clockMs;imuValid=1;yawSign=1;
    assert(processNavCommand("nav init 1") && navInitialized);
    armed=1;assert(processNavCommand("nav goto 400 1200") && navRunning);
    armed=1;assert(processNavCommand("nav goto 1200 1200") && navRunning); /* busy reject */
    navInvalidReports=0;clockMs+=20;imuStamp=clockMs-300;serviceRoute();
    assert(!navInitialized && !navRunning && navInvalidReports==1);
    assert(processNavCommand("nav goto 1200 1200") && !navRunning);
    puts("PASS: routes plus navigation all starts/signs/targets, repeats, busy and invalidation");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_route_runtime.c'
[IO.File]::WriteAllText($generated, $prefix + "`n" + $parser + "`n" + $pair + "`n" + $service + "`n" + $suffix)
& 'E:\Qt\Tools\mingw1310_64\bin\gcc.exe' -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o (Join-Path $root 'Objects/test_route_runtime.exe')
$compileExit = $LASTEXITCODE
if ($compileExit -ne 0) {throw "Runtime test compile failed: exit $compileExit"}
& (Join-Path $root 'Objects/test_route_runtime.exe')
if ($LASTEXITCODE -ne 0) {throw 'Runtime test failed'}
