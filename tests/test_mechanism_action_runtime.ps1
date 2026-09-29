$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
$service = $source.Substring($source.IndexOf('uint8_t mechanismActionStart('))
$service = $service.Substring(0, $service.IndexOf('static void startLine('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include "../mechanism_action.h"
static MechanismState mechanismState;
static const MechanismInitialState *mechanismActionInitial;
static const MechanismAction *mechanismActions;
static uint16_t mechanismActionCount, mechanismActionIndex;
static uint8_t mechanismActionDispatched, mechanismActionActive;
static uint32_t mechanismActionWaitUntil, clockMs;
static uint8_t motionMode, routeActive, rawPickRouteActive, rawPickAtStation, routeWaiting;
static uint8_t missionActive;
static uint8_t servoChannelMoving[3];
static unsigned poseStarts, gripperStarts;
static void startMechanismPose(const MechanismPose *pose)
{
    assert(mechanismStateStart(&mechanismState, pose, clockMs));
    ++poseStarts;
}
static void servoSetTarget(uint8_t channel, uint16_t angle, uint16_t speed)
{
    (void)angle; (void)speed;
    assert(channel == 2U);
    servoChannelMoving[0] = 1U;
    ++gripperStarts;
}
'@
$suffix = @'
int main(void)
{
    const MechanismInitialState initial = MECH_INITIAL_STATE(
        0, 0, 2700, 40, 50, 90, 50, 1200, 1200, 1200,
        1, 1, 70, 35, 30, 150, 270);
    const MechanismAction actions[] = {
        MECH_GRIPPER_OPEN(0),
        MECH_POSE(-200, 0, 2680, 40, 50, 90, 50, 1200, 0),
        MECH_POSE(-200, 1300, 2680, 40, 50, 90, 50, 1200, 0)
    };
    mechanismStateInitialize(&mechanismState, &initial.pose);
    clockMs = 100U;
    assert(mechanismActionStart(&initial, actions, 3U));
    mechanismActionService();
    assert(gripperStarts == 1U && mechanismActionIndex == 0U);
    mechanismActionService();
    assert(mechanismActionIndex == 0U);
    servoChannelMoving[0] = 0U;
    mechanismActionService();
    assert(mechanismActionIndex == 1U);
    mechanismActionService();
    assert(poseStarts == 1U && mechanismState.target.liftDmm == 0U);
    mechanismActionService();
    assert(mechanismActionIndex == 1U && poseStarts == 1U);
    clockMs = mechanismState.deadlineMs;
    assert(mechanismStateService(&mechanismState, clockMs) == MECHANISM_EVENT_DONE);
    mechanismActionService();
    assert(mechanismActionIndex == 2U);
    mechanismActionService();
    assert(poseStarts == 2U && mechanismState.target.liftDmm == 1300U);
    puts("PASS: pickup waits for the prior pose before lift");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_mechanism_action_runtime.c'
$exe = Join-Path $root 'Objects/test_mechanism_action_runtime.exe'
Set-Content -Path $generated -Value ($prefix + $service + $suffix)
$gcc = Join-Path $mingwBin 'gcc.exe'
& $gcc -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
