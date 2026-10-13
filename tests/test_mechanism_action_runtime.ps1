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
#include "../raw_pick_action.h"
#include "../mission_actions.h"
static MechanismState mechanismState;
static const MechanismInitialState *mechanismActionInitial;
static const MechanismAction *mechanismActions;
static uint16_t mechanismActionCount, mechanismActionIndex;
static uint8_t mechanismActionDispatched, mechanismActionActive;
static uint32_t mechanismActionWaitUntil, clockMs;
static uint8_t motionMode, routeActive, rawPickRouteActive, rawPickAtStation, routeWaiting;
static uint8_t missionActive;
enum {MISSION_IDLE, MISSION_RAW_PICK, MISSION_COARSE_PLACE, MISSION_COARSE_PICK, MISSION_TEMP_PLACE};
static uint8_t missionPhase;
static uint16_t missionGrabCount, missionPlaceCount;
static unsigned countUpdates;
static void tjcDisplayCounts(void) { ++countUpdates; }
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
    assert(channel == 2U || channel == 3U);
    servoChannelMoving[channel - 2U] = 1U;
    if (channel == 2U) ++gripperStarts;
    if (channel == 3U) assert(speed == 1800U);
}
'@
$suffix = @'
static void runMissionActions(uint8_t phase, const MechanismInitialState *initial,
                              const MechanismAction *actions, uint16_t count)
{
    unsigned iterations = 0;
    missionPhase = phase;
    mechanismStateInitialize(&mechanismState, &initial->pose);
    assert(mechanismActionStart(initial, actions, count));
    while (mechanismActionActive) {
        uint16_t grabs = missionGrabCount, places = missionPlaceCount;
        assert(++iterations < 1000);
        mechanismActionService();
        if (servoChannelMoving[0]) {
            assert(missionGrabCount == grabs && missionPlaceCount == places);
            mechanismActionService();
            assert(missionGrabCount == grabs && missionPlaceCount == places);
        }
        if (mechanismState.running) {
            mechanismState.current = mechanismState.target;
            mechanismState.running = 0;
        }
        servoChannelMoving[0] = servoChannelMoving[1] = 0;
        clockMs += 1000;
    }
}

static void checkMissionCounts(void)
{
    const uint8_t rings[2][3] = {{3, 2, 1}, {2, 1, 3}};
    MechanismAction raw[RAW_PICK_ACTION_COUNT], place[MISSION_PLACE_COUNT];
    MechanismAction recover[MISSION_PICK_BACK_COUNT];
    missionActive = 1;
    for (unsigned batch = 0; batch < 2; ++batch) {
        for (uint8_t slot = 1; slot <= 3; ++slot) {
            rawPickActionsForSlot(raw, slot);
            runMissionActions(MISSION_RAW_PICK, &rawPickInitial, raw, RAW_PICK_ACTION_COUNT);
            assert(missionGrabCount == batch * 6 + slot && missionPlaceCount == batch * 6);
        }
        missionBuildTaskPlace(place, rings[batch], 1, 1);
        runMissionActions(MISSION_COARSE_PLACE, &missionCoarseInitial, place, MISSION_COARSE_PLACE_COUNT);
        assert(missionGrabCount == batch * 6 + 3 && missionPlaceCount == batch * 6 + 3);
        missionBuildTaskPickBack(recover, rings[batch], rings[batch][2], batch ? -470 : -450);
        runMissionActions(MISSION_COARSE_PICK, &missionCoarseInitial, recover, MISSION_PICK_BACK_COUNT);
        assert(missionGrabCount == (batch + 1) * 6 && missionPlaceCount == batch * 6 + 3);
        missionBuildTaskPlace(place, rings[batch], batch + 1, 0);
        runMissionActions(MISSION_TEMP_PLACE, &missionInitial, place, MISSION_PLACE_COUNT);
        assert(missionGrabCount == (batch + 1) * 6 && missionPlaceCount == (batch + 1) * 6);
    }
    assert(missionGrabCount == 12 && missionPlaceCount == 12 && countUpdates == 24);
    /* A stopped gripper command and manual bench actions must not add counts. */
    mechanismStateInitialize(&mechanismState, &rawPickInitial.pose);
    missionPhase = MISSION_RAW_PICK;
    assert(mechanismActionStart(&rawPickInitial, rawPickActions + 3, 1));
    mechanismActionService();
    assert(servoChannelMoving[0] && missionGrabCount == 12);
    mechanismActionActive = missionActive = 0;
    servoChannelMoving[0] = 0;
    mechanismActionService();
    runMissionActions(MISSION_RAW_PICK, &rawPickInitial, rawPickActions, RAW_PICK_ACTION_COUNT);
    assert(missionGrabCount == 12 && missionPlaceCount == 12 && countUpdates == 24);
    /* Extra waits must not count the same completed command more than once. */
    {
        const MechanismAction close = MECH_GRIPPER_CLOSE(300);
        missionActive = 1;
        runMissionActions(MISSION_RAW_PICK, &rawPickInitial, &close, 1);
        assert(missionGrabCount == 13 && countUpdates == 25);
    }
    puts("PASS: mission screen counts 12 pickups / 12 placements, excludes tray transfers, waits and cancellations");
}

int main(void)
{
    const MechanismInitialState initial = MECH_INITIAL_STATE(
        0, 0, 2720, 40, 50, 90, 50, 1200, 1200, 1800,
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
    assert(mechanismStateService(&mechanismState, clockMs, 0U) == MECHANISM_EVENT_DONE);
    mechanismActionService();
    assert(mechanismActionIndex == 2U);
    mechanismActionService();
    assert(poseStarts == 2U && mechanismState.target.liftDmm == 1300U);
    puts("PASS: pickup waits for the prior pose before lift");
    clockMs = mechanismState.deadlineMs;
    mechanismStateService(&mechanismState, clockMs, 0U);
    mechanismActionService();
    mechanismActionService();
    assert(!mechanismActionActive);
    {
        const MechanismAction waitActions[] = {
            MECH_GRIPPER_CLOSE(300), MECH_GRIPPER_OPEN(0)
        };
        assert(mechanismActionStart(&initial, waitActions, 2U));
        mechanismActionService();
        clockMs += 1000U; /* Extra wait starts after motion, not dispatch. */
        mechanismActionService();
        assert(mechanismActionWaitUntil == 0U);
        servoChannelMoving[0] = 0U;
        mechanismActionService();
        assert(mechanismActionWaitUntil == clockMs + 300U);
        clockMs += 299U;
        mechanismActionService();
        assert(mechanismActionIndex == 0U);
        ++clockMs;
        mechanismActionService();
        assert(mechanismActionIndex == 1U && gripperStarts == 3U);
    }
    puts("PASS: waitMs is added once after motion completes");
    servoChannelMoving[0] = 0U;
    mechanismActionService();
    mechanismActionService();
    const int16_t trayPositions[] = {-500, -450, -470};
    for (unsigned tray=0;tray<3;++tray) {
        int16_t trayH=trayPositions[tray];
        servoChannelMoving[1] = 0U;
        const MechanismAction parallel[] = {
            MECH_PLATFORM(2, 0),
            MECH_POSE(trayH, 0, 1320, 40, 50, 90, 50, 1200, 0),
            MECH_POSE(trayH, 300, 1320, 40, 50, 90, 50, 1200, 0),
            MECH_PLATFORM(3, 0), MECH_PLATFORM(1, 0)
        };
        assert(mechanismActionStart(&initial, parallel, 5U));
        mechanismActionService();
        assert(mechanismActionIndex == 1U && servoChannelMoving[1]);
        mechanismActionService();
        assert(mechanismState.running); /* Pose overlaps platform rotation. */
        clockMs = mechanismState.deadlineMs;
        assert(mechanismStateService(&mechanismState, clockMs, 0U) == MECHANISM_EVENT_DONE);
        mechanismActionService();
        assert(mechanismActionIndex == 2U);
        mechanismActionService();
        assert(!mechanismState.running); /* Tray descent must wait. */
        servoChannelMoving[1] = 0U;
        mechanismActionService();
        assert(mechanismState.running && mechanismState.target.liftDmm == 300U);
        clockMs = mechanismState.deadlineMs;
        mechanismStateService(&mechanismState, clockMs, 0U);
        mechanismActionService();
        mechanismActionService();
        assert(mechanismActionIndex == 4U && servoChannelMoving[1]);
        mechanismActionService();
        assert(mechanismActionIndex == 4U && !mechanismActionDispatched);
        servoChannelMoving[1] = 0U;
        mechanismActionService();
        assert(mechanismActionIndex == 5U);
        mechanismActionService();
        assert(!mechanismActionActive);
    }
    puts("PASS: platform overlaps poses, tray descent and next rotation wait");
    checkMissionCounts();
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
