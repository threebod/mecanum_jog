$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
$observePose = [regex]::Match($source, 'static const MechanismPose missionRingObservePose\s*=\s*\{[^}]+\};').Value
$setter = $source.Substring($source.IndexOf('static uint8_t servoSetTargetMdeg('))
$setter = $setter.Substring(0, $setter.IndexOf('static void serviceHostWatchdog('))
$irq = $source.Substring($source.IndexOf('void TIM2_IRQHandler('))
$irq = $irq.Substring(0, $irq.IndexOf('static uint8_t motionInterrupted('))
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include "../turret_motion.h"
#include "../raw_pick_action.h"
#include "../mission_actions.h"
#define SERVO_UPDATE_HZ 50U
#define SERVO_EMPTY_TURRET_SPEED_DPS10 2500U
#define TIM2 0
#define TIM_IT_Update 0
#define RESET 0
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define TIM_GetITStatus(timer, flag) 1
#define TIM_ClearITPendingBit(timer, flag) ((void)0)
static uint8_t servoChannelEnabled[3], servoChannelMoving[3];
static uint8_t completedServoChannels;
static uint32_t currentAngleMdeg[3], targetAngleMdeg[3], servoStepMdeg[3];
static uint32_t turretStartAngleMdeg, turretElapsedMs, turretDurationMs;
static uint8_t turretLinearMotion, gripperMayHoldMaterial = 1U;
static uint16_t servoAngleToPulse(uint8_t channel, uint32_t angle)
{ (void)channel; (void)angle; return 500U; }
static void servoSetPulse(uint8_t channel, uint16_t pulse)
{ (void)pulse; servoChannelEnabled[channel - 2U] = 1U; }
'@
$suffix = @'
static void checkActionTurrets(const MechanismAction *actions, uint16_t count)
{
    uint16_t i;
    for (i = 0U; i < count; ++i) {
        if (actions[i].type == MECHANISM_ACTION_POSE) {
            uint32_t target = (uint32_t)actions[i].pose.turretDdeg * 100U;
            uint32_t start = currentAngleMdeg[2];
            uint32_t delta = target > start ? target - start : start - target;
            servoSetTargetMdeg(4U, target, actions[i].pose.turretDps10);
            assert(turretLinearMotion == !gripperMayHoldMaterial);
            assert(turretDurationMs == (gripperMayHoldMaterial ?
                turretMotionDurationMs(delta, actions[i].pose.turretDps10) :
                ((delta + 4999U) / 5000U) * 20U));
            while (servoChannelMoving[2]) TIM2_IRQHandler();
            assert(currentAngleMdeg[2] == target);
        } else if (actions[i].type == MECHANISM_ACTION_GRIPPER) {
            servoSetTarget(2U, actions[i].value ? rawPickInitial.gripperOpenDeg : rawPickInitial.gripperCloseDeg, 1800U);
            while (servoChannelMoving[0]) TIM2_IRQHandler();
            assert(gripperMayHoldMaterial == !actions[i].value);
        }
    }
}
int main(void)
{
    unsigned i;
    uint32_t interrupted;
    MechanismAction pickBack[MISSION_PICK_BACK_COUNT];
    MechanismAction storage[MISSION_PLACE_COUNT];
    MechanismAction observations[2] = {{0}};
    assert(!servoSetTarget(2U, rawPickInitial.gripperCloseDeg,
                           rawPickInitial.gripperDps10));
    assert(currentAngleMdeg[0] == 35000U);
    servoSetTarget(2U, 0U, 1200U);
    while (servoChannelMoving[0]) TIM2_IRQHandler();
    assert(!servoSetTarget(4U, 65U, 1800U));
    assert(currentAngleMdeg[2] == 65000U && targetAngleMdeg[2] == 65000U);
    assert(servoSetTarget(4U, 270U, 1200U));
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    assert(currentAngleMdeg[2] == 270000U);
    servoSetTarget(4U, 90U, 1200U);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    assert(!servoSetTarget(4U, 90U, 1200U));
    assert(servoSetTarget(4U, 180U, 1200U));
    assert(turretDurationMs == 1420U);
    servoSetTarget(2U, 0U, 1200U);
    assert(servoSetTarget(2U, 35U, 1200U));
    TIM2_IRQHandler();
    assert(currentAngleMdeg[0] == 2400U); /* Gripper still uses fixed steps. */
    assert(currentAngleMdeg[2] - 90000U < 100U);
    for (i = 1U; i < 71U; ++i) {
        assert(servoChannelMoving[2]);
        TIM2_IRQHandler();
    }
    assert(!servoChannelMoving[2] && currentAngleMdeg[2] == 180000U);
    assert(completedServoChannels & 4U);
    assert(servoSetTarget(4U, 0U, 1200U));
    for (i = 0U; i < 10U; ++i) TIM2_IRQHandler();
    interrupted = currentAngleMdeg[2];
    assert(servoSetTarget(4U, 270U, 1200U));
    assert(turretStartAngleMdeg == interrupted && turretElapsedMs == 0U);
    TIM2_IRQHandler();
    assert(currentAngleMdeg[2] >= interrupted);
    interrupted = currentAngleMdeg[2];
    stopServoMotion();
    for (i = 0U; i < 10U; ++i) TIM2_IRQHandler();
    assert(currentAngleMdeg[2] == interrupted && !servoChannelMoving[2]);
    assert(!servoSetTargetMdeg(4U, interrupted, 1200U));
    assert(servoSetTargetMdeg(4U, interrupted + 1U, 1800U));
    TIM2_IRQHandler();
    assert(!servoChannelMoving[2] && currentAngleMdeg[2] == interrupted + 1U);
    servoSetTarget(4U, 90U, 2200U);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    servoSetTarget(4U, 180U, 2200U);
    assert(!turretLinearMotion && turretDurationMs == 780U);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    puts("PASS: real servo setter/IRQ, loaded 220 dps, retarget and stop");
    servoSetTargetMdeg(4U, (uint32_t)rawPickObservePose.turretDdeg * 100U,
                       rawPickObservePose.turretDps10);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    servoSetTarget(2U, rawPickInitial.gripperOpenDeg, 1800U);
    while (servoChannelMoving[0]) TIM2_IRQHandler();
    assert(!gripperMayHoldMaterial);
    servoSetTarget(4U, 90U, 1200U);
    assert(turretLinearMotion && turretDurationMs == 740U);
    interrupted = currentAngleMdeg[2];
    TIM2_IRQHandler();
    assert(currentAngleMdeg[2] == interrupted - 5000U);
    for (i = 1U; i < 37U; ++i) TIM2_IRQHandler();
    assert(!servoChannelMoving[2] && currentAngleMdeg[2] == 90000U);
    servoSetTarget(4U, 180U, 1800U);
    TIM2_IRQHandler();
    assert(currentAngleMdeg[2] == 95000U);
    stopServoMotion();
    TIM2_IRQHandler();
    assert(currentAngleMdeg[2] == 95000U);
    servoSetTarget(2U, 35U, 1800U);
    while (servoChannelMoving[0]) TIM2_IRQHandler();
    servoSetTarget(4U, 180U, 1200U);
    assert(!turretLinearMotion && gripperMayHoldMaterial);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    puts("PASS: empty turret uses 250 dps fixed steps, stop holds, close restores loaded ramp");
    observations[0].pose = rawPickObservePose;
    observations[1].pose = missionRingObservePose;
    checkActionTurrets(observations, 2U);
    checkActionTurrets(rawPickActions, RAW_PICK_ACTION_COUNT);
    checkActionTurrets(missionCoarsePlaceActions, MISSION_COARSE_PLACE_COUNT);
    missionBuildPickBack(pickBack);
    checkActionTurrets(pickBack, MISSION_PICK_BACK_COUNT);
    missionBuildStoragePlace(storage, 1U);
    checkActionTurrets(storage, MISSION_PLACE_COUNT);
    missionBuildStoragePlace(storage, 2U);
    checkActionTurrets(storage, MISSION_PLACE_COUNT);
    puts("PASS: pickup, coarse place/recovery and both storage layers select empty or loaded turret motion");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_turret_runtime.c'
$exe = Join-Path $root 'Objects/test_turret_runtime.exe'
Set-Content -Path $generated -Value ($prefix + $observePose + $setter + $irq + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
