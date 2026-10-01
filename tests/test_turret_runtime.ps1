$ErrorActionPreference = 'Stop'
$mingwBin = 'E:\Qt\Tools\mingw1310_64\bin'
$env:Path = "$mingwBin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
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
            assert(turretDurationMs == turretMotionDurationMs(delta, actions[i].pose.turretDps10));
            while (servoChannelMoving[2]) TIM2_IRQHandler();
            assert(currentAngleMdeg[2] == target);
        }
    }
}
int main(void)
{
    unsigned i;
    uint32_t interrupted;
    MechanismAction pickBack[MISSION_PICK_BACK_COUNT];
    MechanismAction storage[MISSION_PLACE_COUNT];
    assert(!servoSetTarget(4U, 90U, 1200U)); /* Initial reference unchanged. */
    assert(servoSetTarget(4U, 180U, 1200U));
    assert(turretDurationMs == 1420U);
    servoSetTarget(2U, 0U, 1200U);
    assert(servoSetTarget(2U, 90U, 1200U));
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
    puts("PASS: real servo setter/IRQ, gripper unchanged, retarget and stop");
    servoSetTargetMdeg(4U, (uint32_t)rawPickObservePose.turretDdeg * 100U,
                       rawPickObservePose.turretDps10);
    while (servoChannelMoving[2]) TIM2_IRQHandler();
    checkActionTurrets(rawPickActions, RAW_PICK_ACTION_COUNT);
    checkActionTurrets(missionCoarsePlaceActions, MISSION_COARSE_PLACE_COUNT);
    missionBuildPickBack(pickBack);
    checkActionTurrets(pickBack, MISSION_PICK_BACK_COUNT);
    missionBuildStoragePlace(storage, 1U);
    checkActionTurrets(storage, MISSION_PLACE_COUNT);
    missionBuildStoragePlace(storage, 2U);
    checkActionTurrets(storage, MISSION_PLACE_COUNT);
    puts("PASS: pickup, coarse place/recovery and both storage layers share turret ramp");
    return 0;
}
'@
$generated = Join-Path $root 'Objects/test_turret_runtime.c'
$exe = Join-Path $root 'Objects/test_turret_runtime.exe'
Set-Content -Path $generated -Value ($prefix + $setter + $irq + $suffix)
& (Join-Path $mingwBin 'gcc.exe') -std=c99 -Wall -Wextra -Werror -Wno-unused-function $generated -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
