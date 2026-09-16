#ifndef MECANUM_JOG_MECHANISM_ACTION_H
#define MECANUM_JOG_MECHANISM_ACTION_H

#include <stdint.h>

#define MECHANISM_HORIZONTAL_MIN_DMM (-1220)
#define MECHANISM_HORIZONTAL_MAX_DMM 650
#define MECHANISM_LIFT_MAX_DMM 1350U
#define MECHANISM_TURRET_MAX_DDEG 3600U
#define MECHANISM_RPM_MIN 10U
#define MECHANISM_RPM_MAX 2000U
#define MECHANISM_ACCEL_MIN 1U
#define MECHANISM_ACCEL_MAX 240U
#define MECHANISM_TURRET_SPEED_MIN 10U
#define MECHANISM_TURRET_SPEED_MAX 300U

typedef struct {
    int16_t horizontalDmm;
    uint16_t liftDmm;
    uint16_t turretDdeg;
    uint16_t horizontalRpm;
    uint8_t horizontalAccel;
    uint16_t liftRpm;
    uint8_t liftAccel;
    uint16_t turretDps10;
} MechanismPose;

typedef struct {
    MechanismPose pose;
    uint8_t gripperOpen;
    uint8_t platform;
    uint16_t gripperOpenDeg;
    uint16_t gripperCloseDeg;
    uint16_t platformDeg[3];
} MechanismInitialState;

typedef enum {
    MECHANISM_ACTION_POSE = 0,
    MECHANISM_ACTION_GRIPPER,
    MECHANISM_ACTION_PLATFORM,
    MECHANISM_ACTION_SERVO,
    MECHANISM_ACTION_WAIT
} MechanismActionType;

typedef struct {
    uint8_t type;
    uint8_t channel;
    uint16_t value;
    uint16_t waitMs;
    MechanismPose pose;
} MechanismAction;

#define MECH_INITIAL_STATE(H, L, T, HR, HA, LR, LA, TS, GRIP, PLATFORM, GO, GC, P1, P2, P3) \
    {{(H), (L), (T), (HR), (HA), (LR), (LA), (TS)}, (GRIP), (PLATFORM), \
     (GO), (GC), {(P1), (P2), (P3)}}

#define MECH_POSE(H, L, T, HR, HA, LR, LA, TS, WAIT) \
    {MECHANISM_ACTION_POSE, 0U, 0U, (WAIT), \
     {(H), (L), (T), (HR), (HA), (LR), (LA), (TS)}}
#define MECH_GRIPPER_CLOSE(WAIT) \
    {MECHANISM_ACTION_GRIPPER, 2U, 0U, (WAIT), {0, 0, 0, 0, 0, 0, 0, 0}}
#define MECH_GRIPPER_OPEN(WAIT) \
    {MECHANISM_ACTION_GRIPPER, 2U, 1U, (WAIT), {0, 0, 0, 0, 0, 0, 0, 0}}
#define MECH_PLATFORM(POSITION, WAIT) \
    {MECHANISM_ACTION_PLATFORM, 3U, (POSITION), (WAIT), {0, 0, 0, 0, 0, 0, 0, 0}}
#define MECH_SERVO(CHANNEL, ANGLE, WAIT) \
    {MECHANISM_ACTION_SERVO, (CHANNEL), (ANGLE), (WAIT), {0, 0, 0, 0, 0, 0, 0, 0}}
#define MECH_WAIT(WAIT) \
    {MECHANISM_ACTION_WAIT, 0U, 0U, (WAIT), {0, 0, 0, 0, 0, 0, 0, 0}}

static uint8_t mechanismPoseValid(const MechanismPose *pose)
{
    return pose != (const MechanismPose *)0 &&
           pose->horizontalDmm >= MECHANISM_HORIZONTAL_MIN_DMM &&
           pose->horizontalDmm <= MECHANISM_HORIZONTAL_MAX_DMM &&
           pose->liftDmm <= MECHANISM_LIFT_MAX_DMM &&
           pose->turretDdeg <= MECHANISM_TURRET_MAX_DDEG &&
           pose->horizontalRpm >= MECHANISM_RPM_MIN &&
           pose->horizontalRpm <= MECHANISM_RPM_MAX &&
           pose->liftRpm >= MECHANISM_RPM_MIN &&
           pose->liftRpm <= MECHANISM_RPM_MAX &&
           pose->horizontalAccel >= MECHANISM_ACCEL_MIN &&
           pose->horizontalAccel <= MECHANISM_ACCEL_MAX &&
           pose->liftAccel >= MECHANISM_ACCEL_MIN &&
           pose->liftAccel <= MECHANISM_ACCEL_MAX &&
           pose->turretDps10 >= MECHANISM_TURRET_SPEED_MIN &&
           pose->turretDps10 <= MECHANISM_TURRET_SPEED_MAX;
}

static uint32_t mechanismAbsoluteDelta(int16_t delta)
{
    return delta < 0 ? (uint32_t)(-(int32_t)delta) : (uint32_t)delta;
}

static uint32_t mechanismHorizontalPulses(int16_t deltaDmm)
{
    return (mechanismAbsoluteDelta(deltaDmm) * 25465U + 5000U) / 10000U;
}

static uint32_t mechanismLiftPulses(int16_t deltaDmm)
{
    return mechanismAbsoluteDelta(deltaDmm) * 8U;
}

static uint32_t mechanismMotorDurationMs(uint32_t pulses, uint16_t rpm)
{
    uint32_t denominator = 3200U * (uint32_t)rpm;
    return (pulses * 60000U + denominator - 1U) / denominator + 3000U;
}

static uint32_t mechanismMoveDurationMs(const MechanismPose *current,
                                        const MechanismPose *target)
{
    int16_t horizontalDelta = (int16_t)(target->horizontalDmm -
                                        current->horizontalDmm);
    int16_t liftDelta = (int16_t)((int32_t)target->liftDmm -
                                  (int32_t)current->liftDmm);
    uint16_t turretDelta = target->turretDdeg > current->turretDdeg ?
        (uint16_t)(target->turretDdeg - current->turretDdeg) :
        (uint16_t)(current->turretDdeg - target->turretDdeg);
    uint32_t horizontalMs = mechanismMotorDurationMs(
        mechanismHorizontalPulses(horizontalDelta), target->horizontalRpm);
    uint32_t liftMs = mechanismMotorDurationMs(
        mechanismLiftPulses(liftDelta), target->liftRpm);
    uint32_t turretMs = (uint32_t)turretDelta * 1000U /
                        target->turretDps10 + 500U;
    uint32_t maximum = horizontalMs > liftMs ? horizontalMs : liftMs;
    return maximum > turretMs ? maximum : turretMs;
}

#endif
