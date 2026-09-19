#ifndef MECANUM_JOG_MECHANISM_ACTION_H
#define MECANUM_JOG_MECHANISM_ACTION_H

#include <stdint.h>
#include <string.h>

#define MECHANISM_HORIZONTAL_MIN_DMM (-1220)
#define MECHANISM_HORIZONTAL_MAX_DMM 650
#define MECHANISM_LIFT_MAX_DMM 1350U
#define MECHANISM_TURRET_MAX_DDEG 3600U
#define MECHANISM_RPM_MIN 10U
#define MECHANISM_RPM_MAX 2000U
#define MECHANISM_ACCEL_MIN 1U
#define MECHANISM_ACCEL_MAX 240U
#define MECHANISM_TURRET_SPEED_MIN 10U
#define MECHANISM_TURRET_SPEED_MAX 1800U
#define MECHANISM_MOTOR_SETTLE_MS 500U
#define MECHANISM_TURRET_SETTLE_MS 100U

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
    uint16_t gripperDps10;
    uint16_t platformDps10;
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

typedef enum {
    MECHANISM_COMMAND_NONE = 0,
    MECHANISM_COMMAND_INIT,
    MECHANISM_COMMAND_POSE,
    MECHANISM_COMMAND_STATUS
} MechanismCommandType;

typedef struct {
    uint8_t type;
    MechanismPose pose;
} MechanismCommand;

typedef enum {
    MECHANISM_EVENT_NONE = 0,
    MECHANISM_EVENT_POSITION,
    MECHANISM_EVENT_DONE
} MechanismEvent;

typedef struct {
    uint8_t valid;
    uint8_t running;
    MechanismPose start;
    MechanismPose current;
    MechanismPose target;
    uint32_t startMs;
    uint32_t durationMs;
    uint32_t deadlineMs;
    uint32_t lastReportMs;
} MechanismState;

#define MECH_INITIAL_STATE(H, L, T, HR, HA, LR, LA, TS, GS, PS, GRIP, PLATFORM, GO, GC, P1, P2, P3) \
    {{(H), (L), (T), (HR), (HA), (LR), (LA), (TS)}, (GRIP), (PLATFORM), \
     (GO), (GC), (GS), (PS), {(P1), (P2), (P3)}}

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
    if (pulses == 0U) return 0U;
    return (pulses * 60000U + denominator - 1U) / denominator +
           MECHANISM_MOTOR_SETTLE_MS;
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
    uint32_t turretMs = turretDelta == 0U ? 0U :
        (uint32_t)turretDelta * 1000U / target->turretDps10 +
        MECHANISM_TURRET_SETTLE_MS;
    uint32_t maximum = horizontalMs > liftMs ? horizontalMs : liftMs;
    return maximum > turretMs ? maximum : turretMs;
}

static uint8_t mechanismParseUnsigned(const char **cursor, uint16_t *value)
{
    uint32_t result = 0U;
    uint8_t digits = 0U;
    while (**cursor == ' ') ++*cursor;
    while (**cursor >= '0' && **cursor <= '9') {
        result = result * 10U + (uint32_t)(**cursor - '0');
        if (result > 65535U) return 0U;
        ++*cursor;
        ++digits;
    }
    if (!digits) return 0U;
    *value = (uint16_t)result;
    return 1U;
}

static uint8_t mechanismParseSigned(const char **cursor, int16_t *value)
{
    uint8_t negative = 0U;
    uint16_t magnitude;
    while (**cursor == ' ') ++*cursor;
    if (**cursor == '-') {
        negative = 1U;
        ++*cursor;
    }
    if (!mechanismParseUnsigned(cursor, &magnitude) ||
        magnitude > (negative ? 32768U : 32767U)) return 0U;
    *value = negative ? (int16_t)(-(int32_t)magnitude) : (int16_t)magnitude;
    return 1U;
}

static uint8_t mechanismAtEnd(const char *cursor)
{
    while (*cursor == ' ') ++cursor;
    return *cursor == '\0';
}

static uint8_t mechanismParseCommand(const char *text, MechanismCommand *command)
{
    const char *cursor;
    uint16_t lift, turret, horizontalRpm, horizontalAccel;
    uint16_t liftRpm, liftAccel, turretSpeed;
    int16_t horizontal;
    MechanismPose pose;
    if (text == (const char *)0 || command == (MechanismCommand *)0) return 0U;
    if (strcmp(text, "mech status") == 0) {
        command->type = MECHANISM_COMMAND_STATUS;
        return 1U;
    }
    if (strncmp(text, "mech init ", 10U) == 0) {
        cursor = text + 10U;
        if (!mechanismParseSigned(&cursor, &horizontal) ||
            !mechanismParseUnsigned(&cursor, &lift) ||
            !mechanismParseUnsigned(&cursor, &turret) || !mechanismAtEnd(cursor)) {
            return 0U;
        }
        pose.horizontalDmm = horizontal;
        pose.liftDmm = lift;
        pose.turretDdeg = turret;
        pose.horizontalRpm = 30U;
        pose.horizontalAccel = 50U;
        pose.liftRpm = 30U;
        pose.liftAccel = 50U;
        pose.turretDps10 = 1200U;
        if (!mechanismPoseValid(&pose)) return 0U;
        command->type = MECHANISM_COMMAND_INIT;
        command->pose = pose;
        return 1U;
    }
    if (strncmp(text, "mech pose ", 10U) != 0) return 0U;
    cursor = text + 10U;
    if (!mechanismParseSigned(&cursor, &horizontal) ||
        !mechanismParseUnsigned(&cursor, &lift) ||
        !mechanismParseUnsigned(&cursor, &turret) ||
        !mechanismParseUnsigned(&cursor, &horizontalRpm) ||
        !mechanismParseUnsigned(&cursor, &horizontalAccel) ||
        !mechanismParseUnsigned(&cursor, &liftRpm) ||
        !mechanismParseUnsigned(&cursor, &liftAccel) ||
        !mechanismParseUnsigned(&cursor, &turretSpeed) || !mechanismAtEnd(cursor)) {
        return 0U;
    }
    pose.horizontalDmm = horizontal;
    pose.liftDmm = lift;
    pose.turretDdeg = turret;
    pose.horizontalRpm = horizontalRpm;
    pose.horizontalAccel = (uint8_t)horizontalAccel;
    pose.liftRpm = liftRpm;
    pose.liftAccel = (uint8_t)liftAccel;
    pose.turretDps10 = turretSpeed;
    if (horizontalAccel > 255U || liftAccel > 255U || !mechanismPoseValid(&pose)) {
        return 0U;
    }
    command->type = MECHANISM_COMMAND_POSE;
    command->pose = pose;
    return 1U;
}

static void mechanismStateReset(MechanismState *state)
{
    memset(state, 0, sizeof(*state));
}

static void mechanismStateInitialize(MechanismState *state,
                                     const MechanismPose *pose)
{
    state->valid = 1U;
    state->running = 0U;
    state->current = *pose;
    state->target = *pose;
}

static uint8_t mechanismStateStart(MechanismState *state,
                                   const MechanismPose *target,
                                   uint32_t nowMs)
{
    if (!state->valid || state->running || !mechanismPoseValid(target)) return 0U;
    state->target = *target;
    state->start = state->current;
    state->running = 1U;
    state->startMs = nowMs;
    state->lastReportMs = nowMs;
    state->durationMs = mechanismMoveDurationMs(&state->current, target);
    state->deadlineMs = nowMs + state->durationMs;
    return 1U;
}

static uint8_t mechanismStateService(MechanismState *state, uint32_t nowMs)
{
    if (!state->running) return MECHANISM_EVENT_NONE;
    if ((int32_t)(nowMs - state->deadlineMs) >= 0) {
        state->current = state->target;
        state->running = 0U;
        return MECHANISM_EVENT_DONE;
    }
    if (nowMs - state->lastReportMs >= 200U) {
        uint32_t elapsed = nowMs - state->startMs;
        int32_t horizontalDelta = (int32_t)state->target.horizontalDmm -
                                  state->start.horizontalDmm;
        int32_t liftDelta = (int32_t)state->target.liftDmm -
                            state->start.liftDmm;
        int32_t turretDelta = (int32_t)state->target.turretDdeg -
                              state->start.turretDdeg;
        state->current.horizontalDmm = (int16_t)(state->start.horizontalDmm +
            horizontalDelta * (int32_t)elapsed / (int32_t)state->durationMs);
        state->current.liftDmm = (uint16_t)(state->start.liftDmm +
            liftDelta * (int32_t)elapsed / (int32_t)state->durationMs);
        state->current.turretDdeg = (uint16_t)(state->start.turretDdeg +
            turretDelta * (int32_t)elapsed / (int32_t)state->durationMs);
        state->lastReportMs = nowMs;
        return MECHANISM_EVENT_POSITION;
    }
    return MECHANISM_EVENT_NONE;
}

static void mechanismStateInvalidate(MechanismState *state)
{
    state->valid = 0U;
    state->running = 0U;
}

uint8_t mechanismActionStart(const MechanismInitialState *initial,
                             const MechanismAction *actions,
                             uint16_t count);
void mechanismActionService(void);

#endif
