#ifndef MECANUM_JOG_RAW_PICK_ACTION_H
#define MECANUM_JOG_RAW_PICK_ACTION_H

#include "mechanism_action.h"

/* Single-item bench sequence from 动作组/夹取.json. The operator places the
 * selected color at this sequence's fixed pickup position. */
static const MechanismInitialState rawPickInitial =
    MECH_INITIAL_STATE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 2700, 1800,
                       1, 1, 90, 35, 26, 146, 264);

static const MechanismPose rawPickObservePose =
    { -770, 0, 2720, 1000, 200, 2000, 240, 2200 };

static const MechanismPose rawPickLowerPose =
    { -770, 500, 2720, 1000, 200, 2000, 240, 2200 };

static const MechanismPose rawPickTravelPose =
    { 0, 0, 650, 1000, 200, 2000, 240, 2200 };

static const MechanismAction rawPickActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-200, 500, 2680, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-200, 0, 2680, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 250, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 0)
};

#define RAW_PICK_ACTION_COUNT \
    ((uint16_t)(sizeof(rawPickActions) / sizeof(rawPickActions[0])))

static void rawPickActionsForSlot(MechanismAction *actions, uint8_t slot)
{
    memcpy(actions, rawPickActions, sizeof(rawPickActions));
    actions[0].value = slot;
    actions[10].value = slot < 3U ? (uint16_t)(slot + 1U) : 3U;
    if (slot == 3U) actions[11].pose = rawPickTravelPose;
}

#endif
