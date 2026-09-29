#ifndef MECANUM_JOG_RAW_PICK_ACTION_H
#define MECANUM_JOG_RAW_PICK_ACTION_H

#include "mechanism_action.h"

/* Single-item bench sequence from 动作组/夹取.json. The operator places the
 * selected color at this sequence's fixed pickup position. */
static const MechanismInitialState rawPickInitial =
    MECH_INITIAL_STATE(0, 0, 2700, 40, 50, 90, 50, 1200, 1200, 1200,
                       1, 1, 70, 35, 26, 146, 264);

static const MechanismPose rawPickObservePose =
    { -500, 0, 2700, 40, 50, 90, 50, 1200 };

static const MechanismAction rawPickActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-200, 0, 2680, 40, 50, 90, 50, 1200, 0),
    MECH_POSE(-200, 1300, 2680, 40, 50, 90, 50, 1200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-200, 0, 2680, 40, 50, 90, 50, 1200, 0),
    MECH_POSE(-200, 0, 1320, 40, 50, 90, 50, 1200, 0),
    MECH_POSE(-500, 0, 1320, 50, 50, 30, 50, 1200, 0),
    MECH_POSE(-500, 250, 1320, 30, 50, 50, 50, 1200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 30, 50, 60, 50, 1200, 0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(0, 0, 2700, 30, 50, 90, 50, 1200, 0)
};

#define RAW_PICK_ACTION_COUNT \
    ((uint16_t)(sizeof(rawPickActions) / sizeof(rawPickActions[0])))

static void rawPickActionsForSlot(MechanismAction *actions, uint8_t slot)
{
    memcpy(actions, rawPickActions, sizeof(rawPickActions));
    actions[0].value = slot;
    actions[11].value = slot < 3U ? (uint16_t)(slot + 1U) : 3U;
}

#endif
