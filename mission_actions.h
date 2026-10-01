#ifndef MECANUM_JOG_MISSION_ACTIONS_H
#define MECANUM_JOG_MISSION_ACTIONS_H

#include "mechanism_action.h"

/* 动作组/放置.json: three tray slots to three rings. Lift coordinates grow
 * downward; the second storage layer is 600 dmm above the first layer. */
static const MechanismInitialState missionInitial =
    MECH_INITIAL_STATE(0, 0, 2700, 80, 50, 1000, 200, 1200, 1200, 1800,
                       1, 1, 70, 35, 26, 146, 264);

/* 动作组/放置_升降600rpm无等待.json contains 1000 RPM lift moves. */
static const MechanismInitialState missionCoarseInitial =
    MECH_INITIAL_STATE(0, 0, 2700, 80, 50, 1000, 200, 1800, 1200, 1800,
                       1, 1, 70, 35, 26, 146, 264);

static const MechanismAction missionCoarsePlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2700, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-240, 0, 2670, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-240, 1480, 2670, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-240, 0, 2670, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 260, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(500, 0, 2320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(500, 1300, 2320, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(500, 0, 2320, 80, 50, 1000, 200, 1800, 0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(-500, 0, 3080, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(50, 0, 3080, 80, 50, 1000, 200, 1800, 0),
    MECH_POSE(50, 1300, 3080, 80, 50, 1000, 200, 1800, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(50, 0, 3080, 80, 50, 1000, 200, 1800, 0)
};

#define MISSION_COARSE_PLACE_COUNT \
    ((uint16_t)(sizeof(missionCoarsePlaceActions) / sizeof(missionCoarsePlaceActions[0])))

static const MechanismAction missionPlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2700, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-240, 0, 2670, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-240, 1480, 2670, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-240, 0, 2670, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 260, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(500, 0, 2320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(500, 1400, 2320, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(500, 0, 2320, 80, 50, 1000, 200, 1200, 0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 300, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(-500, 0, 3080, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(50, 0, 3080, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(50, 1450, 3080, 80, 50, 1000, 200, 1200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(50, 0, 3080, 80, 50, 1000, 200, 1200, 0),
    MECH_POSE(0, 0, 2700, 80, 50, 1000, 200, 1200, 0)
};

#define MISSION_PLACE_COUNT \
    ((uint16_t)(sizeof(missionPlaceActions) / sizeof(missionPlaceActions[0])))
#define MISSION_PICK_BACK_COUNT 32U

static void missionBuildStoragePlace(MechanismAction *actions, uint8_t layer)
{
    memcpy(actions, missionPlaceActions, sizeof(missionPlaceActions));
    if (layer == 2U) {
        actions[8].pose.liftDmm -= 600U;
        actions[17].pose.liftDmm -= 600U;
        actions[29].pose.liftDmm -= 600U;
    }
}

/* Reverse the known ring and tray positions. These inferred motions require
 * individual bench validation before the mission may run on the vehicle. */
static void missionBuildPickBack(MechanismAction *actions)
{
    static const int16_t ringH[3] = {-240, 500, 50};
    static const uint16_t ringT[3] = {2670, 2320, 3080};
    static const uint16_t ringL[3] = {1480, 1300, 1300};
    static const uint16_t trayL[3] = {300, 260, 300};
    static const uint8_t order[3] = {2U, 0U, 1U};
    uint8_t item, slot;
    uint16_t i = 0U;
    for (item = 0U; item < 3U; ++item) {
        slot = order[item];
        actions[i++] = (MechanismAction)MECH_PLATFORM(slot + 1U, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[slot], 0, ringT[slot], 80, 50, 1000, 200, 1800, 0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[slot], ringL[slot], ringT[slot], 80, 50, 1000, 200, 1800, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_CLOSE(0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[slot], 0, ringT[slot], 80, 50, 1000, 200, 1800, 0);
        actions[i++] = (MechanismAction)MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0);
        actions[i++] = (MechanismAction)MECH_POSE(-500, trayL[slot], 1320, 80, 50, 1000, 200, 1800, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        actions[i++] = (MechanismAction)MECH_POSE(-500, 0, 1320, 80, 50, 1000, 200, 1800, 0);
    }
    actions[i++] = (MechanismAction)MECH_POSE(0, 0, 2700, 80, 50, 1000, 200, 1800, 0);
    actions[i] = (MechanismAction)MECH_GRIPPER_OPEN(0);
}

#endif
