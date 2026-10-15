#ifndef MECANUM_JOG_MISSION_ACTIONS_H
#define MECANUM_JOG_MISSION_ACTIONS_H

#include "mechanism_action.h"

/* 动作组/放置.json: three tray slots to three rings. Lift coordinates grow
 * downward; the second storage layer is 650 dmm above the first layer. */
static const MechanismInitialState missionInitial =
    MECH_INITIAL_STATE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 2700, 1800,
                       1, 1, 90, 35, 26, 146, 264);

/* 动作组/放置_升降600rpm无等待.json is the source; firmware lift moves use 2000 RPM / acc 240. */
static const MechanismInitialState missionCoarseInitial =
    MECH_INITIAL_STATE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 2700, 1800,
                       1, 1, 90, 35, 26, 146, 264);

static const MechanismAction missionCoarsePlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-240, 0, 2657, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-240, 1300, 2657, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(435, 0, 2320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(435, 1300, 2320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(55, 0, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(55, 1300, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0)
};

#define MISSION_COARSE_PLACE_COUNT \
    ((uint16_t)(sizeof(missionCoarsePlaceActions) / sizeof(missionCoarsePlaceActions[0])))

static const MechanismAction missionPlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-240, 0, 2657, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-240, 1300, 2657, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(435, 0, 2320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(435, 1300, 2320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(0),
    MECH_POSE(-500, 0, 1320, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(55, 0, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(55, 1300, 3065, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(0, 0, 2720, 1000, 200, 2000, 240, 2200, 0)
};

#define MISSION_PLACE_COUNT \
    ((uint16_t)(sizeof(missionPlaceActions) / sizeof(missionPlaceActions[0])))
#define MISSION_PICK_BACK_COUNT 33U

/* Tray slots preserve task pickup order; rings gives each item's destination. */
static inline void missionBuildTaskPlace(MechanismAction *actions,
                                 const uint8_t rings[3], uint8_t layer,
                                 uint8_t coarse)
{
    static const int16_t ringH[3] = {55, -240, 435};
    static const uint16_t ringT[3] = {3065, 2657, 2320};
    static const uint8_t ringPoses[3][2] = {{7, 8}, {15, 16}, {25, 26}};
    uint8_t item, step, index, ring;
    memcpy(actions, coarse ? missionCoarsePlaceActions : missionPlaceActions,
           coarse ? sizeof(missionCoarsePlaceActions) : sizeof(missionPlaceActions));
    for (item = 0U; item < 3U; ++item) {
        ring = (uint8_t)(rings[item] - 1U);
        actions[item == 0U ? 0U : item == 1U ? 10U : 18U].value = (uint16_t)(item + 1U);
        for (step = 0U; step < 2U; ++step) {
            index = ringPoses[item][step];
            actions[index].pose.turretDdeg = ringT[ring];
            actions[index].pose.horizontalDmm = ringH[ring];
        }
        if (layer == 2U)
            actions[item == 0U ? 8U : item == 1U ? 16U : 26U].pose.liftDmm -= 650U;
    }
    actions[24].pose.turretDdeg = ringT[rings[2] - 1U];
}

/* Lift and retract together after release; finish at the last tray release. */
static void missionBuildTaskPickBack(MechanismAction *actions,
                                    const uint8_t rings[3],
                                    int16_t trayHorizontalDmm)
{
    static const int16_t ringH[3] = {55, -240, 435};
    static const uint16_t ringT[3] = {3065, 2657, 2320};
    uint8_t item, ring;
    uint16_t i = 0U;
    actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 0, 1320, 1000, 200, 2000, 240, 2200, 0);
    for (item = 0U; item < 3U; ++item) {
        ring = (uint8_t)(rings[item] - 1U);
        actions[i++] = (MechanismAction)MECH_PLATFORM(item + 1U, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[ring], 0, ringT[ring], 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[ring], 1300, ringT[ring], 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_CLOSE(0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[ring], 0, ringT[ring], 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(ringH[ring], 0, 1320, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 0, 1320, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 300, 1320, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        if (item < 2U)
            actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 0, 1320, 1000, 200, 2000, 240, 2200, 0);
    }
}

static void missionBuildStoragePlace(MechanismAction *actions, uint8_t layer)
{
    memcpy(actions, missionPlaceActions, sizeof(missionPlaceActions));
    if (layer == 2U) {
        /* Tray slots 1/2/3 correspond to rings 2/3/1. Stack rings 1/2/3. */
        actions[0].value = 3U;
        actions[10].value = 1U;
        actions[18].value = 2U;
        actions[4].pose = missionPlaceActions[20].pose;
        actions[12].pose = missionPlaceActions[4].pose;
        actions[7].pose = missionPlaceActions[25].pose;
        actions[8].pose = missionPlaceActions[26].pose;
        actions[15].pose = missionPlaceActions[7].pose;
        actions[16].pose = missionPlaceActions[8].pose;
        actions[20].pose = missionPlaceActions[12].pose;
        actions[21].pose = missionPlaceActions[12].pose;
        actions[24].pose.turretDdeg = missionPlaceActions[15].pose.turretDdeg;
        actions[25].pose = missionPlaceActions[15].pose;
        actions[26].pose = missionPlaceActions[16].pose;
        actions[8].pose.liftDmm -= 650U;
        actions[16].pose.liftDmm -= 650U;
        actions[26].pose.liftDmm -= 650U;
    }
}

/* 动作组/10_粗加工回收xiugai.json: turn at the raised ring position, then retract. */
static void missionBuildPickBack(MechanismAction *actions)
{
    static const uint8_t rings[3] = {2U, 3U, 1U};
    missionBuildTaskPickBack(actions, rings, -500);
}

#endif
