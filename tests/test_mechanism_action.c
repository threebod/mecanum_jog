#include <assert.h>
#include <stdio.h>

#include "../mechanism_action.h"
#include "../raw_pick_action.h"
#include "../mission_actions.h"

static void assertCoarseMotorSettings(const MechanismPose *pose)
{
    assert(pose->horizontalRpm == missionCoarseInitial.pose.horizontalRpm);
    assert(pose->horizontalAccel == missionCoarseInitial.pose.horizontalAccel);
    assert(pose->liftRpm == missionCoarseInitial.pose.liftRpm);
    assert(pose->liftAccel == missionCoarseInitial.pose.liftAccel);
    assert(pose->turretDps10 == 1200U);
}

int main(void)
{
    MechanismCommand command;
    MechanismState state;
    MechanismPose pose = {-1220, 0, 0, 10, 1, 10, 1, 10};
    MechanismPose fast = {650, 1500, 3600, 1200, 200, 1200, 200, 1800};
    MechanismPose invalid = fast;
    MechanismPose liftStart = {0, 0, 0, 1000, 200, 1000, 200, 1200};
    MechanismPose liftTarget = {0, 1300, 0, 1000, 200, 1000, 200, 1200};
    MechanismInitialState initial = MECH_INITIAL_STATE(
        0, 0, 684, 30, 50, 30, 50, 130, 1800, 1700,
        1, 2, 45, 6, 20, 139, 256);
    MechanismAction actions[] = {
        MECH_POSE(-480, 1500, 1350, 30, 50, 30, 50, 130, 500),
        MECH_GRIPPER_CLOSE(300),
        MECH_GRIPPER_OPEN(300),
        MECH_PLATFORM(3, 400),
        MECH_SERVO(4, 180, 200),
        MECH_WAIT(750)
    };
    MechanismAction slotActions[RAW_PICK_ACTION_COUNT];
    MechanismAction storageActions[MISSION_PLACE_COUNT];
    MechanismAction pickBackActions[MISSION_PICK_BACK_COUNT];
    uint16_t index;

    assert(mechanismPoseValid(&pose));
    assert(mechanismPoseValid(&fast));
    invalid.horizontalDmm = -1221;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.liftDmm = 1501;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.turretDdeg = 3601;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.horizontalRpm = 9;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.liftAccel = 201;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.horizontalRpm = 1201;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.turretDps10 = 1801;
    assert(!mechanismPoseValid(&invalid));

    assert(mechanismHorizontalPulses(480) == 1222U);
    assert(mechanismHorizontalPulses(-480) == 1222U);
    assert(mechanismHorizontalPulses(0) == 0U);
    assert(mechanismLiftPulses(250) == 2000U);
    assert(mechanismLiftPulses(-250) == 2000U);
    assert(mechanismMotorDurationMs(0U, 30U, 50U) == 0U);
    /* 130 mm lift: 10400 pulses, acceleration 200 => 2.8 ms per RPM. */
    assert(mechanismMotorDurationMs(10400U, 1000U, 200U) == 1478U);
    assert(mechanismMotorDurationMs(10400U, 1200U, 200U) == 1478U);
    assert(mechanismMotorDurationMs(320000U, 1000U, 200U) == 8800U);
    assert(mechanismMotorDurationMs(320000U, 1000U, 0U) == 6000U);
    assert(mechanismMoveDurationMs(&pose, &pose) == 0U);
    assert(mechanismMoveDurationMs(&pose, &fast) == 3760U);
    {
        MechanismPose turretStart = {0, 0, 0, 30, 50, 30, 50, 1200};
        MechanismPose turretTarget = turretStart;
        turretTarget.turretDdeg = 900U;
        mechanismStateInitialize(&state, &turretStart);
        assert(mechanismStateStart(&state, &turretTarget, 0U));
        assert(state.durationMs == 1420U);
        assert(mechanismStateService(&state, 850U, 1U) != MECHANISM_EVENT_DONE);
        assert(state.running);
        assert(mechanismStateService(&state, 1420U, 0U) == MECHANISM_EVENT_DONE);
    }

    assert(mechanismParseCommand("mech init 0 0 684", &command));
    assert(command.type == MECHANISM_COMMAND_INIT);
    assert(command.pose.horizontalDmm == 0 && command.pose.turretDdeg == 684U &&
           command.pose.turretDps10 == 1200U);
    assert(mechanismParseCommand(
        "mech pose -480 1500 1350 30 50 30 50 130", &command));
    assert(mechanismParseCommand(
        "mech pose -480 1500 1350 1200 200 1200 200 130", &command));
    assert(command.type == MECHANISM_COMMAND_POSE);
    assert(command.pose.horizontalDmm == -480 && command.pose.liftDmm == 1500U);
    assert(mechanismParseCommand("mech status", &command));
    assert(command.type == MECHANISM_COMMAND_STATUS);
    assert(!mechanismParseCommand("mech init -1221 0 0", &command));
    assert(!mechanismParseCommand(
        "mech pose 0 0 0 1201 50 30 50 130", &command));

    mechanismStateReset(&state);
    assert(!state.valid && !state.running);
    mechanismStateInitialize(&state, &pose);
    assert(state.valid && state.current.horizontalDmm == -1220);
    assert(mechanismStateStart(&state, &fast, 100U));
    assert(state.running && mechanismStateService(&state, 299U, 1U) == MECHANISM_EVENT_NONE);
    assert(mechanismStateService(&state, 300U, 1U) == MECHANISM_EVENT_POSITION);
    assert(state.current.horizontalDmm > pose.horizontalDmm &&
           state.current.horizontalDmm < fast.horizontalDmm);
    assert(state.current.liftDmm > pose.liftDmm &&
           state.current.liftDmm < fast.liftDmm);
    state.pendingMotors = 0U;
    assert(mechanismStateService(&state, 400U, 0U) == MECHANISM_EVENT_DONE);
    assert(!state.running && state.current.horizontalDmm == 650);
    mechanismStateInvalidate(&state);
    assert(!state.valid && !state.running);

    mechanismStateInitialize(&state, &liftStart);
    assert(mechanismStateStart(&state, &liftTarget, 0U));
    assert(mechanismStateService(&state, 695U, 0U) == MECHANISM_EVENT_POSITION);
    assert(state.running); /* Former constant-speed deadline must not finish it. */
    assert(mechanismStateService(&state, 1478U, 0U) != MECHANISM_EVENT_DONE);
    assert(mechanismStateService(&state, state.deadlineMs, 0U) == MECHANISM_EVENT_TIMEOUT);
    assert(!state.valid && !state.running);
    mechanismStateInitialize(&state, &liftStart);
    assert(mechanismStateStart(&state, &liftTarget, 0U));
    state.pendingMotors = 0U;
    assert(mechanismStateService(&state, 100U, 0U) == MECHANISM_EVENT_DONE);

    assert(initial.pose.turretDdeg == 684U);
    assert(initial.gripperDps10 == 1800U && initial.platformDps10 == 1700U);
    assert(initial.gripperOpen == 1U && initial.platform == 2U);
    assert(initial.platformDeg[2] == 256U);
    assert(actions[0].type == MECHANISM_ACTION_POSE);
    assert(actions[0].pose.horizontalDmm == -480);
    assert(actions[0].waitMs == 500U);
    assert(actions[1].type == MECHANISM_ACTION_GRIPPER && actions[1].value == 0U);
    assert(actions[2].type == MECHANISM_ACTION_GRIPPER && actions[2].value == 1U);
    assert(actions[3].type == MECHANISM_ACTION_PLATFORM && actions[3].value == 3U);
    assert(actions[4].type == MECHANISM_ACTION_SERVO && actions[4].channel == 4U);
    assert(actions[5].type == MECHANISM_ACTION_WAIT && actions[5].waitMs == 750U);
    assert(RAW_PICK_ACTION_COUNT == 13U);
    assert(rawPickObservePose.horizontalDmm == -500);
    assertCoarseMotorSettings(&rawPickInitial.pose);
    assertCoarseMotorSettings(&rawPickObservePose);
    assert(rawPickInitial.gripperOpenDeg == 70U &&
           rawPickInitial.gripperCloseDeg == 35U);
    assert(rawPickInitial.platformDeg[0] == 26U &&
           rawPickInitial.platformDeg[1] == 146U &&
           rawPickInitial.platformDeg[2] == 264U);
    for (index = 0U; index < RAW_PICK_ACTION_COUNT; ++index) {
        if (rawPickActions[index].type == MECHANISM_ACTION_POSE) {
            assert(mechanismPoseValid(&rawPickActions[index].pose));
            assertCoarseMotorSettings(&rawPickActions[index].pose);
        }
    }
    for (index = 1U; index <= 3U; ++index) {
        rawPickActionsForSlot(slotActions, (uint8_t)index);
        assert(slotActions[0].type == MECHANISM_ACTION_PLATFORM);
        assert(slotActions[0].value == index);
        assert(slotActions[11].value == (index < 3U ? index + 1U : 3U));
        assert(slotActions[4].type == MECHANISM_ACTION_GRIPPER);
        assert(slotActions[3].pose.liftDmm == 500U);
        assert(slotActions[9].type == MECHANISM_ACTION_GRIPPER);
        {
            uint16_t actionIndex;
            for (actionIndex = 0U; actionIndex < RAW_PICK_ACTION_COUNT; ++actionIndex) {
                if (slotActions[actionIndex].type == MECHANISM_ACTION_POSE)
                    assertCoarseMotorSettings(&slotActions[actionIndex].pose);
            }
        }
    }
    assert(rawPickActions[0].value == 1U && rawPickActions[11].value == 2U);

    assert(MISSION_COARSE_PLACE_COUNT == 32U);
    assert(missionCoarseInitial.pose.horizontalRpm == 80U &&
           missionCoarseInitial.pose.liftRpm == 1000U &&
           missionCoarseInitial.pose.liftAccel == 200U &&
           missionCoarseInitial.pose.turretDps10 == 1800U);
    assert(missionCoarsePlaceActions[8].pose.liftDmm == 1480U);
    assert(missionCoarsePlaceActions[17].pose.liftDmm == 1300U);
    assert(missionCoarsePlaceActions[29].pose.liftDmm == 1300U);
    missionBuildPickBack(pickBackActions);
    for (index = 0U; index < MISSION_PICK_BACK_COUNT; ++index) {
        const MechanismAction *action = &pickBackActions[index];
        assert(action->waitMs == 0U);
        if (action->type == MECHANISM_ACTION_POSE) {
            assert(mechanismPoseValid(&action->pose));
            assert(action->pose.horizontalRpm == 80U &&
                   action->pose.liftRpm == 1000U &&
                   action->pose.liftAccel == 200U &&
                   action->pose.turretDps10 == 1800U);
        }
    }
    for (index = 0U; index < 3U; ++index) {
        const MechanismPose *pick = &pickBackActions[index * 10U + 3U].pose;
        static const uint8_t order[3] = {2U, 0U, 1U};
        uint8_t slot = order[index];
        const MechanismPose *place =
            &missionCoarsePlaceActions[slot == 0U ? 8U : slot == 1U ? 17U : 29U].pose;
        assert(pickBackActions[index * 10U].value == slot + 1U);
        assert(pickBackActions[index * 10U + 7U].pose.liftDmm == (slot == 1U ? 260U : 300U));
        assert(pick->horizontalDmm == place->horizontalDmm &&
               pick->liftDmm == place->liftDmm &&
               pick->turretDdeg == place->turretDdeg);
    }
    for (index = 0U; index < MISSION_COARSE_PLACE_COUNT; ++index) {
        assert(missionCoarsePlaceActions[index].waitMs == 0U);
        if (missionCoarsePlaceActions[index].type == MECHANISM_ACTION_POSE)
            assert(mechanismPoseValid(&missionCoarsePlaceActions[index].pose));
    }
    missionBuildStoragePlace(storageActions, 1U);
    assert(storageActions[8].pose.liftDmm == 1480U &&
           storageActions[17].pose.liftDmm == 1400U &&
           storageActions[29].pose.liftDmm == 1450U);
    assert(MISSION_PLACE_COUNT == 33U);
    assert(rawPickInitial.platformDps10 == 1800U && missionInitial.platformDps10 == 1800U &&
           missionCoarseInitial.platformDps10 == 1800U);
    assert(missionCoarsePlaceActions[MISSION_COARSE_PLACE_COUNT - 1U].pose.turretDdeg == 3080U);
    assert(missionCoarsePlaceActions[MISSION_COARSE_PLACE_COUNT - 1U].pose.horizontalDmm == 50);
    for (index = 0U; index < MISSION_PLACE_COUNT; ++index) {
        assert(storageActions[index].waitMs == 0U);
        assert(storageActions[index].type != MECHANISM_ACTION_WAIT);
    }
    assertCoarseMotorSettings(&missionInitial.pose);
    for (index = 0U; index < MISSION_PLACE_COUNT; ++index) {
        if (storageActions[index].type == MECHANISM_ACTION_POSE) {
            assert(mechanismPoseValid(&storageActions[index].pose));
            assertCoarseMotorSettings(&storageActions[index].pose);
        }
    }
    missionBuildStoragePlace(storageActions, 2U);
    assert(storageActions[8].pose.liftDmm == 880U &&
           storageActions[17].pose.liftDmm == 800U &&
           storageActions[29].pose.liftDmm == 850U);
    for (index = 0U; index < MISSION_PLACE_COUNT; ++index) {
        if (storageActions[index].type == MECHANISM_ACTION_POSE) {
            assert(mechanismPoseValid(&storageActions[index].pose));
            assertCoarseMotorSettings(&storageActions[index].pose);
        }
    }

    puts("PASS: mechanism action math and table types");
    return 0;
}
