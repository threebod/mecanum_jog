#include <assert.h>
#include <stdio.h>

#include "../mechanism_action.h"
#include "../raw_pick_action.h"

int main(void)
{
    MechanismCommand command;
    MechanismState state;
    MechanismPose pose = {-1220, 0, 0, 10, 1, 10, 1, 10};
    MechanismPose fast = {650, 1500, 3600, 2000, 240, 2000, 240, 1800};
    MechanismPose invalid = fast;
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
    invalid.liftAccel = 241;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.turretDps10 = 1801;
    assert(!mechanismPoseValid(&invalid));

    assert(mechanismHorizontalPulses(480) == 1222U);
    assert(mechanismHorizontalPulses(-480) == 1222U);
    assert(mechanismHorizontalPulses(0) == 0U);
    assert(mechanismLiftPulses(250) == 2000U);
    assert(mechanismLiftPulses(-250) == 2000U);
    assert(mechanismMotorDurationMs(0U, 30U) == 0U);
    assert(mechanismMoveDurationMs(&pose, &pose) == 0U);
    assert(mechanismMoveDurationMs(&pose, &fast) == 2100U);

    assert(mechanismParseCommand("mech init 0 0 684", &command));
    assert(command.type == MECHANISM_COMMAND_INIT);
    assert(command.pose.horizontalDmm == 0 && command.pose.turretDdeg == 684U &&
           command.pose.turretDps10 == 1200U);
    assert(mechanismParseCommand(
        "mech pose -480 1500 1350 30 50 30 50 130", &command));
    assert(command.type == MECHANISM_COMMAND_POSE);
    assert(command.pose.horizontalDmm == -480 && command.pose.liftDmm == 1500U);
    assert(mechanismParseCommand("mech status", &command));
    assert(command.type == MECHANISM_COMMAND_STATUS);
    assert(!mechanismParseCommand("mech init -1221 0 0", &command));
    assert(!mechanismParseCommand(
        "mech pose 0 0 0 2001 50 30 50 130", &command));

    mechanismStateReset(&state);
    assert(!state.valid && !state.running);
    mechanismStateInitialize(&state, &pose);
    assert(state.valid && state.current.horizontalDmm == -1220);
    assert(mechanismStateStart(&state, &fast, 100U));
    assert(state.running && mechanismStateService(&state, 299U) == MECHANISM_EVENT_NONE);
    assert(mechanismStateService(&state, 300U) == MECHANISM_EVENT_POSITION);
    assert(state.current.horizontalDmm > pose.horizontalDmm &&
           state.current.horizontalDmm < fast.horizontalDmm);
    assert(state.current.liftDmm > pose.liftDmm &&
           state.current.liftDmm < fast.liftDmm);
    assert(mechanismStateService(&state, state.deadlineMs) == MECHANISM_EVENT_DONE);
    assert(!state.running && state.current.horizontalDmm == 650);
    mechanismStateInvalidate(&state);
    assert(!state.valid && !state.running);

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
    assert(rawPickInitial.gripperOpenDeg == 70U &&
           rawPickInitial.gripperCloseDeg == 35U);
    assert(rawPickInitial.platformDeg[0] == 26U &&
           rawPickInitial.platformDeg[1] == 146U &&
           rawPickInitial.platformDeg[2] == 264U);
    for (index = 0U; index < RAW_PICK_ACTION_COUNT; ++index) {
        if (rawPickActions[index].type == MECHANISM_ACTION_POSE)
            assert(mechanismPoseValid(&rawPickActions[index].pose));
    }
    for (index = 1U; index <= 3U; ++index) {
        rawPickActionsForSlot(slotActions, (uint8_t)index);
        assert(slotActions[0].type == MECHANISM_ACTION_PLATFORM);
        assert(slotActions[0].value == index);
        assert(slotActions[11].value == (index < 3U ? index + 1U : 3U));
        assert(slotActions[4].type == MECHANISM_ACTION_GRIPPER);
        assert(slotActions[9].type == MECHANISM_ACTION_GRIPPER);
    }
    assert(rawPickActions[0].value == 1U && rawPickActions[11].value == 2U);

    puts("PASS: mechanism action math and table types");
    return 0;
}
