#include <assert.h>
#include <stdio.h>

#include "../mechanism_action.h"

int main(void)
{
    MechanismPose pose = {-1220, 0, 0, 10, 1, 10, 1, 10};
    MechanismPose fast = {650, 1350, 3600, 2000, 240, 2000, 240, 300};
    MechanismPose invalid = fast;
    MechanismInitialState initial = MECH_INITIAL_STATE(
        0, 0, 684, 30, 50, 30, 50, 130, 1, 2, 45, 6, 20, 139, 256);
    MechanismAction actions[] = {
        MECH_POSE(-480, 250, 1350, 30, 50, 30, 50, 130, 500),
        MECH_GRIPPER_CLOSE(300),
        MECH_GRIPPER_OPEN(300),
        MECH_PLATFORM(3, 400),
        MECH_SERVO(4, 180, 200),
        MECH_WAIT(750)
    };

    assert(mechanismPoseValid(&pose));
    assert(mechanismPoseValid(&fast));
    invalid.horizontalDmm = -1221;
    assert(!mechanismPoseValid(&invalid));
    invalid = fast;
    invalid.liftDmm = 1351;
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
    invalid.turretDps10 = 301;
    assert(!mechanismPoseValid(&invalid));

    assert(mechanismHorizontalPulses(480) == 1222U);
    assert(mechanismHorizontalPulses(-480) == 1222U);
    assert(mechanismHorizontalPulses(0) == 0U);
    assert(mechanismLiftPulses(250) == 2000U);
    assert(mechanismLiftPulses(-250) == 2000U);
    assert(mechanismMoveDurationMs(&pose, &fast) > 3000U);

    assert(initial.pose.turretDdeg == 684U);
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

    puts("PASS: mechanism action math and table types");
    return 0;
}
