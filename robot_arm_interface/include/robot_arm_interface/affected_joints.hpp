#ifndef ROBOT_ARM_AFFECTED_JOINTS_HPP
#define ROBOT_ARM_AFFECTED_JOINTS_HPP

namespace robot_arm {
    enum class AffectedJoints {
        ARM_JOINTS_AND_GRIPPER,
        ARM_JOINTS,
        GRIPPER,
        SINGLE_JOINT,
    };
}

#endif //ROBOT_ARM_AFFECTED_JOINTS_HPP