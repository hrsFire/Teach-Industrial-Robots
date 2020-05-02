#ifndef INTERBOTIX_JOINT_HELPER_HPP
#define INTERBOTIX_JOINT_HELPER_HPP

#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_trajectory_point.hpp>
#include <robot_arm_interface/robot_info.hpp>
#include <robot_arm_interface/joint_state.hpp>
#include <robot_arm_interface/affected_joints.hpp>
#include <interbotix_sdk/OperatingModes.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace interbotix {
    class JointHelper {
    public:
        static std::vector<double> PrepareJointCommands(const std::vector<robot_arm::JointName>& jointNames, const std::vector<double>& values,
            robot_arm::RobotInfo& robotInfo, const std::vector<robot_arm::JointState>& jointStates);
        static int8_t GetAffectedJoints(robot_arm::AffectedJoints affectedJoints);
        static void CopyToJointTrajectoryMessage(const std::vector<robot_arm::JointName>& jointNames, const std::vector<robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints,
            trajectory_msgs::JointTrajectory& message);
    };
}

#endif //INTERBOTIX_JOINT_HELPER_HPP