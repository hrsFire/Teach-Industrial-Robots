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
        static std::unordered_map<robot_arm::JointName, robot_arm::Joint> CreateJoints(const std::vector<robot_arm::JointName>& jointNames, const std::vector<int>& jointIDs,
            const std::vector<double>& lowerJointLimits, const std::vector<double> upperJointLimits, const std::vector<double>& velocityLimits);
        static void PrepareRobotInfoJoints(const std::vector<std::string>& jointNames, std::vector<robot_arm::JointName>& newJointNames,
            const std::vector<int16_t>& jointIDs, std::vector<int>& newJointIDs, const std::vector<double>& lowerJointLimits,
            std::vector<double>& newLowerJointLimits, const std::vector<double>& upperJointLimits, std::vector<double>& newUpperJointLimits,
            double lowerGripperLimit, double upperGripperLimit, bool useGripper);
        static void PrepareJointStates(std::vector<robot_arm::JointState>* orderedJointStates, std::unordered_map<robot_arm::JointName, robot_arm::JointState>* unorderedJointStates,
            const std::vector<std::string>& jointNames, const std::vector<double>& jointPositions, const std::vector<double>& jointVelocities,
            const std::vector<double>& jointEfforts);
    };
}

#endif //INTERBOTIX_JOINT_HELPER_HPP