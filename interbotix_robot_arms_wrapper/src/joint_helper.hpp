#ifndef INTERBOTIX_JOINT_HELPER_HPP
#define INTERBOTIX_JOINT_HELPER_HPP

#include <memory>
#include <chrono>
#include <unordered_map>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>
#include <robot_arm_interface/joint_trajectory_point.hpp>
#include <robot_arm_interface/robot_info.hpp>
#include <robot_arm_interface/joint_state.hpp>
#include <robot_arm_interface/affected_joints.hpp>
#include <robot_arm_interface/operating_mode.hpp>
#include <interbotix_sdk/OperatingModes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include "interbotix_robot_arms_wrapper/interbotix_joint_name.hpp"

namespace interbotix {
    class JointHelper {
    public:
        static std::vector<double> PrepareJointCommands(const std::unordered_map<robot_arm::JointNameImpl, double>& jointValues, robot_arm::RobotInfo& robotInfo,
            const std::vector<robot_arm::JointState>& jointStates);
        static int8_t GetAffectedJoints(const robot_arm::AffectedJoints& affectedJoints);
        static void CopyToJointTrajectoryMessage(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints,
            trajectory_msgs::JointTrajectory& message);
        static void CopyToJointTrajectoryMessage(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints,
            control_msgs::FollowJointTrajectoryGoal& message);
        static std::unordered_map<robot_arm::JointNameImpl, robot_arm::Joint> CreateJoints(const std::vector<robot_arm::JointNameImpl>& jointNames,
            const std::vector<int>& jointIDs, const std::vector<double>& lowerJointLimits, const std::vector<double> upperJointLimits, const std::vector<double>& velocityLimits);
        static void PrepareRobotInfoJoints(const std::vector<std::string>& jointNames, std::vector<robot_arm::JointNameImpl>& newJointNames,
            const std::vector<int16_t>& jointIDs, std::vector<int>& newJointIDs, const std::vector<double>& homePosition,
            std::unordered_map<robot_arm::JointNameImpl, double>& newHomePosition, const std::vector<double>& sleepPosition,
            std::unordered_map<robot_arm::JointNameImpl, double>& newSleepPosition, const std::vector<double>& lowerJointLimits,
            std::vector<double>& newLowerJointLimits, const std::vector<double>& upperJointLimits, std::vector<double>& newUpperJointLimits,
            double lowerGripperLimit, double upperGripperLimit, bool useGripper, InterbotixJointName::DOF dof);
        static void PrepareJointStates(std::vector<robot_arm::JointState>* orderedJointStates, std::unordered_map<robot_arm::JointNameImpl,
            robot_arm::JointState>* unorderedJointStates, const std::vector<std::string>& jointNames, const std::vector<double>& jointPositions, const std::vector<double>& jointVelocities,
            const std::vector<double>& jointEfforts, const std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes,
            InterbotixJointName::DOF dof, std::chrono::system_clock::time_point& jointStatesLastChanged);
        static std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode> GetInitialOperatingModes(InterbotixJointName::DOF dof);
        static void SetOperatingMode(std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes, const robot_arm::JointName& jointName,
            const robot_arm::OperatingMode operatingMode);
        static double CalculateAccelerationDistance(const robot_arm::JointName& jointName, const robot_arm::OperatingMode& operatingMode,
            const std::chrono::milliseconds& duration);
        static double CalculateAccelerationDistance(const std::chrono::milliseconds& duration);
        static InterbotixJointName::DOF DetermineDOF(const robot_arm::RobotInfo& robotInfo);
        static InterbotixJointName::DOF DetermineDOF(uint numberOfJoints);
        static void SetJointState(const robot_arm::JointNameImpl& newJointStateName, double newJointStateValue, std::vector<robot_arm::JointState>& orderedJointStates,
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>& unorderedJointStates, std::chrono::high_resolution_clock::time_point& jointStatesLastChanged,
            const std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes);
        static void SetJointStates(const std::unordered_map<robot_arm::JointNameImpl, double>& newJointStates, std::vector<robot_arm::JointState>& orderedJointStates,
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>& unorderedJointStates, std::chrono::high_resolution_clock::time_point& jointStatesLastChanged,
            const std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes);
        // If true: The joint value is valid. Otherwise: The joint value is invalid and a corrected value is stored in jointValue.
        static bool CheckJointValue(const robot_arm::JointNameImpl& jointName, double& jointValue, const robot_arm::RobotInfo& robotInfo);
        static void CheckJointValues(std::unordered_map<robot_arm::JointNameImpl, double>& jointValues, const robot_arm::RobotInfo& robotInfo);
        static size_t GetJointIndex(const std::vector<robot_arm::JointState> orderedJointStates, const robot_arm::JointName& jointName);
    private:
        static bool HaveJointStatesExpired(const std::chrono::high_resolution_clock::time_point& jointStatesLastChanged);
        static constexpr double GRIPPER_CHANGE = 0.002;
        static constexpr double JOINT_ANGLE_CHANGE = 0.5 * M_PI / 180.0; // = 0.4999999994299069 degrees
        static constexpr double LINEAR_MOVEMENT_CHANGE = 0.0005; // 0.5 mm
    };
}

#endif //INTERBOTIX_JOINT_HELPER_HPP