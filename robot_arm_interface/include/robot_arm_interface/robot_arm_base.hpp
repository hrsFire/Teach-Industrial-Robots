#ifndef INTERBOTIX_INTERBOTIX_ROBOT_ARM_BASE_HPP
#define INTERBOTIX_INTERBOTIX_ROBOT_ARM_BASE_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <chrono>
#include "joint_trajectory_point.hpp"
#include "joint_state.hpp"
#include "operating_mode.hpp"
#include "joint_name.hpp"
#include "robot_info.hpp"
#include "affected_joints.hpp"

namespace robot_arm {
    class InterbotixRobotArmBase {
    public:
        // Reads current joint states from all the motors
        virtual std::unordered_map<JointName, JointState> GetJointStates() = 0;
        // Sends any type of command to a specified joint which was previously configured with an operating mode
        virtual void SendJointCommand(const JointName& jointName, double value) = 0;
        // Sends any type of commands to all joints which were previously configured with an operating mode
        virtual void SendJointCommands(const std::vector<JointName>& jointNames, const std::vector<double>& values) = 0;
        // Sends joint trajectory for the robot arm (excludes gripper)
        virtual void SendJointTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) = 0;
        // Sends any type of gripper command to a gripper which was previously configured with an operating mode
        virtual void SendGripperCommand(double value) = 0;
        // Sends joint trajectory for the gripper only
        virtual void SendGripperTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) = 0;
        // Sets the torque state
        virtual void SetTorqueState(bool on) = 0;
        // Sets the operating mode for a specified joint
        virtual void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) = 0;
        // Get information about the robot arm
        virtual std::shared_ptr<RobotInfo> GetRobotInfo() = 0;
        // The robot arm takes care of the specific operating mode
        virtual double CalculateAcceleration(const JointName& jointName, std::chrono::milliseconds duration) = 0;
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_BASE_HPP