#ifndef ROBOT_ARM_ROBOT_ARM_BASE_HPP
#define ROBOT_ARM_ROBOT_ARM_BASE_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include "joint_trajectory_point.hpp"
#include "joint_state.hpp"
#include "operating_mode.hpp"
#include "joint_name.hpp"
#include "joint_name_impl.hpp"
#include "robot_info.hpp"
#include "affected_joints.hpp"

namespace robot_arm {
    class RobotArmBase {
    public:
        // Reads current joint states from all the motors
        virtual std::unordered_map<JointNameImpl, JointState> GetJointStates() = 0;
        // Gets the current pose
        virtual bool GetCurrentPose(const JointName& endEffectorJointName, geometry_msgs::Pose& pose) = 0;
        // Sends any type of command to a specified joint which was previously configured with an operating mode
        virtual void SendJointCommand(const JointName& jointName, double value) = 0;
        // Sends any type of commands to all joints which were previously configured with an operating mode
        virtual void SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) = 0;
        // Sends joint trajectory for the robot arm (excludes gripper)
        virtual void SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) = 0;
        // Sends any type of gripper command to a gripper which was previously configured with an operating mode
        virtual void SendGripperCommand(double value) = 0;
        // Sends joint trajectory for the gripper only
        virtual void SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) = 0;
        // Sends pose for the specified end effector
        virtual void SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) = 0;
        // Sets the operating mode for a specified joint
        virtual void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) = 0;
        // Get information about the robot arm
        virtual std::shared_ptr<RobotInfo> GetRobotInfo() = 0;
        // Calculate the distance with the available acceleration and duration for the currently active joint. The robot arm takes care of the specific operating mode.
        virtual double CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) = 0;
        // Calculate the distance with the available acceleration and duration for linear movements. The robot arm takes care of the specific operating mode.
        virtual double CalculateAccelerationDistance(const std::chrono::milliseconds& duration) = 0;
    };
}

#endif //ROBOT_ARM_ROBOT_ARM_BASE_HPP