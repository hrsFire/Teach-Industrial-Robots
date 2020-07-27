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
    /**
     * This class provides an abstraction for different robot arms. This class must be extended for new robot arms to implement their capabilities.
     */
    class RobotArmBase {
    public:
        /**
         * Reads the current joint states from all the motors.
         * @return A hash map with all joints and their joint states. The joint name is the key for the specific joint state.
         */
        virtual std::unordered_map<JointNameImpl, JointState> GetJointStates() = 0;
        /**
         * Gets the current pose of the given joint name of the end effector.
         * @param[in]  endEffectorJointName  The joint name of the end effector whose pose should be retrieved.
         * @param[out] pose                  Retrieved pose of the end effector.
         * @return True if the pose could have been retrieved. Otherwise False is returned.
         */
        virtual bool GetCurrentPose(const JointName& endEffectorJointName, geometry_msgs::Pose& pose) = 0;
        /**
         * Sends any type of command to a specified joint which was previously configured with an operating mode.
         * @param[in] jointName  The joint name whose value should be changed.
         * @param     value      The value which should be set for the given joint.
         * @return void
         */
        virtual void SendJointCommand(const JointName& jointName, double value) = 0;
        /**
         * Sends any type of commands to the specified joints which were previously configured with an operating mode.
         * @param[in] jointValues  The joint names and their values which should be changed.
         * @return void
         */
        virtual void SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) = 0;
        /**
         * Sends a joint trajectory to the robot arm. This excludes the gripper value.
         * @param[in] jointTrajectoryPoints  The joint names and their specific joint trajectory point.
         * @return void
         */
        virtual void SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) = 0;
        /**
         * Sends any type of gripper command to the gripper which was previously configured with an operating mode.
         * @param value  The value which should be set for the gripper.
         * @return void
         */
        virtual void SendGripperCommand(double value) = 0;
        /**
         * Sends a gripper trajectory to the robot arm.
         * @param[in] trajectoryPoints The specific trajectory points.
         * @return void
         */
        virtual void SendGripperTrajectory(const std::vector<JointTrajectoryPoint>& trajectoryPoints) = 0;
        /**
         * Sends the pose for the specified end effector.
         * @param[in] pose                 The pose which should be archieved with the end effector.
         * @param[in] endEffectorJointName The joint name for the end effector.
         * @return void
         */
        virtual void SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) = 0;
        /**
         * Sets the operating mode and if necessary a custom profile for a specified joint.
         * @param[in] operatingMode        The operating mode which should be set for the joint.
         * @param[in] affectedJoints       The affected joints whose operating mode should be set.
         * @param[in] jointName            The joint name whose operating mode should be set if the affected joint is set to SINGLE_JOINT.
         * @param     useCustomProfiles    Specifies if a custom profile should be used.
         * @param     profileVelocity      Sets the velocity for the custom profile if the useCustomProfiles parameter is set to True.
         * @param     profileAcceleration  Sets the acceleration for the custom profile if the useCustomProfiles parameter is set to True.
         * @return void
         */
        virtual void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) = 0;
        /**
         * Gets detailed information about the robot arm.
         * @return The detailed information about the robot arm.
         */
        virtual std::shared_ptr<RobotInfo> GetRobotInfo() = 0;
        /**
         * Calculates the distance for the given joint with an internally available acceleration and the given duration. The robot arm takes care of the specific operating mode.
         * @param[in] jointName The joint name whose distance should be calculated.
         * @param[in] duration  The duration in milliseconds of the movement so far.
         * @return The distance which was internally calculated with a valid acceleration. The distance is in radians if the position mode is set.
         */
        virtual double CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) = 0;
        /**
         * Calculates the distance for the given joint with an internally available acceleration and the given duration for a linear movement. The robot arm takes care of the specific operating mode.
         * @param[in] duration  The duration in milliseconds of the movement so far.
         * @return The distance which was internally calculated with a valid acceleration. The distance is in meters.
         */
        virtual double CalculateAccelerationDistance(const std::chrono::milliseconds& duration) = 0;
    };
}

#endif //ROBOT_ARM_ROBOT_ARM_BASE_HPP
