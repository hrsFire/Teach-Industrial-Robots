#ifndef INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP
#define INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP

#include <mutex>
#include <interbotix_sdk/arm_obj.h>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <interbotix_robot_arms_wrapper/interbotix_joint_name.hpp>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include "joint_helper.hpp"

using namespace robot_arm;

namespace interbotix {
    class InterbotixRobotArmDirect : public RobotArmBase {
    public:
        InterbotixRobotArmDirect(int argc, char** argv, std::string robotName, std::string robotModel);
        ~InterbotixRobotArmDirect();
        std::unordered_map<JointNameImpl, JointState> GetJointStates() override;
        bool GetCurrentPose(const JointName& endEffectorJointName, geometry_msgs::Pose& pose) override;
        void SendJointCommand(const JointName& jointName, double value) override;
        void SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) override;
        void SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::vector<JointTrajectoryPoint>& trajectoryPoints) override;
        void SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) override;
        void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<RobotInfo> GetRobotInfo() override;
        double CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) override;
        double CalculateAccelerationDistance(const std::chrono::milliseconds& duration) override;
    private:
        std::vector<JointState> GetOrderedJointStates();
        void SendGripperCommandUnlocked(double value);
        RobotArm* robotArm = nullptr;
        std::mutex jointStatesMutex;
        std::vector<JointState> orderedJointStates;
        std::unordered_map<JointNameImpl, JointState> unorderedJointStates;
        std::shared_ptr<RobotInfo> robotInfo;
        std::unordered_map<JointNameImpl, OperatingMode> operatingModes;
        InterbotixJointName::DOF dof;
        std::chrono::high_resolution_clock::time_point jointStatesLastChanged;
#ifdef COMMUNICATION_MEASUREMENT
        std::ofstream* communicationMeasurementFile = nullptr;
#endif //COMMUNICATION_MEASUREMENT
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP