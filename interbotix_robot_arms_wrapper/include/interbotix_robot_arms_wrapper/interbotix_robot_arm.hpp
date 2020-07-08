#ifndef INTERBOTIX_ROBOT_ARM_HPP
#define INTERBOTIX_ROBOT_ARM_HPP

#include <vector>
#include <unordered_map>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>
#include <robot_arm_interface/joint_trajectory_point.hpp>
#include <robot_arm_interface/operating_mode.hpp>

namespace interbotix {
    class InterbotixRobotArm : public robot_arm::RobotArmBase {
    public:
        InterbotixRobotArm(bool useROS, int argc, char** argv, std::string robotName, std::string robotModel);
        ~InterbotixRobotArm();
        std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> GetJointStates() override;
        bool GetCurrentPose(const robot_arm::JointName& endEffectorJointName, geometry_msgs::Pose& pose) override;
        void SendJointCommand(const robot_arm::JointName& jointName, double value) override;
        void SendJointCommands(const std::unordered_map<robot_arm::JointNameImpl, double>& jointValues) override;
        void SendJointTrajectory(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendPose(const geometry_msgs::Pose& pose, const robot_arm::JointName& endEffectorJointName) override;
        void SetOperatingMode(const robot_arm::OperatingMode& operatingMode, const robot_arm::AffectedJoints& affectedJoints, const robot_arm::JointName& jointName,
            bool useCustomProfiles, int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<robot_arm::RobotInfo> GetRobotInfo() override;
        double CalculateAccelerationDistance(const robot_arm::JointName& jointName, const std::chrono::milliseconds& duration) override;
        double CalculateAccelerationDistance(const std::chrono::milliseconds& duration) override;
    private:
        RobotArmBase* robotArm;
    };
}

#endif //INTERBOTIX_ROBOT_ARM_HPP