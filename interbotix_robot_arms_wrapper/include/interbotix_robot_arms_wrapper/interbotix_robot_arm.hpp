#ifndef INTERBOTIX_ROBOT_ARM_HPP
#define INTERBOTIX_ROBOT_ARM_HPP

#include <vector>
#include <unordered_map>
#include <chrono>
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
        void SendJointCommand(const robot_arm::JointName& jointName, double value) override;
        void SendJointCommands(const std::unordered_map<robot_arm::JointNameImpl, double>& jointValues) override;
        void SendJointTrajectory(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SetTorqueState(bool on) override;
        void SetOperatingMode(const robot_arm::OperatingMode& operatingMode, const robot_arm::AffectedJoints& affectedJoints, const robot_arm::JointName& jointName,
            bool useCustomProfiles, int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<robot_arm::RobotInfo> GetRobotInfo() override;
        double CalculateAcceleration(const robot_arm::JointName& jointName, std::chrono::milliseconds duration, bool isGoingUpwards) override;
    private:
        RobotArmBase* robotArm;
    };
}

#endif //INTERBOTIX_ROBOT_ARM_HPP