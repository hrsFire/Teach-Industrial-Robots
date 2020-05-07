#ifndef INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP
#define INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP

#include <mutex>
#include <interbotix_sdk/arm_obj.h>
#include <robot_arm_interface/robot_arm_base.hpp>
#include "joint_helper.hpp"

using namespace robot_arm;

namespace interbotix {
    class InterbotixRobotArmDirect : public InterbotixRobotArmBase {
    public:
        InterbotixRobotArmDirect(int argc, char** argv, std::string robotName, std::string robotModel);
        ~InterbotixRobotArmDirect();
        std::vector<JointState> GetJointStates() override;
        void SendJointCommand(JointName jointName, double value) override;
        void SendJointCommands(const std::vector<JointName>& jointNames, const std::vector<double>& values) override;
        void SendJointTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SetTorqueState(bool on) override;
        void SetOperatingMode(OperatingMode operatingMode, AffectedJoints affectedJoints, JointName jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<RobotInfo> GetRobotInfo() override;
    private:
        RobotArm* robotArm = nullptr;
        std::shared_ptr<RobotInfo> robotInfo;
        std::mutex jointStatesMutex;
        std::vector<JointState> jointStates;
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP