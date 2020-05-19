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
        std::unordered_map<JointName, JointState> GetJointStates() override;
        void SendJointCommand(const JointName& jointName, double value) override;
        void SendJointCommands(const std::unordered_map<JointName, double>& jointValues) override;
        void SendJointTrajectory(const std::unordered_map<JointName, JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::unordered_map<JointName, JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SetTorqueState(bool on) override;
        void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<RobotInfo> GetRobotInfo() override;
        double CalculateAcceleration(const JointName& jointName, std::chrono::milliseconds duration) override;
    private:
        RobotArm* robotArm = nullptr;
        std::vector<JointState> GetOrderedJointStates();
        std::shared_ptr<RobotInfo> robotInfo;
        std::unordered_map<JointName, OperatingMode> operatingModes;
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_DIRECT_HPP