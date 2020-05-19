#ifndef INTERBOTIX_ROBOT_ARM_HPP
#define INTERBOTIX_ROBOT_ARM_HPP

#include <vector>
#include <unordered_map>
#include <robot_arm_interface/robot_arm_base.hpp>

using namespace robot_arm;

namespace interbotix {
    class InterbotixRobotArm : public InterbotixRobotArmBase {
    public:
        InterbotixRobotArm(bool useRos, int argc, char** argv, std::string robotName, std::string robotModel);
        ~InterbotixRobotArm();
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
        InterbotixRobotArmBase* robotArm;
    };
}

#endif //INTERBOTIX_ROBOT_ARM_HPP