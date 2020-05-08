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
        std::unordered_map<std::string, JointState> GetJointStates() override;
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
        InterbotixRobotArmBase* robotArm;
    };
}

#endif //INTERBOTIX_ROBOT_ARM_HPP