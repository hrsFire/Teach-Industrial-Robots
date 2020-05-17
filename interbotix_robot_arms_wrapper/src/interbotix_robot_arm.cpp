#include "interbotix_robot_arms_wrapper/interbotix_robot_arm.hpp"
#include "interbotix_robot_arm_ros.hpp"
#include "interbotix_robot_arm_direct.hpp"

using namespace interbotix;

InterbotixRobotArm::InterbotixRobotArm(bool useRos, int argc, char** argv, std::string robotName, std::string robotModel) {
    // https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_sdk
    // One is to command the robot via the ROS topics and/or services. In this manner, the developer can code in
    // any language that is capable of sending a ROS message. The other approach is to 'skip' the ROS topic layer
    // and use the publicly available functions directly. All the user would need to do is create an instance of the
    // 'RobotArm' class as shown here to take advantage of these functions.
    // https://github.com/Interbotix/interbotix_ros_arms/blob/master/interbotix_sdk/src/arm_node.cpp

    if (useRos) {
        this->robotArm = new InterbotixRobotArmROS(argc, argv, robotName);
    } else {
        this->robotArm = new InterbotixRobotArmDirect(argc, argv, robotName, robotModel);
    }
}

InterbotixRobotArm::~InterbotixRobotArm() {
    delete robotArm;
}

std::unordered_map<JointName, JointState> InterbotixRobotArm::GetJointStates() {
    return robotArm->GetJointStates();
}

void InterbotixRobotArm::SendJointCommand(const JointName& jointName, double value) {
    robotArm->SendJointCommand(jointName, value);
}

void InterbotixRobotArm::SendJointCommands(const std::vector<JointName>& jointNames, const std::vector<double>& values) {
    robotArm->SendJointCommands(jointNames, values);
}

void InterbotixRobotArm::SendJointTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) {
    robotArm->SendJointTrajectory(jointNames, jointTrajectoryPoints);
}

void InterbotixRobotArm::SendGripperCommand(double value) {
    robotArm->SendGripperCommand(value);
}

void InterbotixRobotArm::SendGripperTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) {
    robotArm->SendGripperTrajectory(jointNames, jointTrajectoryPoints);
}

void InterbotixRobotArm::SetTorqueState(bool on) {
    robotArm->SetTorqueState(on);
}

void InterbotixRobotArm::SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
        int profileVelocity, int profileAcceleration) {
    robotArm->SetOperatingMode(operatingMode, affectedJoints, jointName, useCustomProfiles, profileVelocity,profileAcceleration);
}

std::shared_ptr<RobotInfo> InterbotixRobotArm::GetRobotInfo() {
    return robotArm->GetRobotInfo();
}

double InterbotixRobotArm::CalculateAcceleration(const JointName& jointName, std::chrono::milliseconds duration) {
    return robotArm->CalculateAcceleration(jointName, duration);
}