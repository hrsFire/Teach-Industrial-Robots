#include "interbotix_robot_arms_wrapper/interbotix_robot_arm.hpp"
#include "interbotix_robot_arm_ros.hpp"
#include "interbotix_robot_arm_direct.hpp"

using namespace interbotix;

InterbotixRobotArm::InterbotixRobotArm(bool useROS, int argc, char** argv, std::string robotName, std::string robotModel) {
    // https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_sdk
    // One is to command the robot via the ROS topics and/or services. In this manner, the developer can code in
    // any language that is capable of sending a ROS message. The other approach is to 'skip' the ROS topic layer
    // and use the publicly available functions directly. All the user would need to do is create an instance of the
    // 'RobotArm' class as shown here to take advantage of these functions.
    // https://github.com/Interbotix/interbotix_ros_arms/blob/master/interbotix_sdk/src/arm_node.cpp

    if (useROS) {
        this->robotArm = new InterbotixRobotArmROS(argc, argv, robotName, robotModel);
    } else {
        this->robotArm = new InterbotixRobotArmDirect(argc, argv, robotName, robotModel);
    }
}

InterbotixRobotArm::~InterbotixRobotArm() {
    delete robotArm;
}

std::unordered_map<robot_arm::JointNameImpl, JointState> InterbotixRobotArm::GetJointStates() {
    return robotArm->GetJointStates();
}

bool InterbotixRobotArm::GetCurrentPose(const robot_arm::JointName& endEffectorJointName, geometry_msgs::Pose& pose) {
    return robotArm->GetCurrentPose(endEffectorJointName, pose);
}

void InterbotixRobotArm::SendJointCommand(const JointName& jointName, double value) {
    robotArm->SendJointCommand(jointName, value);
}

void InterbotixRobotArm::SendJointCommands(const std::unordered_map<robot_arm::JointNameImpl, double>& jointValues) {
    robotArm->SendJointCommands(jointValues);
}

void InterbotixRobotArm::SendJointTrajectory(const std::unordered_map<robot_arm::JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    robotArm->SendJointTrajectory(jointTrajectoryPoints);
}

void InterbotixRobotArm::SendGripperCommand(double value) {
    robotArm->SendGripperCommand(value);
}

void InterbotixRobotArm::SendGripperTrajectory(const std::vector<JointTrajectoryPoint>& trajectoryPoints) {
    robotArm->SendGripperTrajectory(trajectoryPoints);
}

void InterbotixRobotArm::SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) {
    robotArm->SendPose(pose, endEffectorJointName);
}

void InterbotixRobotArm::SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
        int profileVelocity, int profileAcceleration) {
    robotArm->SetOperatingMode(operatingMode, affectedJoints, jointName, useCustomProfiles, profileVelocity,profileAcceleration);
}

// This provides among other things the joint limits (http://support.interbotix.com/html/specifications/wx200.html#default-joint-limits)
std::shared_ptr<RobotInfo> InterbotixRobotArm::GetRobotInfo() {
    return robotArm->GetRobotInfo();
}

double InterbotixRobotArm::CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) {
    return robotArm->CalculateAccelerationDistance(jointName, duration);
}

double InterbotixRobotArm::CalculateAccelerationDistance(const std::chrono::milliseconds& duration) {
    return robotArm->CalculateAccelerationDistance(duration);
}