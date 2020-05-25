#include "robot_arm_interface/joint_state.hpp"

using namespace robot_arm;

JointState::JointState(std::shared_ptr<JointName> jointName, double position, double velocity, double effort, OperatingMode operatingMode) :
        jointName(jointName), position(position), velocity(velocity), effort(effort), operatingMode(operatingMode) {
}

std::shared_ptr<JointName> JointState::GetJointName() const {
    return jointName;
}

double JointState::GetPosition() const {
    return position;
}

double JointState::GetVelocity() const {
    return velocity;
}

double JointState::GetEffort() const {
    return effort;
}

OperatingMode JointState::GetOperatingMode() const {
    return operatingMode;
}