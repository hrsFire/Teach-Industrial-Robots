#include "robot_arm_interface/joint_state.hpp"

using namespace robot_arm;

JointState::JointState(JointName jointName, double position, double velocity, double effort, OperatingMode operatingMode) :
        jointName(jointName), position(position), velocity(velocity), effort(effort), operatingMode(operatingMode) {
}