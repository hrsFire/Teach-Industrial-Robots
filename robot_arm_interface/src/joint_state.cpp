#include "robot_arm_interface/joint_state.hpp"

using namespace robot_arm;

JointState::JointState(std::string jointName, double position, double velocity, double effort) :
        jointName(jointName), position(position), velocity(velocity), effort(effort) {
}