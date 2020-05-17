#include "robot_arm_interface/robot_info.hpp"

using namespace robot_arm;

RobotInfo::RobotInfo(std::unordered_map<JointName, Joint> joints, bool useGripper, std::unordered_map<JointName, double> homePosition,
                std::unordered_map<JointName, double> sleepPosition, int numberOfJoints, int numberOfSingleJoints) :
        joints(joints), useGripper(useGripper), homePosition(homePosition), sleepPosition(sleepPosition), numberOfJoints(numberOfJoints),
        numberOfSingleJoints(numberOfSingleJoints) {
}