#include "robot_arm_interface/robot_info.hpp"

using namespace robot_arm;

RobotInfo::RobotInfo(std::unordered_map<JointName, Joint> joints, double lowerGripperLimit, double upperGripperLimit, bool useGripper,
        std::vector<double> homePosition, std::vector<double> sleepPosition, int numberOfJoints, int numberOfSingleJoints) :
        joints(joints), lowerGripperLimit(lowerGripperLimit), upperGripperLimit(upperGripperLimit), useGripper(useGripper), homePosition(homePosition),
        sleepPosition(sleepPosition), numberOfJoints(numberOfJoints), numberOfSingleJoints(numberOfSingleJoints) {
}