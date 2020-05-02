#include "robot_arm_interface/robot_info.hpp"

using namespace robot_arm;

RobotInfo::RobotInfo(std::vector<JointName> jointNames, std::vector<int> jointIDs, std::vector<double> lowerJointLimits, std::vector<double> upperJointLimits,
        std::vector<double> velocityLimits, double lowerGripperLimit, double upperGripperLimit, bool useGripper, std::vector<double> homePosition,
        std::vector<double> sleepPosition, int numberOfJoints, int numberOfSingleJoints) :
        jointNames(jointNames), jointIDs(jointIDs), lowerJointLimits(lowerJointLimits), upperJointLimits(upperJointLimits), velocityLimits(velocityLimits),
        lowerGripperLimit(lowerGripperLimit), upperGripperLimit(upperGripperLimit), useGripper(useGripper), homePosition(homePosition),
        sleepPosition(sleepPosition), numberOfJoints(numberOfJoints), numberOfSingleJoints(numberOfSingleJoints) {
}