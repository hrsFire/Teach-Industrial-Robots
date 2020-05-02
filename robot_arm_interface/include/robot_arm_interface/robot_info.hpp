#ifndef INTERBOTIX_ROBOT_INFO_HPP
#define INTERBOTIX_ROBOT_INFO_HPP

#include <vector>
#include <string>
#include "joint_name.hpp"

namespace robot_arm {
    class RobotInfo {
    public:
        RobotInfo(std::vector<JointName> jointNames, std::vector<int> jointIDs, std::vector<double> lowerJointLimits, std::vector<double> upperJointLimits,
            std::vector<double> velocityLimits, double lowerGripperLimit, double upperGripperLimit, bool useGripper, std::vector<double> homePosition,
            std::vector<double> sleepPosition, int numberOfJoints, int numberOfSingleJoints);
        const std::vector<JointName> jointNames;
        const std::vector<int> jointIDs;
        const std::vector<double> lowerJointLimits;
        const std::vector<double> upperJointLimits;
        const std::vector<double> velocityLimits;
        const double lowerGripperLimit;
        const double upperGripperLimit;
        const bool useGripper;
        const std::vector<double> homePosition;
        const std::vector<double> sleepPosition;
        const int numberOfJoints;
        const int numberOfSingleJoints;
    };
}

#endif //INTERBOTIX_ROBOT_INFO_HPP