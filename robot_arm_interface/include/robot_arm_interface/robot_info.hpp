#ifndef INTERBOTIX_ROBOT_INFO_HPP
#define INTERBOTIX_ROBOT_INFO_HPP

#include <vector>
#include <unordered_map>
#include <string>
#include "joint_name.hpp"
#include "joint.hpp"

namespace robot_arm {
    class RobotInfo {
    public:
        RobotInfo(std::unordered_map<JointName, Joint> joints, double lowerGripperLimit, double upperGripperLimit, bool useGripper, std::vector<double> homePosition,
            std::vector<double> sleepPosition, int numberOfJoints, int numberOfSingleJoints);
        const std::unordered_map<JointName, Joint> joints;
        const double lowerGripperLimit;
        const double upperGripperLimit;
        const bool useGripper;
        const std::vector<double> homePosition;
        const std::vector<double> sleepPosition;
        // The number of joints in the arm (excluding gripper)
        const int numberOfJoints;
        // The number of all joints in the robot (includes gripper and any 'single' joints)
        const int numberOfSingleJoints;
    };
}

#endif //INTERBOTIX_ROBOT_INFO_HPP