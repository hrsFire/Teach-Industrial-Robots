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
        RobotInfo(std::unordered_map<JointName, Joint> joints, bool useGripper, std::unordered_map<JointName, double> homePosition, std::unordered_map<JointName,
            double> sleepPosition, int numberOfJoints, int numberOfSingleJoints);
        const std::unordered_map<JointName, Joint> joints;
        const bool useGripper;
        // Home position for each robot. Is used to command all joints (excluding gripper) to 0 radians
        const std::unordered_map<JointName, double> homePosition;
        // Sleep position for each robot. Is used to command all joints (excluding gripper) to a specific position in radians
        // so that if the driver is shutdown (which torques off all motors), the arm doesn't come crashing down
        const std::unordered_map<JointName, double> sleepPosition;
        // The number of joints in the arm (excluding gripper)
        const int numberOfJoints;
        // The number of all joints in the robot (includes gripper and any 'single' joints)
        const int numberOfSingleJoints;
    };
}

#endif //INTERBOTIX_ROBOT_INFO_HPP