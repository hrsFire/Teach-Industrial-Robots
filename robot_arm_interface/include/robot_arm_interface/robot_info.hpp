#ifndef ROBOT_ARM_ROBOT_INFO_HPP
#define ROBOT_ARM_ROBOT_INFO_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include "joint_name.hpp"
#include "joint_name_impl.hpp"
#include "joint.hpp"

namespace robot_arm {
    /**
     * This class provides detailed information about the robot model.
     */
    class RobotInfo {
    public:
        /**
         * Default constructor for the detailed information about the robot model.
         * @param joints                The joint names and their detailed information for the robot model.
         * @param useGripper	         Specifies if the robot model uses a gripper.
         * @param homePosition          The home position for the robot model.
         * @param sleepPosition         The sleep position for the robot model.
         * @param numberOfJoints        The number of joints for the robot model excluding the gripper.
         * @param numberOfSingleJoints  The number of single joints for the robot model including the gripper.
         */
        RobotInfo(std::unordered_map<JointNameImpl, Joint> joints, bool useGripper, std::unordered_map<JointNameImpl, double> homePosition,
            std::unordered_map<JointNameImpl, double> sleepPosition, int numberOfJoints, int numberOfSingleJoints);
        /**
         * Specifies the joint names of the robot model.
         */
        const std::unordered_map<JointNameImpl, Joint> joints;
        /**
         * Specifies if the robot model uses a gripper.
         */
        const bool useGripper;
        /**
         * The home position for the robot model used to command all joints (excluding gripper) to 0 radians.
         */
        const std::unordered_map<JointNameImpl, double> homePosition;
        /**
         * The sleep position for the robot model. Used to command all joints (excluding gripper) to a specific position in radians so that the arm does not crash when the driver is turned off (which turns off all motors). 
         */
        const std::unordered_map<JointNameImpl, double> sleepPosition;
        /**
         * The number of joints for the robot model excluding the gripper.
         */
        const int numberOfJoints;
        /**
         * The number of single joints for the robot model including the gripper.
         */
        const int numberOfSingleJoints;
    };
}

#endif //ROBOT_ARM_ROBOT_INFO_HPP
