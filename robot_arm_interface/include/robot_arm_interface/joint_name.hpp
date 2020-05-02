#ifndef INTERBOTIX_JOINT_NAME_HPP
#define INTERBOTIX_JOINT_NAME_HPP

#include <string>
#include <stdexcept>

namespace robot_arm {
    class JointName {
    public:
        JointName(std::string joint);
        static JointName WAIST();
        static JointName SHOULDER();
        static JointName ELBOW();
        // Only available with 6 DOF
        static JointName FOREARM_ROLL();
        static JointName WRIST_ANGLE();
        // Only available with 5 or 6 DOF
        static JointName WRIST_ROTATE();
        static JointName GRIPPER();
        static JointName NONE();
        operator std::string();
        bool operator==(const JointName& jointName);
    private:
        enum class Joint {
            WAIST = 0,
            SHOULDER = 1,
            ELBOW = 2,
            FOREARM_ROLL = 3,
            WRIST_ANGLE = 4,
            WRIST_ROTATE = 5,
            GRIPPER = 6,
            NONE = 7
        };
        JointName(Joint joint);
        Joint joint;
    };
}

#endif //INTERBOTIX_JOINT_NAME_HPP