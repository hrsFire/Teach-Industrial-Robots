#ifndef INTERBOTIX_JOINT_HPP
#define INTERBOTIX_JOINT_HPP

#include "joint_name.hpp"

namespace robot_arm {
    class Joint {
    public:
        Joint(JointName name, uint id, double lowerLimit, double upperLimit, double velocityLimit);
        const JointName name;
        const uint id;
        const double lowerLimit;
        const double upperLimit;
        const double velocityLimit;
    };
}

#endif //INTERBOTIX_JOINT_HPP