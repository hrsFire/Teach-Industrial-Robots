#ifndef INTERBOTIX_JOINT_STATE_HPP
#define INTERBOTIX_JOINT_STATE_HPP

#include "joint_name.hpp"

namespace robot_arm {
    class JointState {
    public:
        JointState(JointName jointName, double position, double velocity, double effort);
        const JointName jointName;
        const double position;
        const double velocity;
        const double effort;
    };
}

#endif //INTERBOTIX_JOINT_STATE_HPP