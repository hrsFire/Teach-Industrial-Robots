#ifndef INTERBOTIX_JOINT_STATE_HPP
#define INTERBOTIX_JOINT_STATE_HPP

#include <string>
#include <vector>

namespace robot_arm {
    class JointState {
    public:
        JointState(std::string jointName, double position, double velocity, double effort);
        const std::string jointName;
        const double position;
        const double velocity;
        const double effort;
    };
}

#endif //INTERBOTIX_JOINT_STATE_HPP