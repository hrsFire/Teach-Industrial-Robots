#ifndef INTERBOTIX_JOINT_STATE_HPP
#define INTERBOTIX_JOINT_STATE_HPP

#include <memory>
#include "joint_name.hpp"
#include "operating_mode.hpp"

namespace robot_arm {
    class JointState {
    public:
        JointState(std::shared_ptr<JointName> jointName, double position, double velocity, double effort, OperatingMode operatingMode);
        const std::shared_ptr<JointName> jointName;
        const double position;
        const double velocity;
        const double effort;
        const OperatingMode operatingMode;
    };
}

#endif //INTERBOTIX_JOINT_STATE_HPP