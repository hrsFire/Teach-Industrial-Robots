#ifndef ROBOT_ARM_JOINT_STATE_HPP
#define ROBOT_ARM_JOINT_STATE_HPP

#include <memory>
#include "joint_name.hpp"
#include "operating_mode.hpp"

namespace robot_arm {
    class JointState {
    public:
        JointState(std::shared_ptr<JointName> jointName, double position, double velocity, double effort, OperatingMode operatingMode);
        std::shared_ptr<JointName> GetJointName() const;
        double GetPosition() const;
        double GetVelocity() const;
        double GetEffort() const;
        OperatingMode GetOperatingMode() const;
    private:
        std::shared_ptr<JointName> jointName;
        double position;
        double velocity;
        double effort;
        OperatingMode operatingMode;
    };
}

#endif //ROBOT_ARM_JOINT_STATE_HPP