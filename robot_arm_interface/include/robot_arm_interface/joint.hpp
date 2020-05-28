#ifndef ROBOT_ARM_JOINT_HPP
#define ROBOT_ARM_JOINT_HPP

#include <memory>
#include "joint_name.hpp"

namespace robot_arm {
    class Joint {
    public:
        Joint(std::shared_ptr<JointName> name, uint id, double lowerLimit, double upperLimit, double velocityLimit, bool canRotate);
        std::shared_ptr<JointName> GetName() const;
        uint GetID() const;
        double GetLowerLimit() const;
        double GetUpperLimit() const;
        double GetVelocitiyLimit() const;
        double CanRotate() const;
    private:
        std::shared_ptr<JointName> name;
        uint id;
        double lowerLimit;
        double upperLimit;
        double velocityLimit;
        bool canRotate;
    };
}

#endif //ROBOT_ARM_JOINT_HPP