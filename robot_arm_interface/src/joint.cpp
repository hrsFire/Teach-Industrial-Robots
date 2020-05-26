#include "robot_arm_interface/joint.hpp"

using namespace robot_arm;

Joint::Joint(std::shared_ptr<JointName> name, uint id, double lowerLimit, double upperLimit, double velocityLimit, bool canRotate) :
        name(name), id(id), lowerLimit(lowerLimit), upperLimit(upperLimit), velocityLimit(velocityLimit), canRotate(canRotate) {
}

std::shared_ptr<JointName> Joint::GetName() const {
    return name;
}

uint Joint::GetID() const {
    return id;
}

double Joint::GetLowerLimit() const {
    return lowerLimit;
}

double Joint::GetUpperLimit() const {
    return upperLimit;
}

double Joint::GetVelocitiyLimit() const {
    return velocityLimit;
}

double Joint::CanRotate() const {
    return canRotate;
}