#include "robot_arm_interface/joint.hpp"

using namespace robot_arm;

Joint::Joint(JointName name, uint id, double lowerLimit, double upperLimit, double velocityLimit) :
        name(name), id(id), lowerLimit(lowerLimit), upperLimit(upperLimit), velocityLimit(velocityLimit) {
}