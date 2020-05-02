#include "robot_arm_interface/joint_name.hpp"

using namespace robot_arm;

JointName::JointName(std::string joint) {
    if (joint == "waist") {
        JointName(Joint::WAIST);
    } else if (joint == "shoulder") {
        JointName(Joint::SHOULDER);
    } else if (joint == "elbow") {
        JointName(Joint::ELBOW);
    } else if (joint == "forearm_roll") {
        JointName(Joint::FOREARM_ROLL);
    } else if (joint == "wrist_angle") {
        JointName(Joint::WRIST_ANGLE);
    } else if (joint == "wrist_rotate") {
        JointName(Joint::WRIST_ROTATE);
    } else if (joint == "gripper") {
        JointName(Joint::GRIPPER);
    } else {
        throw std::invalid_argument("'" + joint + "' is not a valid name for a joint");
    }
}

JointName JointName::WAIST() {
    return JointName(Joint::WAIST);
}

JointName JointName::SHOULDER() {
    return JointName(Joint::SHOULDER);
}

JointName JointName::ELBOW() {
    return JointName(Joint::ELBOW);
}

JointName JointName::FOREARM_ROLL() {
    return JointName(Joint::FOREARM_ROLL);
}

JointName JointName::WRIST_ANGLE() {
    return JointName(Joint::WRIST_ANGLE);
}

JointName JointName::WRIST_ROTATE() {
    return JointName(Joint::WRIST_ROTATE);
}

JointName JointName::GRIPPER() {
    return JointName(Joint::GRIPPER);
}

JointName JointName::NONE() {
    return JointName(Joint::NONE);
}

JointName::operator std::string() {
    switch (joint)
    {
        case Joint::WAIST:
            return "waist";
        case Joint::SHOULDER:
            return "shoulder";
        case Joint::ELBOW:
            return "elbow";
        case Joint::FOREARM_ROLL:
            return "forearm_roll";
        case Joint::WRIST_ANGLE:
            return "wrist_angle";
        case Joint::WRIST_ROTATE:
            return "wrist_rotate";
        case Joint::GRIPPER:
            return "gripper";
        case Joint::NONE:
            return "";
        default:
            return "";
    }
}

bool JointName::operator==(const JointName& jointName) {
    return joint == jointName.joint;
}

JointName::JointName(Joint joint) {
    this->joint = joint;
}