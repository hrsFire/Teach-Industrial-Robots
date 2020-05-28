#include "interbotix_robot_arms_wrapper/interbotix_joint_name.hpp"

using namespace interbotix;

InterbotixJointName::DOF InterbotixJointName::defaultDOF = InterbotixJointName::DOF::DOF_4;

InterbotixJointName::InterbotixJointName(std::string joint, DOF dof) {
    if (joint == "waist") {
        this->joint = Joint::WAIST;
    } else if (joint == "shoulder") {
        this->joint = Joint::SHOULDER;
    } else if (joint == "elbow") {
        this->joint = Joint::ELBOW;
    } else if (joint == "forearm_roll") {
        this->joint = Joint::FOREARM_ROLL;
    } else if (joint == "wrist_angle") {
        this->joint = Joint::WRIST_ANGLE;
    } else if (joint == "wrist_rotate") {
        this->joint = Joint::WRIST_ROTATE;
    } else if (joint == "gripper") {
        this->joint = Joint::GRIPPER;
    } else if (joint == "none") {
        this->joint = Joint::NONE;
    } else {
        throw std::invalid_argument("'" + joint + "' is not a valid name for a joint");
    }

    this->dof = dof;
}

InterbotixJointName InterbotixJointName::WAIST() {
    return InterbotixJointName(Joint::WAIST, defaultDOF);
}

InterbotixJointName InterbotixJointName::SHOULDER() {
    return InterbotixJointName(Joint::SHOULDER, defaultDOF);
}

InterbotixJointName InterbotixJointName::ELBOW() {
    return InterbotixJointName(Joint::ELBOW, defaultDOF);
}

InterbotixJointName InterbotixJointName::FOREARM_ROLL() {
    return InterbotixJointName(Joint::FOREARM_ROLL, defaultDOF);
}

InterbotixJointName InterbotixJointName::WRIST_ANGLE() {
    return InterbotixJointName(Joint::WRIST_ANGLE, defaultDOF);
}

InterbotixJointName InterbotixJointName::WRIST_ROTATE() {
    return InterbotixJointName(Joint::WRIST_ROTATE, defaultDOF);
}

InterbotixJointName InterbotixJointName::GRIPPER() {
    return InterbotixJointName(Joint::GRIPPER, defaultDOF);
}

InterbotixJointName InterbotixJointName::NONE() {
    return InterbotixJointName(Joint::NONE, defaultDOF);
}

void InterbotixJointName::SetDefaultDOF(DOF dof) {
    defaultDOF = dof;
}

InterbotixJointName::operator std::string() const {
    switch (joint) {
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
            return "none";
        default:
            return "";
    }
}

InterbotixJointName::operator robot_arm::JointNameImpl() const {
    return robot_arm::JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) *this));
}

bool InterbotixJointName::operator==(const JointName& jointName) const {
    InterbotixJointName jointNamePtr = dynamic_cast<const InterbotixJointName&>(jointName);
    return joint == jointNamePtr.joint && dof == jointNamePtr.dof;
}

std::shared_ptr<robot_arm::JointName> InterbotixJointName::Next() {
    Joint nextJoint = Joint::NONE;

    switch (joint) {
        case Joint::WAIST:
            nextJoint = Joint::SHOULDER;
            break;
        case Joint::SHOULDER:
            nextJoint = Joint::ELBOW;
            break;
        case Joint::ELBOW:
            if (dof == DOF::DOF_6) {
                nextJoint = Joint::FOREARM_ROLL;
            } else {
                nextJoint = Joint::WRIST_ANGLE;
            }

            break;
        case Joint::FOREARM_ROLL:
            nextJoint = Joint::WRIST_ANGLE;
            break;
        case Joint::WRIST_ANGLE:
            if (dof == DOF::DOF_5 || dof == DOF::DOF_6) {
                nextJoint = Joint::WRIST_ROTATE;
            } else {
                nextJoint = Joint::WAIST;
            }

            break;
        case Joint::WRIST_ROTATE:
            nextJoint = Joint::WAIST;
            break;
        case Joint::GRIPPER:
            nextJoint = Joint::GRIPPER;
            break;
        case Joint::NONE:
            nextJoint = Joint::NONE;
            break;
        default:
            nextJoint = Joint::NONE;
    }

    return std::make_shared<InterbotixJointName>(InterbotixJointName(nextJoint, dof));
}

std::shared_ptr<robot_arm::JointName> InterbotixJointName::Prev() {
    Joint prevJoint = Joint::NONE;

    switch (joint) {
        case Joint::WAIST:
            if (dof == DOF::DOF_5 || dof == DOF::DOF_6) {
                prevJoint = Joint::WRIST_ROTATE;
            } else {
                prevJoint = Joint::WRIST_ANGLE;
            }

            break;
        case Joint::SHOULDER:
            prevJoint = Joint::WAIST;
            break;
        case Joint::ELBOW:
            prevJoint = Joint::SHOULDER;
            break;
        case Joint::FOREARM_ROLL:
            prevJoint = Joint::ELBOW;
            break;
        case Joint::WRIST_ANGLE:
            if (dof == DOF::DOF_6) {
                prevJoint = Joint::FOREARM_ROLL;
            } else {
                prevJoint = Joint::ELBOW;
            }

            break;
        case Joint::WRIST_ROTATE:
            prevJoint = Joint::WRIST_ANGLE;
            break;
        case Joint::GRIPPER:
            prevJoint = Joint::GRIPPER;
            break;
        case Joint::NONE:
            prevJoint = Joint::NONE;
            break;
        default:
            prevJoint = Joint::NONE;
    }

    return std::make_shared<InterbotixJointName>(InterbotixJointName(prevJoint, dof));
}

std::string InterbotixJointName::Serialize() const {
    std::string jointName = *this;
    return jointName + ";" + std::to_string(static_cast<int>(dof));
}

std::shared_ptr<robot_arm::JointName> InterbotixJointName::Deserialize(std::string serializedString) const {
    size_t i = serializedString.find(";");
    return std::make_shared<InterbotixJointName>(serializedString.substr(0, i), static_cast<DOF>(std::stoi(serializedString.substr(i+1, serializedString.size()))));
}

InterbotixJointName::InterbotixJointName(Joint joint, DOF dof) {
    this->joint = joint;
    this->dof = dof;
}