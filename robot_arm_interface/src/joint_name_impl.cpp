#include "robot_arm_interface/joint_name_impl.hpp"

using namespace robot_arm;

JointNameImpl::JointNameImpl(std::shared_ptr<JointName> jointName) : jointName(jointName) {
}

JointNameImpl::operator std::string() const {
    return *jointName;
}

JointNameImpl::operator std::shared_ptr<JointName>() const {
    return jointName;
}

bool JointNameImpl::operator==(const JointName& jointName) const {
    return jointName == *(this->jointName);
}

std::shared_ptr<JointName> JointNameImpl::Next() {
    return this->Next();
}

std::shared_ptr<JointName> JointNameImpl::Prev() {
    return this->Prev();
}

size_t std::hash<JointNameImpl>::operator()(JointNameImpl const& jointName) const noexcept {
    return std::hash<std::string>{}((std::string) jointName);
}