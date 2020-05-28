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
    // @TODO: causes a bad cast exception if the items are switched
    return jointName == *(this->jointName);
}

std::shared_ptr<JointName> JointNameImpl::Next() {
    return jointName->Next();
}

std::shared_ptr<JointName> JointNameImpl::Prev() {
    return jointName->Prev();
}

std::string JointNameImpl::Serialize() const {
    return jointName->Serialize();
}

std::shared_ptr<JointName> JointNameImpl::Deserialize(std::string serializedString) const {
    return jointName->Deserialize(serializedString);
}

size_t std::hash<JointNameImpl>::operator()(JointNameImpl const& jointName) const noexcept {
    return std::hash<std::string>{}((std::string) jointName);
}