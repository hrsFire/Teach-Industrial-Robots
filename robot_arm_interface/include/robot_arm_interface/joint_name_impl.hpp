#ifndef INTERBOTIX_JOINT_NAME_IMPL_HPP
#define INTERBOTIX_JOINT_NAME_IMPL_HPP

#include <string>
#include <memory>
#include "joint_name.hpp"

namespace robot_arm {
    class JointNameImpl : public JointName {
    public:
        JointNameImpl(std::shared_ptr<JointName> jointName);
        operator std::string() const override;
        operator std::shared_ptr<JointName>() const;
        bool operator==(const JointName& jointName) const override;
        std::shared_ptr<JointName> Next() override;
        std::shared_ptr<JointName> Prev() override;
    private:
        std::shared_ptr<JointName> jointName;
    };
}

namespace std {
    template<> class hash<robot_arm::JointNameImpl> {
    public:
        std::size_t operator()(robot_arm::JointNameImpl const& jointName) const noexcept;
    };
}

#endif //INTERBOTIX_JOINT_NAME_IMPL_HPP