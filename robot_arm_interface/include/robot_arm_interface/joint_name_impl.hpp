#ifndef ROBOT_ARM_JOINT_NAME_IMPL_HPP
#define ROBOT_ARM_JOINT_NAME_IMPL_HPP

#include <string>
#include <memory>
#include "joint_name.hpp"

namespace robot_arm {
    /**
     * This class provides a default implementation for an abstract joint name.
     */
    class JointNameImpl : public JointName {
    public:
        /**
         * Default constructor for a joint.
         * @param jointName  The name for the joint.
         */
        JointNameImpl(std::shared_ptr<JointName> jointName);
        operator std::string() const override;
        /**
         * Casts the joint name implementation to a shared pointer to an abstract joint name .
         * @return Shared pointer to an abstract joint name..
         */
        operator std::shared_ptr<JointName>() const;
        bool operator==(const JointName& jointName) const override;
        std::shared_ptr<JointName> Next() override;
        std::shared_ptr<JointName> Prev() override;
        std::string Serialize() const override;
        std::shared_ptr<JointName> Deserialize(std::string serializedString) const override;
    private:
        std::shared_ptr<JointName> jointName;
    };
}

namespace std {
    /**
     * This template provides the possibility to hash any joint name implementation. A joint name implementation can be be created by providing a shared pointer to an abstract joint name.
     */
    template<> class hash<robot_arm::JointNameImpl> {
    public:
        /**
         * Calculates the has value for a joint name implementation.
         * @param jointName  A joint name implementation.
         * @return Hash value for a joint name implementation.
         */
        std::size_t operator()(robot_arm::JointNameImpl const& jointName) const noexcept;
    };
}

#endif //ROBOT_ARM_JOINT_NAME_IMPL_HPP
