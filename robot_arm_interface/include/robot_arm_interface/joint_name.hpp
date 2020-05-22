#ifndef INTERBOTIX_JOINT_NAME_HPP
#define INTERBOTIX_JOINT_NAME_HPP

#include <string>
#include <stdexcept>
#include <memory>

namespace robot_arm {
    class JointName {
    public:
        virtual operator std::string() const = 0;
        virtual bool operator==(const JointName& jointName) const = 0;
        virtual std::shared_ptr<JointName> Next() = 0;
        virtual std::shared_ptr<JointName> Prev() = 0;
    };
}

#endif //INTERBOTIX_JOINT_NAME_HPP