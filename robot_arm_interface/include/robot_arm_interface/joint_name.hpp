#ifndef ROBOT_ARM_JOINT_NAME_HPP
#define ROBOT_ARM_JOINT_NAME_HPP

#include <string>
#include <stdexcept>
#include <memory>

/**
* This namespace provides numerous stuff to interact with different robot arm models.
*/
namespace robot_arm {
    /**
     * This class provides abstract joint names for the different robot arm models. This class must be extended for new robot arm models to specify their capabilities.
     */
    class JointName {
    public:
        /**
         * Casts the joint name to a string representation.
         * @return String representation for the joint name.
         */
        virtual operator std::string() const = 0;
        /**
         * Compares two joint names.
         * @param[in] jointName  An abstract joint name.
         * @return Bool value for joint name comparison.
         */
        virtual bool operator==(const JointName& jointName) const = 0;
        /**
         * Returns a shared pointer to an abstract joint name for the logical next joint of the robot arm.
         * @return Shared pointer to an abstract joint name for the logical next joint of the robot arm.
         */
        virtual std::shared_ptr<JointName> Next() = 0;
        /**
         * Returns a shared pointer to an abstract joint name for the logical previous joint of the robot arm.
         * @return Shared pointer to an abstract joint name for the logical previous joint of the robot arm.
         */
        virtual std::shared_ptr<JointName> Prev() = 0;
        /**
         * Returns the serialized string representation of the joint name.
         * @return Serialized string representation of the joint name.
         */
        virtual std::string Serialize() const = 0;
        /**
         * Returns a shared point to an abstract joint name for the given string representation.
         * @param serializedString  String representation for the joint name.
         * @return Serialized string representation for the joint name.
         */
        virtual std::shared_ptr<JointName> Deserialize(std::string serializedString) const = 0;
    };
}

#endif //ROBOT_ARM_JOINT_NAME_HPP
