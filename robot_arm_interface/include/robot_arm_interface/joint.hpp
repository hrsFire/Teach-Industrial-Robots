#ifndef ROBOT_ARM_JOINT_HPP
#define ROBOT_ARM_JOINT_HPP

#include <memory>
#include "joint_name.hpp"

namespace robot_arm {
    /**
     * This class provides information about a joint.
     */
    class Joint {
    public:
        /**
         * Default constructor for a joint.
         * @param name           The name for the joint.
         * @param id             The ID for the joint.
         * @param lowerLimit     The lower limit in rad for the joint.
         * @param upperLimit     The upper limit in rad for the joint.
         * @param velocityLimit  The velocity limit in rad/s for the joint.
         * @param canRotate      Specifies if the joint can rotate.
         */
        Joint(std::shared_ptr<JointName> name, uint id, double lowerLimit, double upperLimit, double velocityLimit, bool canRotate);
        /**
         * Gets the name for the joint.
         * @return The name for the joint.
         */
        std::shared_ptr<JointName> GetName() const;
        /**
         * Gets the ID for the joint.
         * @return The ID for the joint.
         */
        uint GetID() const;
        /**
         * Gets the lower limit for the joint.
         * @return The lower limit for the joint.
         */
        double GetLowerLimit() const;
        /**
         * Gets the upper limit for the joint.
         * @return The upper limit for the joint.
         */
        double GetUpperLimit() const;
        /**
         * Gets the velocity limit for the joint.
         * @return The velocity limit for the joint.
         */
        double GetVelocitiyLimit() const;
        /**
         * Gets if the joint can rotate.
         * @return True if the joint can rotate. Otherwise False is returned.
         */
        double CanRotate() const;
    private:
        std::shared_ptr<JointName> name;
        uint id;
        double lowerLimit;
        double upperLimit;
        double velocityLimit;
        bool canRotate;
    };
}

#endif //ROBOT_ARM_JOINT_HPP
