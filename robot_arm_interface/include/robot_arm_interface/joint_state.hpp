#ifndef ROBOT_ARM_JOINT_STATE_HPP
#define ROBOT_ARM_JOINT_STATE_HPP

#include <memory>
#include "joint_name.hpp"
#include "operating_mode.hpp"

namespace robot_arm {
    /**
     * This class provides the state of a joint.
     */
    class JointState {
    public:
        /**
         * Default constructor for a joint state
         * @param jointName      The name for the joint.
         * @param position       The position for the joint.
         * @param velocity       The velocity for the joint.
         * @param effort         The effor for the joint.
         * @param operatingMode  The operating mode for the joint.
         */
        JointState(std::shared_ptr<JointName> jointName, double position, double velocity, double effort, OperatingMode operatingMode);
        /**
         * Gets the joint name for the joint.
         * @return A shared pointer to a joint name.
         */
        std::shared_ptr<JointName> GetJointName() const;
        /**
         * Gets the position for the joint.
         * @return The position for the joint.
         */
        double GetPosition() const;
        /**
         * Gets the velocity for the joint.
         * @return The velocity for the joint.
         */
        double GetVelocity() const;
        /**
         * Gets the effort for the joint.
         * @return The effort for the joint.
         */
        double GetEffort() const;
        /**
         * Gets the operating mode for the joint.
         * @return The operating mode for the joint.
         */
        OperatingMode GetOperatingMode() const;
    private:
        std::shared_ptr<JointName> jointName;
        double position;
        double velocity;
        double effort;
        OperatingMode operatingMode;
    };
}

#endif //ROBOT_ARM_JOINT_STATE_HPP
