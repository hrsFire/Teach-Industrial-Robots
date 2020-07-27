#ifndef INTERBOTIX_INTERBOTIX_JOINT_NAME_HPP
#define INTERBOTIX_INTERBOTIX_JOINT_NAME_HPP

#include <string>
#include <stdexcept>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>

/**
* This namespace provides numerous stuff to interact with the InterbotiX SDK.
*/
namespace interbotix {
    /**
     * This class provides specific joint names for the different InterbotiX robot arm models.
     */
    class InterbotixJointName : public robot_arm::JointName {
    public:
        /**
         * The degrees of freedom for the numerous InterbotiX robot arm models.
         */
        enum class DOF {
            DOF_4 = 4,
            DOF_5 = 5,
            DOF_6 = 6
        };
        /**
         * Default constructor for creating an InterbotiX joint name.
         * @param joint  The joint name for the joint of the InterbotiX robot arm.
         * @param dof	  The degrees of freedom which are supported by the InterbotiX robot arm.
         */
        InterbotixJointName(std::string joint, DOF dof);
        /**
         * Returns the "waist" joint name of the InterbotiX robot arm.
         * @return The "waist" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName WAIST();
        /**
         * Returns the "shoulder" joint name of the InterbotiX robot arm.
         * @return The "shoulder" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName SHOULDER();
        /**
         * Returns the "elbow" joint name of the InterbotiX robot arm.
         * @return The "elbow" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName ELBOW();
        /**
         * Returns the "forearm roll" joint name of the InterbotiX robot arm. (Only available with 6 DOF)
         * @return The "forearm roll" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName FOREARM_ROLL();
        /**
         * Returns the "wrist angle" joint name of the InterbotiX robot arm.
         * @return The "wrist angle" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName WRIST_ANGLE();
        /**
         * Returns the "wrist rotate" joint name of the InterbotiX robot arm. (Only available with 5 or 6 DOF)
         * @return The "wrist rotate" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName WRIST_ROTATE();
        /**
         * Returns the "gripper" joint name of the InterbotiX robot arm.
         * @return The "gripper" joint name of the InterbotiX robot arm.
         */
        static InterbotixJointName GRIPPER();
        /**
         * Returns an invalid joint name.
         * @return An invalid joint name.
         */
        static InterbotixJointName NONE();
        /**
         * Sets the default DOF for the used InterbotiX robot arm.
         * @param dof  The degrees of freedom which are supported by the InterbotiX robot arm.
         * @return void
         */
        static void SetDefaultDOF(DOF dof);
        operator std::string() const override;
        /**
         * Casts the current joint name to a joint name implementation instance.
         * @return A joint name implementation instance.
         */
        operator robot_arm::JointNameImpl() const;
        /**
         * Compares two joint names.
         * @param jointName  An abstract joint name.
         * @return Bool value for the joint name comparison.
         */
        bool operator==(const JointName& jointName) const override;
        std::shared_ptr<JointName> Next() override;
        std::shared_ptr<JointName> Prev() override;
        std::string Serialize() const override;
        std::shared_ptr<robot_arm::JointName> Deserialize(std::string serializedString) const override;
    private:
        enum class Joint {
            WAIST = 0,
            SHOULDER = 1,
            ELBOW = 2,
            FOREARM_ROLL = 3,
            WRIST_ANGLE = 4,
            WRIST_ROTATE = 5,
            GRIPPER = 6,
            NONE = 7
        };
        static DOF defaultDOF;
        InterbotixJointName(Joint joint, DOF dof);
        Joint joint;
        DOF dof;
    };
}

#endif //INTERBOTIX_INTERBOTIX_JOINT_NAME_HPP
