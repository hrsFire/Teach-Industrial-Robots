#ifndef INTERBOTIX_INTERBOTIX_JOINT_NAME_HPP
#define INTERBOTIX_INTERBOTIX_JOINT_NAME_HPP

#include <string>
#include <stdexcept>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>

namespace interbotix {
    class InterbotixJointName : public robot_arm::JointName {
    public:
        enum class DOF {
            DOF_4 = 4,
            DOF_5 = 5,
            DOF_6 = 6
        };
        InterbotixJointName(std::string joint, DOF dof);
        static InterbotixJointName WAIST();
        static InterbotixJointName SHOULDER();
        static InterbotixJointName ELBOW();
        // Only available with 6 DOF
        static InterbotixJointName FOREARM_ROLL();
        static InterbotixJointName WRIST_ANGLE();
        // Only available with 5 or 6 DOF
        static InterbotixJointName WRIST_ROTATE();
        static InterbotixJointName GRIPPER();
        static InterbotixJointName NONE();
        static void SetDefaultDOF(DOF dof);
        operator std::string() const override;
        operator robot_arm::JointNameImpl() const;
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