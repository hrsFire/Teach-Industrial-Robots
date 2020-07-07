#ifndef INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP
#define INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP

#include <mutex>
#include <unordered_map>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <interbotix_sdk/RobotInfo.h>
#include <interbotix_sdk/OperatingModes.h>
#include <interbotix_sdk/JointCommands.h>
#include <interbotix_sdk/SingleCommand.h>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <boost/bind.hpp>
#include "joint_helper.hpp"
#include "interbotix_helper.hpp"

using namespace robot_arm;

namespace interbotix {
    class InterbotixRobotArmROS : public RobotArmBase {
    public:
        InterbotixRobotArmROS(int argc, char** argv, std::string robotName, std::string robotModel);
        ~InterbotixRobotArmROS();
        std::unordered_map<JointNameImpl, JointState> GetJointStates() override;
        void SendJointCommand(const JointName& jointName, double value) override;
        void SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) override;
        void SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SendGripperCommand(double value) override;
        void SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) override;
        void SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
            int profileVelocity, int profileAcceleration) override;
        std::shared_ptr<RobotInfo> GetRobotInfo() override;
        double CalculateAcceleration(const JointName& jointName, std::chrono::milliseconds duration, bool isGoingUpwards) override;
    private:
        static void JointStatesCallback(InterbotixRobotArmROS& self, const sensor_msgs::JointStateConstPtr& message);
        static const std::string PLANNING_GROUP_INTERBOTIX_GROUP;
        static const std::string PLANNING_GROUP_GRIPPER_GROUP;
        std::vector<JointState> GetOrderedJointStates();
        void SendGripperCommandUnlocked(double value);
        ros::NodeHandlePtr nodeHandlePtr;
        ros::AsyncSpinner* spinner;
        std::mutex jointStatesMutex;
        std::vector<JointState> orderedJointStates;
        std::unordered_map<JointNameImpl, JointState> unorderedJointStates;
        std::shared_ptr<RobotInfo> robotInfo;
        std::unordered_map<JointNameImpl, OperatingMode> operatingModes;
        InterbotixJointName::DOF dof;
        bool isArmInitialized = false;
        std::chrono::high_resolution_clock::time_point jointStatesLastChanged;
        moveit::planning_interface::MoveGroupInterface* interbotixMoveGroup = nullptr;
        moveit::planning_interface::MoveGroupInterface* gripperMoveGroup = nullptr;
        std::string robotName;
        std::string robotModel;
        bool usesGazebo = false;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* jointTrajectoryClient;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* gripperTrajectoryClient;

        ros::Subscriber jointStatesSubscriber;

        ros::ServiceClient setOperatingModeClient;
        ros::ServiceClient getRobotInfoClient;
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP