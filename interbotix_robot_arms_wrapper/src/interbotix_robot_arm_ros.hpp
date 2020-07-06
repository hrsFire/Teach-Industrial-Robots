#ifndef INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP
#define INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP

#include <mutex>
#include <unordered_map>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <interbotix_sdk/RobotInfo.h>
#include <interbotix_sdk/OperatingModes.h>
#include <interbotix_sdk/JointCommands.h>
#include <interbotix_sdk/SingleCommand.h>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <boost/bind.hpp>
#include "joint_helper.hpp"

using namespace robot_arm;

namespace interbotix {
    class InterbotixRobotArmROS : public RobotArmBase {
    public:
        InterbotixRobotArmROS(int argc, char** argv, std::string robotName);
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

        ros::Subscriber jointStatesSubscriber;

        ros::Publisher jointCommandPublisher;
        ros::Publisher jointCommandsPublisher;
        ros::Publisher armControllerTrajectoryPublisher;
        ros::Publisher gripperCommandPublisher;
        ros::Publisher gripperTrajectoryPublisher;

        ros::ServiceClient setOperatingModeClient;
        ros::ServiceClient getRobotInfoClient;
    };
}

#endif //INTERBOTIX_INTERBOTIX_ROBOT_ARM_ROS_HPP