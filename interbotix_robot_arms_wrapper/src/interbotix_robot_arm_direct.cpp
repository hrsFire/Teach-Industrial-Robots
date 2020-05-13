#include "interbotix_robot_arm_direct.hpp"

using namespace interbotix;

InterbotixRobotArmDirect::InterbotixRobotArmDirect(int argc, char** argv, std::string robotName, std::string robotModel) {
    ros::init(argc, argv, "interbotix_robot_arm");

    // TODO: clean up
    /*ros::param::get("~robot_name", robot_name);
    ros::param::get("~robot_model", robot_model);*/
    // https://github.com/Interbotix/interbotix_ros_arms/blob/melodic/interbotix_sdk/launch/arm_run.launch
    ros::param::set("~motor_configs", "/home/ubuntu/interbotix_ws/src/interbotix_ros_arms/interbotix_sdk/config/");
    ros::param::set("~use_default_gripper_bar", true);
    ros::param::set("~use_default_gripper_fingers", true);
    this->robotArm = new RobotArm(robotName, robotModel);
    SetOperatingMode(OperatingMode::POSITION(), AffectedJoints::ARM_JOINTS_AND_GRIPPER, JointName::NONE(), false, 0, 0);
}

InterbotixRobotArmDirect::~InterbotixRobotArmDirect() {
    delete robotArm;
}

std::unordered_map<JointName, JointState> InterbotixRobotArmDirect::GetJointStates() {
    sensor_msgs::JointState states = robotArm->arm_get_joint_states();
    std::unordered_map<JointName, JointState> jointStates;

    JointHelper::PrepareJointStates(nullptr, &jointStates, states.name, states.position, states.velocity, states.effort);

    return jointStates;
}

void InterbotixRobotArmDirect::SendJointCommand(JointName jointName, double value) {
    interbotix_sdk::SingleCommand message;
    message.joint_name = jointName;
    message.cmd = value;

    robotArm->arm_send_single_joint_command(message);
}

void InterbotixRobotArmDirect::SendJointCommands(const std::vector<JointName>& jointNames, const std::vector<double>& values) {
    std::vector<JointState> jointStates;
    jointStates = GetOrderedJointStates();
    std::vector<double> cmds = JointHelper::PrepareJointCommands(jointNames, values, *robotInfo, jointStates);
    interbotix_sdk::JointCommands message;
    message.cmd = values;

    robotArm->arm_send_joint_commands(message);
}

void InterbotixRobotArmDirect::SendJointTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointNames, jointTrajectoryPoints, message);

    robotArm->arm_send_joint_trajectory(message);
}

void InterbotixRobotArmDirect::SendGripperCommand(double value) {
    std_msgs::Float64 message;
    message.data = value;

    robotArm->arm_send_gripper_command(message);
}

void InterbotixRobotArmDirect::SendGripperTrajectory(const std::vector<JointName>& jointNames, const std::vector<JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointNames, jointTrajectoryPoints, message);

    robotArm->send_gripper_trajectory(message);
}

void InterbotixRobotArmDirect::SetTorqueState(bool on) {
    if (on) {
        robotArm->arm_torque_on();
    } else {
        robotArm->arm_torque_off();
    }
}

void InterbotixRobotArmDirect::SetOperatingMode(OperatingMode operatingMode, AffectedJoints affectedJoints, JointName jointName, bool useCustomProfiles,
        int profileVelocity, int profileAcceleration) {
   interbotix_sdk::OperatingModesRequest req;
   interbotix_sdk::OperatingModesResponse res;

   req.mode = operatingMode;
   req.cmd = JointHelper::GetAffectedJoints(affectedJoints);
   req.joint_name = jointName;
   req.use_custom_profiles = useCustomProfiles;
   req.profile_velocity = profileVelocity;
   req.profile_acceleration = profileAcceleration;

   robotArm->arm_set_operating_modes(req);
}

std::shared_ptr<RobotInfo> InterbotixRobotArmDirect::GetRobotInfo() {
    interbotix_sdk::RobotInfoRequest req;
    interbotix_sdk::RobotInfoResponse res;

    if (robotInfo == nullptr) {
        if (robotArm->arm_get_robot_info(res)) {
            std::vector<JointName> jointNames;
            std::vector<int> jointIDs;
            std::vector<double> lowerJointLimits;
            std::vector<double> upperJointLimits;

            JointHelper::PrepareRobotInfoJoints(res.joint_names, jointNames, res.joint_ids, jointIDs, res.lower_joint_limits, lowerJointLimits, res.upper_joint_limits,
                upperJointLimits, res.lower_gripper_limit, res.upper_gripper_limit, res.use_gripper);

            robotInfo.reset(new RobotInfo(JointHelper::CreateJoints(jointNames, jointIDs, lowerJointLimits, upperJointLimits, res.velocity_limits),
                res.use_gripper, res.home_pos, res.sleep_pos, res.num_joints, res.num_single_joints));
        }
    }

    if (robotInfo != nullptr) {
        return robotInfo;
    }

    throw "Could not retrieve the robot info";
}

std::vector<JointState> InterbotixRobotArmDirect::GetOrderedJointStates() {
    sensor_msgs::JointState states = robotArm->arm_get_joint_states();
    std::vector<JointState> jointStates;

    JointHelper::PrepareJointStates(&jointStates, nullptr, states.name, states.position, states.velocity, states.effort);

    return jointStates;
}