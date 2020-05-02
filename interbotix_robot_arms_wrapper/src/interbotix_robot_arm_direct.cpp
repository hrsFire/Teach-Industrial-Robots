#include "interbotix_robot_arm_direct.hpp"

using namespace interbotix;

InterbotixRobotArmDirect::InterbotixRobotArmDirect(int argc, char** argv, std::string robotName, std::string robotModel) {
    ros::init(argc, argv, "interbotix_robot_arm");

    // TODO: clean up
    /*ros::param::get("~robot_name", robot_name);
    ros::param::get("~robot_model", robot_model);*/
    // https://github.com/Interbotix/interbotix_ros_arms/blob/melodic/interbotix_sdk/launch/arm_run.launch
    ros::param::set("~motor_configs", "/home/ubuntu/interbotix_ws/src/interbotix_ros_arms/interbotix_sdk/config/");
    this->robotArm = new RobotArm(robotName, robotModel);
    SetOperatingMode(OperatingMode::POSITION(), AffectedJoints::ARM_JOINTS_AND_GRIPPER, JointName::NONE(), false, 0, 0);
}

InterbotixRobotArmDirect::~InterbotixRobotArmDirect() {
    delete robotArm;
}

std::vector<JointState> InterbotixRobotArmDirect::GetJointStates() {
    sensor_msgs::JointState states = robotArm->arm_get_joint_states();

    std::lock_guard<std::mutex> _(jointStatesMutex);
#
    jointStates.clear();

    for (size_t i = 0; i < states.name.size(); i++) {
        jointStates.push_back(JointState(states.name[i], states.position[i], states.velocity[i], states.effort[i]));  // TODo: mode?
    }

    return std::vector<JointState>();
}

void InterbotixRobotArmDirect::SendJointCommand(JointName jointName, double value) {
    interbotix_sdk::SingleCommand message;
    message.joint_name = jointName;
    message.cmd = value;

    robotArm->arm_send_single_joint_command(message);
}

void InterbotixRobotArmDirect::SendJointCommands(const std::vector<JointName>& jointNames, const std::vector<double>& values) {
    std::vector<JointState> jointStates;
    jointStates = GetJointStates();    
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

    if (robotInfo != nullptr) {
        if (robotArm->arm_get_robot_info(res)) {
            std::vector<JointName> jointNames;
            
            for (std::string joint : res.joint_names) {
                jointNames.push_back(joint);
            }

            std::vector<int> jointIDs;

            for (int16_t id : res.joint_ids) {
                jointIDs.push_back(id);
            }

            robotInfo.reset(new RobotInfo(jointNames, jointIDs, res.lower_joint_limits, res.upper_joint_limits, res.velocity_limits,
                res.lower_gripper_limit, res.upper_gripper_limit, res.use_gripper, res.home_pos, res.sleep_pos, res.num_joints, res.num_single_joints));
        }
    }

    if (robotInfo != nullptr) {
        return robotInfo;
    }

    throw "Could not retrieve the robot info";
}