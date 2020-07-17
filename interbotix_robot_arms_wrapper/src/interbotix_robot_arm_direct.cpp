#include "interbotix_robot_arm_direct.hpp"

#ifdef COMMUNICATION_MEASUREMENT
#include "interbotix_helper.hpp"
#endif //COMMUNICATION_MEASUREMENT

using namespace interbotix;

InterbotixRobotArmDirect::InterbotixRobotArmDirect(int argc, char** argv, std::string robotName, std::string robotModel) {
    ros::init(argc, argv, "interbotix_robot_arm", ros::init_options::NoSigintHandler);

#ifdef COMMUNICATION_MEASUREMENT
    this->communicationMeasurementFile = InterbotixHelper::InitializeMeasurementFile("interbotix_robot_arm_communication_measurement.csv");
#endif //COMMUNICATION_MEASUREMENT

    // https://github.com/Interbotix/interbotix_ros_arms/blob/melodic/interbotix_sdk/launch/arm_run.launch
    std::string interbotixSDKPath = ros::package::getPath("interbotix_sdk");
    ros::param::set("~motor_configs", interbotixSDKPath + "/config/");
    ros::param::set("~use_default_gripper_bar", true);
    ros::param::set("~use_default_gripper_fingers", true);
    this->robotArm = new RobotArm(robotName, robotModel);

    GetRobotInfo();
    SetOperatingMode(OperatingMode::POSITION(), AffectedJoints::ARM_JOINTS_AND_GRIPPER, InterbotixJointName::NONE(), false, 0, 0);
}

InterbotixRobotArmDirect::~InterbotixRobotArmDirect() {
    delete robotArm;

#ifdef COMMUNICATION_MEASUREMENT
    delete communicationMeasurementFile;
#endif //COMMUNICATION_MEASUREMENT
}

std::unordered_map<JointNameImpl, JointState> InterbotixRobotArmDirect::GetJointStates() {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    sensor_msgs::JointState states = robotArm->arm_get_joint_states();

    JointHelper::PrepareJointStates(&orderedJointStates, &unorderedJointStates, states.name, states.position, states.velocity, states.effort, operatingModes, dof, jointStatesLastChanged);

    return unorderedJointStates;
}

bool InterbotixRobotArmDirect::GetCurrentPose(const JointName& endEffectorJointName, geometry_msgs::Pose& pose) {
    return false; // TODO: find an implemenation similiar to ROS ones, because otherwise the tests aren't really statistically representative
}

void InterbotixRobotArmDirect::SendJointCommand(const JointName& jointName, double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());

    interbotix_sdk::SingleCommand message;
    message.joint_name = jointName;
    message.cmd = value;

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    robotArm->arm_send_single_joint_command(message);

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates,
        jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmDirect::SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    std::unordered_map<JointNameImpl, double> newJointValues = jointValues;
    JointHelper::CheckJointValues(newJointValues, *GetRobotInfo());

    std::vector<JointState> jointStates;
    jointStates = GetOrderedJointStates();
    interbotix_sdk::JointCommands message;
    message.cmd = JointHelper::PrepareJointCommands(newJointValues, *robotInfo, jointStates);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    robotArm->arm_send_joint_commands(message);

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

    auto gripper = newJointValues.find(InterbotixJointName::GRIPPER());

    if (gripper != newJointValues.end()) {
#ifdef COMMUNICATION_MEASUREMENT
        startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

        SendGripperCommandUnlocked(gripper->second);

#ifdef COMMUNICATION_MEASUREMENT
        InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
    }

    JointHelper::SetJointStates(newJointValues, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmDirect::SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    robotArm->arm_send_joint_trajectory(message);

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
}

void InterbotixRobotArmDirect::SendGripperCommand(double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>(InterbotixJointName::GRIPPER()));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    SendGripperCommandUnlocked(value);

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates,
        jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmDirect::SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    robotArm->send_gripper_trajectory(message);

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
}

void InterbotixRobotArmDirect::SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) {
     // TODO: find an implemenation similiar to ROS ones, because otherwise the tests aren't really statistically representative
}

void InterbotixRobotArmDirect::SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
        int profileVelocity, int profileAcceleration) {
   interbotix_sdk::OperatingModesRequest req;
   interbotix_sdk::OperatingModesResponse res;

   req.mode = OperatingMode(operatingMode);
   req.cmd = JointHelper::GetAffectedJoints(affectedJoints);
   req.joint_name = InterbotixJointName(jointName, dof);
   req.use_custom_profiles = useCustomProfiles;
   req.profile_velocity = profileVelocity;
   req.profile_acceleration = profileAcceleration;

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

   if (robotArm->arm_set_operating_modes(req)) {
#ifdef COMMUNICATION_MEASUREMENT
        InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

       JointHelper::SetOperatingMode(operatingModes, jointName, operatingMode);
   }
}

std::shared_ptr<RobotInfo> InterbotixRobotArmDirect::GetRobotInfo() {
    interbotix_sdk::RobotInfoRequest req;
    interbotix_sdk::RobotInfoResponse res;

    if (robotInfo == nullptr) {
        if (robotArm->arm_get_robot_info(res)) {
            std::vector<JointNameImpl> jointNames;
            std::vector<int> jointIDs;
            std::vector<double> lowerJointLimits;
            std::vector<double> upperJointLimits;
            std::unordered_map<JointNameImpl, double> homePosition;
            std::unordered_map<JointNameImpl, double> sleepPosition;

            // Initialize DOF and operating modes
            InterbotixJointName::DOF dof = JointHelper::DetermineDOF(res.num_joints);
            InterbotixJointName::SetDefaultDOF(dof);
            this->dof = dof;
            this->operatingModes = JointHelper::GetInitialOperatingModes(dof);

            JointHelper::PrepareRobotInfoJoints(res.joint_names, jointNames, res.joint_ids, jointIDs, res.home_pos, homePosition, res.sleep_pos, sleepPosition,
                res.lower_joint_limits, lowerJointLimits, res.upper_joint_limits, upperJointLimits, res.lower_gripper_limit, res.upper_gripper_limit, res.use_gripper,
                dof);

            robotInfo.reset(new RobotInfo(JointHelper::CreateJoints(jointNames, jointIDs, lowerJointLimits, upperJointLimits, res.velocity_limits),
                res.use_gripper, homePosition, sleepPosition, res.num_joints, res.num_single_joints));
        }
    }

    if (robotInfo != nullptr) {
        return robotInfo;
    }

    throw "Could not retrieve the robot info";
}

double InterbotixRobotArmDirect::CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) {
    return JointHelper::CalculateAccelerationDistance(jointName,
        operatingModes.at(JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName))), duration);
}

double InterbotixRobotArmDirect::CalculateAccelerationDistance(const std::chrono::milliseconds& duration) {
    return JointHelper::CalculateAccelerationDistance(duration);
}

std::vector<JointState> InterbotixRobotArmDirect::GetOrderedJointStates() {
    sensor_msgs::JointState states = robotArm->arm_get_joint_states();

    JointHelper::PrepareJointStates(&orderedJointStates, &unorderedJointStates, states.name, states.position, states.velocity, states.effort, operatingModes, dof, jointStatesLastChanged);

    return orderedJointStates;
}

void InterbotixRobotArmDirect::SendGripperCommandUnlocked(double value) {
    std_msgs::Float64 message;
    message.data = value;

    robotArm->arm_send_gripper_command(message);
}