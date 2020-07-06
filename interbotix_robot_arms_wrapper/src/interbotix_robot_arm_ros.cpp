#include "interbotix_robot_arm_ros.hpp"

using namespace interbotix;

InterbotixRobotArmROS::InterbotixRobotArmROS(int argc, char** argv, std::string robotName) {
    ros::init(argc, argv, "interbotix_robot_arm");
    this->nodeHandlePtr = ros::NodeHandlePtr(new ros::NodeHandle());

    this->jointStatesSubscriber = this->nodeHandlePtr->subscribe<sensor_msgs::JointState>(robotName + "/joint_states", 100, boost::bind(JointStatesCallback, boost::ref(*this), _1));

    this->jointCommandPublisher = this->nodeHandlePtr->advertise<interbotix_sdk::SingleCommand>(robotName + "/single_joint/command", 100);
    this->jointCommandsPublisher = this->nodeHandlePtr->advertise<interbotix_sdk::JointCommands>(robotName + "/joint/commands", 100);
    this->armControllerTrajectoryPublisher = this->nodeHandlePtr->advertise<trajectory_msgs::JointTrajectory>(robotName + "/arm_controller/joint_trajectory", 100);
    this->gripperCommandPublisher = this->nodeHandlePtr->advertise<std_msgs::Float64>(robotName + "/gripper/command", 100);
    this->gripperTrajectoryPublisher = this->nodeHandlePtr->advertise<trajectory_msgs::JointTrajectory>(robotName + "/gripper_controller/gripper_trajectory", 100);

    this->setOperatingModeClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::OperatingModes>(robotName + "/set_operating_modes");
    this->getRobotInfoClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::RobotInfo>(robotName + "/get_robot_info");

    this->spinner = new ros::AsyncSpinner(0);
    this->spinner->start();

    // This is normally the time required to get the robot information from the subscriptions
    ros::Duration(0.5).sleep();

    while (!isArmInitialized) {
        ROS_INFO("Try to get information about the robot arm");
        ros::Duration(0.5).sleep();
    }

    GetRobotInfo();
}

InterbotixRobotArmROS::~InterbotixRobotArmROS() {
    spinner->stop();
}

std::unordered_map<JointNameImpl, JointState> InterbotixRobotArmROS::GetJointStates() {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    return unorderedJointStates;
}

void InterbotixRobotArmROS::SendJointCommand(const JointName& jointName, double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());

    interbotix_sdk::SingleCommand message;
    message.joint_name = jointName;
    message.cmd = value;

    jointCommandPublisher.publish(message);

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates,
        jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    std::unordered_map<JointNameImpl, double> newJointValues = jointValues;
    JointHelper::CheckJointValues(newJointValues, *GetRobotInfo());

    interbotix_sdk::JointCommands message;
    std::shared_ptr<RobotInfo> robotInfo = GetRobotInfo();
    std::vector<JointState> jointStates = GetOrderedJointStates();
    message.cmd = JointHelper::PrepareJointCommands(newJointValues, *robotInfo, jointStates);

    jointCommandsPublisher.publish(message);

    auto gripper = newJointValues.find(InterbotixJointName::GRIPPER());

    if (gripper != newJointValues.end()) {
        SendGripperCommandUnlocked(gripper->second);
    }

    JointHelper::SetJointStates(newJointValues, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

    armControllerTrajectoryPublisher.publish(message);

    // @TODO: perhaps use MoveIt! ?
}

void InterbotixRobotArmROS::SendGripperCommand(double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>(InterbotixJointName::GRIPPER()));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());
    SendGripperCommandUnlocked(value);

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    trajectory_msgs::JointTrajectory message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

    gripperTrajectoryPublisher.publish(message);

    // @TODO: perhaps use MoveIt! ?
}

void InterbotixRobotArmROS::SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName, bool useCustomProfiles,
        int profileVelocity, int profileAcceleration) {
    interbotix_sdk::OperatingModesRequest req;
    interbotix_sdk::OperatingModesResponse res;

    req.mode = OperatingMode(operatingMode);
    req.cmd = JointHelper::GetAffectedJoints(affectedJoints);
    req.joint_name = jointName;
    req.use_custom_profiles = useCustomProfiles;
    req.profile_velocity = profileVelocity;
    req.profile_acceleration = profileAcceleration;

    if (!setOperatingModeClient.call(req, res)) {
        ROS_ERROR("Could not set operating mode for '%s'", ((std::string) jointName).c_str());
    } else {
        JointHelper::SetOperatingMode(operatingModes, jointName, operatingMode);
    }
}

std::shared_ptr<RobotInfo> InterbotixRobotArmROS::GetRobotInfo() {
    interbotix_sdk::RobotInfoRequest req;
    interbotix_sdk::RobotInfoResponse res;

    if (robotInfo == nullptr && getRobotInfoClient.call(req, res)) {
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
            res.lower_joint_limits, lowerJointLimits, res.upper_joint_limits, upperJointLimits, res.lower_gripper_limit, res.upper_gripper_limit, res.use_gripper, dof);

        robotInfo.reset(new RobotInfo(JointHelper::CreateJoints(jointNames, jointIDs, lowerJointLimits, upperJointLimits, res.velocity_limits),
            res.use_gripper, homePosition, sleepPosition, res.num_joints, res.num_single_joints));
    }

    if (robotInfo != nullptr) {
        return robotInfo;
    }

    throw "Could not retrieve the robot info";
}

double InterbotixRobotArmROS::CalculateAcceleration(const JointName& jointName, std::chrono::milliseconds duration, bool isGoingUpwards) {
    return JointHelper::CalculateAcceleration(jointName,
        operatingModes.at(JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName))), duration, isGoingUpwards);
}

void InterbotixRobotArmROS::JointStatesCallback(InterbotixRobotArmROS& self, const sensor_msgs::JointStateConstPtr& message) {
    std::lock_guard<std::mutex> lock(self.jointStatesMutex);

    if (!self.isArmInitialized) {
        self.GetRobotInfo();
        self.isArmInitialized = true;
    }

    JointHelper::PrepareJointStates(&self.orderedJointStates, &self.unorderedJointStates, message->name, message->position, message->velocity, message->effort,
        self.operatingModes, self.dof, self.jointStatesLastChanged);
}

std::vector<JointState> InterbotixRobotArmROS::GetOrderedJointStates() {
    return orderedJointStates;
}

void InterbotixRobotArmROS::SendGripperCommandUnlocked(double value) {
    std_msgs::Float64 message;
    message.data = value;

    gripperCommandPublisher.publish(message);
}