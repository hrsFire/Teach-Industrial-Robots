#include "interbotix_robot_arm_ros.hpp"

using namespace interbotix;

const std::string InterbotixRobotArmROS::PLANNING_GROUP_INTERBOTIX_GROUP = "interbotix_arm";
const std::string InterbotixRobotArmROS::PLANNING_GROUP_GRIPPER_GROUP = "interbotix_gripper";

InterbotixRobotArmROS::InterbotixRobotArmROS(int argc, char** argv, std::string robotName, std::string robotModel) {
    ros::init(argc, argv, "interbotix_robot_arm");
    this->robotName = robotName;
    this->robotModel = robotModel;
    this->nodeHandlePtr = ros::NodeHandlePtr(new ros::NodeHandle());

    // "export ROS_NAMESPACE=wx200" needs to be set before
    moveit::planning_interface::MoveGroupInterface::Options interbotixMoveGroupOptions(PLANNING_GROUP_INTERBOTIX_GROUP, robotName + "/robot_description", ros::NodeHandle());
    this->interbotixMoveGroup = new moveit::planning_interface::MoveGroupInterface(interbotixMoveGroupOptions);
    moveit::planning_interface::MoveGroupInterface::Options gripperMoveGroupOptions(PLANNING_GROUP_GRIPPER_GROUP, robotName + "/robot_description", ros::NodeHandle());
    this->gripperMoveGroup = new moveit::planning_interface::MoveGroupInterface(gripperMoveGroupOptions);

    this->jointStatesSubscriber = this->nodeHandlePtr->subscribe<sensor_msgs::JointState>("/" + robotName + "/joint_states", 100, boost::bind(JointStatesCallback, boost::ref(*this), _1));

    this->setOperatingModeClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::OperatingModes>(robotName + "/" + robotName + "/set_operating_modes");
    this->getRobotInfoClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::RobotInfo>("/" + robotName + "/get_robot_info");

    jointTrajectoryClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/" + robotName + "/arm_controller/joint_trajectory");
    gripperTrajectoryClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/" + robotName + "/gripper_controller/gripper_trajectory");

    ros::master::V_TopicInfo topicInfos;
    bool isTopicTestRunning = true;

    while (isTopicTestRunning) {
        if (ros::master::getTopics(topicInfos)) {
            for (auto topicInfo : topicInfos) {
                if (topicInfo.name.rfind("/gazebo") == 0) {
                    this->usesGazebo = true;
                    isTopicTestRunning = false;
                    break;
                }
            }

            break;
        }
    }

    this->spinner = new ros::AsyncSpinner(0);
    this->spinner->start();

    while (!this->isArmInitialized) {
        ROS_INFO("Try to get information about the robot arm");
        // This is normally the time required to get the robot information from the subscriptions
        ros::Duration(0.5).sleep();
    }
}

InterbotixRobotArmROS::~InterbotixRobotArmROS() {
    spinner->stop();
    delete jointTrajectoryClient;
    delete gripperTrajectoryClient;
    interbotixMoveGroup->stop();
    delete interbotixMoveGroup;
    gripperMoveGroup->stop();
    delete gripperMoveGroup;
}

std::unordered_map<JointNameImpl, JointState> InterbotixRobotArmROS::GetJointStates() {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    return unorderedJointStates;
}

void InterbotixRobotArmROS::SendJointCommand(const JointName& jointName, double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());

    this->interbotixMoveGroup->setJointValueTarget(jointName, value);
    this->interbotixMoveGroup->move();

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates,
        jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendJointCommands(const std::unordered_map<JointNameImpl, double>& jointValues) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    std::unordered_map<JointNameImpl, double> newJointValues = jointValues;
    JointHelper::CheckJointValues(newJointValues, *GetRobotInfo());

    std::map<std::string, double> tmpJointValues;
    bool containsGripperValue = false;
    double gripperValue;

    for (auto jointValue : jointValues) {
        if (jointValue.first == InterbotixJointName::GRIPPER()) {
           containsGripperValue = true;
           gripperValue = jointValue.second; 
        }

        tmpJointValues.emplace(jointValue.first, jointValue.second);
    }

    this->interbotixMoveGroup->setJointValueTarget(tmpJointValues);
    this->interbotixMoveGroup->move();

    if (containsGripperValue) {
        SendGripperCommandUnlocked(gripperValue);
    }

    JointHelper::SetJointStates(newJointValues, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    control_msgs::FollowJointTrajectoryGoal message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

    jointTrajectoryClient->sendGoal(message);
}

void InterbotixRobotArmROS::SendGripperCommand(double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>(InterbotixJointName::GRIPPER()));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());
    SendGripperCommandUnlocked(value);

    JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void InterbotixRobotArmROS::SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    control_msgs::FollowJointTrajectoryGoal message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

    gripperTrajectoryClient->sendGoal(message);
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
    bool isSuccessful = false;

    if (usesGazebo) {
        // Gazebo does not publish robot arm information for interbotix robot arms. Therefore get the information from the configuration files.
        isSuccessful = InterbotixHelper::GetRobotInfoFromConfigFiles(this->robotName, res);
    } else if (robotInfo == nullptr) {
        isSuccessful = getRobotInfoClient.call(req, res);
    }

    if (isSuccessful) {
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
    std::map<std::string, double> jointValues;
    jointValues.emplace("left_finger", value / 2.0);
    jointValues.emplace("right_finger", -value / 2.0);
    this->gripperMoveGroup->setJointValueTarget(jointValues);
    this->gripperMoveGroup->move();
}