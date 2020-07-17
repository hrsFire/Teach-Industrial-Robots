#include "interbotix_robot_arm_ros.hpp"

using namespace interbotix;

const std::string InterbotixRobotArmROS::PLANNING_GROUP_INTERBOTIX_GROUP = "interbotix_arm";
const std::string InterbotixRobotArmROS::PLANNING_GROUP_GRIPPER_GROUP = "interbotix_gripper";

InterbotixRobotArmROS::InterbotixRobotArmROS(int argc, char** argv, std::string robotName, std::string robotModel) {
    ros::init(argc, argv, "interbotix_robot_arm", ros::init_options::NoSigintHandler);
    this->robotName = robotName;
    this->robotModel = robotModel;
    this->nodeHandlePtr = ros::NodeHandlePtr(new ros::NodeHandle());

#ifdef COMMUNICATION_MEASUREMENT
    this->communicationMeasurementFile = InterbotixHelper::InitializeMeasurementFile("interbotix_robot_arm_communication_measurement.csv");
#endif //COMMUNICATION_MEASUREMENT

    // "export ROS_NAMESPACE=wx200" needs to be set before
    moveit::planning_interface::MoveGroupInterface::Options interbotixMoveGroupOptions(PLANNING_GROUP_INTERBOTIX_GROUP, robotName + "/robot_description", ros::NodeHandle());
    this->interbotixMoveGroup = new moveit::planning_interface::MoveGroupInterface(interbotixMoveGroupOptions);
    this->interbotixMoveGroup->setPlanningTime(PLANNING_TIME);
    moveit::planning_interface::MoveGroupInterface::Options gripperMoveGroupOptions(PLANNING_GROUP_GRIPPER_GROUP, robotName + "/robot_description", ros::NodeHandle());
    this->gripperMoveGroup = new moveit::planning_interface::MoveGroupInterface(gripperMoveGroupOptions);
    this->gripperMoveGroup->setPlanningTime(PLANNING_TIME);

    this->jointStatesSubscriber = this->nodeHandlePtr->subscribe<sensor_msgs::JointState>("/" + robotName + "/joint_states", 100, boost::bind(JointStatesCallback, boost::ref(*this), _1));

    this->setOperatingModeClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::OperatingModes>(robotName + "/" + robotName + "/set_operating_modes");
    this->getRobotInfoClient = this->nodeHandlePtr->serviceClient<interbotix_sdk::RobotInfo>("/" + robotName + "/get_robot_info");

    this->jointTrajectoryClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/" + robotName + "/arm_controller/joint_trajectory");
    this->gripperTrajectoryClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/" + robotName + "/gripper_controller/gripper_trajectory");

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

#ifdef COMMUNICATION_MEASUREMENT
    delete communicationMeasurementFile;
#endif //COMMUNICATION_MEASUREMENT
}

std::unordered_map<JointNameImpl, JointState> InterbotixRobotArmROS::GetJointStates() {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    return unorderedJointStates;
}

bool InterbotixRobotArmROS::GetCurrentPose(const JointName& endEffectorJointName, geometry_msgs::Pose& pose) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    if (endEffectorJointName == InterbotixJointName::GRIPPER()) {
        if (isCurrentPoseValid) {
            pose = currentPose;
        } else {
            pose = currentPose = interbotixMoveGroup->getCurrentPose().pose;
            isCurrentPoseValid = true;
        }

        return true;
    } else {
        pose = interbotixMoveGroup->getCurrentPose(endEffectorJointName).pose;
        return true;
    }
}

void InterbotixRobotArmROS::SendJointCommand(const JointName& jointName, double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());

    SetCurrentJointValuesAfterPoseMode();

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    if (interbotixMoveGroup->setJointValueTarget(jointName, value)) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (interbotixMoveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#ifdef COMMUNICATION_MEASUREMENT
            if (interbotixMoveGroup->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#else
            if (interbotixMoveGroup->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#endif //COMMUNICATION_MEASUREMENT
                JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates,
                    jointStatesLastChanged, operatingModes);
                isCurrentPoseValid = false;

#ifdef COMMUNICATION_MEASUREMENT
                InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
            }
        }
    } else {
        interbotixMoveGroup->setJointValueTarget(interbotixMoveGroup->getCurrentJointValues());
        std::cout << (std::string) jointName << ": Could not set joint values" << std::endl;
    }
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

    SetCurrentJointValuesAfterPoseMode();
    bool isSuccessful = false;

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    if (interbotixMoveGroup->setJointValueTarget(tmpJointValues)) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (interbotixMoveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#ifdef COMMUNICATION_MEASUREMENT
            if (interbotixMoveGroup->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#else
            if (interbotixMoveGroup->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#endif //COMMUNICATION_MEASUREMENT
#ifdef COMMUNICATION_MEASUREMENT
                InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

                if (containsGripperValue) {
                    if (SendGripperCommandUnlocked(gripperValue)) {
                        isSuccessful = true;
                    }
                } else {
                    isSuccessful = true;
                }

                if (isSuccessful) {
                    JointHelper::SetJointStates(newJointValues, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
                }
            }
        }
    } else {
        interbotixMoveGroup->setJointValueTarget(interbotixMoveGroup->getCurrentJointValues());
    }

    if (!isSuccessful) {
        std::cout << "Could not set joint values" << std::endl;
    }

    isCurrentPoseValid = false;
}

void InterbotixRobotArmROS::SendJointTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    control_msgs::FollowJointTrajectoryGoal message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT

    jointTrajectoryClient->sendGoal(message);
    isCurrentPoseValid = false;
}

void InterbotixRobotArmROS::SendGripperCommand(double value) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    JointNameImpl alteredJointName = JointNameImpl(std::make_shared<InterbotixJointName>(InterbotixJointName::GRIPPER()));
    JointHelper::CheckJointValue(alteredJointName, value, *GetRobotInfo());
    SetCurrentJointValuesAfterPoseMode();

    if (SendGripperCommandUnlocked(value)) {
        JointHelper::SetJointState(alteredJointName, value, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
        isCurrentPoseValid = false;
    }
}

void InterbotixRobotArmROS::SendGripperTrajectory(const std::unordered_map<JointNameImpl, JointTrajectoryPoint>& jointTrajectoryPoints) {
    std::lock_guard<std::mutex> lock(jointStatesMutex);

    control_msgs::FollowJointTrajectoryGoal message;
    JointHelper::CopyToJointTrajectoryMessage(jointTrajectoryPoints, message);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    gripperTrajectoryClient->sendGoal(message);
    isCurrentPoseValid = false;

#ifdef COMMUNICATION_MEASUREMENT
    InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
}

void InterbotixRobotArmROS::SendPose(const geometry_msgs::Pose& pose, const JointName& endEffectorJointName) {
    if (isSendingMove) {
        return;
    }

    std::lock_guard<std::mutex> lock(jointStatesMutex);
    isSendingMove = true;
    std::string jointName;

    if (endEffectorJointName == InterbotixJointName::GRIPPER()) {
        jointName = interbotixMoveGroup->getEndEffectorLink();
    } else {
        jointName = endEffectorJointName;
    }

    if (SetPose(pose, jointName)) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

#ifdef COMMUNICATION_MEASUREMENT
        std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

        if (interbotixMoveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#ifdef COMMUNICATION_MEASUREMENT
            if (interbotixMoveGroup->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#else
            if (interbotixMoveGroup->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#endif //COMMUNICATION_MEASUREMENT
                currentPose = pose;
                positionedWithPose = true;

#ifdef COMMUNICATION_MEASUREMENT
                InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
            }
        }
    } else {
        interbotixMoveGroup->setJointValueTarget(interbotixMoveGroup->getCurrentJointValues());
        std::cout << (std::string) endEffectorJointName << ": Could not set joint values" << std::endl;
    }

    isSendingMove = false;
}

void InterbotixRobotArmROS::SetOperatingMode(const OperatingMode& operatingMode, const AffectedJoints& affectedJoints, const JointName& jointName,
        bool useCustomProfiles, int profileVelocity, int profileAcceleration) {
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

double InterbotixRobotArmROS::CalculateAccelerationDistance(const JointName& jointName, const std::chrono::milliseconds& duration) {
    return JointHelper::CalculateAccelerationDistance(jointName,
        operatingModes.at(JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName))), duration);
}

double InterbotixRobotArmROS::CalculateAccelerationDistance(const std::chrono::milliseconds& duration) {
    return JointHelper::CalculateAccelerationDistance(duration);
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

bool InterbotixRobotArmROS::SendGripperCommandUnlocked(double value) {
    std::map<std::string, double> jointValues;
    jointValues.emplace("left_finger", value / 2.0);
    jointValues.emplace("right_finger", -value / 2.0);

#ifdef COMMUNICATION_MEASUREMENT
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
#endif //COMMUNICATION_MEASUREMENT

    if (gripperMoveGroup->setJointValueTarget(jointValues)) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (gripperMoveGroup->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#ifdef COMMUNICATION_MEASUREMENT
            if (interbotixMoveGroup->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#else
            if (interbotixMoveGroup->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
#endif //COMMUNICATION_MEASUREMENT
#ifdef COMMUNICATION_MEASUREMENT
                InterbotixHelper::SaveCommunicationMeasurement(startTime, *communicationMeasurementFile);
#endif //COMMUNICATION_MEASUREMENT
            }

            return true;
        }
    } else {
        gripperMoveGroup->setJointValueTarget(gripperMoveGroup->getCurrentJointValues());
        std::cout << "Gripper: Could not set joint values" << std::endl;
    }

    return false;
}

void InterbotixRobotArmROS::SetCurrentJointValuesAfterPoseMode() {
    // By switching between the pose and joint mode MoveIt or/and the interbotiX node behave strangely.
    // The method "setJointValueTarget" should normally set the required joint and the non specified joints should be the same as before.
    // But this doesn't work, because the other joint values are set to 0 instead of the expected current values for the joints.
    if (positionedWithPose) {
        interbotixMoveGroup->setJointValueTarget(interbotixMoveGroup->getCurrentJointValues());
        gripperMoveGroup->setJointValueTarget(gripperMoveGroup->getCurrentJointValues());
        positionedWithPose = false;
    }
}

bool InterbotixRobotArmROS::SetPose(const geometry_msgs::Pose& pose, const std::string& endEffectorJointName) {
    bool isSuccessful = interbotixMoveGroup->setPositionTarget(pose.position.x, pose.position.y, pose.position.z, endEffectorJointName);
    std::shared_ptr<RobotInfo> robotInfo = GetRobotInfo();

    if (this->dof == InterbotixJointName::DOF::DOF_6) {
        isSuccessful = isSuccessful && interbotixMoveGroup->setOrientationTarget(pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w, endEffectorJointName);
    }

    return isSuccessful;
}