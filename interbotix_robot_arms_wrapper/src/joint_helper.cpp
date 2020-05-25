#include "joint_helper.hpp"

using namespace interbotix;

// Sort and add additonal values
std::vector<double> JointHelper::PrepareJointCommands(const std::unordered_map<robot_arm::JointNameImpl, double>& jointValues, robot_arm::RobotInfo& robotInfo,
        const std::vector<robot_arm::JointState>& jointStates) {
    std::vector<double> sortedValues;
    sortedValues.reserve(jointStates.size());

    for (size_t i = 0; i < jointStates.size(); i++) {
        for (auto jointValue : jointValues) {
            // @TODO: causes a bad cast exception if the items are switched
            if (jointValue.first == *(jointStates[i].GetJointName())) {
                sortedValues.push_back(jointValue.second);
                break;
            }
        }
    }

    return sortedValues;
}

int8_t JointHelper::GetAffectedJoints(const robot_arm::AffectedJoints& affectedJoints) {
    switch (affectedJoints) {
        case robot_arm::AffectedJoints::ARM_JOINTS_AND_GRIPPER:
            return interbotix_sdk::OperatingModesRequest::ARM_JOINTS_AND_GRIPPER;
        case robot_arm::AffectedJoints::ARM_JOINTS:
            return interbotix_sdk::OperatingModesRequest::ARM_JOINTS;
        case robot_arm::AffectedJoints::GRIPPER:
            return interbotix_sdk::OperatingModesRequest::GRIPPER;
        case robot_arm::AffectedJoints::SINGLE_JOINT:
            return interbotix_sdk::OperatingModesRequest::SINGLE_JOINT;
    }

    return -1;
}

void JointHelper::CopyToJointTrajectoryMessage(const std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints,
        trajectory_msgs::JointTrajectory& message) {
    for (auto jointTrajectoryPoint : jointTrajectoryPoints) {
        message.joint_names.push_back(jointTrajectoryPoint.first);

        robot_arm::JointTrajectoryPoint& point = jointTrajectoryPoint.second;
        trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
        trajectoryPoint.accelerations = point.GetAccelerations();
        trajectoryPoint.effort = point.GetEfforts();
        trajectoryPoint.positions = point.GetPositions();
        trajectoryPoint.time_from_start = ros::Duration(point.GetSecondsFromStart());
        trajectoryPoint.velocities = point.GetVelocities();

        message.points.push_back(trajectoryPoint);
    }
}

std::unordered_map<robot_arm::JointNameImpl, robot_arm::Joint> JointHelper::CreateJoints(const std::vector<robot_arm::JointNameImpl>& jointNames,
        const std::vector<int>& jointIDs, const std::vector<double>& lowerJointLimits, const std::vector<double> upperJointLimits,
        const std::vector<double>& velocityLimits) {
    std::unordered_map<robot_arm::JointNameImpl, robot_arm::Joint> joints(jointNames.size());
    double lowerJointDeviation;

    for (size_t i = 0; i < jointNames.size(); i++) {
        if (jointNames[i] == InterbotixJointName::GRIPPER()) {
            lowerJointDeviation = 0;
        } else {
            // WORKAROUND: + 0.001739 prevents the driver from throwing warnings
            lowerJointDeviation = 0.001739;
        }

        joints.emplace(jointNames[i], robot_arm::Joint(jointNames[i], jointIDs[i], lowerJointLimits[i] + lowerJointDeviation, upperJointLimits[i], velocityLimits[i]));
    }

    return joints;
}

void JointHelper::PrepareRobotInfoJoints(const std::vector<std::string>& jointNames, std::vector<robot_arm::JointNameImpl>& newJointNames,
            const std::vector<int16_t>& jointIDs, std::vector<int>& newJointIDs, const std::vector<double>& homePosition,
            std::unordered_map<robot_arm::JointNameImpl, double>& newHomePosition, const std::vector<double>& sleepPosition,
            std::unordered_map<robot_arm::JointNameImpl, double>& newSleepPosition, const std::vector<double>& lowerJointLimits, std::vector<double>& newLowerJointLimits,
            const std::vector<double>& upperJointLimits, std::vector<double>& newUpperJointLimits, double lowerGripperLimit, double upperGripperLimit, bool useGripper,
            InterbotixJointName::DOF dof) {
    int gripperIndex = 0;
    bool gripperIndexFound = false;
    newJointNames.reserve(jointNames.size());
    newJointIDs.reserve(jointIDs.size());
    newLowerJointLimits.reserve(lowerJointLimits.size());
    newUpperJointLimits.reserve(upperJointLimits.size());
    newHomePosition.reserve(homePosition.size());
    newSleepPosition.reserve(sleepPosition.size());

    for (const std::string& joint : jointNames) {
        newJointNames.push_back(robot_arm::JointNameImpl(std::make_shared<InterbotixJointName>(joint, dof)));

        if (!gripperIndexFound) {
            if (joint == (std::string) InterbotixJointName::GRIPPER()) {
                gripperIndexFound = true;
            } else {
                gripperIndex++;
            }
        }
    }

    for (int16_t id : jointIDs) {
        newJointIDs.push_back(id);
    }

    for (size_t i = 0; i < lowerJointLimits.size(); i++) {
        if (i != gripperIndex || !useGripper) {
            newLowerJointLimits.push_back(lowerJointLimits[i]);
        } else {
            newLowerJointLimits.push_back(lowerGripperLimit + 0.01);
        }
    }

    for (size_t i = 0; i < upperJointLimits.size(); i++) {
        if (i != gripperIndex || !useGripper) {
            newUpperJointLimits.push_back(upperJointLimits[i]);
        } else {
            newUpperJointLimits.push_back(upperGripperLimit * 2.0);
        }
    }

    for (size_t i = 0; i < homePosition.size(); i++) {
        newHomePosition.emplace(std::make_shared<InterbotixJointName>(jointNames[i], dof), homePosition[i]);
    }

    for (size_t i = 0; i < sleepPosition.size(); i++) {
        newSleepPosition.emplace(std::make_shared<InterbotixJointName>(jointNames[i], dof), sleepPosition[i]);
    }
}

void JointHelper::PrepareJointStates(std::vector<robot_arm::JointState>* orderedJointStates, std::unordered_map<robot_arm::JointNameImpl,
        robot_arm::JointState>* unorderedJointStates, const std::vector<std::string>& jointNames, const std::vector<double>& jointPositions,
        const std::vector<double>& jointVelocities, const std::vector<double>& jointEfforts, const std::unordered_map<robot_arm::JointNameImpl,
        robot_arm::OperatingMode>& operatingModes, InterbotixJointName::DOF dof, std::chrono::system_clock::time_point& jointStatesLastChanged) {
    std::string jointName;
    double position;

    if (!JointHelper::HaveJointStatesExpired(jointStatesLastChanged)) {
        return;
    }

    if (orderedJointStates != nullptr) {
        orderedJointStates->clear();
        orderedJointStates->reserve(jointNames.size());
    }

    if (unorderedJointStates != nullptr) {
        unorderedJointStates->clear();
        unorderedJointStates->reserve(jointNames.size());
    }

    for (size_t i = 0; i < jointNames.size(); i++) {
        // Ignore some unused joint names
        if (jointNames[i] == (std::string) InterbotixJointName::GRIPPER() || jointNames[i] == "right_finger") {
            continue;
        } else if (jointNames[i] == "left_finger") {
            jointName = InterbotixJointName::GRIPPER();
            position = jointPositions[i] * 2.0;
        } else {
            jointName = jointNames[i];
            position = jointPositions[i];
        }

        std::shared_ptr<robot_arm::JointName> jointNamePtr;

        if (orderedJointStates != nullptr || unorderedJointStates != nullptr) {
            jointNamePtr.reset(new InterbotixJointName(jointName, dof));
        }

        robot_arm::JointNameImpl jointNameImpl = robot_arm::JointNameImpl(std::make_shared<InterbotixJointName>(jointName, dof));

        if (orderedJointStates != nullptr) {
            orderedJointStates->push_back(robot_arm::JointState(jointNamePtr, jointPositions[i], jointVelocities[i], jointEfforts[i],
                operatingModes.at(jointNameImpl)));
        }

        if (unorderedJointStates != nullptr) {
            unorderedJointStates->emplace(jointNameImpl, robot_arm::JointState(jointNamePtr, position, jointVelocities[i], jointEfforts[i],
                operatingModes.at(jointNameImpl)));
        }
    }

    jointStatesLastChanged = std::chrono::high_resolution_clock::now();
}

void JointHelper::SetOperatingMode(std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes,
        const robot_arm::JointName& jointName, const robot_arm::OperatingMode operatingMode) {
    auto it = operatingModes.find(robot_arm::JointNameImpl(std::make_shared<InterbotixJointName>((const InterbotixJointName&) jointName)));

    if (it != operatingModes.end()) {
        it->second = operatingMode;
    }
}

std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode> JointHelper::GetInitialOperatingModes(InterbotixJointName::DOF dof) {
    std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode> operatingModes((int) dof);

    operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::WAIST()), robot_arm::OperatingMode::POSITION());
    operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::SHOULDER()), robot_arm::OperatingMode::POSITION());
    operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::ELBOW()), robot_arm::OperatingMode::POSITION());

    if (dof == InterbotixJointName::DOF::DOF_6) {
        operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::FOREARM_ROLL()), robot_arm::OperatingMode::POSITION());
    }

    operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::WRIST_ANGLE()), robot_arm::OperatingMode::POSITION());

    if (dof == InterbotixJointName::DOF::DOF_5 || dof == InterbotixJointName::DOF::DOF_6) {
        operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::WRIST_ROTATE()), robot_arm::OperatingMode::POSITION());
    }

    operatingModes.emplace(std::make_shared<InterbotixJointName>(InterbotixJointName::GRIPPER()), robot_arm::OperatingMode::POSITION());

    return operatingModes;
}

double JointHelper::CalculateAcceleration(const robot_arm::JointName& jointName, const robot_arm::OperatingMode& operatingMode, std::chrono::milliseconds duration,
        bool isGoingUpwards) {
    if (operatingMode == robot_arm::OperatingMode::POSITION() || operatingMode == robot_arm::OperatingMode::POSITION_MULTIPLE_REVOLUTIONS()) {
        // Empirically determined the best values for the logarithm base for the WidowX 200 seem to be in the range e < x < 3.1.
        // This smooths down the movement and prevents to abrupt movements.
        double base = 2.9;
        std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        // This handles the cases for log(0) = undefined, log(1) = 0, log(x) < 0 when x < 1.0 and log(x)/log(base) <= 1 when x <= base
        double factor = seconds >= std::chrono::seconds(0) && seconds <= std::chrono::seconds(2) && seconds.count() <= base ? 1.0 : log(seconds.count()) / log(base);

        if (jointName == InterbotixJointName::GRIPPER()) {
            return GRIPPER_CHANGE * factor;
        } else {
            return JOINT_ANGLE_CHANGE * factor;
        }
    } else if (operatingMode == robot_arm::OperatingMode::VELOCITY()) {
        return 1;
    } else if (operatingMode == robot_arm::OperatingMode::CURRENT()) {
        return 0;
    } else if (operatingMode == robot_arm::OperatingMode::PWM()) {
        return 0;
    } else {
        return 0;
    }
}

InterbotixJointName::DOF JointHelper::DetermineDOF(const robot_arm::RobotInfo& robotInfo) {
    return DetermineDOF(robotInfo.numberOfJoints);
}

InterbotixJointName::DOF JointHelper::DetermineDOF(uint numberOfJoints) {
    switch (numberOfJoints) {
        case 4:
            return InterbotixJointName::DOF::DOF_4;
        case 5:
            return InterbotixJointName::DOF::DOF_5;
        case 6:
            return InterbotixJointName::DOF::DOF_6;
        default:
            throw "This Interbotix robot arm isn't supported yet. It has %i DOF (degrees of freedom). Only 4, 5 and 6 DOF are currently supported.";
    }
}

void JointHelper::SetJointState(const robot_arm::JointNameImpl& newJointStateName, double newJointStateValue, std::vector<robot_arm::JointState>& orderedJointStates,
        std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>& unorderedJointStates, std::chrono::high_resolution_clock::time_point& jointStatesLastChanged,
        const std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes) {
    std::unordered_map<robot_arm::JointNameImpl, double> newStates;
    newStates.emplace(newJointStateName, newJointStateValue);
    JointHelper::SetJointStates(newStates, orderedJointStates, unorderedJointStates, jointStatesLastChanged, operatingModes);
}

void JointHelper::SetJointStates(const std::unordered_map<robot_arm::JointNameImpl, double>& newJointStates, std::vector<robot_arm::JointState>& orderedJointStates,
        std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>& unorderedJointStates, std::chrono::high_resolution_clock::time_point& jointStatesLastChanged,
        const std::unordered_map<robot_arm::JointNameImpl, robot_arm::OperatingMode>& operatingModes) {
    std::shared_ptr<robot_arm::JointName> jointName;
    double position;
    double velocity;

    for (size_t i = 0; i < orderedJointStates.size(); i++) {
        jointName = orderedJointStates[i].GetJointName();
        auto newJointState = newJointStates.find(jointName);

        if (newJointState != newJointStates.end()) {
            position = orderedJointStates[i].GetPosition();
            velocity = orderedJointStates[i].GetVelocity();
            robot_arm::OperatingMode operatingMode = operatingModes.at(jointName);

            if (operatingMode == robot_arm::OperatingMode::POSITION() || operatingMode == robot_arm::OperatingMode::POSITION_MULTIPLE_REVOLUTIONS()) {
                position = newJointState->second;
            } else if (operatingMode == robot_arm::OperatingMode::VELOCITY()) {
                velocity = newJointState->second;
            }

            robot_arm::JointState tmpJointState = robot_arm::JointState(jointName, position, velocity, orderedJointStates[i].GetEffort(), orderedJointStates[i].GetOperatingMode());
            unorderedJointStates.at(jointName) = tmpJointState;
            orderedJointStates[i] = tmpJointState;
        }
    }

    jointStatesLastChanged = std::chrono::high_resolution_clock::now();
}

bool JointHelper::HaveJointStatesExpired(const std::chrono::high_resolution_clock::time_point& jointStatesLastChanged) {
    // If true: This prevents issues when reading the current joint states
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - jointStatesLastChanged) >= std::chrono::seconds(1);
}