#include "joint_helper.hpp"

using namespace interbotix;

// Sort and add additonal values
std::vector<double> JointHelper::PrepareJointCommands(const std::vector<robot_arm::JointName>& jointNames, const std::vector<double>& values,
        robot_arm::RobotInfo& robotInfo, const std::vector<robot_arm::JointState>& jointStates) {
    std::vector<double> sortedValues(jointStates.size());
    bool jointAdded;

    for (size_t i = 0; i < sortedValues.size(); i++) {
        jointAdded = false;

        for (size_t n = 0; n < jointNames.size(); n++) {
            if ((robot_arm::JointName) jointStates[i].jointName == jointNames[n]) {
                sortedValues.push_back(values[n]);
                jointAdded = true;
            }

            if (!jointAdded) {
                sortedValues.push_back(jointStates[i].position); // TODO: detect acceleration mode
            }
        }
    }

    return sortedValues;
}

int8_t JointHelper::GetAffectedJoints(robot_arm::AffectedJoints affectedJoints) {
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

void JointHelper::CopyToJointTrajectoryMessage(const std::vector<robot_arm::JointName>& jointNames, const std::vector<robot_arm::JointTrajectoryPoint>& jointTrajectoryPoints,
        trajectory_msgs::JointTrajectory& message) {
    for (robot_arm::JointName jointName : jointNames) {
        message.joint_names.push_back(jointName);
    }

    for (robot_arm::JointTrajectoryPoint point : jointTrajectoryPoints) {
        trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
        trajectoryPoint.accelerations = point.GetAccelerations();
        trajectoryPoint.effort = point.GetEfforts();
        trajectoryPoint.positions = point.GetPositions();
        trajectoryPoint.time_from_start = ros::Duration(point.GetSecondsFromStart());
        trajectoryPoint.velocities = point.GetVelocities();

        message.points.push_back(trajectoryPoint);
    }
}

std::unordered_map<robot_arm::JointName, robot_arm::Joint> JointHelper::CreateJoints(const std::vector<robot_arm::JointName>& jointNames, const std::vector<int>& jointIDs,
            const std::vector<double>& lowerJointLimits, const std::vector<double> upperJointLimits, const std::vector<double>& velocityLimits) {
    std::unordered_map<robot_arm::JointName, robot_arm::Joint> joints(jointNames.size());
    double lowerJointDeviation;

    for (size_t i = 0; i < jointNames.size(); i++) {
        if (jointNames[i] == robot_arm::JointName::GRIPPER()) {
            lowerJointDeviation = 0;
        } else {
            // WORKAROUND: + 0.001739 prevents the driver from throwing warnings
            lowerJointDeviation = 0.001739;
        }

        joints.emplace((robot_arm::JointName) jointNames[i], robot_arm::Joint(jointNames[i], jointIDs[i], lowerJointLimits[i] + lowerJointDeviation, upperJointLimits[i], velocityLimits[i]));
    }

    return joints;
}

void JointHelper::PrepareRobotInfoJoints(const std::vector<std::string>& jointNames, std::vector<robot_arm::JointName>& newJointNames,
            const std::vector<int16_t>& jointIDs, std::vector<int>& newJointIDs, const std::vector<double>& lowerJointLimits,
            std::vector<double>& newLowerJointLimits, const std::vector<double>& upperJointLimits, std::vector<double>& newUpperJointLimits,
            double lowerGripperLimit, double upperGripperLimit, bool useGripper) {
    int gripperIndex = 0;
    bool gripperIndexFound = false;

    for (std::string joint : jointNames) {
        newJointNames.push_back(robot_arm::JointName(joint));

        if (!gripperIndexFound) {
            if (joint == (std::string) robot_arm::JointName::GRIPPER()) {
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
}

void JointHelper::PrepareJointStates(std::vector<robot_arm::JointState>* orderedJointStates, std::unordered_map<robot_arm::JointName, robot_arm::JointState>* unorderedJointStates,
        const std::vector<std::string>& jointNames, const std::vector<double>& jointPositions, const std::vector<double>& jointVelocities,
        const std::vector<double>& jointEfforts) {
    std::string jointName;
    double position;

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
        if (jointNames[i] == (std::string) robot_arm::JointName::GRIPPER() || jointNames[i] == "right_finger") {
            continue;
        } else if (jointNames[i] == "left_finger") {
            jointName = robot_arm::JointName::GRIPPER();
            position = jointPositions[i] * 2.0;
        } else {
            jointName = jointNames[i];
            position = jointPositions[i];
        }

        if (orderedJointStates != nullptr) {
            orderedJointStates->push_back(robot_arm::JointState(jointName, jointPositions[i], jointVelocities[i], jointEfforts[i]));  // TODO: mode?
        }

        if (unorderedJointStates != nullptr) {
            unorderedJointStates->emplace(jointName, robot_arm::JointState(jointName, position, jointVelocities[i], jointEfforts[i]));  // TODO: mode?
        }
    }
}