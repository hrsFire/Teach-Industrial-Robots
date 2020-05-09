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

    for (size_t i = 0; i < jointNames.size(); i++) {
        // WORKAROUND: + 0.001739 prevents the driver from throwing warnings
        joints.emplace((robot_arm::JointName) jointNames[i], robot_arm::Joint(jointNames[i], jointIDs[i], lowerJointLimits[i] + 0.001739, upperJointLimits[i], velocityLimits[i]));
    }

    return joints;
}