#include "robot_arm_interface/joint_trajectory_point.hpp"

using namespace robot_arm;

JointTrajectoryPoint::JointTrajectoryPoint(std::vector<double> positions, std::vector<double> velocities,
        std::vector<double> accelerations, std::vector<double> efforts, double secondsFromStart) :
        positions(positions), velocities(velocities), accelerations(accelerations), efforts(efforts), secondsFromStart(secondsFromStart) {
}

const std::vector<double>& JointTrajectoryPoint::GetPositions() {
    return positions;
}

const std::vector<double>& JointTrajectoryPoint::GetVelocities() {
    return velocities;
}

const std::vector<double>& JointTrajectoryPoint::GetAccelerations() {
    return accelerations;
}

const std::vector<double>& JointTrajectoryPoint::GetEfforts() {
    return efforts;
}

double JointTrajectoryPoint::GetSecondsFromStart() const {
    return secondsFromStart;
}