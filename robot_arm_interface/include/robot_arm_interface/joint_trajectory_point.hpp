#ifndef ROBOT_ARM_JOINT_TRAJECTORY_POINT_HPP
#define ROBOT_ARM_JOINT_TRAJECTORY_POINT_HPP

#include <vector>

namespace robot_arm {
    class JointTrajectoryPoint {
    public:
        JointTrajectoryPoint(std::vector<double> positions, std::vector<double> velocities,
            std::vector<double> accelerations, std::vector<double> efforts, double secondsFromStart);
        const std::vector<double>& GetPositions();
        const std::vector<double>& GetVelocities();
        const std::vector<double>& GetAccelerations();
        const std::vector<double>& GetEfforts();
        double GetSecondsFromStart() const;
    private:
        std::vector<double> positions;;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        std::vector<double> efforts;
        double secondsFromStart;
    };
}

#endif //ROBOT_ARM_JOINT_TRAJECTORY_POINT_HPP