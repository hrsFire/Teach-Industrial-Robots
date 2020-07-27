#ifndef ROBOT_ARM_JOINT_TRAJECTORY_POINT_HPP
#define ROBOT_ARM_JOINT_TRAJECTORY_POINT_HPP

#include <vector>

namespace robot_arm {
    /**
     * This class provides a joint trajectory point for all ordered joint names of the robot arm.
     */
    class JointTrajectoryPoint {
    public:
        /**
         * Default constructor for a trajectory point for a joint.
         * @param positions         The positions for the trajectory point.
         * @param velocities        The velocities for the trajectory point.
         * @param accelerations     The accelerations for the trajectory point.
         * @param efforts           The efforts for the trajectory point.
         * @param secondsFromStart  The seconds from the start for the trajectory point.
         */
        JointTrajectoryPoint(std::vector<double> positions, std::vector<double> velocities,
            std::vector<double> accelerations, std::vector<double> efforts, double secondsFromStart);
        /**
         * Gets the positions for the trajectory point
         * @return The positions for the trajectory point.
         */
        const std::vector<double>& GetPositions();
        /**
         * Gets the velocities for the trajectory point
         * @return The velocities for the trajectory point.
         */
        const std::vector<double>& GetVelocities();
        /**
         * Gets the accelerations for the trajectory point
         * @return The accelerations for the trajectory point.
         */
        const std::vector<double>& GetAccelerations();
        /**
         * Gets the efforts for the trajectory point
         * @return The efforts for the trajectory point.
         */
        const std::vector<double>& GetEfforts();
        /**
         * Gets the seconds from the start for the trajectory point
         * @return The seconds from the start for the trajectory point.
         */
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
