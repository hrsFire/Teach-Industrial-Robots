#ifndef INTERBOTIX_INTERBOTIX_HELPER_HPP
#define INTERBOTIX_INTERBOTIX_HELPER_HPP

#include <interbotix_sdk/arm_obj.h>
#include <ros/package.h>

#ifdef COMMUNICATION_MEASUREMENT
#include <iostream>
#include <fstream>
#include <sstream>
#endif //COMMUNICATION_MEASUREMENT

namespace interbotix {
    class InterbotixHelper {
    public:
        static void GetMotorConfigs(std::string robot_name, std::vector<double>& home_positions, std::vector<double>& sleep_positions,
            std::vector<Motor>& all_joints, size_t& joint_num_write);
        static bool GetRobotInfoFromConfigFiles(std::string robot_name, interbotix_sdk::RobotInfo::Response& res);
#ifdef COMMUNICATION_MEASUREMENT
        static std::ofstream* InitializeMeasurementFile(const std::string& filePath);
        static void SaveCommunicationMeasurement(const std::chrono::system_clock::time_point& timePoint, std::ofstream& measurementFile);
#endif //COMMUNICATION_MEASUREMENT
    };
}

#endif //INTERBOTIX_INTERBOTIX_HELPER_HPP