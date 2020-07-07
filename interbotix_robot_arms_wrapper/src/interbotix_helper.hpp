#ifndef INTERBOTIX_INTERBOTIX_HELPER_HPP
#define INTERBOTIX_INTERBOTIX_HELPER_HPP

#include <interbotix_sdk/arm_obj.h>
#include <ros/package.h>

namespace interbotix {
    class InterbotixHelper {
    public:
        static void GetMotorConfigs(std::string robot_name, std::vector<double>& home_positions, std::vector<double>& sleep_positions,
            std::vector<Motor>& all_joints, size_t& joint_num_write);
        static bool GetRobotInfoFromConfigFiles(std::string robot_name, interbotix_sdk::RobotInfo::Response& res);
    };
}

#endif //INTERBOTIX_INTERBOTIX_HELPER_HPP