#ifndef ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP
#define ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>
#include <robot_arm_interface/joint_state.hpp>

namespace robot_arm_common {
    class ConfigurationStorage {
    public:
        ConfigurationStorage();
        bool SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, bool overwrite = false);
        bool SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, std::string directoryPath,
            std::string fileName, bool overwrite = false);
        bool LoadPositions(const robot_arm::JointName& jointNameDeserializer,
            std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions);
        bool LoadPositions(std::string directoryPath, std::string fileName, const robot_arm::JointName& jointNameDeserializer,
            std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions);
    private:
        std::string defaultDirectoryPath;
        std::string defaultFileName;
    };
}

#endif //ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP