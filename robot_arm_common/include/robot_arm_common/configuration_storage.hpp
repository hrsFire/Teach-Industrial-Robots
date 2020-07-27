#ifndef ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP
#define ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <robot_arm_interface/joint_name.hpp>
#include <robot_arm_interface/joint_name_impl.hpp>
#include <robot_arm_interface/joint_state.hpp>

/**
* This namespace provides different stuff for common robot arm tasks.
*/
namespace robot_arm_common {
    /**
     * This class provides the possibility to store and load configuration files.
     */
    class ConfigurationStorage {
    public:
        /**
         * Default constructor for the configuration storage.
         */
        ConfigurationStorage();
        /**
         * Saves the joint positions to the default configuration file.
         * @param allJointPositions  All joint names and their corresponding joint positions.
         * @param overwrite          Specifies if the configuration file should be overwritten.
         * @return True if the configuration file could be saved. Otherwise False is returned.
         */
        bool SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, bool overwrite = false);
        /**
         * Saves the joint positions to the configuration file with the given file path.
         * @param allJointPositions  All joint names and their corresponding joint positions.
         * @param overwrite          Specifies if the configuration file should be overwritten.
         * @param directoryPath      The directory path where the configuration file should be saved.
         * @param fileName           The file name which is used to store the configuration file.
         * @return True if the configuration file could be saved. Otherwise False is returned.
         */
        bool SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, std::string directoryPath,
            std::string fileName, bool overwrite = false);
        /**
         * Saves the joint positions to the configuration file with the given file path.
         * @param allJointPositions  All joint names and their corresponding joint positions.
         * @param overwrite          Specifies if the configuration file should be overwritten.
         * @param filePath           The file path where the configuration file should be saved.
         * @return True if the configuration file could be saved. Otherwise False is returned.
         */
        bool SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, std::string filePath,
            bool overwrite = false);
        /**
         * Loads the joint positions from the default configuration file.
         * @param jointNameDeserializer  The joint name deserializer which should be used to deserialize the configuration file.
         * @param allJointPositions      All joint names and their corresponding joint positions.
         * @return True if the configuration file could be loaded. Otherwise False is returned.
         */
        bool LoadPositions(const robot_arm::JointName& jointNameDeserializer,
            std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions);
        /**
         * Loads the joint positions from the configuration file from the given file path.
         * @param directoryPath          The directory path from where the configuration file should be loaded.
         * @param fileName               The file name which is used to load the configuration file.
         * @param jointNameDeserializer  The joint name deserializer which should be used to deserialize the configuration file.
         * @param allJointPositions      All joint names and their corresponding joint positions.
         * @return True if the configuration file could be loaded. Otherwise False is returned.
         */
        bool LoadPositions(std::string directoryPath, std::string fileName, const robot_arm::JointName& jointNameDeserializer,
            std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions);
        /**
         * Loads the joint positions from the configuration file from the given file path.
         * @param filePath               The file path from where the configuration file should be loaded.
         * @param jointNameDeserializer  The joint name deserializer which should be used to deserialize the configuration file.
         * @param allJointPositions      All joint names and their corresponding joint positions.
         * @return True if the configuration file could be loaded. Otherwise False is returned.
         */
        bool LoadPositions(std::string filePath, const robot_arm::JointName& jointNameDeserializer,
            std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions);
    private:
        std::string defaultDirectoryPath;
        std::string defaultFileName;
    };
}

#endif //ROBOT_ARM_COMMON_CONFIGURATON_STORAGE_HPP
