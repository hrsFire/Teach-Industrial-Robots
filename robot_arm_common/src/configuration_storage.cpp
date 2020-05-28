#include "robot_arm_common/configuration_storage.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <rapidjson/stream.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/encodings.h>
#include <rapidjson/document.h>
#include <rapidjson/reader.h>
#include <rapidjson/prettywriter.h>

#ifdef unix
#include <unistd.h>
#include <pwd.h>
#endif

using namespace robot_arm_common;

ConfigurationStorage::ConfigurationStorage() {
    const char* homedir;

#ifdef unix
    // https://stackoverflow.com/questions/2910377/get-home-directory-in-linux/26696759
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
#elif defined(_WIN32)
    homedir = getenv("USERPROFILE");
#endif

    defaultDirectoryPath = homedir;
    defaultFileName = "robot_arm_positions.json";

    std::cout << "Default file path for robot arm positions: " << defaultDirectoryPath << "/" << defaultFileName << std::endl;
}

bool ConfigurationStorage::SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions, bool overwrite) {
    return ConfigurationStorage::SavePositions(allJointPositions, defaultDirectoryPath, defaultFileName, overwrite);
}

bool ConfigurationStorage::SavePositions(const std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions,
        std::string directoryPath, std::string fileName, bool overwrite) {
    std::string filePath = directoryPath + "/" + fileName;
    std::ifstream tmpFile(filePath);

    if (tmpFile.good() && !overwrite) {
        return false;
    }

    std::cout << "Save robot arm positions to: " << directoryPath << "/" << fileName << std::endl;
    std::ofstream file(filePath, std::ofstream::out | std::ofstream::trunc);
    bool hasError = false;

    if (file.is_open()) {
        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        writer.StartObject();
        writer.Key("all_positions");
        writer.StartArray();

        for (auto& allPositionsItem : allJointPositions) {
            writer.StartObject();
            writer.Key("single_positions");
            writer.StartArray();

            for (auto& singlePositionsItem : allPositionsItem) {
                writer.StartObject();
                writer.Key("joint_name");
                writer.String(singlePositionsItem.first.Serialize().c_str());

                writer.Key("joint_state");
                writer.StartObject();

                writer.Key("position");
                writer.Double(singlePositionsItem.second.GetPosition());

                writer.Key("velocity");
                writer.Double(singlePositionsItem.second.GetVelocity());

                writer.Key("effort");
                writer.Double(singlePositionsItem.second.GetEffort());

                writer.Key("operating_mode");
                writer.String(static_cast<std::string>(singlePositionsItem.second.GetOperatingMode()).c_str());

                writer.EndObject();

                writer.EndObject();
            }

            writer.EndArray();
            writer.EndObject();
        }

        writer.EndArray();
        writer.EndObject();

        file << buffer.GetString();
        file.flush();
        file.close();

        return true;
    }

    return false;
}

bool ConfigurationStorage::LoadPositions(const robot_arm::JointName& jointNameDeserializer,
        std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions) {
    return ConfigurationStorage::LoadPositions(defaultDirectoryPath, defaultFileName, jointNameDeserializer, allJointPositions);
}

bool ConfigurationStorage::LoadPositions(std::string directoryPath, std::string fileName, const robot_arm::JointName& jointNameDeserializer,
        std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>>& allJointPositions) {
    std::cout << "Load robot arm positions from: " << directoryPath << "/" << fileName << std::endl;
    std::ifstream file(directoryPath + "/" + fileName, std::ifstream::in);
    bool hasError = false;

    if (file.is_open()) {
        std::stringstream ss;
        ss << file.rdbuf();
        rapidjson::Document document;
        rapidjson::ParseResult parseResult = document.Parse(ss.str().c_str());
        file.close();

        if (parseResult.IsError()) {
            return false;
        }

        if (document.HasMember("all_positions")) {
            rapidjson::Value& allPositionsNode = document["all_positions"];

            if (allPositionsNode.IsArray()) {
                auto allPositionsArray = allPositionsNode.GetArray();

                for (auto allPositionsItemItr = allPositionsArray.begin(); allPositionsItemItr != allPositionsArray.end(); allPositionsItemItr++) {
                    if (allPositionsItemItr->IsObject()) {
                        auto allPositionsItemObject = allPositionsItemItr->GetObject();

                        if (allPositionsItemObject.HasMember("single_positions")) {
                            rapidjson::Value& singlePositionsNode = allPositionsItemObject["single_positions"];

                            if (singlePositionsNode.IsArray()) {
                                auto singlePositionsArray = singlePositionsNode.GetArray();
                                std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointPositions;

                                for (auto singlePositionsItemItr = singlePositionsArray.begin(); singlePositionsItemItr != singlePositionsArray.end(); singlePositionsItemItr++) {
                                    if (singlePositionsItemItr->IsObject()) {
                                        auto singlePositionsItemObject = singlePositionsItemItr->GetObject();
                                        std::string jointName;
                                        double position, velocity, effort;
                                        std::string operatingMode;

                                        if (singlePositionsItemObject.HasMember("joint_name")) {
                                            rapidjson::Value& jointNameItem = singlePositionsItemObject["joint_name"];

                                            if (jointNameItem.IsString()) {
                                                jointName = jointNameItem.GetString();
                                            } else {
                                                hasError = true;
                                                break;
                                            }
                                        } else {
                                            hasError = true;
                                            break;
                                        }

                                        if (singlePositionsItemObject.HasMember("joint_state")) {
                                            rapidjson::Value& jointStateNode = singlePositionsItemObject["joint_state"];

                                            if (jointStateNode.IsObject()) {
                                                auto jointStateObject = jointStateNode.GetObject();

                                                if (jointStateObject.HasMember("position")) {
                                                    rapidjson::Value& positionNode = jointStateObject["position"];

                                                    if (positionNode.IsDouble()) {
                                                        position = positionNode.GetDouble();
                                                    } else {
                                                        hasError = true;
                                                        break;
                                                    }
                                                } else {
                                                    hasError = true;
                                                    break;
                                                }

                                                if (jointStateObject.HasMember("velocity")) {
                                                    rapidjson::Value& velocityNode = jointStateObject["velocity"];

                                                    if (velocityNode.IsDouble()) {
                                                        velocity = velocityNode.GetDouble();
                                                    } else {
                                                        hasError = true;
                                                        break;
                                                    }
                                                } else {
                                                    hasError = true;
                                                    break;
                                                }

                                                if (jointStateObject.HasMember("effort")) {
                                                    rapidjson::Value& effortNode = jointStateObject["effort"];

                                                    if (effortNode.IsDouble()) {
                                                        effort = effortNode.GetDouble();
                                                    } else {
                                                        hasError = true;
                                                        break;
                                                    }
                                                } else {
                                                    hasError = true;
                                                    break;
                                                }

                                                if (jointStateObject.HasMember("operating_mode")) {
                                                    rapidjson::Value& operatingModeNode = jointStateObject["operating_mode"];

                                                    if (operatingModeNode.IsString()) {
                                                        operatingMode = operatingModeNode.GetString();
                                                    } else {
                                                        hasError = true;
                                                        break;
                                                    }
                                                } else {
                                                    hasError = true;
                                                    break;
                                                }
                                            } else {
                                                hasError = true;
                                                break;
                                            }
                                        } else {
                                            hasError = true;
                                            break;
                                        }

                                        robot_arm::JointNameImpl jointNameImpl = robot_arm::JointNameImpl(jointNameDeserializer.Deserialize(jointName));
                                        jointPositions.emplace(jointNameImpl, robot_arm::JointState(jointNameImpl, position, velocity, effort, operatingMode));
                                    } else {
                                        hasError = true;
                                        break;
                                    }
                                }

                                if (hasError) {
                                    break;
                                }

                                allJointPositions.push_back(jointPositions);
                            } else {
                                hasError = true;
                                break;
                            }
                        } else {
                            hasError = true;
                            break;
                        }
                    } else {
                        hasError = true;
                        break;
                    }
                }
            }
        } else {
            hasError = true;
        }

        if (hasError) {
            allJointPositions.clear();
        }

        return !hasError;
    }

    return false;
}