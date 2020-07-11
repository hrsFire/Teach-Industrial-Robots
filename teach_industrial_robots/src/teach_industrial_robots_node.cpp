#include <iostream>
#include <memory>
#include <csignal>
#include <mutex>
#include <unordered_map>
#include <locale>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <simple_gestures/kinect/azure_kinect_gestures.hpp>
#include <simple_gestures/kinect/azure_kinect.hpp>
#include <simple_gestures/gestures/gestures_engine.hpp>
#include <simple_gestures/gestures/gesture.hpp>
#include <simple_gestures/gestures/gesture_group.hpp>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <robot_arm_common/configuration_storage.hpp>
#include <interbotix_robot_arms_wrapper/interbotix_robot_arm.hpp>
#include <interbotix_robot_arms_wrapper/interbotix_joint_name.hpp>

gestures::GesturesEngine* gesturesEngine = nullptr;
robot_arm::RobotArmBase* robotArm = nullptr;
std::mutex exitSafelyMutex;
bool exitSafelyDone = false;

void ExitSafely() {
    std::lock_guard<std::mutex> lock(exitSafelyMutex);

    if (!exitSafelyDone) {
        if (gesturesEngine != nullptr) {
            std::cout << "Close Gestures Engine" << std::endl;
            gesturesEngine->Stop();
        }

        if (robotArm != nullptr) {
            std::cout << "Move to home position" << std::endl;
            robotArm->SendJointCommands(robotArm->GetRobotInfo()->sleepPosition);
            delete robotArm;
            robotArm = nullptr;
        }

        exitSafelyDone = true;
    }
}

void PrintHelp() {
    std::cout << std::endl << std::endl;
    std::cout << "Command line options:" << std::endl;
    std::cout << "  " << "-pfp, --positions-file-path" << "\t\t" << "The file path to save or load the recorded positions" << std::endl;
    std::cout << "  " << "-opf, --overwrite-positions-file" << "\t" << "Enables to overwrite the positions file."
        << " Make a backup of the positions file, becasue otherwise this leads to data loss." << std::endl;
    std::cout << "  " << "-rrp, --repeat-recorded-positions" << "\t" << "Repeat the recorded positions from the positions file" << std::endl;
    std::cout << "  " << "-tp, --teach-positions" << "\t\t" << "Teach positions" << std::endl;
    std::cout << "  " << "-rn, --robot-name" << "\t\t\t" << "Robot name" << std::endl;
    std::cout << "  " << "-rm, --robot-model" << "\t\t\t" << "Robot model (typically the same as the robot name)" << std::endl;
    std::cout << "  " << "-ur, --use-ros" << "\t\t\t" << "Use ROS or use a direct connection" << std::endl;
    std::cout << "  " << "-mhaex, --move-home-at-exit" << "\t\t\t" << "Move home at exit" << std::endl;
    std::cout << "  " << "-mhaer, --move-home-at-error" << "\t\t\t" << "Move home at error" << std::endl;
    std::cout << "  " << "--help" << "\t\t\t\t" << "Shows this help" << std::endl;
}

bool IsLookingAtDevice(const gestures::GesturesQuery& gesturesImpl) {
    return gesturesImpl.IsJointVisible(K4ABT_JOINT_EYE_LEFT) && gesturesImpl.IsJointVisible(K4ABT_JOINT_EYE_RIGHT);
}

enum CoordinateSystemMode {
    JOINT_MODE,
    WORLD_COORDINATE_MODE
};

enum WorldAxis {
    UP, // Z
    NORTH, // Y
    EAST // X
};

std::string GetStringForWorldAxis(WorldAxis worldAxis) {
    std::string axisName = "";

    if (worldAxis == WorldAxis::EAST) {
        axisName = "EAST (X)";
    } else if (worldAxis == WorldAxis::NORTH) {
        axisName = "NORTH (Y)";
    } else if (worldAxis == WorldAxis::UP) {
        axisName = "UP (Z)";
    }

    return axisName;
}

int main(int argc, char** argv) {
    std::string positionsFilePath;
    bool overwritePositionsFile = false;
    bool repeatRecordedPositions = false;
    bool teachPositions = false;
    std::string robotName = "wx200";
    std::string robotModel = "wx200"; // Typically, this will be the same as robotName
    bool useROS = false;
    bool moveHomeAtExit = false;
    bool moveHomeAtError = false;

#ifndef NDEBUG
    overwritePositionsFile = false;
    repeatRecordedPositions = false;
    teachPositions = true;
    useROS = true;
    moveHomeAtExit = true;
    moveHomeAtError = true;
#else
    if (argc == 1) {
        PrintHelp();
        return 0;
    }

    bool requiresValue;
    std::string arg;
    std::string key;
    std::string value;
    std::size_t index;
    bool numberOfHyphens;

    for (size_t i = 1; i < argc; i++) {
        arg = argv[i];
        requiresValue = false;
        numberOfHyphens = arg.rfind("--", 0) == 0 ? 2 : 0;

        if (numberOfHyphens == 0) {
            numberOfHyphens = arg.rfind("-", 0) == 0 ? 1 : 0;
        }

        if (numberOfHyphens > 0) {
            index = arg.find("=");

            if (index == std::string::npos) {
                key = arg;
                value = "";
            } else {
                key = arg.substr(0, index);
                key.erase(key.begin(), key.begin() + (numberOfHyphens -1));
                value = arg.substr(index +1);
            }

            if (key == "--positions-file-path" || key == "-pfp") {
                if (value.empty()) {
                    requiresValue = true;
                } else {
                    positionsFilePath = value;
                }
            } else if (key == "--overwrite-positions-file" || key == "-opf") {
                overwritePositionsFile = true;
            } else if (key == "--repeat-recorded-positions" || key == "-rrp") {
                repeatRecordedPositions = true;
            } else if (key == "--teach-positions" || key == "-tp") {
                teachPositions = true;
            } else if (key == "--robot-name" || key == "-rn") {
                if (value.empty()) {
                    requiresValue = true;
                } else {
                    robotName = value;
                }
            } else if (key == "--robot-model" || key == "-rm") {
                if (value.empty()) {
                    requiresValue = true;
                } else {
                    robotModel = value;
                }
            } else if (key == "--use-ros" || key == "-ur") {
                useROS = true;
            } else if (key == "--move-home-at-exit" || key == "-mhaex") {
                moveHomeAtExit = true;
            } else if (key == "--move-home-at-error" || key == "-mhaer") {
                moveHomeAtError = true;
            } else if (key == "--help") {
                PrintHelp();
                return 0;
            } else {
                std::cout << std::endl << "> Invalid key: " << key << std::endl;
                PrintHelp();
                return 0;
            }

            if (requiresValue) {
                std::cout << std::endl << "> Key requires value: " << key << std::endl;
                PrintHelp();
                return 0;
            }
        } else {
            std::cout << std::endl << "> Invalid parameter: " << arg << std::endl;
            PrintHelp();
            return 0;
        }
    }
#endif //NDEBUG

    std::cout << "#################################" << std::endl;
    std::cout << "###  Teach Industrial Robots  ###" << std::endl;
    std::cout << "#################################" << std::endl << std::endl;

    if (repeatRecordedPositions || teachPositions) {
        if (moveHomeAtExit) {
            signal(SIGINT, [](int i) {
                ExitSafely();
            });

            signal(SIGQUIT, [](int i) {
                ExitSafely();
            });

            signal(SIGHUP, [](int i) {
                ExitSafely();
            });
        }

        if (moveHomeAtError) {
            signal(SIGTERM, [](int i) {
                ExitSafely();
            });
        }

        robotArm = new interbotix::InterbotixRobotArm(useROS, argc, argv, robotName, robotModel);
    }

    if (repeatRecordedPositions) {
        robot_arm_common::ConfigurationStorage configurationStorage;
        std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>> recordedPositions;
        bool isSuccessful;

        if (positionsFilePath.empty()) {
            isSuccessful = configurationStorage.LoadPositions(interbotix::InterbotixJointName::NONE(), recordedPositions);
        } else {
            isSuccessful = configurationStorage.LoadPositions(positionsFilePath, interbotix::InterbotixJointName::NONE(), recordedPositions);
        }

        if (isSuccessful) {
            std::cout << "Successfully loaded recorded positions" << std::endl;
            std::cout << "Start to repeat the recorded positions" << std::endl;
            int numberOfJointPositionsReached;
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> previousJointPositions;
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> currentJointPositions;
            size_t positionCount = 0;

            for (auto& singlePositions : recordedPositions) {
                positionCount++;
                std::unordered_map<robot_arm::JointNameImpl, double> tmpSinglePositions;

                for (auto& singlePosition : singlePositions) {
                    tmpSinglePositions.emplace(singlePosition.first, singlePosition.second.GetPosition());
                }

                std::cout << "Move to " << positionCount << ". recorded position" << std::endl;
                robotArm->SendJointCommands(tmpSinglePositions);

                previousJointPositions = robotArm->GetJointStates();
                // Use a timeout to prevent getting the same joint state
                sleep(1);

                while (true) {
                    numberOfJointPositionsReached = 0;
                    currentJointPositions = robotArm->GetJointStates();

                    for (auto& item : currentJointPositions) {
                        double previousJointPosition = previousJointPositions.at(item.first).GetPosition();
                        double currentPosition = item.second.GetPosition();

#ifndef NDEBUG
                        std::cout << "Joint Name: " << (std::string) item.first << std::endl;
                        std::cout << "Previous position: " << previousJointPosition << std::endl;
                        std::cout << "Current positon: " << currentPosition << std::endl;
                        std::cout << "-----------------------" << std::endl;
#endif //NDEBUG

                        if (currentPosition == previousJointPosition) {
                            numberOfJointPositionsReached++;
                        }
                    }

                    if (numberOfJointPositionsReached == singlePositions.size()) {
                        break;
                    }

                    previousJointPositions = currentJointPositions;
                    // Use a timeout to prevent getting the same joint state
                    sleep(1);
                }
            }
        } else {
            std::cout << "Failed to load the recorded positions" << std::endl;
        }

        ExitSafely();

        return 0;
    } else if (!teachPositions) {
        return 0;
    }

    uint32_t installedDevices = k4a::device::get_installed_count();
    std::cout << "Installed devices: " << installedDevices << std::endl;

    if (installedDevices > 0) {
        uint32_t deviceIndex = 0;
        std::cout << "Open device at index: " << deviceIndex << std::endl;
        k4a::device device = nullptr;

        try {
            device = k4a::device::open(deviceIndex);
        } catch (k4a::error e) {
            std::cerr << e.what() << std::endl;
            return 0;
        }

        try {
            std::string serialNumber = device.get_serialnum();
            std::cout << "Serial number: " << serialNumber << std::endl;
        } catch (k4a::error e) {
            std::cerr << e.what() << std::endl;
            device.close();
            return 0;
        }

        k4a_device_configuration_t deviceConfig;

        if (!kinect::AzureKinect::StartSensors(device, deviceConfig)) {
            return 0;
        }

        k4abt::tracker bodyTracker = nullptr;

        if (!kinect::AzureKinect::StartBodyTracking(device, deviceConfig, bodyTracker)) {
            return 0;
        }

        // A value of 1 seems to have no performance impact in CPU mode
        bodyTracker.set_temporal_smoothing(0);

        std::vector<std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState>> recordedPositions;
        std::shared_ptr<robot_arm::JointName> currentJoint = std::make_shared<interbotix::InterbotixJointName>(interbotix::InterbotixJointName::ELBOW());
        std::chrono::system_clock::time_point switchToPrevJointTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchToNextJointTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchPrecisionModeTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchGripperLockTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point saveConfigurationTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchTeachModeTime = std::chrono::system_clock::now();
        std::unique_ptr<bool> isPreciseMode = std::make_unique<bool>(true);
        std::unique_ptr<bool> isGripperLocked = std::make_unique<bool>(true);
        std::unique_ptr<bool> successfullySavedConfiguration = std::make_unique<bool>(false);
        std::unique_ptr<CoordinateSystemMode> currentCoordinateSystemMode = std::make_unique<CoordinateSystemMode>(
            robotName == "wx200" && !useROS ? CoordinateSystemMode::JOINT_MODE : CoordinateSystemMode::WORLD_COORDINATE_MODE);
        std::unique_ptr<WorldAxis> currentWorldAxis = std::make_unique<WorldAxis>(WorldAxis::EAST);
        std::unique_ptr<robot_arm::JointNameImpl> endEffectorJointName = std::make_unique<robot_arm::JointNameImpl>(interbotix::InterbotixJointName::GRIPPER());

        gesturesEngine = new gestures::GesturesEngine(new kinect::AzureKinectGestures(&bodyTracker, &device, false));
        gestures::GestureGroup singleJointGestureGroup = gesturesEngine->AddGestureGroup("single_joint", 0, {});
        gestures::GestureGroup gripperGestureGroup = gesturesEngine->AddGestureGroup("gripper", 1, {});
        gestures::GestureGroup switchJointGestureGroup = gesturesEngine->AddGestureGroup("switch_joint", 2, { singleJointGestureGroup });
        gestures::GestureGroup switchPrecisionModeGestureGroup = gesturesEngine->AddGestureGroup("switch_precision_mode", 3, { singleJointGestureGroup,
            switchJointGestureGroup });
        gestures::GestureGroup switchTeachModeGestureGroup = gesturesEngine->AddGestureGroup("switch_teach_mode", 4, { singleJointGestureGroup,
            gripperGestureGroup, switchJointGestureGroup, switchPrecisionModeGestureGroup });
        gestures::GestureGroup configurationGestureGroup = gesturesEngine->AddGestureGroup("configuration", 5, { singleJointGestureGroup, gripperGestureGroup,
            switchJointGestureGroup, switchPrecisionModeGestureGroup });

        std::function<void(bool)> saveConfiguratonFile = [&positionsFilePath, &recordedPositions, &successfullySavedConfiguration,
                &overwritePositionsFile](bool initialize) -> void {
            robot_arm_common::ConfigurationStorage configurationStorage;
            recordedPositions.push_back(robotArm->GetJointStates());
            bool isSuccessful = false;

            if (positionsFilePath.empty()) {
                isSuccessful = configurationStorage.SavePositions(recordedPositions, overwritePositionsFile);
            } else {
                isSuccessful = configurationStorage.SavePositions(recordedPositions, positionsFilePath, overwritePositionsFile);
            }

            if (isSuccessful) {
                if (!initialize) {
                    std::cout << "Saved recorded positions" << std::endl;
                }

                successfullySavedConfiguration.reset(new bool(true));
                overwritePositionsFile = true;
            } else {
                if (initialize) {
                    std::cout << "Warning: Could not create file for \"recorded positions\"";

                    if (!overwritePositionsFile) {
                        std::cout << ", because it isn't allowed to overwrite the file. Please make a backup or use the specific command line option." << std::endl;
                    }
                } else {
                    std::cout << "Could not save recorded positions";
                }

                std::cout << std::endl;
            }
        };

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 65.0);
        }, [&currentJoint, &isPreciseMode, &currentCoordinateSystemMode, &currentWorldAxis, &endEffectorJointName](std::chrono::milliseconds duration) {
            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            if (*currentCoordinateSystemMode == CoordinateSystemMode::JOINT_MODE) {
                std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
                double jointAngle = jointStates.at(currentJoint).GetPosition();

                jointAngle += robotArm->CalculateAccelerationDistance(*currentJoint, duration);
                robotArm->SendJointCommand(*currentJoint, jointAngle);
            } else if (*currentCoordinateSystemMode == CoordinateSystemMode::WORLD_COORDINATE_MODE) {
                geometry_msgs::Pose pose;

                if (robotArm->GetCurrentPose(*endEffectorJointName, pose)) {
                    double coordinateChange = robotArm->CalculateAccelerationDistance(duration);

                    if (*currentWorldAxis == WorldAxis::EAST) {
                        pose.position.x += coordinateChange;
                    } else if (*currentWorldAxis == WorldAxis::NORTH) {
                        pose.position.y += coordinateChange;
                    } else if (*currentWorldAxis == WorldAxis::UP) {
                        pose.position.z += coordinateChange;
                    }

                    robotArm->SendPose(pose, *endEffectorJointName);
                }
            }
        }), singleJointGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 65.0);
        }, [&currentJoint, &isPreciseMode, &currentCoordinateSystemMode, &currentWorldAxis, &endEffectorJointName](std::chrono::milliseconds duration) {
            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            if (*currentCoordinateSystemMode == CoordinateSystemMode::JOINT_MODE) {
                std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
                double jointAngle = jointStates.at(currentJoint).GetPosition();

                jointAngle -= robotArm->CalculateAccelerationDistance(*currentJoint, duration);
                robotArm->SendJointCommand(*currentJoint, jointAngle);
            } else if (*currentCoordinateSystemMode == CoordinateSystemMode::WORLD_COORDINATE_MODE) {
                geometry_msgs::Pose pose;

                if (robotArm->GetCurrentPose(*endEffectorJointName, pose)) {
                    double coordinateChange = robotArm->CalculateAccelerationDistance(duration);

                    if (*currentWorldAxis == WorldAxis::EAST) {
                        pose.position.x -= coordinateChange;
                    } else if (*currentWorldAxis == WorldAxis::NORTH) {
                        pose.position.y -= coordinateChange;
                    } else if (*currentWorldAxis == WorldAxis::UP) {
                        pose.position.z -= coordinateChange;
                    }

                    robotArm->SendPose(pose, *endEffectorJointName);
                }
            }
        }), singleJointGestureGroup, 1, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
        }, [&isPreciseMode, &isGripperLocked](std::chrono::milliseconds duration) {
            if (*isGripperLocked) {
                return;
            }

            interbotix::InterbotixJointName gripperJointNamePtr = interbotix::InterbotixJointName::GRIPPER();
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
            double gripperDistance = jointStates.at(gripperJointNamePtr).GetPosition();

            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            gripperDistance += robotArm->CalculateAccelerationDistance(interbotix::InterbotixJointName::GRIPPER(), duration);
            robotArm->SendGripperCommand(gripperDistance);
        }), gripperGestureGroup, 0, { interbotix::InterbotixJointName::GRIPPER() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
        }, [&isPreciseMode, &isGripperLocked](std::chrono::milliseconds duration) {
            if (*isGripperLocked) {
                return;
            }

            interbotix::InterbotixJointName gripperJointNamePtr = interbotix::InterbotixJointName::GRIPPER();
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
            double gripperDistance = jointStates.at(gripperJointNamePtr).GetPosition();

            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            gripperDistance -= robotArm->CalculateAccelerationDistance(interbotix::InterbotixJointName::GRIPPER(), duration);
            robotArm->SendGripperCommand(gripperDistance);
        }), gripperGestureGroup, 1, { interbotix::InterbotixJointName::GRIPPER() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl)&& (
                gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0));
        }, [&isGripperLocked, &switchGripperLockTime](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchGripperLockTime) > std::chrono::milliseconds(100)) {
                switchGripperLockTime = currentTime;
                isGripperLocked.reset(new bool(!(*isGripperLocked)));

                std::cout << "Gripper lock mode: ";

                if (*isGripperLocked) {
                    std::cout << "locked";
                } else {
                    std::cout << "unlocked";
                }

                std::cout << std::endl;
            }
        }), gripperGestureGroup, 2, { interbotix::InterbotixJointName::GRIPPER() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && gesturesImpl.IsGesture(K4ABT_JOINT_EAR_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
        }, [&currentJoint, &switchToPrevJointTime, &currentCoordinateSystemMode, &currentWorldAxis](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchToPrevJointTime) > std::chrono::milliseconds(50)) {
                switchToPrevJointTime = currentTime;

                if (*currentCoordinateSystemMode == CoordinateSystemMode::JOINT_MODE) {
                    currentJoint = currentJoint->Prev();
                    std::cout << "Set previous joint: "<< (std::string) *currentJoint << std::endl;
                } else if (*currentCoordinateSystemMode == CoordinateSystemMode::WORLD_COORDINATE_MODE) {
                    if (*currentWorldAxis == WorldAxis::EAST) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::UP));
                    } else if (*currentWorldAxis == WorldAxis::NORTH) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::EAST));
                    } else if (*currentWorldAxis == WorldAxis::UP) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::NORTH));
                    }

                    std::cout << "Set world axis: " << GetStringForWorldAxis(*currentWorldAxis) << std::endl;
                }
            }
        }), switchJointGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && gesturesImpl.IsGesture(K4ABT_JOINT_EAR_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
        }, [&currentJoint, &switchToNextJointTime, &currentCoordinateSystemMode, &currentWorldAxis](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchToNextJointTime) > std::chrono::milliseconds(50)) {
                switchToNextJointTime = currentTime;

                if (*currentCoordinateSystemMode == CoordinateSystemMode::JOINT_MODE) {
                    currentJoint = currentJoint->Next();
                    std::cout << "Set next joint: " << (std::string) *currentJoint << std::endl;
                } else if (*currentCoordinateSystemMode == CoordinateSystemMode::WORLD_COORDINATE_MODE) {
                    if (*currentWorldAxis == WorldAxis::EAST) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::NORTH));
                    } else if (*currentWorldAxis == WorldAxis::NORTH) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::UP));
                    } else if (*currentWorldAxis == WorldAxis::UP) {
                        currentWorldAxis.reset(new WorldAxis(WorldAxis::EAST));
                    }

                    std::cout << "Set world axis: " << GetStringForWorldAxis(*currentWorldAxis) << std::endl;
                }
            }
        }), switchJointGestureGroup, 1, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && (
                gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0));
        }, [&currentJoint, &switchPrecisionModeTime, &isPreciseMode](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchPrecisionModeTime) > std::chrono::milliseconds(150)) {
                switchPrecisionModeTime = currentTime;
                isPreciseMode.reset(new bool(!(*isPreciseMode)));

                std::cout << "Switch precision mode: ";

                if (*isPreciseMode) {
                    std::cout << "precise";
                } else {
                    std::cout << "imprecise (accelerated)";
                }

                std::cout << std::endl;
            }
        }), switchPrecisionModeGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
        }, [&saveConfigurationTime, &recordedPositions, &overwritePositionsFile, &successfullySavedConfiguration, &saveConfiguratonFile](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - saveConfigurationTime) > std::chrono::milliseconds(100)) {
                saveConfigurationTime = currentTime;
                saveConfiguratonFile(false);
            }
        }), configurationGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && (
                gesturesImpl.IsGesture(K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0));
        }, [&switchTeachModeTime, &currentCoordinateSystemMode, &currentJoint, &currentWorldAxis, &robotName, &useROS](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchTeachModeTime) > std::chrono::milliseconds(50)) {
                switchTeachModeTime = currentTime;

                if (*currentCoordinateSystemMode == CoordinateSystemMode::JOINT_MODE) {
                    if (robotName == "wx200" && !useROS) {
                        std::cout << "World coordinate system mode: Not fully supported" << std::endl;
                        return;
                    }

                    currentCoordinateSystemMode.reset(new CoordinateSystemMode(CoordinateSystemMode::WORLD_COORDINATE_MODE));
                    std::cout << "Teach Mode: World coordinate system mode (" << GetStringForWorldAxis(*currentWorldAxis) << ")" << std::endl;
                } else if (*currentCoordinateSystemMode == CoordinateSystemMode::WORLD_COORDINATE_MODE) {
                    currentCoordinateSystemMode.reset(new CoordinateSystemMode(CoordinateSystemMode::JOINT_MODE));
                    std::string jointName = *currentJoint;
                    std::cout << "Teach Mode: Joint Mode (" << jointName << ")" << std::endl;
                }
            }
        }), switchTeachModeGestureGroup, 0, { });

#ifdef MEASUREMENT
    robotArm->SendJointCommands(robotArm->GetRobotInfo()->sleepPosition);
#endif

        std::string tmpJointName = *currentJoint;

#ifdef unix
        int tmpSystemResult = std::system("clear");
#elif defined(_WIN32)
        int tmpSystemResult = std::system("cls");
#endif
        std::cout << "Teach Mode: Joint Mode (" << tmpJointName << ")" << std::endl;
        std::cout << "Teach Mode: World coordinate system mode (" << GetStringForWorldAxis(*currentWorldAxis) << ")" << std::endl;
        saveConfiguratonFile(true);
        std::cout << std::endl << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        gesturesEngine->Start();

        delete gesturesEngine;
        gesturesEngine = nullptr;
    }
}