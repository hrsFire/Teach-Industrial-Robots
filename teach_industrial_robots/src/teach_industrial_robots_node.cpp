#include <iostream>
#include <memory>
#include <csignal>
#include <mutex>
#include <unordered_map>
#include <locale>
#include <ros/ros.h>
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
    std::cout << "  " << "-opf, --overwrite-positions-file" << "\t" << "Enables to overwrite the positions file."
        << " Make a backup of the positions file, becasue otherwise this leads to data loss." << std::endl;
    std::cout << "  " << "-rrp, --repeat-recorded-positions" << "\t" << "Repeat the recorded positions from the positions file" << std::endl;
    std::cout << "  " << "-tp, --teach-positions" << "\t\t" << "Teach positions" << std::endl;
    std::cout << "  " << "-rn, --robot-name" << "\t\t\t" << "Robot name" << std::endl;
    std::cout << "  " << "-rm, --robot-model" << "\t\t\t" << "Robot model (typically the same as the robot name)" << std::endl;
    std::cout << "  " << "-ur, --use-ros" << "\t\t\t" << "Use ROS or use a direct connection" << std::endl;
    std::cout << "  " << "-mhaex, --move-home-at-exit" << "\t\t\t" << "Move home at exit" << std::endl;
    std::cout << "  " << "-mhaer, --move-home-at-error" << "\t\t\t" << "Move home at error" << std::endl;
    std::cout << "  " << "--help" << "\t\t\t\t" << "This help" << std::endl;
}

bool IsLookingAtDevice(const gestures::GesturesQuery& gesturesImpl) {
    return gesturesImpl.IsJointVisible(K4ABT_JOINT_EYE_LEFT) && gesturesImpl.IsJointVisible(K4ABT_JOINT_EYE_RIGHT);
}

int main(int argc, char** argv) {
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
    useROS = false;
    moveHomeAtExit = true;
    moveHomeAtError = true;
#else
    if (argc == 1) {
        PrintHelp();
        return 0;
    }

    bool keyFound = false;
    std::string key;

    for (size_t i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (!keyFound) {
            if (arg.rfind("-", 0) == 0 || arg.rfind("--", 0) == 0) {
                key = arg;
                bool requiresValue = false;

                if (key == "--overwrite-positions-file" || key == "-opf") {
                    overwritePositionsFile = true;
                } else if (key == "--repeat-recorded-positions" || key == "-rrp") {
                    repeatRecordedPositions = true;
                } else if (key == "--teach-positions" || key == "-tp") {
                    teachPositions = true;
                } else if (key == "--robot-name" || key == "-rn") {
                    requiresValue = true;
                } else if (key == "--robot-model" || key == "-rm") {
                    requiresValue = true;
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

                if (requiresValue && i == argc -1) {
                    std::cout << std::endl << "> Key requires value: " << key << std::endl;
                    PrintHelp();
                    return 0;
                }

                if (requiresValue) {
                    keyFound = true;
                }
            } else {
                keyFound == false;
                std::cout << std::endl << "> Invalid parameter: " << arg << std::endl;
                PrintHelp();
                return 0;
            }
        } else {
            // Check for keys with values
            if (key == "--robot-name" || key == "-rn") {
                robotName = arg;
            } else if (key == "--robot-model" || key == "-rm") {
                robotModel = arg;
            } else {
                PrintHelp();
                return 0;
            }

            keyFound = false;
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

        if (configurationStorage.LoadPositions(interbotix::InterbotixJointName::NONE(), recordedPositions)) {
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
        std::unique_ptr<bool> isPreciseMode = std::make_unique<bool>(true);
        std::unique_ptr<bool> isGripperLocked = std::make_unique<bool>(true);
        std::unique_ptr<bool> successfullySavedConfiguration = std::make_unique<bool>(false);

        gesturesEngine = new gestures::GesturesEngine(new kinect::AzureKinectGestures(&bodyTracker, &device, true));
        gestures::GestureGroup singleJointGestureGroup = gesturesEngine->AddGestureGroup("single_joint", 0, {});
        gestures::GestureGroup gripperGestureGroup = gesturesEngine->AddGestureGroup("gripper", 1, {});
        gestures::GestureGroup switchJointGestureGroup = gesturesEngine->AddGestureGroup("switch_joint", 2, { singleJointGestureGroup });
        gestures::GestureGroup switchPrecisionModeGestureGroup = gesturesEngine->AddGestureGroup("switch_precision_mode", 3, { singleJointGestureGroup,
            switchJointGestureGroup });
        gestures::GestureGroup configurationGestureGroup = gesturesEngine->AddGestureGroup("configuration", 4, { singleJointGestureGroup, gripperGestureGroup,
            switchJointGestureGroup, switchPrecisionModeGestureGroup });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 65.0);
        }, [&currentJoint, &isPreciseMode](std::chrono::milliseconds duration) {
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
            double jointAngle = jointStates.at(currentJoint).GetPosition();

            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            jointAngle += robotArm->CalculateAcceleration(*currentJoint, duration, true);
            robotArm->SendJointCommand(*currentJoint, jointAngle);
        }), singleJointGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) &&
                gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 65.0);
        }, [&currentJoint, &isPreciseMode](std::chrono::milliseconds duration) {
            std::shared_ptr<robot_arm::RobotInfo> robotInfo = robotArm->GetRobotInfo();
            double minJointAngle = robotInfo->joints.at(currentJoint).GetLowerLimit();
            std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
            double jointAngle = jointStates.at(currentJoint).GetPosition();

            if (*isPreciseMode) {
                duration = std::chrono::milliseconds(0);
            }

            jointAngle -= robotArm->CalculateAcceleration(*currentJoint, duration, false);
            robotArm->SendJointCommand(*currentJoint, jointAngle);
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

            gripperDistance += robotArm->CalculateAcceleration(interbotix::InterbotixJointName::GRIPPER(), duration, false);
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

            gripperDistance -= robotArm->CalculateAcceleration(interbotix::InterbotixJointName::GRIPPER(), duration, false);
            robotArm->SendGripperCommand(gripperDistance);
        }), gripperGestureGroup, 1, { interbotix::InterbotixJointName::GRIPPER() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl)&& (
                gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0));
        }, [&isGripperLocked, &switchGripperLockTime](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchGripperLockTime) > std::chrono::seconds(1)) {
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
        }, [&currentJoint, &switchToPrevJointTime](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchToPrevJointTime) > std::chrono::seconds(1)) {
                switchToPrevJointTime = currentTime;
                currentJoint = currentJoint->Prev();
                std::cout << "Set previous joint: "<< (std::string) *currentJoint << std::endl;
            }
        }), switchJointGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && gesturesImpl.IsGesture(K4ABT_JOINT_EAR_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
        }, [&currentJoint, &switchToNextJointTime](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchToNextJointTime) > std::chrono::seconds(1)) {
                switchToNextJointTime = currentTime;
                currentJoint = currentJoint->Next();
                std::cout << "Set next joint: " << (std::string) *currentJoint << std::endl;
            }
        }), switchJointGestureGroup, 1, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return IsLookingAtDevice(gesturesImpl) && (
                gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0));
        }, [&currentJoint, &switchPrecisionModeTime, &isPreciseMode](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - switchPrecisionModeTime) > std::chrono::seconds(1)) {
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
        }, [&saveConfigurationTime, &recordedPositions, &overwritePositionsFile, &successfullySavedConfiguration](std::chrono::milliseconds duration) {
            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

            if (duration == std::chrono::milliseconds(0) && std::chrono::duration_cast<std::chrono::seconds>(currentTime - saveConfigurationTime) > std::chrono::seconds(1)) {
                saveConfigurationTime = currentTime;
                robot_arm_common::ConfigurationStorage configurationStorage;
                recordedPositions.push_back(robotArm->GetJointStates());

                if (*successfullySavedConfiguration) {
                    overwritePositionsFile = true;
                }

                if (configurationStorage.SavePositions(recordedPositions, overwritePositionsFile)) {
                    std::cout << "Saved recorded positions" << std::endl;
                    successfullySavedConfiguration.reset(new bool(true));
                } else {
                    std::cout << "Could not save recorded positions";

                    if (!overwritePositionsFile) {
                        std::cout << ", because it isn't allowed to overwrite the file. Please make a backup or use the specific command line option." << std::endl;
                    }

                    std::cout << std::endl;
                }
            }
        }), configurationGestureGroup, 0, { interbotix::InterbotixJointName::WAIST(), interbotix::InterbotixJointName::SHOULDER(), interbotix::InterbotixJointName::ELBOW(),
            interbotix::InterbotixJointName::FOREARM_ROLL(), interbotix::InterbotixJointName::WRIST_ANGLE(), interbotix::InterbotixJointName::WRIST_ROTATE() });

        gesturesEngine->Start();

        delete gesturesEngine;
        gesturesEngine = nullptr;
    }
}