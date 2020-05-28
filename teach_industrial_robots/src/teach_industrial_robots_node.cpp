#include <iostream>
#include <memory>
#include <csignal>
#include <mutex>
#include <unordered_map>
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

int main(int argc, char** argv) {
    std::cout << "#################################" << std::endl;
    std::cout << "###  Teach Industrial Robots  ###" << std::endl;
    std::cout << "#################################" << std::endl << std::endl;

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

        std::string robotName = "wx200";
        std::string robotModel = "wx200"; // Typically, this will be the same as robotName
        bool useROS = false;
        robotArm = new interbotix::InterbotixRobotArm(useROS, argc, argv, robotName, robotModel);

        std::unordered_map<robot_arm::JointNameImpl, robot_arm::JointState> jointStates = robotArm->GetJointStates();
        std::shared_ptr<robot_arm::JointName> currentJoint = std::make_shared<interbotix::InterbotixJointName>(interbotix::InterbotixJointName::ELBOW());
        std::chrono::system_clock::time_point switchToPrevJointTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchToNextJointTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchPrecisionModeTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point switchGripperLockTime = std::chrono::system_clock::now();
        std::unique_ptr<bool> isPreciseMode = std::make_unique<bool>(true);
        std::unique_ptr<bool> isGripperLocked = std::make_unique<bool>(true);

        gesturesEngine = new gestures::GesturesEngine(new kinect::AzureKinectGestures(&bodyTracker, &device, true));
        gestures::GestureGroup singleJointGestureGroup = gesturesEngine->AddGestureGroup("single_joint", 0, {});
        gestures::GestureGroup gripperGestureGroup = gesturesEngine->AddGestureGroup("gripper", 1, {});
        gestures::GestureGroup switchJointGestureGroup = gesturesEngine->AddGestureGroup("switch_joint", 2, { singleJointGestureGroup });
        gestures::GestureGroup switchPrecisionModeGestureGroup = gesturesEngine->AddGestureGroup("switch_precision_mode", 3, { singleJointGestureGroup, switchJointGestureGroup });

        gesturesEngine->AddGesture(gestures::Gesture([](const gestures::GesturesQuery& gesturesImpl) -> bool {
            return gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 65.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 65.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_EAR_LEFT, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_EAR_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
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
            return gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0) ||
                gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
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

        signal(SIGINT, [](int i) {
            ExitSafely();
        });

        signal(SIGTERM, [](int i) {
            ExitSafely();
        });

        signal(SIGQUIT, [](int i) {
            ExitSafely();
        });

        gesturesEngine->Start();

        delete gesturesEngine;
        gesturesEngine = nullptr;
    }
}