#include <iostream>
#include <ros/ros.h>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <simple_gestures/kinect/azure_kinect_gestures.hpp>
#include <simple_gestures/kinect/azure_kinect.hpp>
#include <simple_gestures/gestures/gestures_engine.hpp>
#include <simple_gestures/gestures/gesture.hpp>
#include <robot_arm_interface/robot_arm_base.hpp>
#include <interbotix_robot_arms_wrapper/interbotix_robot_arm.hpp>

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

        gestures::GesturesBase* gesturesImpl = new kinect::AzureKinectGestures(&bodyTracker, &device, true);
        std::string robotName = "wx200";
        std::string robotModel = "wx200"; // Typically, this will be the same as robotName
        robot_arm::InterbotixRobotArmBase* robotArm = new interbotix::InterbotixRobotArm(false, argc, argv, robotName, robotModel);

        ros::Duration(0.5).sleep();

        std::unordered_map<JointName, JointState> jointStates = robotArm->GetJointStates();
        double gripperDistance = jointStates.at(JointName::GRIPPER()).position;
        double jointAngle = jointStates.at(JointName::ELBOW()).position;
        std::shared_ptr<RobotInfo> robotInfo = robotArm->GetRobotInfo();

        // http://support.interbotix.com/html/specifications/wx200.html#default-joint-limits
        double minGripperDistance = robotInfo->joints.at(JointName::GRIPPER()).lowerLimit;
        double maxGripperDistance = robotInfo->joints.at(JointName::GRIPPER()).upperLimit;

        double minJointAngle = robotInfo->joints.at(JointName::ELBOW()).lowerLimit;
        double maxJointAngle = robotInfo->joints.at(JointName::ELBOW()).upperLimit;

        gestures::GesturesEngine gesturesEngine(gesturesImpl);

        gesturesEngine.AddGesture(gestures::Gesture([](gestures::GesturesQuery& gesturesImpl) -> bool {
            return gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 65.0);
        }, [&robotArm, &jointAngle, maxJointAngle](std::chrono::milliseconds duration) {
            jointAngle += robotArm->CalculateAcceleration(JointName::ELBOW(), duration);

            if (jointAngle > maxJointAngle) {
                jointAngle = maxJointAngle;
            }

            robotArm->SendJointCommand(JointName::ELBOW(), jointAngle);
        }), "single_joint", 0, 0, { JointName::ELBOW() });

        gesturesEngine.AddGesture(gestures::Gesture([](gestures::GesturesQuery& gesturesImpl) -> bool {
            return gesturesImpl.IsGesture(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 65.0);
        }, [&robotArm, &jointAngle, minJointAngle](std::chrono::milliseconds duration) {
            jointAngle -= robotArm->CalculateAcceleration(JointName::ELBOW(), duration);

            if (jointAngle < minJointAngle) {
                jointAngle = minJointAngle;
            }

            robotArm->SendJointCommand(JointName::ELBOW(), jointAngle);
        }), "single_joint", 0, 1, { JointName::ELBOW() });

        gesturesEngine.AddGesture(gestures::Gesture([](gestures::GesturesQuery& gesturesImpl) -> bool {
            return gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0);
        }, [&robotArm, &gripperDistance, maxGripperDistance](std::chrono::milliseconds duration) {
            gripperDistance += robotArm->CalculateAcceleration(JointName::GRIPPER(), duration);

            if (gripperDistance > maxGripperDistance) {
                gripperDistance = maxGripperDistance;
            }

            robotArm->SendGripperCommand(gripperDistance);
        }), "gripper", 1, 0, { JointName::GRIPPER() });

        gesturesEngine.AddGesture(gestures::Gesture([](gestures::GesturesQuery& gesturesImpl) -> bool {
            return gesturesImpl.IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0);
        }, [&robotArm, &gripperDistance, minGripperDistance](std::chrono::milliseconds duration) {
            gripperDistance -= robotArm->CalculateAcceleration(JointName::GRIPPER(), duration);

            if (gripperDistance < minGripperDistance) {
                gripperDistance = minGripperDistance;
            }

            robotArm->SendGripperCommand(gripperDistance);
        }), "gripper", 1, 1, { JointName::GRIPPER() });

        gesturesEngine.Start();

        std::cout << "Close the device" << std::endl;
        gesturesEngine.Stop();
        bodyTracker.shutdown();
        bodyTracker.destroy();
        device.stop_cameras();
        device.stop_imu();
        device.close();
        kinect::AzureKinect::Destroy();
    }
}