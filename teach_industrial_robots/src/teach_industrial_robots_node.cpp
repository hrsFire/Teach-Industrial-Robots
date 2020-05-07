#include <iostream>
#include <ros/ros.h>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <simple_gestures/kinect/azure_kinect_gestures.hpp>
#include <simple_gestures/kinect/azure_kinect.hpp>
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

        gestures::GesturesBase* gestures = new kinect::AzureKinectGestures(&bodyTracker, &device, true);
        std::string robotName = "wx200";
        std::string robotModel = "wx200"; // Typically, this will be the same as robotName
        robot_arm::InterbotixRobotArmBase* robotArm = new interbotix::InterbotixRobotArm(false, argc, argv, robotName, robotModel);
        float gripperDistance = 0;
        float jointAngle = 0;

        ros::Duration(0.5).sleep();

        std::vector<JointState> jointStates = robotArm->GetJointStates();
        int numberOfJointStates = jointStates.size();

        while (true) {
            gestures->NextCycle();

            if (gestures->IsNewDataAvailable()) {
                // http://support.interbotix.com/html/specifications/wx200.html#default-joint-limits
                float gripperDiff = 0.002;
                float minGripperDistance = 0.025;
                float maxGripperDistance = 0.076;

                float jointAngleDiff = 1.0 * M_PI / 180.0;
                float minJointAngle = -90.0 * M_PI / 180.0;
                float maxJointAngle = 90.0 * M_PI / 180.0;

                if (gestures->IsGesture(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT, 0, 65.0)) {
                    jointAngle += jointAngleDiff;

                    if (jointAngle > maxJointAngle) {
                        jointAngle = maxJointAngle;
                    }

                    robotArm->SendJointCommand(JointName::ELBOW(), jointAngle);
                } else if (gestures->IsGesture(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, 0, 65.0)) {
                    jointAngle -= jointAngleDiff;

                    if (jointAngle < minJointAngle) {
                        jointAngle = minJointAngle;
                    }

                    robotArm->SendJointCommand(JointName::ELBOW(), jointAngle);
                } else if (gestures->IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_RIGHT, 0, 140.0)) {
                    gripperDistance += gripperDiff;

                    if (gripperDistance > maxGripperDistance) {
                        gripperDistance = maxGripperDistance;
                    }

                    robotArm->SendGripperCommand(gripperDistance);
                } else if (gestures->IsGesture(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_LEFT, 0, 140.0)) {
                    gripperDistance -= gripperDiff;

                    if (gripperDistance < minGripperDistance) {
                        gripperDistance = minGripperDistance;
                    }

                    robotArm->SendGripperCommand(gripperDistance);
                }

                gestures->MarkDataAsUsed();
            }
        }

        std::cout << "Close the device" << std::endl;
        delete gestures;
        bodyTracker.shutdown();
        bodyTracker.destroy();
        device.stop_cameras();
        device.stop_imu();
        device.close();
        kinect::AzureKinect::Destroy();
    }
}