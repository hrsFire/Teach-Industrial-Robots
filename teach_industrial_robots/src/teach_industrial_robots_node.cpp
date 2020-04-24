#include <iostream>
#include <ros/ros.h>
#include <interbotix_sdk/arm_obj.h>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <simple_gestures/kinect/azure_kinect_gestures.hpp>
#include <simple_gestures/kinect/azure_kinect.hpp>

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

        // https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_sdk
        // One is to command the robot via the ROS topics and/or services. In this manner, the developer can code in
        // any language that is capable of sending a ROS message. The other approach is to 'skip' the ROS topic layer
        // and use the publicly available functions directly. All the user would need to do is create an instance of the
        // 'RobotArm' class as shown here to take advantage of these functions.
        // https://github.com/Interbotix/interbotix_ros_arms/blob/master/interbotix_sdk/src/arm_node.cpp
        ros::init(argc, argv, "teach_industrial_robots");
        ros::NodeHandle nodeHandle;

        std::string robotName = "wx200";
        std::string robotModel = "wx200"; // Typically, this will be the same as robotName
        /*ros::param::get("~robot_name", robot_name);
        ros::param::get("~robot_model", robot_model);*/
        // https://github.com/Interbotix/interbotix_ros_arms/blob/melodic/interbotix_sdk/launch/arm_run.launch
        ros::param::set("~motor_configs", "/home/ubuntu/interbotix_ws/src/interbotix_ros_arms/interbotix_sdk/config/");
        RobotArm robotArm(&nodeHandle, robotName, robotModel);
        //ros::spin();
        float gripperDistance = 0;
        float jointAngle = 0;

        while (true) {
            gestures->NextCycle();

            if (gestures->IsNewDataAvailable()) {
                double distanceToRightShoulder = gestures->PerpendicularDistance(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_HANDTIP_LEFT);
                std::cout << "Distance (to right shoulder): " << distanceToRightShoulder * 0.1 << " cm" << std::endl;

                double distanceToLeftShoulder = gestures->PerpendicularDistance(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_HANDTIP_RIGHT);
                std::cout << "Distance (to left shoulder): " << distanceToLeftShoulder * 0.1 << " cm" << std::endl;

                double distanceToChestRight = gestures->PerpendicularDistance(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT);
                std::cout << "Distance (to chest, right): " << distanceToChestRight * 0.1 << " cm" << std::endl;

                double distanceToChestLeft = gestures->PerpendicularDistance(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_HANDTIP_LEFT, K4ABT_JOINT_HANDTIP_LEFT);
                std::cout << "Distance (to spine naval, left): " << distanceToChestLeft * 0.1 << " cm" << std::endl;
                //gestures->ShowDebugInfo();

                // http://support.interbotix.com/html/specifications/wx200.html#default-joint-limits
                float gripperDiff = 0.002;
                float minGripperDistance = 0.025;
                float maxGripperDistance = 0.076;

                float jointAngleDiff = 1.0 * M_PI / 180.0;
                float minJointAngle = -90.0 * M_PI / 180.0;
                float maxJointAngle = 90.0 * M_PI / 180.0;

                if (distanceToRightShoulder > 0 && distanceToRightShoulder < 65.0) {
                    jointAngle += jointAngleDiff;

                    if (jointAngle > maxJointAngle) {
                        jointAngle = maxJointAngle;
                    }

                    robotArm.arm_set_single_joint_angular_position("elbow", jointAngle);
                } else if (distanceToLeftShoulder > 0 && distanceToLeftShoulder < 65.0) {
                    jointAngle -= jointAngleDiff;

                    if (jointAngle < minJointAngle) {
                        jointAngle = minJointAngle;
                    }

                    robotArm.arm_set_single_joint_angular_position("elbow", jointAngle);
                } else if (distanceToChestRight > 0 && distanceToChestRight < 0.01) {
                    gripperDistance += gripperDiff;

                    if (gripperDistance > maxGripperDistance) {
                        gripperDistance = maxGripperDistance;
                    }

                    robotArm.arm_set_gripper_linear_position(gripperDistance);
                } else if (distanceToChestLeft > 0 && distanceToChestLeft < 0.01) {
                    gripperDistance -= gripperDiff;

                    if (gripperDistance < minGripperDistance) {
                        gripperDistance = minGripperDistance;
                    }

                    robotArm.arm_set_gripper_linear_position(gripperDistance);
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