#include "simple_gestures/kinect/azure_kinect_gestures.hpp"
#include "simple_gestures/kinect/azure_kinect.hpp"
#include "../common/angle_converter.hpp"
#include "../common/vector3.hpp"

using namespace kinect;

AzureKinectGestures::AzureKinectGestures(k4abt::tracker* bodyTracker, k4a::device* device, bool allowDifferentBodies) {
    this->bodyTracker = bodyTracker;
    this->device = device;
    this->allowDifferentBodies = allowDifferentBodies;
}

AzureKinectGestures::~AzureKinectGestures() {
}

void AzureKinectGestures::NextCycle() {
    try {
        // Capture a depth frame
        if (device->get_capture(&sensorCapture, captureTimeout)) {
            if (bodyTracker->enqueue_capture(sensorCapture, enqueueCaptureTimeout)) {
                k4abt::frame frame = bodyTracker->pop_result(trackerTimeout);

                uint32_t numberOfBodies = frame.get_num_bodies();
                std::chrono::microseconds timestamp = frame.get_device_timestamp();
                std::cout << "Number of bodies: " << numberOfBodies << std::endl;
                std::cout << "Timestamp: " << timestamp.count() * 1e-6 << " s" << std::endl;

                if (numberOfBodies == 1 || (allowDifferentBodies && numberOfBodies > 1)) {
                    k4abt_body_t* tmpBody = new k4abt_body_t(frame.get_body(0));
                    std::cout << "Body ID: " << tmpBody->id << std::endl;

                    // Each person has it's own body ID for temporal correlation between frames and the kinematic skeleton
                    // https://docs.microsoft.com/de-de/azure/Kinect-dk/body-joints
                    if ((allowDifferentBodies &&
                            (std::chrono::duration_cast<std::chrono::seconds>(timestamp - bodyTimestamp).count() > MAX_WAIT_TIME_FOR_OTHER_BODY ||
                            bodyId == tmpBody->id)) || bodyId == std::numeric_limits<uint32_t>::max()) {
                        delete previousBody;
                        previousBody = body;
                        body = tmpBody;
                        bodyId = tmpBody->id;
                        bodyTimestamp = timestamp;
                        dataWasUsed = false;
                    } else {
                        body = nullptr;
                        std::cout << "This isn't the same person as before" << std::endl;
                    }
                } else if (numberOfBodies > 1) {
                    body = nullptr;
                    std::cout << "Multiple persons have been detected" << std::endl;
                } else {
                    body = nullptr;
                    std::cout << "No person has been detected" << std::endl;
                }

                // Release the sensor capture
                sensorCapture.reset();

                std::cout << std::endl << "------------------------" << std::endl << std::endl;
            }
        }
    } catch (k4a::error e) {
        std::cerr << "Could not retrieve the data from the body tracker" << std::endl;
    }
}

bool AzureKinectGestures::IsNewDataAvailable() {
    return dataWasUsed == false && body != nullptr && &previousBody != &body;
}

double AzureKinectGestures::PerpendicularDistance(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex) {
    k4abt_joint_t startJoint = body->skeleton.joints[startJointIndex];
    k4abt_joint_t endJoint = body->skeleton.joints[endJointIndex];
    k4abt_joint_t joint = body->skeleton.joints[jointIndex];

    if ((startJoint.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM || startJoint.confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH) &&
            (endJoint.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM || endJoint.confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH) &&
            (joint.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM || joint.confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH)) {

        return common::Vector3::PerpendicularDistance(glm::vec3(startJoint.position.xyz.x, startJoint.position.xyz.y, startJoint.position.xyz.z),
            glm::vec3(endJoint.position.xyz.x, endJoint.position.xyz.y, endJoint.position.xyz.z),
            glm::vec3(joint.position.xyz.x, joint.position.xyz.y, joint.position.xyz.z));
    }

    return 0;
}

void AzureKinectGestures::ShowDebugInfo() {
    if (body != nullptr) {
        k4abt_joint_t joint = body->skeleton.joints[K4ABT_JOINT_HAND_RIGHT];

        // The confidence level needs to be at least medium. Otherwise the joint is out of range or not visible likely due to occlusion.
        if (joint.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM || joint.confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH) {
            std::cout << "Joint visible" << std::endl << std::endl;
            kinect::AzureKinect::PrintConfidenceLevel(joint);
            std::cout << std::endl;

            // The position of the joint is specified in millimeters
            // https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/1.x.x/structk4abt__joint__t.html
            std::cout << "Coordinates:" << std::endl;
            std::cout << "  * x: " << joint.position.xyz.x / 10.0 << " cm" << std::endl;
            std::cout << "  * y: " << joint.position.xyz.y / 10.0 << " cm" << std::endl;
            std::cout << "  * z: " << joint.position.xyz.z / 10.0 << " cm" << std::endl << std::endl;
            
            /*std::cout << "Quaternion Orientation:" << std::endl;
            std::cout << "  * w: " << joint.orientation.wxyz.w << std::endl;
            std::cout << "  * x: " << joint.orientation.wxyz.x << std::endl;
            std::cout << "  * y: " << joint.orientation.wxyz.y << std::endl;
            std::cout << "  * z: " << joint.orientation.wxyz.z << std::endl << std::endl;

            // https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
            Eigen::Quaternionf orientation(joint.orientation.wxyz.w, joint.orientation.wxyz.x, joint.orientation.wxyz.y, joint.orientation.wxyz.z);
            Eigen::Vector3f eulerOrientation = orientation.toRotationMatrix().eulerAngles(0, 0, 0);

            std::cout << "Euler Orientation:" << std::endl;
            std::cout << "  * x: " << common::AngleConverter::DegreeFromRadian(eulerOrientation[0]) << "°" << std::endl;
            std::cout << "  * y: " << common::AngleConverter::DegreeFromRadian(eulerOrientation[1]) << "°" << std::endl;
            std::cout << "  * z: " << common::AngleConverter::DegreeFromRadian(eulerOrientation[2]) << "°" << std::endl << std::endl;*/

            std::cout << "------------------------" << std::endl << std::endl;
        }
    }
}