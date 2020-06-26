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
    std::cout << "Close the device" << std::endl;
    bodyTracker->shutdown();
    bodyTracker->destroy();
    device->stop_cameras();
    device->stop_imu();
    device->close();
    kinect::AzureKinect::Destroy();
}

void AzureKinectGestures::NextCycle() {
    try {
        // Capture a depth frame
        if (device->get_capture(&sensorCapture, captureTimeout)) {
            if (bodyTracker->enqueue_capture(sensorCapture, enqueueCaptureTimeout)) {
                k4abt::frame frame = bodyTracker->pop_result(trackerTimeout);

                uint32_t numberOfBodies = frame.get_num_bodies();
                std::chrono::microseconds timestamp = frame.get_device_timestamp();
#ifndef NDEBUG
                std::cout << "Number of bodies: " << numberOfBodies << std::endl;
                std::cout << "Timestamp: " << timestamp.count() * 1e-6 << " s" << std::endl;
#endif

                if (numberOfBodies == 1 || (allowDifferentBodies && numberOfBodies > 1)) {
                    k4abt_body_t* tmpBody = new k4abt_body_t(frame.get_body(0));
#ifndef NDEBUG
                    std::cout << "Body ID: " << tmpBody->id << std::endl;
#endif

                    // Each person has it's own body ID for temporal correlation between frames and the kinematic skeleton
                    // https://docs.microsoft.com/de-de/azure/Kinect-dk/body-joints
                    if ((allowDifferentBodies &&
                            std::chrono::duration_cast<std::chrono::seconds>(timestamp - bodyTimestamp).count() > MAX_WAIT_TIME_FOR_OTHER_BODY) ||
                            (allowDifferentBodies && bodyId == tmpBody->id) ||
                            (!allowDifferentBodies && numberOfBodies == 1)) {
                        if (previousBody != nullptr) {
                            delete previousBody;
                        }

                        previousBody = body;
                        body = tmpBody;
                        bodyId = tmpBody->id;
                        bodyTimestamp = timestamp;
                        dataWasUsed = false;
                    } else {
                        if (body != nullptr) {
                            delete previousBody;
                            delete body;
                        }

                        body = nullptr;
                        std::cout << "This isn't the same person as before" << std::endl;
                    }
                } else if (numberOfBodies > 1) {
                    if (body != nullptr) {
                        delete previousBody;
                        delete body;
                    }

                    body = nullptr;
                    std::cout << "Multiple persons have been detected" << std::endl;
                } else {
                    if (body != nullptr) {
                        delete previousBody;
                        delete body;
                    }

                    body = nullptr;
                    std::cout << "No person has been detected" << std::endl;
                }

                // Release the sensor capture
                sensorCapture.reset();

#ifndef NDEBUG
                std::cout << std::endl << "------------------------" << std::endl << std::endl;
#endif
            }
        }
    } catch (k4a::error e) {
        std::cerr << "Could not retrieve the data from the body tracker" << std::endl;
    }
}

bool AzureKinectGestures::IsNewDataAvailable() {
    return dataWasUsed == false && body != nullptr && &previousBody != &body;
}

bool AzureKinectGestures::IsGesture(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex, double minDistance, double maxDistance) const {
    k4abt_joint_t startJoint = body->skeleton.joints[startJointIndex];
    k4abt_joint_t endJoint = body->skeleton.joints[endJointIndex];
    k4abt_joint_t joint = body->skeleton.joints[jointIndex];

    // The confidence level needs to be at least medium. Otherwise the joint is out of range or not visible likely due to occlusion.
    if (IsJointVisible(startJointIndex) && IsJointVisible(endJointIndex) && IsJointVisible(jointIndex)) {
        glm::vec3 startPoint = glm::vec3(startJoint.position.xyz.x, startJoint.position.xyz.y, startJoint.position.xyz.z);
        glm::vec3 endPoint = glm::vec3(endJoint.position.xyz.x, endJoint.position.xyz.y, endJoint.position.xyz.z);
        glm::vec3 point = glm::vec3(joint.position.xyz.x, joint.position.xyz.y, joint.position.xyz.z);

        double perpendicularDistance = common::Vector3::PerpendicularDistance(startPoint, endPoint, point);
        double deviationOnLine = common::Vector3::DeviationOnLine(startPoint, endPoint, point);

#ifndef NDEBUG
        std::cout << "Distance: " << perpendicularDistance << " mm, " << deviationOnLine << " mm" << std::endl;
#endif //NDEBUG

        return perpendicularDistance >= minDistance && perpendicularDistance <= maxDistance && deviationOnLine >= -DEVIATION;
    }

    return false;
}

bool AzureKinectGestures::IsGesture(uint32_t jointIndex1, uint32_t jointIndex2, double minDistance, double maxDistance) const {
    k4abt_joint_t joint1 = body->skeleton.joints[jointIndex1];
    k4abt_joint_t joint2 = body->skeleton.joints[jointIndex2];

    // The confidence level needs to be at least medium. Otherwise the joint is out of range or not visible likely due to occlusion.
    if (IsJointVisible(jointIndex1) && IsJointVisible(jointIndex2)) {
        // Get the distance to the center of the sphere
        double distance = common::Vector3::Distance(glm::vec3(joint1.position.xyz.x, joint1.position.xyz.y, joint1.position.xyz.z),
            glm::vec3(joint2.position.xyz.x, joint2.position.xyz.y, joint2.position.xyz.z));

#ifndef NDEBUG
        std::cout << "Distance: " << distance << " mm" << std::endl;
#endif //NDEBUG

        return distance >= minDistance && distance <= maxDistance;
    }

    return false;
}

bool AzureKinectGestures::IsJointVisible(uint32_t jointIndex) const {
    k4abt_joint_t joint = body->skeleton.joints[jointIndex];

    return joint.confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM || joint.confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH;
}