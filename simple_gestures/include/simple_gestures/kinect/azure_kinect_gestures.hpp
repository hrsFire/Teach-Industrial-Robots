#ifndef KINECT_AZURE_KINECT_GESTURES_HPP
#define KINECT_AZURE_KINECT_GESTURES_HPP

#include <iostream>
#include <limits>
#include <k4abt.hpp>
#include <eigen3/Eigen/Geometry>
#include "../gestures/gestures_base.hpp"

namespace kinect {
    /**
     * This class provides an implementation of the gesture recognition abstraction layer for the Azure Kinect.
     */
    class AzureKinectGestures : public gestures::GesturesBase {
    public:
        /**
         * Default constructor for the gesture recognition abstraction layer for the Azure Kinect. The given device and body tracking handler will be managed by this class afterwards.
         * @param bodyTracker           A handler for the Azure Kinect body tracking.
         * @param device                A device handler for the Azure Kinect.
         * @param allowDifferentBodies  Specifies if more than two bodies are allowed.
         */
        AzureKinectGestures(k4abt::tracker* bodyTracker, k4a::device* device, bool allowDifferentBodies);
        /**
         * Destructor for the gesture recognition abstraction layer for the Azure Kinect. The given device and body tracking handler will be destroyed.
         */
        ~AzureKinectGestures() override;
        void NextCycle() override;
        bool IsNewDataAvailable() override;
        bool IsGesture(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex, double minDistance, double maxDistance) const override;
        bool IsGesture(uint32_t jointIndex1, uint32_t jointIndex2, double minDistance, double maxDistance) const override;
        bool IsJointVisible(uint32_t jointIndex) const override;
    private:
        const int64_t MAX_WAIT_TIME_FOR_OTHER_BODY = 5;
        const double DEVIATION = 3.0;
        k4abt::tracker* bodyTracker = nullptr;
        k4a::device* device = nullptr;
        bool allowDifferentBodies = false;
        k4a::capture sensorCapture = nullptr;
        k4abt_body_t* previousBody = nullptr;
        k4abt_body_t* body = nullptr;
        uint32_t bodyId = std::numeric_limits<uint32_t>::max();
        bool hasPersonBeenDetected = false;
        std::chrono::milliseconds captureTimeout = std::chrono::milliseconds(0);
        std::chrono::milliseconds enqueueCaptureTimeout = std::chrono::milliseconds(0);
        std::chrono::milliseconds trackerTimeout = std::chrono::milliseconds(0);
        std::chrono::microseconds bodyTimestamp = std::chrono::microseconds(0);
    };
}

#endif //KINECT_AZURE_KINECT_GESTURES_HPP
