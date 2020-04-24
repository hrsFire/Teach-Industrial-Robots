#ifndef KINECT_AZURE_KINECT_GESTURES_HPP
#define KINECT_AZURE_KINECT_GESTURES_HPP

#include <iostream>
#include <limits>
#include <k4abt.hpp>
#include <eigen3/Eigen/Geometry>
#include "../gestures/gestures_base.hpp"

namespace kinect {
    class AzureKinectGestures : public virtual gestures::GesturesBase
    {
    public:
        AzureKinectGestures(k4abt::tracker* bodyTracker, k4a::device* device, bool allowDifferentBodies);
        ~AzureKinectGestures();
        void NextCycle() override;
        bool IsNewDataAvailable() override;
        double PerpendicularDistance(uint32_t startJointIndex, uint32_t endJointIndex, uint32_t jointIndex) override;
        void ShowDebugInfo() override;
    private:
        const int64_t MAX_WAIT_TIME_FOR_OTHER_BODY = 5;
        k4abt::tracker* bodyTracker = nullptr;
        k4a::device* device = nullptr;
        bool allowDifferentBodies = false;
        k4a::capture sensorCapture = nullptr;
        k4abt_body_t* previousBody = nullptr;
        k4abt_body_t* body = nullptr;
        uint32_t bodyId = std::numeric_limits<uint32_t>::max();
        std::chrono::milliseconds captureTimeout = std::chrono::milliseconds(0);
        std::chrono::milliseconds enqueueCaptureTimeout = std::chrono::milliseconds(0);
        std::chrono::milliseconds trackerTimeout = std::chrono::milliseconds(0);
        std::chrono::microseconds bodyTimestamp = std::chrono::microseconds(0);
    };
}

#endif //KINECT_AZURE_KINECT_GESTURES_HPP