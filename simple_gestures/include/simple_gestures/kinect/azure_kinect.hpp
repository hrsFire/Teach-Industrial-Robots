#ifndef KINECT_AZURE_KINECT_HPP
#define KINECT_AZURE_KINECT_HPP

#include <iostream>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>

namespace kinect {
    class AzureKinect {
    public:
        bool static StartSensors(k4a::device& device, k4a_device_configuration_t& deviceConfig);
        bool static StartBodyTracking(k4a::device& device, const k4a_device_configuration_t& deviceConfig, k4abt::tracker& bodyTracker);
        void static PrintConfidenceLevel(k4abt_joint_t joint);
        void static Destroy();
    private:
        static bool* canUseGPU;
        static bool CanUseGPU(k4a::device& device, const k4a_device_configuration_t& deviceConfig);
    };
}

#endif //KINECT_AZURE_KINECT_HPP