#ifndef KINECT_AZURE_KINECT_HPP
#define KINECT_AZURE_KINECT_HPP

#include <iostream>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>

/**
* This namespace provides an implementation of the gesture recognition abstraction layer for the kinect device family.
*/
namespace kinect {
    /**
     * This class provides common tasks for interacting with the Azure Kinect.
     */
    class AzureKinect {
    public:
        /**
         * Starts the sensors of the Azure Kinect.
         * @param device        A device handler for the Azure Kinect.
         * @param deviceConfig  The configuration which should be used to start the sensors of the Azure Kinect.
         * @return True if the Azure Kinect sensors could be started. Otherwise False is returned.
         */
        bool static StartSensors(k4a::device& device, k4a_device_configuration_t& deviceConfig);
        /**
         * Starts the body tracking of the Azure Kinect.
         * @param device        A device handler for the Azure Kinect.
         * @param deviceConfig  The configuration which should be used to start the sensors of the Azure Kinect.
         * @param bodyTracker   A handler for the Azure Kinect body tracking.
         * @return True if the Azure Kinect body tracking could be started. Otherwise False is returned.
         */
        bool static StartBodyTracking(k4a::device& device, const k4a_device_configuration_t& deviceConfig, k4abt::tracker& bodyTracker);
        /**
         * Prints the confidence level of a joint to the console.
         * @param joint  The joint whose confidence level should be printed to the console.
         * @return void
         */
        void static PrintConfidenceLevel(k4abt_joint_t joint);
        /**
         * Resets the usage of this helper class.
         * @return void
         */
        void static Destroy();
    private:
        static bool* canUseGPU;
        static bool CanUseGPU(k4a::device& device, const k4a_device_configuration_t& deviceConfig);
    };
}

#endif //KINECT_AZURE_KINECT_HPP
