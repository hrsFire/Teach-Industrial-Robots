#include "simple_gestures/kinect/azure_kinect.hpp"

using namespace kinect;

bool* AzureKinect::canUseGPU = nullptr;

bool AzureKinect::CanUseGPU(k4a::device& device, const k4a_device_configuration_t& deviceConfig) {
    if (canUseGPU == nullptr) {
        k4abt_tracker_configuration_t trackerConfiguration = K4ABT_TRACKER_CONFIG_DEFAULT;
        k4a_calibration_t sensorCalibration;
        sensorCalibration = device.get_calibration(deviceConfig.depth_mode, deviceConfig.color_resolution);
        
        try {
            // Try to use CUDA
            trackerConfiguration.processing_mode = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_GPU;
            k4abt::tracker bodyTracker = k4abt::tracker::create(sensorCalibration, trackerConfiguration);
            bodyTracker.shutdown();
            std::cout << std::endl << std::endl;
            canUseGPU = new bool(true);
        } catch (k4a::error e) {
            canUseGPU = new bool(false);
        }
    }

    return *canUseGPU;
}

bool AzureKinect::StartSensors(k4a::device& device, k4a_device_configuration_t& deviceConfig) {
    // https://docs.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
    // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/release/1.4.x/structk4a__device__configuration__t.html
    // https://github.com/MicrosoftDocs/azure-docs/blob/master/articles/kinect-dk/build-first-body-app.md#open-device-and-start-the-camera
    // "For best results set the depth mode to K4A_DEPTH_MODE_NFOV_UNBINNED or K4A_DEPTH_MODE_WFOV_2X2BINNED."
    // NFOV_UNBINNED - Narraw Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]
    // WFOV_BINNED   - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]
    deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    //deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    //deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    //deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    //deviceConfig.disable_streaming_indicator = false;
    //deviceConfig.synchronized_images_only = true;

    try {
        std::cout << "Start cameras" << std::endl;
        device.start_cameras(&deviceConfig);
    } catch (k4a::error e) {
        std::cout << e.what() << std::endl;
        device.close();
        return false;
    }

    return true;
}

bool AzureKinect::StartBodyTracking(k4a::device& device, const k4a_device_configuration_t& deviceConfig, k4abt::tracker& bodyTracker) {
    k4a_calibration_t sensorCalibration;

    try {
        sensorCalibration = device.get_calibration(deviceConfig.depth_mode, deviceConfig.color_resolution);
    } catch (k4a::error e) {
        std::cout << e.what() << std::endl;
        device.close();
        return 0;
    }

    k4abt_tracker_configuration_t trackerConfiguration = K4ABT_TRACKER_CONFIG_DEFAULT;

    bool canUseGPU;

    try {
        canUseGPU = CanUseGPU(device, deviceConfig);
    } catch (k4a::error e) {
        return false;
    }

    if (canUseGPU) {
        std::cout << std::endl << std::endl << "There is no Nvidia GPU available which supports CUDA. Therefore the CPU mode is used as fallback!" << std::endl;

        try {
            trackerConfiguration.processing_mode = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_GPU;
            bodyTracker = k4abt::tracker::create(sensorCalibration, trackerConfiguration);
        } catch (k4a::error e) {
            std::cerr << e.what() << std::endl;
            device.close();
            return false;
        }
    } else {
        std::cout << std::endl << std::endl << "There is no Nvidia GPU available which supports CUDA. Therefore the CPU mode is used as fallback!" << std::endl;
        trackerConfiguration.processing_mode = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_CPU;

        try {
            bodyTracker = k4abt::tracker::create(sensorCalibration, trackerConfiguration);
        } catch (k4a::error e) {
            std::cerr << e.what() << std::endl;
            device.close();
            return false;
        }
    }

    return true;
}

void AzureKinect::PrintConfidenceLevel(k4abt_joint_t joint) {
    switch (joint.confidence_level)
    {
        case K4ABT_JOINT_CONFIDENCE_NONE:
            std::cout << "NONE confidence" << std::endl;
            break;
        case K4ABT_JOINT_CONFIDENCE_LOW:
            std::cout << "LOW confidence" << std::endl;
            break;
        case K4ABT_JOINT_CONFIDENCE_MEDIUM:
            std::cout << "MEDIUM confidence" << std::endl;
            break;
        case K4ABT_JOINT_CONFIDENCE_HIGH:
            std::cout << "HIGH confidence" << std::endl;
            break;
        default:
            break;
    }
}

void AzureKinect::Destroy() {
    delete canUseGPU;
}