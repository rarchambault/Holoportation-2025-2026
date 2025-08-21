/***************************************************************************\

Module Name:  OrbbecCaptureManager.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module uses the Orbbec SDK to retrieve data from a connected Orbbec
camera.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#include "orbbecCaptureManager.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

OrbbecCaptureManager::OrbbecCaptureManager(int deviceIndex) : deviceIndex(deviceIndex), lastFrameTime(0)
{
    documentDetector = std::make_unique<DocumentDetector>();

    documentDetector->SetDetectionCallback([=](const DetectionResult& result) {
        // Save the new detection result
        lastDocumentHeight = result.height;
        lastDocumentWidth = result.width;
        lastDocumentData = result.data;
        lastDocumentScore = result.score;
        hasNewDocument = true;
    });
}

OrbbecCaptureManager::~OrbbecCaptureManager()
{
    // Release the device and stop the pipeline
    Close();
}

/// <summary>
/// Sets the logging function to be used to append messages to the logging file.
/// </summary>
/// <param name="loggerFunc">Function to be used for logging. Should be passed by liveScanClient.cpp.</param>
void OrbbecCaptureManager::SetLogger(std::function<void(const std::string&)> loggerFunc) {
    logFn = loggerFunc;

    // Propagate to document detector
    documentDetector->SetLogger(loggerFunc);
}

/// <summary>
/// Opens the device associated with this instance and sets the right configuration. 
/// </summary>
/// <param name="state">Sync State with which to initialize the device</param>
/// <param name="syncOffsetMultiplier">Multiplier used to determine the offset with which the
/// device should capture to be synchronized with the others (this is set by the server).</param>
/// <returns></returns>
bool OrbbecCaptureManager::Initialize(SyncState state, int syncOffsetMultiplier)
{
    bool res = TryOpenDevice();

    if (!res)
    {
        isInitialized = false;
        return isInitialized;
    }

    // Set sync configuration on the device
    OBMultiDeviceSyncConfig syncConfig = device->getMultiDeviceSyncConfig();

    if (state == Master) {
        syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
    }
    else if (state == Subordinate) {
        syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
        syncConfig.trigger2ImageDelayUs = SyncDelayUs * syncOffsetMultiplier;
    }
    else {
        syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
    }

    device->setMultiDeviceSyncConfig(syncConfig);

    // Create a pipeline with the current device
    pipeline = std::make_shared<ob::Pipeline>(device);

    // Create a configuration to set color and depth sensor parameters
    auto config = std::make_shared<ob::Config>();

    // Configure color stream
    auto colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr <ob::VideoStreamProfile> colorProfile;

    if (colorProfiles) {
        try {
            // Find the corresponding Profile according to the specified format
            colorProfile = colorProfiles->getVideoStreamProfile(2560, 1440, OB_FORMAT_RGB888, 30);
        }
        catch (ob::Error& e) {
            // If the specified format is not found, select the first one (default stream profile)
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
        }
    }

    config->enableStream(colorProfile);

    // Configure depth stream
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode alignMode = ALIGN_DISABLE;

    depthProfileList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);

    if (depthProfileList->count() > 0) {
        std::shared_ptr<ob::StreamProfile> depthProfile;
        try {
            // Select the profile with the same frame rate as color and the specified parameters
            if (colorProfile) {
                depthProfile = depthProfileList->getVideoStreamProfile(640, 576, OB_FORMAT_Y16, colorProfile->fps());
            }
        }
        catch (...) {
            depthProfile = nullptr;
        }

        if (!depthProfile) {
            // If no matching profile is found, select the default profile
            depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
        }

        config->enableStream(depthProfile);
    }

    // Enable D2C alignment to generate RGBD point clouds
    config->setAlignMode(alignMode);

    // Start the pipeline with the new configuration
    try {
        pipeline->start(config);
        isInitialized = true;
    }
    catch (const ob::Error& e) {
        if (logFn) logFn("[OrbbecCaptureManager] Failed to start pipeline: " + std::string(e.getMessage()));
        isInitialized = false;
    }

    if (autoExposureEnabled == false) {
        SetExposureState(false, exposureTimeStep);
    }

    // Wait a bit before starting capture
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Check that the device is able to capture a frame in under 5 seconds
    // If this device is a subordinate, it only starts capturing when the master has started, so skip this check
    if (state != Subordinate) {
        auto start = std::chrono::system_clock::now();
        bool bTemp;

        do {
            bTemp = AcquireFrame(false);
            auto elapsedSeconds = std::chrono::duration<double>(std::chrono::system_clock::now() - start);

            if (elapsedSeconds.count() > 5.0) {
                isInitialized = false;
                break;
            }
        } 
        while (!bTemp);
    }

    return isInitialized;
}

/// <summary>
/// Acquires a frame from the camera and stores relevant data in local variables.
/// </summary>
/// <param name="isCalibrationDataRequested">Indicates whether or not to save data to be used for calibration</param>
/// <returns>True if a frame was acquired successfully; false otherwise.</returns>
bool OrbbecCaptureManager::AcquireFrame(bool isCalibrationDataRequested)
{
    if (!isInitialized || !pipeline) {
        return false;
    }

    try {
        // Wait for a frameset (color + depth)
        std::shared_ptr<ob::FrameSet> frameset = pipeline->waitForFrames(CaptureTimeoutMs);

        if (!frameset || !frameset->colorFrame() || !frameset->depthFrame() || (frameset->colorFrame()->globalTimeStampUs() != frameset->depthFrame()->globalTimeStampUs())) {
            return false;
        }

        // Get color and depth frames
        auto colorFrame = frameset->colorFrame();
        auto depthFrame = frameset->depthFrame();

        int colorWidth = colorFrame->width();
        int colorHeight = colorFrame->height();

        int depthWidth = depthFrame->width();
        int depthHeight = depthFrame->height();

        // Resize color frame buffer if needed
        if (!colorData || colorFrameWidth != colorWidth || colorFrameHeight != colorHeight) {
            colorFrameWidth = colorWidth;
            colorFrameHeight = colorHeight;
            if (colorData) delete[] colorData;
            colorData = new RGB[colorWidth * colorHeight];
        }

        // Resize depth frame buffer if needed
        if (!depthData || depthFrameWidth != depthWidth || depthFrameHeight != depthHeight) {
            depthFrameWidth = depthWidth;
            depthFrameHeight = depthHeight;
            if (depthData) delete[] depthData;
            depthData = new UINT16[depthWidth * depthHeight];
        }

        // Copy color frame data into color frame buffer
        if (colorFrame->format() != OB_FORMAT_RGB888) {
            logFn("[OrbbecCaptureManager] Warning: Expected RGB888 format but got " + std::to_string(colorFrame->format()));
        }

        const uint8_t* colorSrc = static_cast<const uint8_t*>(colorFrame->data());
        for (int i = 0; i < colorWidth * colorHeight; ++i) {
            colorData[i].Red = colorSrc[i * 3 + 0];
            colorData[i].Green = colorSrc[i * 3 + 1];
            colorData[i].Blue = colorSrc[i * 3 + 2];
        }

        // Copy depth frame data into depth frame buffer
        if (depthFrame->format() != OB_FORMAT_Y16) {
            logFn("[OrbbecCaptureManager] Warning: Expected Y16 format but got " + std::to_string(depthFrame->format()));
        }

        const uint16_t* depthSrc = static_cast<const uint16_t*>(depthFrame->data());
        for (int i = 0; i < depthWidth * depthHeight; ++i) {
            depthData[i] = depthSrc[i];
        }

        // Generate point cloud from Orbbec SDK
        UpdatePointCloud();

        // Store timestamp
        currentTimeStamp = colorFrame->globalTimeStampUs();

        // Send the latest frame to the document detection
        // Get the current time and check the interval
        auto now = std::chrono::steady_clock::now();
        auto nowMs = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();

        if (nowMs - lastFrameTime.count() >= DocumentServerSendDelayMs)
        {
            documentDetector->SubmitFrame(colorFrame, alignedDepthFrame);
            lastFrameTime = std::chrono::milliseconds(nowMs);
        }

        return true;
    }
    catch (const ob::Error& e) {
        if (logFn) logFn("[OrbbecCaptureManager] Failed to acquire frame: " + std::string(e.getMessage()));
        return false;
    };
}

/// <summary>
/// Enables/Disables Auto Exposure and/or sets the exposure to a step value between 1 and 300
/// </summary>
/// <param name="exposureStep">The Exposure Step between 1 and 300</param>
void OrbbecCaptureManager::SetExposureState(bool enableAutoExposure, int exposureStep)
{
    if (!isInitialized || !device) {
        return;
    }

    try {
        if (enableAutoExposure) {
            device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);
            autoExposureEnabled = true;
        }
        else {
            device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
            device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exposureStep);
            autoExposureEnabled = false;
            exposureTimeStep = exposureStep;
        }
    }
    catch (const ob::Error& e) {
        if (logFn) logFn("[OrbbecCaptureManager] Failed to set exposure: " + std::string(e.getMessage()));
    }
}

uint64_t OrbbecCaptureManager::GetTimeStamp()
{
    return currentTimeStamp;
}

int OrbbecCaptureManager::GetDeviceIndex()
{
    return deviceIDForRestart;
}

bool OrbbecCaptureManager::TryOpenDevice()
{
    bool opened = false;

    // Find the requested device in the device list
    ob::Context ctx;
    ctx.setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);

    auto devList = ctx.queryDeviceList();
    int count = static_cast<int>(devList->deviceCount());

    if (count < (deviceIndex - 1)) {
        if (logFn) logFn("[OrbbecCaptureManager] Device not found!");
        return opened;
    }

    int deviceIdx = deviceIndex;

    // Save the deviceId of this Client; when the cameras are reinitialized during runtime, they will then use the same ID as before
    if (deviceIDForRestart != -1) {
        deviceIdx = deviceIDForRestart;
    }

    // Open the device at the specified index
    std::shared_ptr<ob::Device> newDevice;

    try {
        newDevice = devList->getDevice(deviceIdx);

        if (logFn) {
            logFn("[OrbbecCaptureManager] Device opened successfully at index: " + std::to_string(deviceIdx));
        }

        opened = true;
    }
    catch (const ob::Error& e) {
        if (logFn) logFn("[OrbbecCaptureManager] Failed to open device at index: " + std::to_string(deviceIdx) +
            " - Error: " + e.getMessage());
    }

    // Store device if opened
    if (opened) {
        device = newDevice;
        deviceIDForRestart = deviceIdx;

        // Get device info to store serial number
        auto devInfo = newDevice->getDeviceInfo();
        serialNumber = devInfo->serialNumber();
    }

    return opened;
}

/// <summary>
/// Generates a new point cloud from the latest acquired frameset
/// </summary>
void OrbbecCaptureManager::UpdatePointCloud() {
    // Get camera parameters
    auto cameraParams = pipeline->getCameraParam();
    const auto& depthIntrinsics = cameraParams.depthIntrinsic; // Depth camera intrinsics (used to go from depth pixel -> depth camera space)
    const auto& colorIntrinsics = cameraParams.rgbIntrinsic; // Color camera intrinsics (used to project from color camera space -> color image)
    const auto& extrinsics = cameraParams.transform; // This transforms a point in depth camera space into color camera space

    float fx = depthIntrinsics.fx; // focal length x (depth cam)
    float fy = depthIntrinsics.fy; // focal length y (depth cam)
    float cx = depthIntrinsics.cx; // principal point x (depth cam)
    float cy = depthIntrinsics.cy; // principal point y (depth cam)

    // Reset point cloud buffers
    lastFrameVertices.clear();
    lastFrameColors.clear();
    lastFrameColors.reserve(depthFrameWidth * depthFrameHeight);

    alignedDepthFrame = cv::Mat::zeros(depthFrameHeight, depthFrameWidth, CV_16U);

    // Align the color frame to the depth frame and compute point cloud
    for (int v = 0; v < depthFrameHeight; ++v) {
        for (int u = 0; u < depthFrameWidth; ++u) {
            int depthIdx = v * depthFrameWidth + u;
            uint16_t d = depthData[depthIdx]; // depth in millimeters

            if (d == 0)
            {
                // No depth data for the current point: store zero vertex and black color
                lastFrameVertices.emplace_back(0.0f, 0.0f, 0.0f);
                lastFrameColors.push_back({ 0, 0, 0 });
                continue;
            }

            // Convert from depth pixel to depth camera space (in meters)
            float z = d / 1000.0f; // convert mm to meters
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            // Transform point from depth to color camera space
            float X = extrinsics.rot[0] * x + extrinsics.rot[1] * y + extrinsics.rot[2] * z + extrinsics.trans[0] / 1000.0f;
            float Y = extrinsics.rot[3] * x + extrinsics.rot[4] * y + extrinsics.rot[5] * z + extrinsics.trans[1] / 1000.0f;
            float Z = extrinsics.rot[6] * x + extrinsics.rot[7] * y + extrinsics.rot[8] * z + extrinsics.trans[2] / 1000.0f;

            if (Z <= 0)
            {
                // Invalid projection: store zero vertex and black color
                lastFrameVertices.emplace_back(0.0f, 0.0f, 0.0f);
                lastFrameColors.push_back({ 0, 0, 0 });
                continue;
            }

            // Project from color camera space to color image pixel coordinates
            float projU = colorIntrinsics.fx * X / Z + colorIntrinsics.cx;
            float projV = colorIntrinsics.fy * Y / Z + colorIntrinsics.cy;

            // Fill alignedDepthFrame (original depth frame aligned to a scaled down version of the color frame)
            // Scale from color frame dimensions to depth frame dimensions
            int alignedU = static_cast<int>(round(projU * depthFrameWidth / colorFrameWidth));
            int alignedV = static_cast<int>(round(projV * depthFrameHeight / colorFrameHeight));

            if (alignedU >= 0 && alignedV >= 0 && alignedU < depthFrameWidth && alignedV < depthFrameHeight) {
                uint16_t& existingDepth = alignedDepthFrame.at<uint16_t>(alignedV, alignedU);
                if (existingDepth == 0 || d < existingDepth) {
                    existingDepth = d; // Keep nearest depth
                }
            }

            // Sample color image using bilinear interpolation
            uint8_t r = 0, g = 0, b = 0;
            int u0 = static_cast<int>(floor(projU));
            int v0 = static_cast<int>(floor(projV));
            float du = projU - u0;
            float dv = projV - v0;

            if (u0 >= 0 && v0 >= 0 && u0 + 1 < colorFrameWidth && v0 + 1 < colorFrameHeight) {
                RGB c00 = colorData[v0 * colorFrameWidth + u0];
                RGB c10 = colorData[v0 * colorFrameWidth + u0 + 1];
                RGB c01 = colorData[(v0 + 1) * colorFrameWidth + u0];
                RGB c11 = colorData[(v0 + 1) * colorFrameWidth + u0 + 1];

                // Interpolate red channel
                r = static_cast<uint8_t>(
                    (1 - du) * (1 - dv) * c00.Red +
                    du * (1 - dv) * c10.Red +
                    (1 - du) * dv * c01.Red +
                    du * dv * c11.Red);

                // Interpolate green channel
                g = static_cast<uint8_t>(
                    (1 - du) * (1 - dv) * c00.Green +
                    du * (1 - dv) * c10.Green +
                    (1 - du) * dv * c01.Green +
                    du * dv * c11.Green);

                // Interpolate blue channel
                b = static_cast<uint8_t>(
                    (1 - du) * (1 - dv) * c00.Blue +
                    du * (1 - dv) * c10.Blue +
                    (1 - du) * dv * c01.Blue +
                    du * dv * c11.Blue);
            }

            // Store results
            lastFrameVertices.emplace_back(X, Y, Z); // Position in color camera space
            lastFrameColors.push_back({ b, g, r });
        }
    }
}

bool OrbbecCaptureManager::Close()
{
    if (!isInitialized)
        return false;

    try {
        if (pipeline) {
            pipeline->stop(); // Stop streaming
            pipeline.reset(); // Release the pipeline
        }

        if (device) {
            device.reset(); // Release the device
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Allow SDK internals to clean up

        isInitialized = false;
        return true;
    }
    catch (const ob::Error& e) {
        if (logFn) logFn("[OrbbecCaptureManager] Error during Close(): " + std::string(e.getMessage()));
        return false;
    }
}
