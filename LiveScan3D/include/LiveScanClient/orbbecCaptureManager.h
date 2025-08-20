/***************************************************************************\

Module Name:  OrbbecCaptureManager.h
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

#pragma once

#include "libobsensor/ObSensor.hpp"
#include "ICaptureManager.h"
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

class OrbbecCaptureManager : public ICaptureManager
{
public:
    OrbbecCaptureManager(int deviceIndex = 0);
    ~OrbbecCaptureManager();
    
    bool Initialize(SyncState state, int syncOffset);
    bool AcquireFrame(bool isCalibrationDataRequested);
    uint64_t GetTimeStamp();
    int GetDeviceIndex();
    void SetExposureState(bool enableAutoExposure, int exposureStep);
    void SetLogger(std::function<void(const std::string&)> loggerFunc);

private:
    const int SyncDelayUs = 160;
    const int DocumentServerSendDelayMs = 1000;
    const int CaptureTimeoutMs = 1000;

    int deviceIndex = 0;
    int deviceIDForRestart = -1;
    int restartAttempts = 0;
    int counter = 0;

    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Pipeline> pipeline;

    cv::Mat alignedDepthFrame;

    uint64_t currentTimeStamp = 0;
    std::chrono::milliseconds lastFrameTime;

    bool autoExposureEnabled = true;
    int exposureTimeStep = 0;

    std::function<void(const std::string&)> logFn;

    bool TryOpenDevice();
    void UpdatePointCloud();
    bool Close();
};

