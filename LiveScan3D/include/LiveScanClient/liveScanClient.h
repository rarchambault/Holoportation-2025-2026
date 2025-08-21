/***************************************************************************\

Module Name:  LiveScanClient.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module handles all logic related to retrieving data from one camera and
setting its parameters. It also sends data back to the C# LiveScanServer.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once

#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCKAPI_

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "liveScanClientWrapper.h"
#include "resource.h"
#include "calibration.h"
#include "orbbecCaptureManager.h"
#include "frameIOHandler.h"
#include "transferObjectUtils.h"
#include <thread>
#include <mutex>
#include <functional>
#include <voxelGridFilter.h>

class LiveScanClient
{
public:
    LiveScanClientWrapper* wrapper = nullptr;

    LiveScanClient(int index);
    ~LiveScanClient();

    void Run();
    void StartFrameRecording();
    void Calibrate();
    void SetSettings(const CameraSettings& settings);
    void RequestRecordedFrame();
    void RequestLatestFrame();
    void ReceiveCalibration(const AffineTransform& transform);
    void ClearRecordedFrames();
    void EnableSync(int syncState, int syncOffset);
    void DisableSync();
    void StartMaster();
    void RequestExit();

    std::function<void(const std::string&)> GetLogger();

private:
    const float Range = 0.3f;
    const float HalfRange = Range / 2.0f;
    const float MinPrecision = Range / 255; // Min precision (max resolution) with the set range and the number of values in a byte (255)
    const int GridResolution = Range / MinPrecision;

    const float XRangeCenter = 0.0f;
    const float YRangeCenter = 0.0f;
    const float ZRangeCenter = HalfRange;

    const float DocumentDiffThreshold = 0.50;
    const int DocumentSendTimeout = 30000; // In milliseconds

    int clientIndex = -1;
    bool isClientThreadRunning;

    bool isCalibrateRequested;
    bool isRecordFrameRequested;
    bool isConfirmRecordedRequested;
    bool isConfirmSyncStateRequested;
    bool isConfirmRestartAsMasterRequested;
    bool isConfirmCalibratedRequested;
    bool isSendDocumentRequested;
    
    bool isFilterEnabled;
    int numFilterNeighbors;
    float filterThreshold;

    bool isAutoExposureEnabled;
    int numExposureSteps;

    bool isRestartingCamera;

    volatile bool isExitRequested = false;

    SyncState currentSyncState;

    ICaptureManager* captureManager;
    Calibration calibration;
    VoxelGridFilter voxelGridFilter;
    FrameIOHandler framesFileWriterReader;

    std::vector<float> bounds;

    std::vector<Point3s> lastFrameVertices;
    std::vector<RGB> lastFrameColors;

    cv::Mat lastDocumentData;
    float lastDocumentScore;
    short lastDocumentWidth;
    short lastDocumentHeight;
    std::chrono::milliseconds lastDocumentSendTime;

    Point3f* cameraSpaceCoordinates;

    std::ofstream logFile;

    void UpdateFrame();
    void ProcessFrame();
    void ProcessDocument();
    float ComputeImageDifference(cv::Mat& newDocumentData);
    void SendSerialNumber();
    void ConfirmRecorded();
    void ConfirmCalibrated();
    void SendLatestFrame();
    void SendRecordedFrame(vector<Point3s>& vertices, vector<RGB>& RGB, bool noMoreFrames);
    void ConfirmSyncState();
    void ConfirmMasterRestart();
    void SendDocument();
    void SendClientConfirmations();
    void SetupLogging(int clientIndex);
    void Log(const std::string& message);
};
