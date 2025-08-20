/***************************************************************************\

Module Name:  DocumentDetector.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module uses a YOLO machine learning model to detect documents from a
provided color frame and ranks its detections based on their size and blur.

\***************************************************************************/

#pragma once
#include <opencv2/opencv.hpp>
#include "libobsensor/ObSensor.hpp"
#include <utils.h>
#include <vector>
#include <string>
#include <mutex>

class DocumentDetector
{
public:
    // Type alias for the detection callback
    using DetectionCallback = std::function<void(const DetectionResult&)>;

    DocumentDetector(int deviceIdx);
    ~DocumentDetector();

    void DocumentDetector::SubmitFrame(std::shared_ptr<ob::ColorFrame> color, cv::Mat depth);

    bool DocumentDetector::Detect(
        const std::shared_ptr<ob::ColorFrame>& colorFrame,
        cv::Mat depthFrame,
        cv::Mat& documentData,
        float& documentPictureWidth,
        float& documentPicutreHeight,
        float& documentScore
    );

    void SetDetectionCallback(DetectionCallback callback);
    void SetLogger(std::function<void(const std::string&)> loggerFunc);

private:
    int counter = 0;
    int deviceIndex = -1;

    std::mutex frameMutex;
    std::condition_variable frameCond;

    std::shared_ptr<ob::ColorFrame> pendingColorFrame = nullptr;
    cv::Mat pendingDepthFrame;

    int numBackgroundSamples = 0;
    int numRequiredBackgroundSamples = 5;
    std::vector<cv::Mat> backgroundDepthSamples;
    cv::Mat averageBackgroundDepth;

    bool newFrameAvailable = false;
    bool stopThread = false;
    std::thread detectThread;

    DetectionCallback resultCallback;

    void StartDetectionThread();
    void StopDetectionThread();
    std::function<void(const std::string&)> logFn;
};