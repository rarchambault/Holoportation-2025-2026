/***************************************************************************\

Module Name:  DocumentDetector.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module uses a YOLO machine learning model to detect documents from a 
provided color frame and ranks its detections based on their size and blur.

\***************************************************************************/

#include "documentDetector.h"

DocumentDetector::DocumentDetector(int deviceIdx)
{
    deviceIndex = deviceIdx;

    StartDetectionThread();
}

DocumentDetector::~DocumentDetector()
{
    // Stop the detection thread
    StopDetectionThread();
}

void DocumentDetector::SetDetectionCallback(DetectionCallback callback) {
    resultCallback = std::move(callback);
}

/// <summary>
/// Sets the logging function to be used to append messages to the logging file.
/// </summary>
/// <param name="loggerFunc">Function to be used for logging. Should be passed by orbbecCaptureManager.cpp.</param>
void DocumentDetector::SetLogger(std::function<void(const std::string&)> loggerFunc) {
    logFn = loggerFunc;
}

/// <summary>
/// Submits a new frame for document detection.
/// </summary>
/// <param name="color">Color frame on which to perform the document detection</param>
/// <param name="depth">Depth frame on which to perform the document detection, aligned with the color frame</param>
void DocumentDetector::SubmitFrame(std::shared_ptr<ob::ColorFrame> color, cv::Mat depth)
{
    // Store the newly submitted frame in local variables for thread processing
    std::lock_guard<std::mutex> lock(frameMutex);
    pendingColorFrame = color;
    pendingDepthFrame = depth;
    newFrameAvailable = true;
    frameCond.notify_one();
}

/// <summary>
/// Starts the detection thread and detects document from provided frames
/// </summary>
void DocumentDetector::StartDetectionThread()
{
    stopThread = false;

    detectThread = std::thread([this]() {

        while (!stopThread)
        {
            std::shared_ptr<ob::ColorFrame> localColor;
            cv::Mat localDepth;

            // Wait for new frame
            {
                std::unique_lock<std::mutex> lock(frameMutex);
                frameCond.wait(lock, [this]() { return newFrameAvailable || stopThread; });

                if (stopThread) break;

                // Store latest frame in local variables
                localColor = pendingColorFrame;
                localDepth = pendingDepthFrame;

                newFrameAvailable = false;
            }

            // Try to detect a document from the frame
            cv::Mat data;
            float score = 0.0f, width = 0.0f, height = 0.0f;
            bool found = Detect(localColor, localDepth, data, width, height, score);

            // Call the detection callback if a document has been detected
            if (found && resultCallback) {
                DetectionResult result;
                result.data = std::move(data);
                result.width = width;
                result.height = height;
                result.score = score;

                resultCallback(result);
            }
        }
    });
}

void DocumentDetector::StopDetectionThread()
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        stopThread = true;
        frameCond.notify_all();  // Wake the thread if waiting
    }

    if (detectThread.joinable())
        detectThread.join();
}



/// <summary>
/// Uses computer vision techniques to detect any documents in the provided frame
/// </summary>
/// <param name="colorFrame">Color frame from the camera from which to detect documents</param>
/// <param name="depthMat">Depth frame, converted to an OpenCV Mat, from the camera from which to detect documents</param>
/// <param name="documentData">Output pixels composing the detected document</param>
/// <param name="documentPictureWidth">Output width of the detected document, in pixels</param>
/// <param name="documentPictureHeight">Output height of the detected document, in pixels</param>
/// <param name="documentScore">Score of the detected document to compare it with other detections</param>
/// <returns>True if a document was detected, false otherwise</returns>
bool DocumentDetector::Detect(
    const std::shared_ptr<ob::ColorFrame>& colorFrame,
    cv::Mat depthMat,
    cv::Mat& documentData,
    float& documentPictureWidth,
    float& documentPictureHeight,
    float& bestScore
)
{
    // Convert Orbbec color frame to OpenCV Mat
    cv::Mat originalImage(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::cvtColor(originalImage, originalImage, cv::COLOR_BGR2RGB);

    cv::Mat resizedImage;
    cv::resize(originalImage, resizedImage, depthMat.size());

    if (numBackgroundSamples < numRequiredBackgroundSamples)
    {
        backgroundDepthSamples.push_back(depthMat.clone());
        numBackgroundSamples++;

        if (numBackgroundSamples >= numRequiredBackgroundSamples)
        {
            // Accumulators
            cv::Mat depthSum = cv::Mat::zeros(resizedImage.size(), CV_32F);
            cv::Mat depthCount = cv::Mat::zeros(resizedImage.size(), CV_32S);

            for (size_t i = 0; i < numRequiredBackgroundSamples; ++i)
            {
                cv::Mat depthSample = backgroundDepthSamples[i];

                cv::Mat depthSampleFloat;
                depthSample.convertTo(depthSampleFloat, CV_32F);

                // Binary mask: 255 where valid, 0 elsewhere
                cv::Mat validMaskBinary = (depthSample >= 0);

                // For sum: convert to float in [0.0, 1.0]
                cv::Mat validMaskFloat;
                validMaskBinary.convertTo(validMaskFloat, CV_32F, 1.0 / 255.0);

                // For count: convert to int (0 or 1)
                cv::Mat validMaskInt;
                validMaskBinary.convertTo(validMaskInt, CV_32S, 1.0f / 255.0f);

                // Add valid depths to sum
                depthSum += depthSampleFloat.mul(validMaskFloat);

                // Add 1 to count where valid
                depthCount += validMaskInt;
            }

            // Compute average: where count > 0
            averageBackgroundDepth = cv::Mat::zeros(resizedImage.size(), CV_16U);

            cv::Mat avgDepthFloat;
            cv::divide(depthSum, depthCount, avgDepthFloat, 1, CV_32F);

            // Convert to 16U safely
            avgDepthFloat.convertTo(averageBackgroundDepth, CV_16U);
        }
        else
        {
            return false;
        }
    }

    // Create a mask where depth has changed significantly (i.e., foreground)
    cv::Mat mask = cv::Mat::zeros(resizedImage.size(), CV_8U);
    cv::Mat depthForegroundMask = cv::Mat::zeros(resizedImage.size(), CV_8U);

    for (int y = 0; y < resizedImage.rows; ++y) {
        for (int x = 0; x < resizedImage.cols; ++x) {
            // Depth difference
            uint16_t bgDepth = averageBackgroundDepth.at<uint16_t>(y, x);
            uint16_t currDepth = depthMat.at<uint16_t>(y, x);
            int depthDiff = static_cast<int>(bgDepth) - static_cast<int>(currDepth);
            bool isDepthForeground = depthDiff > 15 || (bgDepth == 0 && depthDiff < -15);

            if (isDepthForeground) {
                depthForegroundMask.at<uint8_t>(y, x) = 255;
            }
        }
    }

    // Depth mask pre-processing
    // Erode then dilate to clean up small noise
    cv::Mat morphKernelDepth = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::morphologyEx(depthForegroundMask, depthForegroundMask, cv::MORPH_OPEN, morphKernelDepth); // Cleans small noise
    cv::morphologyEx(depthForegroundMask, depthForegroundMask, cv::MORPH_CLOSE, morphKernelDepth); // Fills small holes in objects

    // Apply mask: set color to black for masked pixels
    resizedImage.setTo(cv::Scalar(0, 0, 0), depthForegroundMask == 0);

    // Preprocess: Convert to grayscale and blur slightly
    cv::Mat gray;
    cv::cvtColor(resizedImage, gray, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // Edge detection
    cv::Mat edges;
    cv::Canny(gray, edges, 100, 200);
    cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1), 1);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat debugImage = resizedImage.clone();
    cv::drawContours(debugImage, contours, -1, cv::Scalar(0, 255, 0), 2);

    // Look through all predictions and save the best one
    bestScore = 0.0f;
    bool found = false;

    int imageWidth = resizedImage.cols;
    int imageHeight = resizedImage.rows;
    double imageArea = imageWidth * imageHeight;

    int quadCount = 0;

    for (size_t idx = 0; idx < contours.size(); ++idx)
    {
        const auto& contour = contours[idx];

        // Approximate contour to polygon
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.018, true);

        // Check for quadrilaterals
        if (approx.size() == 4 && cv::isContourConvex(approx)) {
            ++quadCount;

            cv::Rect boundingBox = cv::boundingRect(approx);
            float areaRatio = static_cast<float>(boundingBox.area()) / imageArea;
            if (boundingBox.area() < imageArea * 0.01)
            {
                continue;
            }
            
            float aspectRatio = static_cast<float>(boundingBox.width) / boundingBox.height;
            
            if (aspectRatio < 0.5f || aspectRatio > 2.0f)
            {
                continue;
            }

            // Project bounding box back to the original image's resolution
            float scaleX = static_cast<float>(originalImage.size().width) / resizedImage.size().width;
            float scaleY = static_cast<float>(originalImage.size().height) / resizedImage.size().height;
            cv::Rect origBox = cv::Rect(
                cv::Point(cvRound(boundingBox.x * scaleX), cvRound(boundingBox.y * scaleY)), 
                cv::Size(cvRound(boundingBox.width * scaleX),
                cvRound(boundingBox.height * scaleY))
            );
            
            // Crop image
            cv::Mat cropped = originalImage(origBox).clone();

            // Sharpness score
            cv::Mat croppedGray;
            cv::cvtColor(cropped, croppedGray, cv::COLOR_RGB2GRAY);
            cv::Mat lap;
            cv::Laplacian(croppedGray, lap, CV_64F);
            cv::Scalar mean, stddev;
            cv::meanStdDev(lap, mean, stddev);
            double sharpnessScore = stddev[0] * stddev[0];
            double newScore = (0.9 * sharpnessScore / 1000.0) + (0.1 * areaRatio);

            if (newScore > bestScore) {
                documentData = cropped.clone();
                documentPictureWidth = cropped.cols;
                documentPictureHeight = cropped.rows;
                bestScore = newScore;
                found = true;
            }
        }
    }

    return found;
}
