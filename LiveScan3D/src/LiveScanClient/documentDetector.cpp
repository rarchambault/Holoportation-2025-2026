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

#include <algorithm>
#include <numeric>
#ifdef _WIN32
#include <Windows.h>
#endif


DocumentDetector::DocumentDetector()
{
    // Attempt to load the model early so we can fail fast in logs.
    LoadModelIfNeeded();
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


void DocumentDetector::SetModelPath(const std::string& onnxPath) {
    modelPath = onnxPath;
    modelLoaded = false;
    ortSession.reset();
    inputNames.clear();
    outputNames.clear();
    inputNameStrs.clear();
    outputNameStrs.clear();
}

static std::basic_string<ORTCHAR_T> ToOrtString(const std::string& s) {
#ifdef _WIN32
    if (s.empty()) return std::basic_string<ORTCHAR_T>();
    int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), NULL, 0);
    std::wstring w(sizeNeeded, 0);
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), &w[0], sizeNeeded);
    return w;
#else
    return s;
#endif
}

bool DocumentDetector::LoadModelIfNeeded() {
    if (modelLoaded && ortSession) return true;

    try {
        if (!ortEnv) {
            ortEnv = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "DocumentDetector");
        }

        ortSessionOptions = Ort::SessionOptions();
        ortSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        // CPU-only inference: limit threads to avoid starving the rest of the pipeline.
        ortSessionOptions.SetIntraOpNumThreads(1);
        ortSessionOptions.SetInterOpNumThreads(1);

        auto ortPath = ToOrtString(modelPath);
        ortSession = std::make_unique<Ort::Session>(*ortEnv, ortPath.c_str(), ortSessionOptions);

        Ort::AllocatorWithDefaultOptions allocator;

        // Cache input/output names (must keep strings alive!)
        size_t numInputs = ortSession->GetInputCount();
        inputNameStrs.clear(); 
        inputNames.clear();
        inputNameStrs.reserve(numInputs);
        inputNames.reserve(numInputs);
        for (size_t i = 0; i < numInputs; ++i) {
            auto nameAllocated = ortSession->GetInputNameAllocated(i, allocator); 
            inputNameStrs.emplace_back(nameAllocated.get()); // copy into std::string
        }
        for (auto& s : inputNameStrs) inputNames.push_back(s.c_str());

        size_t numOutputs = ortSession->GetOutputCount();
        outputNameStrs.clear(); 
        outputNames.clear();
        outputNameStrs.reserve(numOutputs);
        outputNames.reserve(numOutputs);
        for (size_t i = 0; i < numOutputs; ++i) {
            auto nameAllocated = ortSession->GetOutputNameAllocated(i, allocator); 
            outputNameStrs.emplace_back(nameAllocated.get()); // copy into std::string
        }
        for (auto& s : outputNameStrs) outputNames.push_back(s.c_str());

        modelLoaded = true;
        if (logFn) logFn(std::string("[DocumentDetector] Loaded ONNX model: ") + modelPath);
        return true;
    }
    catch (const Ort::Exception& e) {
        modelLoaded = false;
        if (logFn) logFn(std::string("[DocumentDetector] Failed to load ONNX model: ") + e.what());
        return false;
    }
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
            float score = 0.0f;
            short width = 0, height = 0;
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


// Helper functions for NMS 
static float IoU(const cv::Rect& a, const cv::Rect& b) { 
    int x1 = (std::max)(a.x, b.x); 
    int y1 = (std::max)(a.y, b.y); 
    int x2 = (std::min)(a.x + a.width, b.x + b.width); 
    int y2 = (std::min)(a.y + a.height, b.y + b.height); 
    int interArea = (std::max)(0, x2 - x1) * (std::max)(0, y2 - y1); 
    int unionArea = a.area() + b.area() - interArea; 
    
    return unionArea > 0 ? static_cast<float>(interArea) / unionArea : 0.0f; 
} 

// Helper function to perform Non-Maximum Suppression (NMS) on detected bounding boxes 
static void NMS(const std::vector<cv::Rect>& boxes, const std::vector<float>& scores, float iouThreshold, std::vector<int>& keep) { 
    keep.clear(); 
    std::vector<int> idxs(boxes.size()); 
    std::iota(idxs.begin(), idxs.end(), 0); 
    std::sort(idxs.begin(), idxs.end(), [&](int a, int b) { return scores[a] > scores[b]; });
    
    while (!idxs.empty()) { 
        int best = idxs.front(); 
        keep.push_back(best); 
        idxs.erase(idxs.begin()); 
        
        idxs.erase(std::remove_if(idxs.begin(), idxs.end(), [&](int i) { return IoU(boxes[best], boxes[i]) > iouThreshold; }), idxs.end() ); 
    } 
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
    cv::Mat /*depthMat*/,
    cv::Mat& documentData,
    short& documentPictureWidth,
    short& documentPictureHeight,
    float& bestScore
)
{
    bestScore = 0.0f;

    if (!colorFrame || colorFrame->data() == nullptr) {
        return false;
    }
    if (!LoadModelIfNeeded()) {
        // If model is not available, we can't detect.
        return false;
    }

    // Convert Orbbec color frame to OpenCV Mat (RGB)
    cv::Mat originalImage(colorFrame->height(), colorFrame->width(), CV_8UC3, (void*)colorFrame->data());
    //cv::cvtColor(originalImage, originalImage, cv::COLOR_BGR2RGB);

    // --- Preprocess: letterbox to model input size (default 640) ---
    constexpr int kInputSize = 640;
    const float confThreshold = 0.25f;
    const float nmsThreshold = 0.45f;

    struct LetterboxInfo {
        float scale = 1.0f;
        int padW = 0;
        int padH = 0;
        int newW = 0;
        int newH = 0;
    } lb;

    auto letterbox = [&](const cv::Mat& src, cv::Mat& dst, LetterboxInfo& info) {
        const int w = src.cols;
        const int h = src.rows;

        info.scale = (std::min)(static_cast<float>(kInputSize) / static_cast<float>(w),
            static_cast<float>(kInputSize) / static_cast<float>(h));
        info.newW = static_cast<int>(std::round(w * info.scale));
        info.newH = static_cast<int>(std::round(h * info.scale));

        cv::Mat resized;
        cv::resize(src, resized, cv::Size(info.newW, info.newH), 0, 0, cv::INTER_LINEAR);

        info.padW = (kInputSize - info.newW) / 2;
        info.padH = (kInputSize - info.newH) / 2;

        dst = cv::Mat(cv::Size(kInputSize, kInputSize), CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(dst(cv::Rect(info.padW, info.padH, info.newW, info.newH)));
        };

    cv::Mat inputImage;
    letterbox(originalImage, inputImage, lb);

    // Convert to float tensor NCHW, normalized to [0,1]
    std::vector<float> inputTensor(1 * 3 * kInputSize * kInputSize);
    for (int y = 0; y < kInputSize; ++y) {
        const cv::Vec3b* row = inputImage.ptr<cv::Vec3b>(y);
        for (int x = 0; x < kInputSize; ++x) {
            // inputImage is RGB
            const float r = row[x][0] / 255.0f;
            const float g = row[x][1] / 255.0f;
            const float b = row[x][2] / 255.0f;

            const int idx = y * kInputSize + x;
            inputTensor[0 * kInputSize * kInputSize + idx] = r;
            inputTensor[1 * kInputSize * kInputSize + idx] = g;
            inputTensor[2 * kInputSize * kInputSize + idx] = b;
        }
    }

    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 4> inputShape = { 1, 3, kInputSize, kInputSize };

    Ort::Value inputOrt = Ort::Value::CreateTensor<float>(
        memInfo,
        inputTensor.data(),
        inputTensor.size(),
        inputShape.data(),
        inputShape.size()
    );

    // --- Inference ---
    std::vector<Ort::Value> outputs;
    try {
        outputs = ortSession->Run(
            Ort::RunOptions{ nullptr },
            inputNames.data(),
            &inputOrt,
            1,
            outputNames.data(),
            outputNames.size()
        );
    }
    catch (const Ort::Exception& e) {
        if (logFn) logFn(std::string("[DocumentDetector] ONNX inference failed: ") + e.what());
        return false;
    }

    if (outputs.empty()) return false;

    // YOLOv8-seg ONNX typically returns 2 outputs:
    //  - output0: detections, shape [1, C, N] or [1, N, C]
    //  - output1: prototypes (we ignore for bbox-only crop)
    const Ort::Value& det = outputs[0];
    auto detInfo = det.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> detShape = detInfo.GetShape();
    if (detShape.size() != 3) {
        if (logFn) logFn("[DocumentDetector] Unexpected detection output shape.");
        return false;
    }

    int64_t dim1 = detShape[1];
    int64_t dim2 = detShape[2];

    // Determine layout
    int64_t C = 0, N = 0;
    bool layoutCHW = true; // [1, C, N]
    if (dim1 < dim2) {
        C = dim1; N = dim2; layoutCHW = true;
    }
    else {
        C = dim2; N = dim1; layoutCHW = false; // [1, N, C]
    }

    const float* detData = det.GetTensorData<float>();
    if (!detData) return false;

    // If a prototype output exists, use it to infer mask dimension,
    // otherwise fall back to YOLOv8 default (32).
    int64_t maskDim = 32;
    if (outputs.size() >= 2) {
        auto protoInfo = outputs[1].GetTensorTypeAndShapeInfo();
        auto protoShape = protoInfo.GetShape(); // [1, maskDim, mh, mw]
        if (protoShape.size() >= 2 && protoShape[1] > 0) maskDim = protoShape[1];
    }

    int64_t clsCount = C - 4 - maskDim;
    if (clsCount <= 0) {
        // Some exports may omit mask coefficients in output0.
        clsCount = C - 4;
        maskDim = 0;
    }

    auto at = [&](int64_t i, int64_t c) -> float {
        if (layoutCHW) {
            // [1, C, N] contiguous as C-major
            return detData[c * N + i];
        }
        else {
            // [1, N, C]
            return detData[i * C + c];
        }
        };

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    boxes.reserve(static_cast<size_t>(N));
    scores.reserve(static_cast<size_t>(N));

    for (int64_t i = 0; i < N; ++i) {
        float cx = at(i, 0);
        float cy = at(i, 1);
        float w = at(i, 2);
        float h = at(i, 3);

        // class scores start at index 4
        float bestCls = 0.0f;
        for (int64_t c = 0; c < clsCount; ++c) {
            float s = at(i, 4 + c);
            if (s > bestCls) bestCls = s;
        }

        float conf = bestCls;
        if (conf < confThreshold) continue;

        float x1 = cx - w * 0.5f;
        float y1 = cy - h * 0.5f;
        float x2 = cx + w * 0.5f;
        float y2 = cy + h * 0.5f;

        // Map from letterboxed 640x640 back to original image coordinates
        x1 = (x1 - static_cast<float>(lb.padW)) / lb.scale;
        y1 = (y1 - static_cast<float>(lb.padH)) / lb.scale;
        x2 = (x2 - static_cast<float>(lb.padW)) / lb.scale;
        y2 = (y2 - static_cast<float>(lb.padH)) / lb.scale;

        int left = (std::max)(0, static_cast<int>(std::floor(x1)));
        int top = (std::max)(0, static_cast<int>(std::floor(y1)));
        int right = (std::min)(originalImage.cols - 1, static_cast<int>(std::ceil(x2)));
        int bottom = (std::min)(originalImage.rows - 1, static_cast<int>(std::ceil(y2)));

        int width = right - left;
        int height = bottom - top;
        if (width <= 2 || height <= 2) continue;

        boxes.emplace_back(left, top, width, height);
        scores.emplace_back(conf);
    }

    if (boxes.empty()) return false;

    // NMS
    std::vector<int> keep; 
    NMS(boxes, scores, nmsThreshold, keep); 
    
    if (keep.empty()) return false;

    // Choose best detection with a strong preference for bigger boxes (documents)
    const double imgArea = static_cast<double>(originalImage.cols) * static_cast<double>(originalImage.rows);
    bool found = false;

    for (int idx : keep) {
        const cv::Rect& box = boxes[idx];
        const float conf = scores[idx];

        // Require some confidence so we don't pick random large regions
        if (conf < 0.25f) continue;

        const double areaRatio = static_cast<double>(box.area()) / imgArea;

        // Strongly prefer larger detections so we don't end up with tiny crops (e.g., 785px wide)
        const double score = 0.4 * conf + 0.6 * areaRatio;

        if (score > bestScore) {
            // Add padding around the bbox to capture more of the page (10%)
            int padX = static_cast<int>(box.width * 0.10f);
            int padY = static_cast<int>(box.height * 0.10f);

            cv::Rect padded(
                box.x - padX,
                box.y - padY,
                box.width + 2 * padX,
                box.height + 2 * padY
            );

            // Clamp to image bounds
            cv::Rect safeBox = padded & cv::Rect(0, 0, originalImage.cols, originalImage.rows);
            if (safeBox.width <= 0 || safeBox.height <= 0) continue;

            documentData = originalImage(safeBox).clone();

            static int dbg = 0;
            if (dbg++ % 5 == 0) { // save every ~30 detections
                cv::Mat bgr;
                cv::cvtColor(documentData, bgr, cv::COLOR_RGB2BGR);
                cv::imwrite("doc_debug.png", bgr);   // lossless
            }

            documentPictureWidth = static_cast<short>(documentData.cols);
            documentPictureHeight = static_cast<short>(documentData.rows);
            bestScore = static_cast<float>(score);
            found = true;

            if (logFn) {
                logFn("[DocumentDetector] Selected crop: " +
                    std::to_string(documentPictureWidth) + "x" +
                    std::to_string(documentPictureHeight) +
                    " conf=" + std::to_string(conf) +
                    " areaRatio=" + std::to_string(areaRatio));
            }
        }
    }

    return found;

}
