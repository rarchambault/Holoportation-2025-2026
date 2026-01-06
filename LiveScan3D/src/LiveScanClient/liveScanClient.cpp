/***************************************************************************\

Module Name:  LiveScanClient.cpp
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

#include "stdafx.h"
#include "resource.h"
#include "liveScanClient.h"
#include "filter.h"
#include <chrono>
#include <strsafe.h>
#include <fstream>
#include <shellapi.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

LiveScanClient::LiveScanClient(int index) :
	clientIndex(index),
	cameraSpaceCoordinates(NULL),
	isCalibrateRequested(false),
	isFilterEnabled(false),
	isRecordFrameRequested(false),
	isConfirmRecordedRequested(false),
	isConfirmCalibratedRequested(false),
	isConfirmRestartAsMasterRequested(false),
	isClientThreadRunning(true),
	numFilterNeighbors(10),
	filterThreshold(0.01f),
	isRestartingCamera(false),
	isAutoExposureEnabled(true),
	numExposureSteps(-5),
	voxelGridFilter(MinPrecision, XRangeCenter, YRangeCenter, ZRangeCenter, HalfRange)
{
	SetupLogging(clientIndex);

	captureManager = new OrbbecCaptureManager(clientIndex);
	captureManager->SetLogger(GetLogger());
	calibration.SetLogger(GetLogger());

	bounds.push_back(-0.5);
	bounds.push_back(-0.5);
	bounds.push_back(-0.5);
	bounds.push_back(0.5);
	bounds.push_back(0.5);
	bounds.push_back(0.5);
}

LiveScanClient::~LiveScanClient()
{
	if (captureManager)
	{
		delete captureManager;
		captureManager = NULL;
	}

	if (cameraSpaceCoordinates)
	{
		delete[] cameraSpaceCoordinates;
		cameraSpaceCoordinates = NULL;
	}
}

/// <summary>
/// Initializes the camera and starts the main loop to retrieve its data
/// </summary>
void LiveScanClient::Run()
{
	// First initialize the camera as standalone (sync disabled)
	bool res = captureManager->Initialize(Standalone, 0);

	if (res)
	{
		SendSerialNumber();

		// Try to load calibration data from a previous run
		calibration.LoadCalibration(captureManager->serialNumber);

		if (calibration.isCalibrated)
			isConfirmCalibratedRequested = true;

		cameraSpaceCoordinates = new Point3f[captureManager->colorFrameWidth * captureManager->colorFrameHeight];
		captureManager->SetExposureState(true, 0);
	}
	else
	{
		Log("[LiveScanClient] Failed to initialize capture device.");
	}

	// Start a thread to handle some client callbacks to the server in parallel to the main data loop
	std::thread t1(&LiveScanClient::SendClientConfirmations, this);

	// Start the main loop to retrieve data from the camera
	while (!isExitRequested)
	{
		UpdateFrame();
	}

	isClientThreadRunning = false;
	t1.join();
}

void LiveScanClient::StartFrameRecording()
{
	isRecordFrameRequested = true;
}

void LiveScanClient::Calibrate()
{
	isCalibrateRequested = true;
}

void LiveScanClient::SetSettings(const CameraSettings& settings)
{
	bounds = { settings.MinBounds[0], settings.MinBounds[1], settings.MinBounds[2],
				  settings.MaxBounds[0], settings.MaxBounds[1], settings.MaxBounds[2] };

	isFilterEnabled = settings.Filter;
	numFilterNeighbors = settings.FilterNeighbors;
	filterThreshold = settings.FilterThreshold;

	// Copy marker poses to calibration data
	calibration.markerPoses.resize(settings.NumMarkers);

	for (int i = 0; i < settings.NumMarkers; i++) {
		calibration.markerPoses[i].MarkerId = settings.MarkerPoses[i].MarkerId;
		memcpy(calibration.markerPoses[i].R, settings.MarkerPoses[i].R, sizeof(float) * 9);
		memcpy(calibration.markerPoses[i].T, settings.MarkerPoses[i].T, sizeof(float) * 3);
	}

	isAutoExposureEnabled = settings.AutoExposureEnabled;
	numExposureSteps = settings.ExposureStep;

	captureManager->SetExposureState(isAutoExposureEnabled, numExposureSteps);
}

void LiveScanClient::RequestRecordedFrame()
{
	// Read the first recorded frame saved during recording
	vector<Point3s> points;
	vector<RGB> colors;
	bool res = framesFileWriterReader.ReadFrame(points, colors);

	SendRecordedFrame(points, colors, !res);
}

void LiveScanClient::RequestLatestFrame()
{
	SendLatestFrame();
}

void LiveScanClient::ReceiveCalibration(const AffineTransform& transform)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			calibration.worldR[i][j] = transform.R[i][j];

		calibration.worldT[i] = transform.T[i];
	}
}

void LiveScanClient::ClearRecordedFrames()
{
	framesFileWriterReader.CloseFile();
}

void LiveScanClient::EnableSync(int syncState, int syncOffset)
{
	bool res = false;

	switch (syncState)
	{
	case 0:
		currentSyncState = Subordinate;
		isRestartingCamera = true;

		// Close camera
		res = captureManager->Close();
		if (!res) {
			Log("[LiveScanClient] Subordinate device failed to close! Restart Application!");
			return;
		}

		// Re-initialize as Subordinate with a unique syncOffset (sent by the server)
		res = captureManager->Initialize(Subordinate, syncOffset);
		if (!res) {
			Log("[LiveScanClient] Subordinate device failed to reinitialize! Restart Application!");
			return;
		}

		// Confirm reinitialization as Subordinate to the server
		isConfirmSyncStateRequested = true;
		isRestartingCamera = false;
		break;

	case 1:
		currentSyncState = Master;
		isRestartingCamera = true;

		// Close camera; need to wait until all Subordinates have reinitialized before restarting the Master
		res = captureManager->Close();
		if (!res) {
			Log("[LiveScanClient] Master device failed to close! Restart Application!");
			return;
		}

		// Confirm reinitialization as Master to the server
		isConfirmSyncStateRequested = true;
		break;

	case 2:
		currentSyncState = Standalone;
		isRestartingCamera = true;

		// Close camera
		res = captureManager->Close();
		if (!res) {
			Log("[LiveScanClient] Capture device failed to close! Restart Application!");
			return;
		}

		// Re-initialize as Standalone
		res = captureManager->Initialize(Standalone, 0);

		if (!res) {
			Log("[LiveScanClient] Capture device failed to reinitialize! Restart Application!");
			return;
		}

		// Confirm reinitialization as Standalone to the server
		isConfirmSyncStateRequested = true;
		isRestartingCamera = false;
		break;

	default:
		break;
	}
}

void LiveScanClient::DisableSync()
{
	// Set this device as Standalone
	currentSyncState = Standalone;
	isRestartingCamera = true;

	bool res;

	// Close the camera
	res = captureManager->Close();
	if (!res) {
		Log("[LiveScanClient] Capture device failed to close! Restart Application!");
		return;
	}

	// Re-initialize as Standalone
	res = captureManager->Initialize(Standalone, 0);

	if (!res) {
		Log("[LiveScanClient] Capture device failed to reinitialize! Restart Application!");
		return;
	}

	// Confirm reinitialization as Standalone to the server
	isConfirmSyncStateRequested = true;
	isRestartingCamera = false;
}

void LiveScanClient::StartMaster()
{
	// This is called by the server once all Subordinates have been re-initialized, meaning the Master can now start
	if (currentSyncState == Master)
	{
		bool res = captureManager->Initialize(Master, 0);
		if (!res) {
			Log("[LiveScanClient] Master device failed to reinitialize! Restart Application!");
			return;
		}

		isConfirmRestartAsMasterRequested = true;
		isRestartingCamera = false;
	}
}

void LiveScanClient::RequestExit()
{
	isExitRequested = true;
}

void LiveScanClient::SendClientConfirmations()
{
	while (isClientThreadRunning)
	{
		if (isConfirmRecordedRequested)
		{
			ConfirmRecorded();
		}

		if (isConfirmCalibratedRequested)
		{
			ConfirmCalibrated();
		}

		if (isConfirmSyncStateRequested)
		{
			ConfirmSyncState();
		}

		if (isConfirmRestartAsMasterRequested)
		{
			ConfirmMasterRestart();
		}

		if (isSendDocumentRequested)
		{
			SendDocument();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

/// <summary>
/// Retrieves point cloud data from the camera and stores it into local variables for sending to the server
/// </summary>
void LiveScanClient::UpdateFrame()
{
	// Check that the capture manager is initialized
	if (!captureManager->isInitialized)
	{
		return;
	}

	// Acquire a new point cloud frame from the camera
	bool newFrameAcquired = captureManager->AcquireFrame(isCalibrateRequested);

	if (!newFrameAcquired)
	{
		return;
	}

	// Apply some processing to the data that was just retrieved and store it in local variables
	ProcessFrame();

	// Process the document data from the frame
	if (captureManager->hasNewDocument) 
	{
		ProcessDocument();
		captureManager->hasNewDocument = false;
	}
	

	if (isRecordFrameRequested)
	{
		// If we are recording frames, save the frame that was just processed
		uint64_t timeStamp = captureManager->GetTimeStamp();
		framesFileWriterReader.WriteFrame(lastFrameVertices, lastFrameColors, timeStamp, captureManager->GetDeviceIndex());

		isConfirmRecordedRequested = true;
		isRecordFrameRequested = false;
	}

	if (isCalibrateRequested)
	{
		// Calibrate the camera by using the marker(s) and their positions as specified in the settings
		int totalPixels = captureManager->depthFrameWidth * captureManager->depthFrameHeight;
		Point3f* floatPoints = new Point3f[totalPixels];
		RGB* colors = new RGB[totalPixels];

		for (int i = 0; i < totalPixels; i++) {
			floatPoints[i].X = captureManager->lastFrameVertices[i].X;
			floatPoints[i].Y = captureManager->lastFrameVertices[i].Y;
			floatPoints[i].Z = captureManager->lastFrameVertices[i].Z;

			colors[i].Red = captureManager->lastFrameColors[i].Red;
			colors[i].Green = captureManager->lastFrameColors[i].Green;
			colors[i].Blue = captureManager->lastFrameColors[i].Blue;
		} 

		bool res = calibration.Calibrate(colors, floatPoints, captureManager->depthFrameWidth, captureManager->depthFrameHeight);

		delete[] floatPoints;
		delete[] colors;

		if (res)
		{
			// Save the new calibration to a file to reuse in a later run
			calibration.SaveCalibration(captureManager->serialNumber);
			isConfirmCalibratedRequested = true;
			isCalibrateRequested = false;
		}
	}
}

/// <summary>
/// Applies some processing steps to the last retrieved point cloud such as filtering and removing points outside the bounds
/// </summary>
void LiveScanClient::ProcessFrame()
{
	unsigned int numVertices = captureManager->lastFrameVertices.size();

	// To save some processing cost, we allocate a full frame size (numVertices) of a Point3f Vector beforehand
	// instead of using push_back for each vertex. Even though we have to copy the vertices into a clean array
	// later and it uses a little bit more RAM, this gives us a nice speed increase for this function, around 25-50%.

	// Create a placeholder for a point to remove
	Point3f invalidPoint = Point3f(0, 0, 0, true);

	vector<Point3f> allVertices(numVertices);
	int goodVerticesCount = 0;

	voxelGridFilter.Reset();

	// Apply calibration and remove points outside bounds
	for (unsigned int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++)
	{
		Point3f temp = captureManager->lastFrameVertices[vertexIndex];

		if (calibration.isCalibrated)
		{
			// Rotate the point to match the calibration
			temp.X += calibration.worldT[0];
			temp.Y += calibration.worldT[1];
			temp.Z += calibration.worldT[2];
			temp = RotatePoint(temp, calibration.worldR);

			// Remove the point if it is outside the bounds specified in the settings
			if (temp.X < bounds[0] || temp.X > bounds[3]
				|| temp.Y < bounds[1] || temp.Y > bounds[4]
				|| temp.Z < bounds[2] || temp.Z > bounds[5])
			{
				allVertices[vertexIndex] = invalidPoint;
				continue;
			}
			// Only keep the point if there is not already data for the same reduced point when considering the range
			else if (!voxelGridFilter.Insert(temp.X, temp.Y, temp.Z))
			{
				allVertices[vertexIndex] = invalidPoint;
				continue;
			}
		}

		allVertices[vertexIndex] = temp;
		goodVerticesCount++;
	}

	// Apply simple voxel density-based filter
	const float voxelSize = 0.006f;
	const int minPointsPerVoxel = 12;

	// Count points per voxel
	std::map<uint64_t, int> voxelCounts;
	auto HashVoxel = [](int x, int y, int z) -> uint64_t {
		return (static_cast<uint64_t>(x) & 0x1FFFFF) << 42 |
			(static_cast<uint64_t>(y) & 0x1FFFFF) << 21 |
			(static_cast<uint64_t>(z) & 0x1FFFFF);
		};

	vector<uint64_t> vertexVoxelKeys(numVertices);
	for (unsigned int i = 0; i < allVertices.size(); ++i)
	{
		Point3f& pt = allVertices[i];
		if (!pt.Invalid)
		{
			int vx = static_cast<int>(floor(pt.X / voxelSize));
			int vy = static_cast<int>(floor(pt.Y / voxelSize));
			int vz = static_cast<int>(floor(pt.Z / voxelSize));
			uint64_t key = HashVoxel(vx, vy, vz);
			vertexVoxelKeys[i] = key;
			voxelCounts[key]++;
		}
		else
		{
			vertexVoxelKeys[i] = 0; // placeholder
		}
	}

	int filteredCount = 0;

	// Mark isolated points as invalid
	for (unsigned int i = 0; i < allVertices.size(); ++i)
	{
		if (!allVertices[i].Invalid)
		{
			if (voxelCounts[vertexVoxelKeys[i]] < minPointsPerVoxel)
			{
				allVertices[i] = invalidPoint;
				filteredCount++;
			}
		}
	}

	// Copy all valid vertices into a clean vector 
	vector<Point3f> goodVertices(goodVerticesCount);
	vector<RGB> goodColorPoints(goodVerticesCount);
	int goodVerticesShortCounter = 0;

	for (unsigned int i = 0; i < allVertices.size(); i++)
	{
		if (!allVertices[i].Invalid)
		{
			goodVertices[goodVerticesShortCounter] = allVertices[i];
			goodColorPoints[goodVerticesShortCounter] = captureManager->lastFrameColors[i];
			goodVerticesShortCounter++;
		}
	}

	// If the more complex filtering step is enabled, apply it now
	if (isFilterEnabled)
	{
		Filter(goodVertices, goodColorPoints, numFilterNeighbors, filterThreshold);
	}

	// Convert the remaining vertices to shorts to save memory
	vector<Point3s> goodVerticesShort(goodVertices.size());

	for (size_t i = 0; i < goodVertices.size(); i++)
	{
		goodVerticesShort[i] = goodVertices[i];
	}

	lastFrameVertices = goodVerticesShort;
	lastFrameColors = goodColorPoints;
}

void LiveScanClient::ProcessDocument()
{
	if (captureManager->lastDocumentData.empty())
	{
		return;
	}

	cv::Mat newDocumentData = captureManager->lastDocumentData;
	float newDocumentScore = captureManager->lastDocumentScore;
	short newDocumentWidth = captureManager->lastDocumentWidth;
	short newDocumentHeight = captureManager->lastDocumentHeight;

	// Check if LiveScanClient has no data yet
	if (lastDocumentData.empty()) {
		lastDocumentData = newDocumentData;
		lastDocumentScore = newDocumentScore;
		lastDocumentWidth = newDocumentWidth;
		lastDocumentHeight = newDocumentHeight;
		isSendDocumentRequested = true;
		return;
	}

	float diff = ComputeImageDifference(newDocumentData);

	auto now = std::chrono::steady_clock::now();
	auto nowMs = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();

	if (nowMs - lastDocumentSendTime.count() >= DocumentSendTimeout || diff > DocumentDiffThreshold || newDocumentScore > lastDocumentScore)
	{
		lastDocumentData = newDocumentData;
		lastDocumentScore = newDocumentScore;
		lastDocumentWidth = newDocumentWidth;
		lastDocumentHeight = newDocumentHeight;
		isSendDocumentRequested = true;
		lastDocumentSendTime = std::chrono::milliseconds(nowMs);
	}
}

float LiveScanClient::ComputeImageDifference(cv::Mat& newDocumentData)
{
	if (newDocumentData.empty())
		return 1.0f; // Completely different if new is empty

	if (lastDocumentData.empty())
	{
		lastDocumentData = newDocumentData;
		return 1.0f; // No previous frame, assume max difference
	}

	// Resize to same dimensions
	cv::Mat resizedLast;
	cv::resize(lastDocumentData, resizedLast, newDocumentData.size());

	// Compute absolute difference
	cv::Mat diff;
	cv::absdiff(newDocumentData, resizedLast, diff);

	// Convert to grayscale to simplify metric
	cv::Mat grayDiff;
	cv::cvtColor(diff, grayDiff, cv::COLOR_BGR2GRAY);

	// Compute mean difference
	double meanDiff = cv::mean(grayDiff)[0]; // average intensity difference (0–255)

	// Normalize to 0.0–1.0
	float normalizedDiff = static_cast<float>(meanDiff / 255.0);

	// Update stored frame
	lastDocumentData = newDocumentData;

	return normalizedDiff;
}

void LiveScanClient::SendSerialNumber()
{
	if (wrapper && wrapper->sendSerialNumberCallback)
		wrapper->sendSerialNumberCallback(clientIndex, captureManager->serialNumber.c_str());
}

void LiveScanClient::ConfirmRecorded()
{
	if (wrapper && wrapper->confirmRecordedCallback)
		wrapper->confirmRecordedCallback(clientIndex);

	isConfirmRecordedRequested = false;
}

void LiveScanClient::ConfirmCalibrated()
{
	if (wrapper && wrapper->confirmCalibratedCallback)
	{
		float* R = new float[9] {
			calibration.worldR[0][0], calibration.worldR[0][1], calibration.worldR[0][2],
			calibration.worldR[1][0], calibration.worldR[1][1], calibration.worldR[1][2],
			calibration.worldR[2][0], calibration.worldR[2][1], calibration.worldR[2][2]
		};

		float* t = calibration.worldT.data();

		wrapper->confirmCalibratedCallback(clientIndex, calibration.usedMarkerId, R, t);
	}

	isConfirmCalibratedRequested = false;
}

void LiveScanClient::SendLatestFrame()
{
	if (wrapper && wrapper->sendLatestFrameCallback)
	{
		int count = static_cast<int>(lastFrameVertices.size());
		if (count != lastFrameColors.size())
		{
			Log("[LiveScanClient] Warning: size mismatch! There were " + std::to_string(count) + " vertices and " + std::to_string(lastFrameColors.size()) + " colors. Sending smallest size.");

			if (count < lastFrameColors.size())
				count = lastFrameColors.size();
		}

		wrapper->sendLatestFrameCallback(clientIndex, lastFrameVertices.data(), lastFrameColors.data(), count);
	}
}

void LiveScanClient::SendRecordedFrame(std::vector<Point3s>& vertices, std::vector<RGB>& RGB, bool noMoreFrames)
{
	if (wrapper && wrapper->sendStoredFrameCallback)
	{
		int count = static_cast<int>(vertices.size());
		if (count != RGB.size())
		{
			Log("[LiveScanClient] Warning: size mismatch! There were " + std::to_string(count) + " vertices and " + std::to_string(RGB.size()) + " colors. Sending smallest size.");

			if (count < RGB.size())
				count = RGB.size();
		}

		wrapper->sendStoredFrameCallback(clientIndex, vertices.data(), RGB.data(), count, noMoreFrames);
	}
}

void LiveScanClient::ConfirmSyncState()
{
	if (wrapper && wrapper->confirmSyncStateCallback)
	{
		int syncStateToSend = 2; // default: Standalone
		switch (currentSyncState)
		{
		case Subordinate: 
			syncStateToSend = 0;
			break;
		case Master:      
			syncStateToSend = 1;
			break;
		case Standalone:  
			syncStateToSend = 2;
			break;
		}

		wrapper->confirmSyncStateCallback(clientIndex, syncStateToSend);
	}

	isConfirmSyncStateRequested = false;
}

void LiveScanClient::ConfirmMasterRestart()
{
	if (wrapper && wrapper->confirmMasterRestartCallback)
	{
		wrapper->confirmMasterRestartCallback(clientIndex);
	}

	isConfirmRestartAsMasterRequested = false;
}

void LiveScanClient::SendDocument()
{
	if (wrapper && wrapper->sendDocumentCallback)
	{
		wrapper->sendDocumentCallback(clientIndex, lastDocumentData.data, lastDocumentScore, lastDocumentWidth, lastDocumentHeight);
	}

	isSendDocumentRequested = false;
}

/// <summary>
/// Creates a log file for this particular instance of the LiveScanClient
/// </summary>
/// <param name="clientIndex">Index of the current client instance</param>
void LiveScanClient::SetupLogging(int clientIndex)
{
	wchar_t buffer[MAX_PATH];
	GetModuleFileNameW(NULL, buffer, MAX_PATH);
	std::wstring path(buffer);
	std::wstring dir = path.substr(0, path.find_last_of(L"\\/")) + L"\\Log";

	CreateDirectoryW(dir.c_str(), NULL);

	std::wstring logPath = dir + L"\\LiveScanClient_" + std::to_wstring(clientIndex) + L"_Log.txt";
	logFile.open(logPath, std::ios::out | std::ios::app);

	if (!logFile.is_open())
	{
		OutputDebugStringW(L"Failed to open log file.\n");
		return;
	}

	Log("==== Application Started (Client " + std::to_string(clientIndex) + ") ====");
}

/// <summary>
/// Returns a reference to the Log function. Can be used to pass this function to other modules of the 
/// LiveScanClient project to enable logging.
/// </summary>
/// <returns>A reference to the Log function</returns>
std::function<void(const std::string&)> LiveScanClient::GetLogger() {
	return [this](const std::string& msg) { this->Log(msg); };
}

/// <summary>
/// Appends the provided message to the end of the previously opened log file.
/// </summary>
/// <param name="message">Message to append to the log file</param>
void LiveScanClient::Log(const std::string& message)
{
	// Get current time
	auto now = std::chrono::system_clock::now();
	auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
	auto value = now_ms.time_since_epoch();
	long duration = value.count();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);

	std::tm tm;
	localtime_s(&tm, &now_c); // Windows-specific thread-safe function

	// Extract milliseconds
	int milliseconds = static_cast<int>(duration % 1000);

	// Format time to string with milliseconds
	std::ostringstream timestamp;
	timestamp << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S")
		<< "." << std::setfill('0') << std::setw(3) << milliseconds
		<< "] " << message;

	std::string logEntry = timestamp.str();

	if (logFile.is_open())
	{
		logFile << logEntry << std::endl;
		logFile.flush();
	}
	else
	{
		OutputDebugStringA((logEntry + "\n").c_str()); // fallback to debugger output
	}
}

