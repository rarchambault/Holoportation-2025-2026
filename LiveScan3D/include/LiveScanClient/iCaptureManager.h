/***************************************************************************\

Module Name:  iCaptureManager.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module defines some functions and fields which should be implemented by
any capture manager modules used for retrieving data from a device for the
point cloud reconstruction.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once

#include "utils.h"
#include <functional>
#include <documentDetector.h>

class ICaptureManager
{
public:
	bool isInitialized;

	int colorFrameHeight, colorFrameWidth;
	int depthFrameHeight, depthFrameWidth;

	UINT16* depthData;
	RGB* colorData;

	std::vector<Point3f> lastFrameVertices;
	std::vector<RGB> lastFrameColors;

	cv::Mat lastDocumentData;
	float lastDocumentScore;
	float lastDocumentWidth;
	float lastDocumentHeight;

	std::string serialNumber;

	std::unique_ptr<DocumentDetector> documentDetector;

	ICaptureManager();
	~ICaptureManager();

	virtual bool Initialize(SyncState state, int syncOffset) = 0;
	virtual bool AcquireFrame(bool isCalibrationDataRequested) = 0;
	virtual bool Close() = 0;
	virtual uint64_t GetTimeStamp() = 0;
	virtual int GetDeviceIndex() = 0;
	virtual void SetExposureState(bool enableAutoExposure, int exposureStep) = 0;
	virtual void SetLogger(std::function<void(const std::string&)> loggerFunc) = 0;
};