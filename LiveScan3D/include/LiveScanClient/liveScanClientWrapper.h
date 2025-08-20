/***************************************************************************\

Module Name:  LiveScanClientWrapper.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is a wrapper around each instance of LiveScanClient to handle
the thread it runs on and the callback functions it can call on the server
if they have been previously registered.

\***************************************************************************/

#pragma once

#include "utils.h"
#include <memory>
#include <functional>
#include <thread>
#include <string>

// Forward declaration
class LiveScanClient;

// Typedefs for the callback signatures
typedef void(*SendSerialNumberCallback)(int clientIndex, const char* serialNumber);
typedef void(*ConfirmRecordedCallback)(int clientIndex);
typedef void(*ConfirmCalibratedCallback)(int clientIndex, int markerId, const float* R, const float* t);
typedef void(*SendLatestFrameCallback)(int clientIndex, const Point3s* vertices, const RGB* colors, int count);
typedef void(*SendRecordedFrameCallback)(int clientIndex, const Point3s* vertices, const RGB* colors, int count, bool noMoreFrames);
typedef void(*ConfirmSyncStateCallback)(int clientIndex, int tempSyncState);
typedef void(*ConfirmMasterRestartCallback)(int clientIndex);
typedef void(*SendDocumentCallback)(int clientIndex, const unsigned char* data, float score, float width, float height);

struct LiveScanClientWrapper {
	std::unique_ptr<LiveScanClient> client;
	std::thread thread;

	SendSerialNumberCallback sendSerialNumberCallback = nullptr;
	ConfirmRecordedCallback confirmRecordedCallback = nullptr;
	ConfirmCalibratedCallback confirmCalibratedCallback = nullptr;
	SendLatestFrameCallback sendLatestFrameCallback = nullptr;
	SendRecordedFrameCallback sendStoredFrameCallback = nullptr;
	ConfirmSyncStateCallback confirmSyncStateCallback = nullptr;
	ConfirmMasterRestartCallback confirmMasterRestartCallback = nullptr;
	SendDocumentCallback sendDocumentCallback = nullptr;
};