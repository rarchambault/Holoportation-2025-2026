/***************************************************************************\

Module Name:  LiveScanClientApi.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module defines all API calls made available to the C# server through
the LiveScanClient DLL to allow communication from the C# server to the
C++ clients and vice-versa.

\***************************************************************************/

#pragma once

#ifdef LIVESCANCLIENT_EXPORTS
#define LIVESCAN_API __declspec(dllexport)
#else
#define LIVESCAN_API __declspec(dllimport)
#endif

#include "LiveScanClient.h"
#include "transferObjectUtils.h"

extern "C" {

	typedef void* LiveScanClientHandle;

	// Server to client (inbound) calls
	LIVESCAN_API LiveScanClientHandle CreateClient(int index);
	LIVESCAN_API void StartClient(LiveScanClientHandle handle);
	LIVESCAN_API void StopClient(LiveScanClientHandle handle);
	LIVESCAN_API void DestroyClient(LiveScanClientHandle handle);

	LIVESCAN_API void StartFrameRecording(LiveScanClientHandle handle);
	LIVESCAN_API void Calibrate(LiveScanClientHandle handle);
    LIVESCAN_API void SetSettings(LiveScanClientHandle handle, const CameraSettings* settings);
	LIVESCAN_API void RequestRecordedFrame(LiveScanClientHandle handle);
	LIVESCAN_API void RequestLatestFrame(LiveScanClientHandle handle);
	LIVESCAN_API void ReceiveCalibration(LiveScanClientHandle handle, const AffineTransform* transform);
	LIVESCAN_API void ClearRecordedFrames(LiveScanClientHandle handle);
	LIVESCAN_API void EnableSync(LiveScanClientHandle handle, int syncState, int syncOffset);
	LIVESCAN_API void DisableSync(LiveScanClientHandle handle);
	LIVESCAN_API void StartMaster(LiveScanClientHandle handle);

	// Client to server (outbound) calls
	LIVESCAN_API void SetSendSerialNumberCallback(LiveScanClientHandle handle, SendSerialNumberCallback cb);
	LIVESCAN_API void SetConfirmRecordedCallback(LiveScanClientHandle handle, ConfirmRecordedCallback cb);
	LIVESCAN_API void SetConfirmCalibratedCallback(LiveScanClientHandle handle, ConfirmCalibratedCallback cb);
	LIVESCAN_API void SetSendLatestFrameCallback(LiveScanClientHandle handle, SendLatestFrameCallback cb);
	LIVESCAN_API void SetSendRecordedFrameCallback(LiveScanClientHandle handle, SendRecordedFrameCallback cb);
	LIVESCAN_API void SetConfirmSyncStateCallback(LiveScanClientHandle handle, ConfirmSyncStateCallback cb);
	LIVESCAN_API void SetConfirmMasterRestartCallback(LiveScanClientHandle handle, ConfirmMasterRestartCallback cb);
	LIVESCAN_API void SetSendDocumentCallback(LiveScanClientHandle handle, SendDocumentCallback cb);
}