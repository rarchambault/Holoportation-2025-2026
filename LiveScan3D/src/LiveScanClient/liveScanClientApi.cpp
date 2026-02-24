/***************************************************************************\

Module Name:  LiveScanClientApi.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module defines all API calls made available to the C# server through
the LiveScanClient DLL to allow communication from the C# server to the
C++ clients and vice-versa.

\***************************************************************************/

#include "LiveScanClientApi.h"
#include <thread>
#include <memory>
#include <map>
#include <locale>
#include <codecvt> 

/*
* Server to client (inbound) calls
*/
LiveScanClientHandle CreateClient(int index)
{
	auto* wrapper = new LiveScanClientWrapper();

	wrapper->client = std::make_unique<LiveScanClient>(index);
	wrapper->client->wrapper = wrapper;
	return wrapper;
}

void StartClient(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->thread = std::thread([wrapper]() {
		wrapper->client->Run();
		});
}

void StopClient(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->RequestExit();

	if (wrapper->thread.joinable())
		wrapper->thread.join();
}

void DestroyClient(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	delete wrapper;
}

void StartFrameRecording(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->StartFrameRecording();
}

void Calibrate(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->Calibrate();
}

void SetSettings(LiveScanClientHandle handle, const CameraSettings* settings)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper || !settings) return;

	wrapper->client->SetSettings(*settings);
}

void RequestRecordedFrame(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->RequestRecordedFrame();
}

void RequestLatestFrame(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->RequestLatestFrame();
}

void ReceiveCalibration(LiveScanClientHandle handle, const AffineTransform* transform)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper || !wrapper->client || !transform)
		return;

	wrapper->client->ReceiveCalibration(*transform);
}

void ClearRecordedFrames(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->ClearRecordedFrames();
}

void EnableSync(LiveScanClientHandle handle, int syncState, int syncOffset)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->EnableSync(syncState, syncOffset);
}

void DisableSync(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->DisableSync();
}

void StartMaster(LiveScanClientHandle handle)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (!wrapper) return;

	wrapper->client->StartMaster();
}

/*
* Client to server (outbound) calls
*/
void SetSendSerialNumberCallback(LiveScanClientHandle handle, SendSerialNumberCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->sendSerialNumberCallback = cb;
}

void SetConfirmRecordedCallback(LiveScanClientHandle handle, ConfirmRecordedCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->confirmRecordedCallback = cb;
}

void SetConfirmCalibratedCallback(LiveScanClientHandle handle, ConfirmCalibratedCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->confirmCalibratedCallback = cb;
}

void SetSendLatestFrameCallback(LiveScanClientHandle handle, SendLatestFrameCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->sendLatestFrameCallback = cb;
}

void SetSendRecordedFrameCallback(LiveScanClientHandle handle, SendRecordedFrameCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->sendStoredFrameCallback = cb;
}

void SetConfirmSyncStateCallback(LiveScanClientHandle handle, ConfirmSyncStateCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->confirmSyncStateCallback = cb;
}

void SetConfirmMasterRestartCallback(LiveScanClientHandle handle, ConfirmMasterRestartCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->confirmMasterRestartCallback = cb;
}

void SetSendDocumentCallback(LiveScanClientHandle handle, SendDocumentCallback cb)
{
	auto* wrapper = static_cast<LiveScanClientWrapper*>(handle);
	if (wrapper)
		wrapper->sendDocumentCallback = cb;
}