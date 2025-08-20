/***************************************************************************\

Module Name:  FrameIOHandler.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module reads and writes point cloud frames to files for recording and
playback purposes.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once

#include <stdio.h>
#include <string>
#include <vector>
#include <chrono>
#include "utils.h"

class FrameIOHandler
{
public:
	~FrameIOHandler();

	void WriteFrame(std::vector<Point3s> points, std::vector<RGB> colors, uint64_t timestamp, int deviceID);
	bool ReadFrame(std::vector<Point3s> &outPoints, std::vector<RGB> &outColors);
	void CloseFile();

private:
	FILE* fileHandle = nullptr;
	std::string filename = "";
	bool isFileOpenForWriting = false;
	bool isFileOpenForReading = false;

	std::chrono::steady_clock::time_point recordingStartTime;

	void OpenNewFileForWriting(int deviceID);
	void OpenFileForReading();

	void ResetRecordingTimer();
	int GetElapsedRecordingTimeMs();
};