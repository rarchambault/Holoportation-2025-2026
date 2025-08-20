/***************************************************************************\

Module Name:  FrameIOHandler.cpp
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

#include "frameIOHandler.h"
#include <ctime>

FrameIOHandler::~FrameIOHandler()
{
	CloseFile();
}

void FrameIOHandler::CloseFile()
{
	if (fileHandle == nullptr)
		return;

	fclose(fileHandle);
	fileHandle = nullptr; 
	isFileOpenForReading = false;
	isFileOpenForWriting = false;
}

void FrameIOHandler::ResetRecordingTimer()
{
	recordingStartTime = std::chrono::steady_clock::now();
}

int FrameIOHandler::GetElapsedRecordingTimeMs()
{
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds >(end - recordingStartTime).count());
}

void FrameIOHandler::OpenFileForReading()
{
	CloseFile();

	fileHandle = fopen(filename.c_str(), "rb");

	isFileOpenForReading = true;
	isFileOpenForWriting = false;
}

void FrameIOHandler::OpenNewFileForWriting(int deviceID)
{
	CloseFile();

	char filename[1024];
	time_t t = time(0);
	struct tm * now = localtime(&t);
	sprintf(filename, "recording_%01d_%04d_%02d_%02d_%02d_%02d_%02d.bin", deviceID, now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	this->filename = filename; 
	fileHandle = fopen(filename, "wb");

	isFileOpenForReading = false;
	isFileOpenForWriting = true;

	ResetRecordingTimer();
}

bool FrameIOHandler::ReadFrame(std::vector<Point3s> &outPoints, std::vector<RGB> &outColors)
{
	if (!isFileOpenForReading)
		OpenFileForReading();

	outPoints.clear();
	outColors.clear();

	FILE *fp = fileHandle;

	int numPoints = 0;
	int timestamp = 0;
	char headerLabelBuffer[1024]; 

	// Read and discard header labels, parse number of points and timestamp
	int nRead = fscanf_s(fp, "%s %d %s %d", headerLabelBuffer, (int)sizeof(headerLabelBuffer), &numPoints, headerLabelBuffer, (int)sizeof(headerLabelBuffer), &timestamp);

	// If fewer than 4 items are read, this is a malformed or incomplete frame
	if (nRead < 4)
		return false;

	// If no points, this is a valid but empty frame
	if (numPoints == 0)
		return true;

	// Skip newline character after header
	fgetc(fp);

	outPoints.resize(numPoints);
	outColors.resize(numPoints);

	fread((void*)outPoints.data(), sizeof(Point3s), numPoints, fp);
	fread((void*)outColors.data(), sizeof(RGB), numPoints, fp);

	// Skip trailing newline after binary block
	fgetc(fp);

	return true;
}


void FrameIOHandler::WriteFrame(const std::vector<Point3s> points, const std::vector<RGB> colors, uint64_t timestamp, int deviceID)
{
	if (!isFileOpenForWriting)
		OpenNewFileForWriting(deviceID);

	FILE *fp = fileHandle;

	int numPoints = static_cast<int>(points.size());
	
	// Write text header
	fprintf(fp, "n_points= %d\nframe_timestamp= %d\n", numPoints, (int)timestamp);

	if (numPoints > 0)
	{
		// Write binary point data
		fwrite((void*)points.data(), sizeof(points[0]), numPoints, fp);
		fwrite((void*)colors.data(), sizeof(colors[0]), numPoints, fp);
	}

	// Write a newline to mark the end of this frame
	fprintf(fp, "\n");
}