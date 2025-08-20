/***************************************************************************\

Module Name:  Calibration.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module computes transformations to bring points from the local space of 
the camera from they were captured to the globalworld space of the Holoport. 

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once
#include "stdafx.h"
#include "markerDetector.h"
#include "utils.h"
#include <functional>

vector<float> RotatePoint(vector<float> &point, std::vector<std::vector<float>> &R);
vector<float> InverseRotatePoint(vector<float> &point, std::vector<std::vector<float>> &R);

struct MarkerPose
{
	int MarkerId;
	float R[3][3];
	float T[3];
};

class Calibration
{
public:
	vector<float> worldT;
	vector<vector<float>> worldR;
	int usedMarkerId;

	vector<MarkerPose> markerPoses;

	bool isCalibrated;

	Calibration();
	~Calibration();

	bool Calibrate(RGB *colorFrame, Point3f *alignedDepthFrame, int colorFrameWidth, int colorFrameHeight);
	bool LoadCalibration(const string &serialNumber);
	void SaveCalibration(const string &serialNumber);
	void SetLogger(std::function<void(const std::string&)> loggerFunc);

private:
	const int NumRequiredSamples = 20; // Number of samples required to average marker position

	int numSamples;
	IMarkerDetector *markerDetector;
	vector<vector<Point3f>> markerSamplePositions;

	void Procrustes(MarkerInfo &marker, vector<Point3f> &markerInWorld, vector<float> &markerT, vector<vector<float>> &markerR);
	bool Get3DMarkerCorners(vector<Point3f> &marker3D, MarkerInfo &marker, Point3f *alignedDepthFrame, int colorFrameWidth, int colorFrameHeight);
	std::function<void(const std::string&)> logFn;
};

