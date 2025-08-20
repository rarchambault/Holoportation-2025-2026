/***************************************************************************\

Module Name:  Calibration.cpp
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

#include "calibration.h"
#include "opencv\cv.h"

#include <fstream>
#include <functional>

Calibration::Calibration() : usedMarkerId(-1)
{
	// Initialize variables
	isCalibrated = false;
	numSamples = 0;

	worldT = vector<float>(3, 0.0f);

	for (int i = 0; i < 3; i++)
	{
		worldR.push_back(vector<float>(3, 0.0f));
		worldR[i][i] = 1.0f;
	}

	markerDetector = new MarkerDetector();
}

Calibration::~Calibration()
{
	if (markerDetector != NULL)
	{
		delete markerDetector;
		markerDetector = NULL;
	}
}

/// <summary>
/// Finds the transformations required to project local points into global space by finding a marker from a color frame.
/// </summary>
/// <param name="colorFrame">A color frame (RGB data) from the camera</param>
/// <param name="depthFrame">A depth frame from the same camera, aligned with the color frame and providing depth
/// data for all of it (both frames should have the same dimensions)</param>
/// <param name="frameWidth">Width of the color and depth frames</param>
/// <param name="frameHeight">Height of the color  and depth frames</param>
/// <returns></returns>
bool Calibration::Calibrate(RGB *colorFrame, Point3f *depthFrame, int frameWidth, int frameHeight)
{
	if (colorFrame == NULL || depthFrame == NULL) {
		return false;
	}

	MarkerInfo marker;

	// Try to find a marker in the color frame provided
	bool res = markerDetector->DetectMarkersInImage(colorFrame, frameHeight, frameWidth, marker);

	if (!res) {
		return false;
	}

	// Find which of the markers was found in provided list (from settings)
	int indexInPoses = -1;

	for (unsigned int j = 0; j < markerPoses.size(); j++)
	{
		if (marker.Id == markerPoses[j].MarkerId)
		{
			indexInPoses = j;
			break;
		}
	}

	if (indexInPoses == -1)
	{
		// No matching marker was found in the provided list
		return false;
	}

	MarkerPose markerPose = markerPoses[indexInPoses];
	usedMarkerId = markerPose.MarkerId;

	// Find the marker's corners
	vector<Point3f> marker3D(marker.Corners.size());
	bool success = Get3DMarkerCorners(marker3D, marker, depthFrame, frameWidth, frameHeight);

	if (!success)
	{
		return false;
	}

	// Save the found marker position and wait until enough samples have been saved
	markerSamplePositions.push_back(marker3D);
	numSamples++;

	if (numSamples < NumRequiredSamples) {
		return false;
	}
		
	// Calculate the average 3D position of the marker from all samples
	for (size_t i = 0; i < marker3D.size(); i++)
	{
		marker3D[i] = Point3f();
		for (int j = 0; j < NumRequiredSamples; j++)
		{
			marker3D[i].X += markerSamplePositions[j][i].X / (float)NumRequiredSamples;
			marker3D[i].Y += markerSamplePositions[j][i].Y / (float)NumRequiredSamples;
			marker3D[i].Z += markerSamplePositions[j][i].Z / (float)NumRequiredSamples;
		}
	}

	// Apply the Procrustes algorithm to find the world position of the marker
	Procrustes(marker, marker3D, worldT, worldR);

	vector<vector<float>> Rcopy = worldR;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			worldR[i][j] = 0;

			for (int k = 0; k < 3; k++)
			{
				worldR[i][j] += markerPose.R[i][k] * Rcopy[k][j];
			}
		}
	}

	vector<float> translationIncr(3);
	translationIncr[0] = markerPose.T[0];
	translationIncr[1] = markerPose.T[1];
	translationIncr[2] = markerPose.T[2];;

	translationIncr = InverseRotatePoint(translationIncr, worldR);

	worldT[0] += translationIncr[0];
	worldT[1] += translationIncr[1];
	worldT[2] += translationIncr[2];

	isCalibrated = true;

	markerSamplePositions.clear();
	numSamples = 0;

	return true;
}

/// <summary>
/// Attempts to load calibration data for the current camera from a file.
/// </summary>
/// <param name="serialNumber">Serial number of the current camera</param>
/// <returns></returns>
bool Calibration::LoadCalibration(const string &serialNumber)
{
	ifstream file;
	file.open("calibration_" + serialNumber + ".txt");

	if (!file.is_open())
		return false;

	for (int i = 0; i < 3; i++)
		file >> worldT[i];

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			file >> worldR[i][j];
	}

	file >> usedMarkerId;
	file >> isCalibrated;

	return true;
}

/// <summary>
/// Saves the current calibration to a file.
/// </summary>
/// <param name="serialNumber">Serial number of the current camera</param>
void Calibration::SaveCalibration(const string &serialNumber)
{
	ofstream file;
	file.open("calibration_" + serialNumber + ".txt");

	for (int i = 0; i < 3; i++)
		file << worldT[i] << " ";

	file << endl;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			file << worldR[i][j];

		file << endl;
	}

	file << usedMarkerId << endl;
	file << isCalibrated << endl;

	file.close();
}

/// <summary>
/// Sets the logging function to be used to append messages to the logging file.
/// </summary>
/// <param name="loggerFunc">Function to be used for logging. Should be passed by liveScanClient.cpp.</param>
void Calibration::SetLogger(std::function<void(const std::string&)> loggerFunc) {
	logFn = loggerFunc;
}

/// <summary>
/// Applies the Procrustes algorithm to find the transformation (rotation and translation)
/// that maps the detected marker points in camera space to their known positions in world space.
/// </summary>
/// <param name="marker">Information of the marker found in the color frame</param>
/// <param name="markerInWorld">Position of the marker in camera</param>
/// <param name="worldToMarkerT">Resulting transformation of world coordinates to obtain the marker position</param>
/// <param name="worldToMarkerR">Resulting transformation of world coordinates to obtain the marker rotation</param>
void Calibration::Procrustes(MarkerInfo &marker, vector<Point3f> &markerInWorld, vector<float> &worldToMarkerT, vector<vector<float>> &worldToMarkerR)
{
	int nVertices = marker.Points.size();

	// Compute centroids of both point sets
	Point3f markerCenterInWorld;
	Point3f markerCenter;

	for (int i = 0; i < nVertices; i++)
	{
		markerCenterInWorld.X += markerInWorld[i].X / nVertices;
		markerCenterInWorld.Y += markerInWorld[i].Y / nVertices;
		markerCenterInWorld.Z += markerInWorld[i].Z / nVertices;

		markerCenter.X += marker.Points[i].X / nVertices;
		markerCenter.Y += marker.Points[i].Y / nVertices;
		markerCenter.Z += marker.Points[i].Z / nVertices;
	}

	// Compute translation from world to camera
	worldToMarkerT.resize(3);
	worldToMarkerT[0] = -markerCenterInWorld.X;
	worldToMarkerT[1] = -markerCenterInWorld.Y;
	worldToMarkerT[2] = -markerCenterInWorld.Z;

	// Center the point clouds
	vector<Point3f> markerInWorldTranslated(nVertices);
	vector<Point3f> markerTranslated(nVertices);

	for (int i = 0; i < nVertices; i++)
	{
		markerInWorldTranslated[i].X = markerInWorld[i].X + worldToMarkerT[0];
		markerInWorldTranslated[i].Y = markerInWorld[i].Y + worldToMarkerT[1];
		markerInWorldTranslated[i].Z = markerInWorld[i].Z + worldToMarkerT[2];

		markerTranslated[i].X = marker.Points[i].X - markerCenter.X;
		markerTranslated[i].Y = marker.Points[i].Y - markerCenter.Y;
		markerTranslated[i].Z = marker.Points[i].Z - markerCenter.Z;
	}

	// Convert to OpenCV matrices
	cv::Mat A(nVertices, 3, CV_64F); // Camera (translated)
	cv::Mat B(nVertices, 3, CV_64F); // World (translated

	for (int i = 0; i < nVertices; i++)
	{
		A.at<double>(i, 0) = markerTranslated[i].X;
		A.at<double>(i, 1) = markerTranslated[i].Y;
		A.at<double>(i, 2) = markerTranslated[i].Z;

		B.at<double>(i, 0) = markerInWorldTranslated[i].X;
		B.at<double>(i, 1) = markerInWorldTranslated[i].Y;
		B.at<double>(i, 2) = markerInWorldTranslated[i].Z;
	}

	// Compute rotation using SVD
	cv::Mat M = A.t() * B;

	cv::SVD svd;
	svd(M);
	cv::Mat R = svd.u * svd.vt;

	double det = cv::determinant(R);

	// Handle reflection case (if determinant is negative)
	if (det < 0)
	{
		cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
		temp.at<double>(2, 2) = -1;
		R = svd.u * temp * svd.vt;
	}

	// Copy rotation matrix to output
	worldToMarkerR.resize(3);

	for (int i = 0; i < 3; i++)
	{
		worldToMarkerR[i].resize(3);
		for (int j = 0; j < 3; j++)
		{
			worldToMarkerR[i][j] = static_cast<float>(R.at<double>(i, j));
		}
	}
}

/// <summary>
/// Uses bilinear interpolation to find marker corner positions in 3D (camera space) from a depth frame.
/// </summary>
/// <param name="marker3D">Output vector of 3D positions for each marker corner (camera space)</param>
/// <param name="marker">Information of the marker found in the color frame</param>
/// <param name="depthFrame">A depth frame, aligned with the color frame</param>
/// <param name="frameWidth">Width of the color and depth frames</param>
/// <param name="frameHeight">Height of the color and depth frames</param>
/// <returns></returns>
bool Calibration::Get3DMarkerCorners(vector<Point3f> &marker3D, MarkerInfo &marker, Point3f *depthFrame, int frameWidth, int frameHeight)
{
	for (unsigned int i = 0; i < marker.Corners.size(); i++)
	{
		// Get pixel coordinates of the corner
		int minX = static_cast<int>(marker.Corners[i].X);
		int maxX = minX + 1;
		int minY = static_cast<int>(marker.Corners[i].Y);
		int maxY = minY + 1;

		// Compute how far the actual corner is from the top-left pixel
		float dx = marker.Corners[i].X - minX;
		float dy = marker.Corners[i].Y - minY;

		// Fetch 3D points at surrounding pixels
		Point3f pointMin = depthFrame[minX + minY * frameWidth];
		Point3f pointXMaxYMin = depthFrame[maxX + minY * frameWidth];
		Point3f pointXMinYMax = depthFrame[minX + maxY * frameWidth];
		Point3f pointMax = depthFrame[maxX + maxY * frameWidth];

		// If any of the depth values are invalid (Z <= 0), abort
		if (pointMin.Z <= 0 || pointXMaxYMin.Z <= 0 || pointXMinYMax.Z <= 0 || pointMax.Z <= 0)
			return false;

		// Bilinear interpolation for X, Y, Z
		marker3D[i].X = (1 - dx) * (1 - dy) * pointMin.X + dx * (1 - dy) * pointXMaxYMin.X + (1 - dx) * dy * pointXMinYMax.X + dx * dy * pointMax.X;
		marker3D[i].Y = (1 - dx) * (1 - dy) * pointMin.Y + dx * (1 - dy) * pointXMaxYMin.Y + (1 - dx) * dy * pointXMinYMax.Y + dx * dy * pointMax.Y;
		marker3D[i].Z = (1 - dx) * (1 - dy) * pointMin.Z + dx * (1 - dy) * pointXMaxYMin.Z + (1 - dx) * dy * pointXMinYMax.Z + dx * dy * pointMax.Z;
	}

	return true;
}

/// <summary>
/// Applies the inverse rotation to a 3D point using the transpose of R.
/// For rotation matrices, inverse(R) = transpose(R).
/// This is equivalent to: result = R^T * point.
/// </summary>
/// <param name="point">Input 3D point as vector [x, y, z]</param>
/// <param name="R">3x3 rotation matrix</param>
/// <returns>Inverse-rotated 3D point</returns>
vector<float> InverseRotatePoint(vector<float> &point, std::vector<std::vector<float>> &R)
{
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[1][0] + point[2] * R[2][0];
	res[1] = point[0] * R[0][1] + point[1] * R[1][1] + point[2] * R[2][1];
	res[2] = point[0] * R[0][2] + point[1] * R[1][2] + point[2] * R[2][2];

	return res;
}

/// <summary>
/// Rotates a 3D point using a 3x3 rotation matrix R.
/// This is equivalent to: result = R * point.
/// </summary>
/// <param name="point">Input 3D point as vector [x, y, z]</param>
/// <param name="R">3x3 rotation matrix</param>
/// <returns>Rotated 3D point</returns>
vector<float> RotatePoint(vector<float> &point, std::vector<std::vector<float>> &R)
{
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[0][1] + point[2] * R[0][2];
	res[1] = point[0] * R[1][0] + point[1] * R[1][1] + point[2] * R[1][2];
	res[2] = point[0] * R[2][0] + point[1] * R[2][1] + point[2] * R[2][2];

	return res;
}
