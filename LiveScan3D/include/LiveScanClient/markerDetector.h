/***************************************************************************\

Module Name:  MarkerDetector.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module uses the iMarker interface to detect markers in a provided
2D color frame.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/
#pragma once

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "IMarkerDetector.h"

using namespace std;

class MarkerDetector : public IMarkerDetector
{
public:
	bool DetectMarkersInImage(RGB *img, int height, int width, MarkerInfo &marker);
private:
	const int NumMarkerCorners = 5; // Number of corners to find in the detected marker

	// Maximum and minimum sizes with which a marker can be detected
	const int MinSize = 100;
	const int MaxSize = 1000000000;

	const int ColorFrameBitThreshold = 120; // Threshold used to obtain a binary (black and white) image from the color frame
	const double ApproxPolyCoefficient = 0.12; // Coefficient used to map the image to the polygon representing the markers

	// Normalized point coordinates which make up the shape of the markers
	const vector<cv::Point2f> NormalizedMarkerPoints = {
	{0.0f, 1.0f}, // Bottom center point
	{-1.0f, 1.6667f}, // Bottom left corner
	{-1.0f, -1.0f}, // Top left corner
	{1.0f, -1.0f}, // Top right corner
	{1.0f, 1.6667f} // Bottom right corner
	};

	// Normalized point coordinates which make up the shape of the markers in 3D
	const vector<Point3f> NormalizedMarkerPoints3D = {
	{0.0f, -1.0f, 0.0f}, // Bottom center point
	{-1.0f, -1.6667f, 0.0f}, // Bottom left corner
	{-1.0f, 1.0f, 0.0f}, // Top left corner
	{1.0f, 1.0f, 0.0f}, // Top right corner
	{1.0f, -1.6667f, 0.0f} // Bottom right corner
	};

	// Parameters used for extracting marker code from the marker image
	const double NormalizedMarkerSize = 2.0; // Arbitrary size to which detected markers are mapped to retrieve their code
	const double NormalizedMarkerBorderSize = 0.4; // Size of the border relative to the normalized size (e.g., 0.4 out of 2 means 20% border on each side)
	const int WarpedMarkerResolutionPerUnit = 50; // Pixel resolution (for each unit square in the normalized size marker)
	const int BitGridSize = 3; // Number of grid cells on each side of the square representing the marker code
	const int CodeDetectionBitThreshold = 128; // Threshold used to determine if a grid cell is black or white

	const bool DrawOnOriginalImage = false;

	bool DetectMarkers(cv::Mat &img, MarkerInfo &marker);
	bool OrderCorners(vector<cv::Point2f> &corners);
	int GetCode(cv::Mat &img, vector<cv::Point2f> points, vector<cv::Point2f> corners);
	void RefineCornerPositions(vector<cv::Point2f> &corners, vector<cv::Point> contour, bool order);
	cv::Point2f GetIntersection(cv::Vec4f lin1, cv::Vec4f lin2);
	double GetMarkerArea(MarkerInfo &marker);
};