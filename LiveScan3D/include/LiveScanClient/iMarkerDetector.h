/***************************************************************************\

Module Name:  iMarkerDetector.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module stores data associated with a calibration marker.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/
#pragma once

#include "utils.h"

typedef struct MarkerStruct
{
	int Id;

	// Positions of the marker corners in a 2D color frame
	std::vector<Point2f> Corners;

	// Positions of the marker corners in local marker space
	std::vector<Point3f> Points;

	MarkerStruct()
	{
		Id = -1;
	}

	MarkerStruct(int id, std::vector<Point2f> corners, std::vector<Point3f> points)
	{
		this->Id = id;

		this->Corners = corners;
		this->Points = points;
	}
} MarkerInfo;

class IMarkerDetector
{
public:
	IMarkerDetector() {};

	// Finds all markers in the provided 2D color frame and keeps the best detected one
	virtual bool DetectMarkersInImage(RGB *img, int height, int width, MarkerInfo &marker) = 0;
};