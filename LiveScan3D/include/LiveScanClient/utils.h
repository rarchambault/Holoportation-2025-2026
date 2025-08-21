/***************************************************************************\

Module Name:  Utils.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module contains some definitions for structs and methods which
are used in several other modules of the LiveScanClient project.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once

#include "stdafx.h"
#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

enum SyncState 
{ 
	Subordinate, 
	Master, 
	Standalone 
};

typedef struct Point3f
{
	float X;
	float Y;
	float Z;
	bool Invalid = false;

	Point3f()
	{
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
		this->Invalid = false;
	}

	Point3f(float X, float Y, float Z)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
		this->Invalid = false;
	}

	Point3f(float X, float Y, float Z, bool invalid)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
		this->Invalid = invalid;
	}
} Point3f;

#pragma pack(push, 1)
typedef struct Point3s
{
	short X;
	short Y;
	short Z;

	Point3s()
	{
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}

	Point3s(short X, short Y, short Z)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}

	// Convert meters to milimeters
	Point3s(Point3f &other)
	{
		this->X = static_cast<short>(1000 * other.X);
		this->Y = static_cast<short>(1000 * other.Y);
		this->Z = static_cast<short>(1000 * other.Z);
	}
} Point3s;
#pragma pack(pop)

typedef struct Point2f
{
	float X;
	float Y;

	Point2f()
	{
		this->X = 0;
		this->Y = 0;
	}

	Point2f(float X, float Y)
	{
		this->X = X;
		this->Y = Y;
	}
} Point2f;

#pragma pack(push, 1)
typedef struct RGB
{
	BYTE    Blue;
	BYTE    Green;
	BYTE    Red;
} RGB;
#pragma pack(pop)

typedef struct DetectionResult {
	cv::Mat data;
	short width;
	short height;
	float score;
};

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R);
Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R);
