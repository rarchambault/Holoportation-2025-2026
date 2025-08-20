/***************************************************************************\

Module Name:  iCaptureManager.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module defines some functions and fields which should be implemented by
any capture manager modules used for retrieving data from a device for the
point cloud reconstruction.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#include "ICaptureManager.h"

ICaptureManager::ICaptureManager()
{
	isInitialized = false;

	colorFrameHeight = 0;
	colorFrameWidth = 0;

	depthData = NULL;
	colorData = NULL;
}

ICaptureManager::~ICaptureManager()
{
	if (depthData != NULL)
	{
		delete[] depthData;
		depthData = NULL;
	}

	if (colorData != NULL)
	{
		delete[] colorData;
		colorData = NULL;
	}
}