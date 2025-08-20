/***************************************************************************\

Module Name:  TransferObjectUtils.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module contains some definitions for structs which are used as transfer 
objects to share data between C++ and C# modules of the application.

\***************************************************************************/

#pragma once
#include <calibration.h>

struct CameraSettings
{
    float MinBounds[3];
    float MaxBounds[3];

    bool Filter;
    int FilterNeighbors;
    float FilterThreshold;

    MarkerPose* MarkerPoses;
    int NumMarkers;

    bool AutoExposureEnabled;
    int ExposureStep;
};

struct AffineTransform
{
    float R[3][3];
    float T[3];
};