/***************************************************************************\

Module Name:  Utils.cpp
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

#include "utils.h"

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	return res;
}

Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[1][0] + point.Z * R[2][0];
	res.Y = point.X * R[0][1] + point.Y * R[1][1] + point.Z * R[2][1];
	res.Z = point.X * R[0][2] + point.Y * R[1][2] + point.Z * R[2][2];

	return res;
}
