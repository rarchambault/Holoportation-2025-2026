/***************************************************************************\

Module Name:  Filter.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module applies a KNN filter to a point cloud to remove some unwanted outlier
points.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#pragma once

#include <vector>
#include "nanoflann.h"
#include "utils.h"

struct KNNResult
{
	std::vector<size_t> Neighbors;
	std::vector<float> Distances;
	float KthNeighbourDistance;
};

struct PointCloud
{
	std::vector<Point3f> Points;

	// nanoflann adaptor methods
	inline size_t kdtree_get_point_count() const { return Points.size(); }

	// Computes the distance between the vector "queryPoint[0:size-1]" and the data point with index "targetIdx" stored in the class
	inline float kdtree_distance(const float *queryPoint, const size_t targetIdx, size_t /*size*/) const
	{
		const float dx = queryPoint[0] - Points[targetIdx].X;
		const float dy = queryPoint[1] - Points[targetIdx].Y;
		const float dz = queryPoint[2] - Points[targetIdx].Z;

		return dx * dx + dy * dy + dz * dz;
	}

	// Returns the dimemsion-th component of the point at the specified index in the class:
	inline float kdtree_get_pt(const size_t index, int dimension) const
	{
		if (dimension == 0) return Points[index].X;
		else if (dimension == 1) return Points[index].Y;
		else return Points[index].Z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	// Return true if the BBOX was already computed by the class and returned in "bb" to avoid computing it again.
	// Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

using KdTree3D = nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<float, PointCloud>,
	PointCloud, 3>;

void Filter(std::vector<Point3f> &vertices, std::vector<RGB> &colors, int k = 10, float maxDist = 0.01);