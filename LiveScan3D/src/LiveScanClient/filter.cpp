/***************************************************************************\

Module Name:  Filter.cpp
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

#include "filter.h"

using namespace std;

/// <summary>
/// Computes the k-nearest neighbours for each point in a point cloud using the KD-tree.
/// </summary>
/// <param name="cloud">Point cloud in which to compute the k-nearest neighbours</param>
/// <param name="tree">KD-tree to use to compute the k-nearest neighbours</param>
/// <param name="k">Number of neighbours to evaluate</param>
/// <returns>A list of KNNResult, each containing the indices and distances of the k-nearest neighbors.</returns>
vector<KNNResult> ComputeKNearestNeighbours(PointCloud &cloud, KdTree3D &tree, int k)
{
	vector<KNNResult> results(cloud.Points.size());
	int numPoints = static_cast<int>(cloud.Points.size());

#pragma omp parallel for
	for (int i = 0; i < numPoints; i++)
	{
		results[i].Neighbors.resize(k);
		results[i].Distances.resize(k);

		// Perform the k-NN search using nanoflann
		tree.knnSearch((float*)(cloud.Points.data() + i), k, (size_t*)results[i].Neighbors.data(), results[i].Distances.data());

		// Store the distance to the k-th nearest neighbor
		results[i].KthNeighbourDistance = results[i].Distances[k - 1];
	}

	return results;
}

/// <summary>
/// Removes outlier points from the input point cloud based on k-nearest neighbor distance.
/// </summary>
/// <param name="vertices">Vertices of the input point cloud (this vector will be modified directly)</param>
/// <param name="colors">Colors of the input point cloud (this vector will be modified directly)</param>
/// <param name="k">Number of neighbours to evaluate</param>
/// <param name="maxDist">Maximum distance with the k nearest neighbours for a point to be kept</param>
void Filter(std::vector<Point3f> &vertices, std::vector<RGB> &colors, int k, float maxDist)
{
	if (k <= 0 || maxDist <= 0)
		return;

	// Wrap input points into a PointCloud object
	PointCloud cloud;
	cloud.Points = vertices;

	// Build the KD-tree
	KdTree3D tree(3, cloud);
	tree.buildIndex();

	// Compute k-nearest neighbors for all points
	vector<KNNResult> knnResults = ComputeKNearestNeighbours(cloud, tree, k);

	float distanceThresholdSquared = maxDist * maxDist;
	vector<int> outlierIndices;

	// Identify outliers whose k-th neighbor is too far away
	for (unsigned int i = 0; i < cloud.Points.size(); i++)
	{
		if (knnResults[i].KthNeighbourDistance > distanceThresholdSquared)
			outlierIndices.push_back(i);
	}

	// Remove the identified outliers (in-place compaction)
	int writeIndex = 0;
	unsigned int removeIndexCursor = 0;
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		if (removeIndexCursor < outlierIndices.size() && i == outlierIndices[removeIndexCursor])
		{
			removeIndexCursor++;
			continue;
		}

		vertices[writeIndex] = vertices[i];
		colors[writeIndex] = colors[i];

		writeIndex++;
	}

	vertices.resize(writeIndex);
	colors.resize(writeIndex);
}