/***************************************************************************\

Module Name:  ICP.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module applies the Iterative Closest Point algorithm to align two sets
of 3D points.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#include "icp.h"

/// <summary>
/// Performs Iterative Closest Point (ICP) alignment to compute a rigid transformation 
/// (rotation and translation) that aligns source vertices to the target vertices.
/// </summary>
/// <param name="targetVerts">Target point cloud (fixed)</param>
/// <param name="sourceVerts">Source point cloud (will be transformed in place)</param>
/// <param name="numTargetVerts">Number of points in targetVerts</param>
/// <param name="numSourceVerts">Number of points in sourceVerts</param>
/// <param name="R">Output 3x3 rotation matrix (row-major)</param>
/// <param name="t">Output 3D translation vector</param>
/// <param name="maxIter">Maximum number or ICP iterations</param>
/// <returns>Final alignment error</returns>
ICP_API float __stdcall ICP(Point3f* targetVerts, Point3f* sourceVerts, int numTargetVerts, int numSourceVerts, float* R, float* t, int maxIter)
{
	// Convert targetVerts into a point cloud structure for KD-tree
	PointCloud targetCloud;
	targetCloud.Points = vector<Point3f>(targetVerts, targetVerts + numTargetVerts);

	// Wrap output rotation and translation as cv::Mat
	cv::Mat matR(3, 3, CV_32F, R);
	cv::Mat matT(1, 3, CV_32F, t);

	// Convert sourceVerts into a CV matrix for matrix operations
	cv::Mat sourceVertsMat(numSourceVerts, 3, CV_32F, (float*)sourceVerts);

	float error = 1.0f;

	for (int iter = 0; iter < maxIter; iter++)
	{
		vector<Point3f> matchedTargetVerts;
		vector<Point3f> matchedSourceVerts;

		vector<float> distances(numSourceVerts);
		vector<size_t> indices(numSourceVerts);

		// Find nearest neighbors from sourceVerts to targetVerts
		FindNearestNeighbours(targetCloud, sourceVertsMat, distances, indices);

		// For each source point, keep only the best match
		vector<float> matchDistances;
		vector<int> matchMap(numTargetVerts, -1); // Maps targetVerts index to matched index

		for (int i = 0; i < numSourceVerts; ++i)
		{
			int targetIdx = indices[i];
			float currentDistance = distances[i];

			int existingMatchPos = matchMap[targetIdx];

			if (existingMatchPos != -1 && matchDistances[existingMatchPos] < currentDistance)
				continue;

			// Get matched point from sourceVerts
			Point3f queryPoint;
			queryPoint.X = sourceVertsMat.at<float>(i, 0);
			queryPoint.Y = sourceVertsMat.at<float>(i, 1);
			queryPoint.Z = sourceVertsMat.at<float>(i, 2);

			if (existingMatchPos == -1)
			{
				matchedTargetVerts.push_back(targetVerts[targetIdx]);
				matchedSourceVerts.push_back(queryPoint);
				matchDistances.push_back(currentDistance);
				matchMap[targetIdx] = matchedSourceVerts.size() - 1;
			}
			else
			{
				matchedSourceVerts[existingMatchPos] = queryPoint;
				matchDistances[existingMatchPos] = currentDistance;
			}
		}

		// Remove outliers based on distance threshold
		RejectOutlierMatches(matchedTargetVerts, matchedSourceVerts, matchDistances, 2.5f);

		// Estimate translation (centroid difference)
		cv::Mat matchedTargetMat(matchedTargetVerts.size(), 3, CV_32F, matchedTargetVerts.data());
		cv::Mat matchedSourceMat(matchedSourceVerts.size(), 3, CV_32F, matchedSourceVerts.data());

		cv::Mat centroidShift;
		cv::reduce(matchedTargetMat - matchedSourceMat, centroidShift, 0, CV_REDUCE_AVG);

		// Apply translation shift to sourceVerts and matchedSourceVerts
		for (int i = 0; i < sourceVertsMat.rows; ++i)
		{
			sourceVertsMat.row(i) += centroidShift;
		}

		for (int i = 0; i < matchedSourceMat.rows; ++i)
		{
			matchedSourceMat.row(i) += centroidShift;
		}

		// Estimate optimal rotation using SVD
		cv::Mat crossCov = matchedSourceMat.t() * matchedTargetMat;
		cv::SVD svd;
		svd(crossCov);

		cv::Mat rotationUpdate = svd.u * svd.vt;
		
		// Ensure a proper rotation (handle reflection case)
		if (cv::determinant(rotationUpdate) < 0)
		{
			cv::Mat reflexionFix = cv::Mat::eye(3, 3, CV_32F);
			reflexionFix.at<float>(2, 2) = -1.0f;
			rotationUpdate = svd.u * reflexionFix * svd.vt;
		}

		// Apply rotation update
		sourceVertsMat = sourceVertsMat * rotationUpdate;

		matT += centroidShift * matR.t();
		matR = matR * rotationUpdate;

		// Optionally compute and print alignment error
		
		error = 0.0f;
		for (float d : matchDistances)
			error += std::sqrt(d);
		error /= matchDistances.size();
	}

	// Copy the transformed sourceVerts data back to original buffer
	memcpy(sourceVerts, sourceVertsMat.data, sourceVertsMat.rows * sizeof(float) * 3);

	// Copy the final transformation results
	memcpy(R, matR.data, 9 * sizeof(float));
	memcpy(t, matT.data, 3 * sizeof(float));;

	return error;
}

/// <summary>
/// Finds the closest point in the source point cloud for each point in the destination set.
/// </summary>
/// <param name="sourceCloud">Original point cloud to compare with the query points</param>
/// <param name="queryPoints">Query points to compare with the point cloud</param>
/// <param name="distances">Output squared distances to the nearest neighbours for each query point</param>
/// <param name="indices">Output indices of the closest points in sourceCloud for each query point</param>
void FindNearestNeighbours(PointCloud &sourceCloud, cv::Mat &queryPoints, vector<float> &distances, vector<size_t> &indices)
{
	int numQueryPoints = queryPoints.rows;

	// Define a 3D KD-tree using the source point cloud
	using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, PointCloud>,
		PointCloud, 3>;

	KDTree kdTree(3, sourceCloud);
	kdTree.buildIndex();

	// Parallel search for the nearest neighbor of each query point
#pragma omp parallel for
	for (int i = 0; i < numQueryPoints; i++)
	{
		nanoflann::KNNResultSet<float> resultSet(1); // Only 1 nearest neighbor
		resultSet.init(&indices[i], &distances[i]);

		// Query point is assumed to be a row in the Mat (3 floats)
		kdTree.findNeighbors(resultSet, (float*)queryPoints.row(i).data, nanoflann::SearchParams());
	}
}

/// <summary>
/// Filters out outlier correspondences between two point sets based on match distance
/// </summary>
/// <param name="matches1">Input/output 3D points from the first set (in-place update)</param>
/// <param name="matches2">Input/output 3D points from the second set (in-place update)</param>
/// <param name="matchDistances">Precomputed distances between point pairs</param>
/// <param name="maxStdDev">Maximum allowed distance in terms of standard deviations from the mean</param>
void RejectOutlierMatches(vector<Point3f>& matches1, vector<Point3f>& matches2, vector<float>& matchDistances, float maxStdDev)
{
	float distanceStdDev = GetStandardDeviation(matchDistances);

	vector<Point3f> filteredMatches1;
	vector<Point3f> filteredMatches2;

	for (size_t i = 0; i < matches1.size(); i++)
	{
		// Reject match if its distance is too far from the mean
		if (matchDistances[i] > maxStdDev * distanceStdDev)
			continue;

		filteredMatches1.push_back(matches1[i]);
		filteredMatches2.push_back(matches2[i]);
	}

	matches1 = filteredMatches1;
	matches2 = filteredMatches2;
}

/// <summary>
/// Calculates the standard deviation of a set of floating-point values.
/// </summary>
/// <param name="data">Data from which to compute the standard deviation</param>
/// <returns>The standard deviation of the data</returns>
float GetStandardDeviation(vector<float> &data)
{
	if (data.empty()) return 0.0f;

	// Calculate mean
	float mean = 0.0f;

	for (float value : data)
	{
		mean += value;
	}

	mean /= static_cast<float>(data.size());

	// Calculate variance
	float variance = 0.0f;

	for (float value : data)
	{
		float diff = value - mean;
		variance += diff * diff;
	}

	variance /= static_cast<float>(data.size());

	// Standard deviation is the square root of variance
	return std::sqrt(variance);
}
