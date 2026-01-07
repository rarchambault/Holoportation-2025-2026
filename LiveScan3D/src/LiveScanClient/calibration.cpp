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

//modifications

#include <algorithm>
#include <cmath>
#include <numeric>

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

// modifications

static inline float Sq(float v) { return v * v; }

static inline float DistSq3(const Point3f& a, const Point3f& b)
{
	return Sq(a.X - b.X) + Sq(a.Y - b.Y) + Sq(a.Z - b.Z);
}

static Point3f MedianPoint3f(std::vector<Point3f>& pts)
{
	// pts must be non-empty
	std::vector<float> xs, ys, zs;
	xs.reserve(pts.size());
	ys.reserve(pts.size());
	zs.reserve(pts.size());

	for (const auto& p : pts)
	{
		xs.push_back(p.X);
		ys.push_back(p.Y);
		zs.push_back(p.Z);
	}

	auto medianOf = [](std::vector<float>& v) -> float {
		size_t mid = v.size() / 2;
		std::nth_element(v.begin(), v.begin() + mid, v.end());
		float med = v[mid];
		if ((v.size() % 2) == 0)
		{
			std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
			med = 0.5f * (med + v[mid - 1]);
		}
		return med;
		};

	Point3f out;
	out.X = medianOf(xs);
	out.Y = medianOf(ys);
	out.Z = medianOf(zs);
	return out;
}

static inline float Dist3(const Point3f& a, const Point3f& b)
{
	float dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static inline float Dist2(const Point2f& a, const Point2f& b)
{
	float dx = a.X - b.X, dy = a.Y - b.Y;
	return std::sqrt(dx * dx + dy * dy);
}

// Returns true if this sample is "reasonable"
static bool PassMarkerSampleGates(
	const MarkerInfo& marker,
	const std::vector<Point3f>& marker3D)
{
	// Assumptions:
	// - marker has 4 corners in consistent order (0..3)
	// - marker3D contains corresponding 3D points
	if (marker.Corners.size() < 4 || marker3D.size() < 4) return false;

	// (A) 2D geometry sanity: opposite edges should be similar length
	float e01 = Dist2(marker.Corners[0], marker.Corners[1]);
	float e12 = Dist2(marker.Corners[1], marker.Corners[2]);
	float e23 = Dist2(marker.Corners[2], marker.Corners[3]);
	float e30 = Dist2(marker.Corners[3], marker.Corners[0]);

	// avoid degenerate detections
	if (e01 < 2.0f || e12 < 2.0f || e23 < 2.0f || e30 < 2.0f) return false;

	float opp1 = std::max(e01, e23) / std::min(e01, e23);
	float opp2 = std::max(e12, e30) / std::min(e12, e30);

	// best-effort threshold: if the rectangle is extremely skewed, reject
	if (opp1 > 2.5f || opp2 > 2.5f) return false;

	// (B) 3D depth sanity: corner depths shouldn't wildly disagree
	float zmin = marker3D[0].Z, zmax = marker3D[0].Z;
	for (int i = 1; i < 4; i++)
	{
		zmin = std::min(zmin, marker3D[i].Z);
		zmax = std::max(zmax, marker3D[i].Z);
	}

	// Best-effort: allow some tilt; reject extreme depth spread (often bad depth)
	// Use a relative gate to be unit-agnostic.
	float zSpread = zmax - zmin;
	float zMean = 0.25f * (marker3D[0].Z + marker3D[1].Z + marker3D[2].Z + marker3D[3].Z);
	if (zMean <= 0.0f) return false;

	// e.g. spread > 25% of mean distance is suspicious
	if (zSpread > 0.25f * zMean) return false;

	// (C) 3D size sanity: adjacent 3D edges shouldn't collapse
	float d01 = Dist3(marker3D[0], marker3D[1]);
	float d12 = Dist3(marker3D[1], marker3D[2]);
	float d23 = Dist3(marker3D[2], marker3D[3]);
	float d30 = Dist3(marker3D[3], marker3D[0]);

	if (d01 <= 0.0f || d12 <= 0.0f || d23 <= 0.0f || d30 <= 0.0f) return false;

	// Reject if 3D edges are extremely inconsistent (often indicates depth holes)
	float dOpp1 = std::max(d01, d23) / std::min(d01, d23);
	float dOpp2 = std::max(d12, d30) / std::min(d12, d30);
	if (dOpp1 > 3.0f || dOpp2 > 3.0f) return false;

	return true;
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
bool Calibration::Calibrate(RGB* colorFrame, Point3f* depthFrame, int frameWidth, int frameHeight)
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

	if (!PassMarkerSampleGates(marker, marker3D))
		return false;


	// Save the found marker position and wait until enough samples have been saved
	markerSamplePositions.push_back(marker3D);
	numSamples++;

	if (numSamples < NumRequiredSamples) {
		return false;
	}

	// Calculate the average 3D position of the marker from all samples
	// --- Robust frame outlier rejection + averaging ---
// 1) Build a robust per-corner reference using the median across samples.
// 2) Score each sample by how far its corners deviate from that reference.
// 3) Keep only the best samples (drop worst outliers).
// 4) Average only inliers.

	const int S = NumRequiredSamples;
	const int C = (int)marker3D.size();

	// (A) robust reference per corner (median across samples)
	std::vector<Point3f> refCorners(C);
	for (int c = 0; c < C; c++)
	{
		std::vector<Point3f> cornerPts;
		cornerPts.reserve(S);
		for (int s = 0; s < S; s++)
			cornerPts.push_back(markerSamplePositions[s][c]);

		refCorners[c] = MedianPoint3f(cornerPts);
	}

	// (B) compute per-sample error score
	struct SampleScore { int idx; float err; };
	std::vector<SampleScore> scores;
	scores.reserve(S);

	for (int s = 0; s < S; s++)
	{
		float err = 0.0f;
		for (int c = 0; c < C; c++)
			err += DistSq3(markerSamplePositions[s][c], refCorners[c]);

		scores.push_back({ s, err });
	}

	// (C) keep best fraction (drop worst outliers)
	std::sort(scores.begin(), scores.end(),
		[](const SampleScore& a, const SampleScore& b) { return a.err < b.err; });

	// Best-effort choice: keep 80% of samples, at least 3
	int keepCount = (int)std::floor(0.8f * (float)S);
	if (keepCount < 3) keepCount = std::min(3, S);

	// (D) average only inliers
	marker3D.assign(C, Point3f());
	for (int k = 0; k < keepCount; k++)
	{
		int s = scores[k].idx;
		for (int c = 0; c < C; c++)
		{
			marker3D[c].X += markerSamplePositions[s][c].X;
			marker3D[c].Y += markerSamplePositions[s][c].Y;
			marker3D[c].Z += markerSamplePositions[s][c].Z;
		}
	}

	const float invKeep = 1.0f / (float)keepCount;
	for (int c = 0; c < C; c++)
	{
		marker3D[c].X *= invKeep;
		marker3D[c].Y *= invKeep;
		marker3D[c].Z *= invKeep;
	}
	// --- end robust averaging ---


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
bool Calibration::LoadCalibration(const string& serialNumber)
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
void Calibration::SaveCalibration(const string& serialNumber)
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
void Calibration::Procrustes(MarkerInfo& marker, vector<Point3f>& markerInWorld, vector<float>& worldToMarkerT, vector<vector<float>>& worldToMarkerR)
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

//modifications

static inline bool IsValidDepthPoint(const Point3f& p)
{
	// Best-effort assumption:
	// - invalid if Z <= 0
	// - also reject NaN/Inf
	return (p.Z > 0.0f) && std::isfinite(p.X) && std::isfinite(p.Y) && std::isfinite(p.Z);
}

static inline float MedianOf(std::vector<float>& v)
{
	// v must be non-empty
	size_t mid = v.size() / 2;
	std::nth_element(v.begin(), v.begin() + mid, v.end());
	float med = v[mid];

	// If even size, average two middle values (slightly nicer)
	if ((v.size() % 2) == 0)
	{
		std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
		med = 0.5f * (med + v[mid - 1]);
	}
	return med;
}

static bool SampleRobustPointFromPatch(
	const Point3f* depthFrame,
	int frameWidth,
	int frameHeight,
	float x,
	float y,
	int radius,               // radius=2 => 5x5
	int minValidSamples,      // minimum valid points needed
	Point3f& outPoint)
{
	// Nearest pixel center (best-effort assumption)
	int cx = static_cast<int>(std::lround(x));
	int cy = static_cast<int>(std::lround(y));

	int x0 = std::max(0, cx - radius);
	int x1 = std::min(frameWidth - 1, cx + radius);
	int y0 = std::max(0, cy - radius);
	int y1 = std::min(frameHeight - 1, cy + radius);

	std::vector<float> xs;
	std::vector<float> ys;
	std::vector<float> zs;
	xs.reserve((2 * radius + 1) * (2 * radius + 1));
	ys.reserve((2 * radius + 1) * (2 * radius + 1));
	zs.reserve((2 * radius + 1) * (2 * radius + 1));

	for (int yy = y0; yy <= y1; yy++)
	{
		int row = yy * frameWidth;
		for (int xx = x0; xx <= x1; xx++)
		{
			const Point3f p = depthFrame[row + xx];
			if (!IsValidDepthPoint(p)) continue;

			xs.push_back(p.X);
			ys.push_back(p.Y);
			zs.push_back(p.Z);
		}
	}

	if (static_cast<int>(zs.size()) < minValidSamples)
		return false;

	// Median independently for X/Y/Z (robust, simple, works well for small patches)
	float mx = MedianOf(xs);
	float my = MedianOf(ys);
	float mz = MedianOf(zs);

	outPoint.X = mx;
	outPoint.Y = my;
	outPoint.Z = mz;
	return true;
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
bool Calibration::Get3DMarkerCorners(vector<Point3f>& marker3D, MarkerInfo& marker, Point3f* depthFrame, int frameWidth, int frameHeight)
{
	// Best-effort assumptions:
	// - depthFrame is aligned to the color frame
	// - invalid depth is Z <= 0 (and/or NaN/Inf)
	// - robust sampling is better than bilinear at edges/corners

	const int patchRadius = 2;            // 5x5 patch
	const int minValidSamples = 8;        // require at least 8 valid points in the patch

	for (unsigned int i = 0; i < marker.Corners.size(); i++)
	{
		Point3f robustPoint;
		bool ok = SampleRobustPointFromPatch(
			depthFrame,
			frameWidth,
			frameHeight,
			marker.Corners[i].X,
			marker.Corners[i].Y,
			patchRadius,
			minValidSamples,
			robustPoint
		);

		if (!ok)
			return false;

		marker3D[i] = robustPoint;
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
vector<float> InverseRotatePoint(vector<float>& point, std::vector<std::vector<float>>& R)
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
vector<float> RotatePoint(vector<float>& point, std::vector<std::vector<float>>& R)
{
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[0][1] + point[2] * R[0][2];
	res[1] = point[0] * R[1][0] + point[1] * R[1][1] + point[2] * R[1][2];
	res[2] = point[0] * R[2][0] + point[1] * R[2][1] + point[2] * R[2][2];

	return res;
}
