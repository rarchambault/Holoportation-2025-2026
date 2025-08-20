/***************************************************************************\

Module Name:  MarkerDetector.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module uses the iMarkerDetector interface to detect markers in a provided
2D color frame.

This code was adapted from the following research:
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV),
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

#include "markerDetector.h"
#include <opencv2/opencv.hpp>

using namespace std;

/// <summary>
/// Finds all markers in the provided 2D color frame and outputs the best detected one;
/// modifies the original image to draw markers if requested.
/// </summary>
/// <param name="img">Color frame from which to detect markers</param>
/// <param name="height">Height of the color frame</param>
/// <param name="width">Width of the color frame</param>
/// <param name="marker">Output information on the detected marker</param>
/// <returns>True if a marker was detected, false otherwise</returns>
bool MarkerDetector::DetectMarkersInImage(RGB* img, int height, int width, MarkerInfo& marker)
{
	// Create an OpenCV 8-bit 3-channel image to hold the RGB data
	cv::Mat cvImg(height, width, CV_8UC3);

	// Copy raw RGB pixel data into the OpenCV Mat format (BGR order)
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			cvImg.at<cv::Vec3b>(i, j)[0] = img[j + width * i].Blue;
			cvImg.at<cv::Vec3b>(i, j)[1] = img[j + width * i].Green;
			cvImg.at<cv::Vec3b>(i, j)[2] = img[j + width * i].Red;
		}
	}

	bool res = DetectMarkers(cvImg, marker);

	// If the original image was modified to add marker drawings, copy it back to the original one
	if (DrawOnOriginalImage)
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				img[j + width * i].Blue = cvImg.at<cv::Vec3b>(i, j)[0];
				img[j + width * i].Green = cvImg.at<cv::Vec3b>(i, j)[1];
				img[j + width * i].Red = cvImg.at<cv::Vec3b>(i, j)[2];
			}
		}
	}

	return res;
}

/// <summary>
/// Finds all markers in the provided 2D color frame and outputs the best detected one.
/// </summary>
/// <param name="img">Color frame from which to detect markers</param>
/// <param name="marker">Output information on the detected marker</param>
/// <returns>True if a marker was detected, false otherwise</returns>
bool MarkerDetector::DetectMarkers(cv::Mat &img, MarkerInfo &marker)
{
	vector<MarkerInfo> markers;

	// Convert color image to grayscale for thresholding
	cv::Mat grayImg, thresholdedGrayImg;
	cv::cvtColor(img, grayImg, CV_BGR2GRAY);

	// Apply binary thresholding to extract high-contrast areas
	cv::threshold(grayImg, grayImg, ColorFrameBitThreshold, 255, CV_THRESH_BINARY);

	// Copy binary image for contour detection
	grayImg.copyTo(thresholdedGrayImg);

	// Find contours (connected regions of white pixels)
	vector<vector<cv::Point>> contours;	
	cv::findContours(thresholdedGrayImg, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// Process each contour found
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		vector<cv::Point> corners;
		double area = cv::contourArea(contours[i]);

		// Skip contours that are too small or too large
		if (area < MinSize || area > MaxSize)
			continue;

		// Approximate the contour to a polygon with fewer vertices
		cv::approxPolyDP(contours[i], corners, sqrt(area)*ApproxPolyCoefficient, true);

		// Convert corners to floating-point for further processing
		vector<cv::Point2f> cornersFloatDetected;
		for (unsigned int j = 0; j < corners.size(); j++)
		{
			cornersFloatDetected.push_back(cv::Point2f((float)corners[j].x, (float)corners[j].y));
		}

		// Check if the shape resembles a marker (not convex and has expected corner count)
		if (!cv::isContourConvex(corners) && corners.size() == NumMarkerCorners && OrderCorners(cornersFloatDetected))
		{	
			bool order = true;

			// Try to read marker code with the current corner order
			int code = GetCode(grayImg, NormalizedMarkerPoints, cornersFloatDetected);

			// If the first try failed, reverse corner order and try again
			if (code < 0)
			{
				reverse(cornersFloatDetected.begin() + 1, cornersFloatDetected.end());
				code = GetCode(grayImg, NormalizedMarkerPoints, cornersFloatDetected);

				if (code < 0)
					continue; // Still unreadable, skip

				order = false;
			}

			// Optional: refine corners using subpixel accuracy (disabled due to stability concerns)
			// CornersSubPix(cornersFloat, contours[i], order);

			// Only keep the expected number of corners
			vector<Point2f> cornersFloat(NumMarkerCorners);		

			for (int i = 0; i < NumMarkerCorners; i++)
			{
				cornersFloat[i] = Point2f(cornersFloatDetected[i].x, cornersFloatDetected[i].y);
			}

			// Store detected marker info
			markers.push_back(MarkerInfo(code, cornersFloat, NormalizedMarkerPoints3D));

			// Optional: draw marker on the image
			if (DrawOnOriginalImage)
			{
				for (unsigned int j = 0; j < corners.size(); j++)
				{
					cv::circle(img, cornersFloatDetected[j], 2, cv::Scalar(0, 50 * j, 0), 1);
					cv::line(img, cornersFloatDetected[j], cornersFloatDetected[(j + 1) % cornersFloatDetected.size()], cv::Scalar(0, 0, 255), 2);
				}
			}
		}
	}

	// If one or more markers were found, select the largest one and return it
	if (markers.size() > 0)
	{
		double maxArea = 0;
		int maxInd = 0;

		// Find marker with the largest area
		for (unsigned int i = 0; i < markers.size(); i++)
		{
			if (GetMarkerArea(markers[i]) > maxArea)
			{
				maxInd = i;
				maxArea = GetMarkerArea(markers[i]);
			}
		}

		marker = markers[maxInd];

		// Optional: draw the final selected marker with green outline
		if (DrawOnOriginalImage)
		{
			for (int j = 0; j < NumMarkerCorners; j++)
			{
				cv::Point2f pt1 = cv::Point2f(marker.Corners[j].X, marker.Corners[j].Y);
				cv::Point2f pt2 = cv::Point2f(marker.Corners[(j + 1) % NumMarkerCorners].X, marker.Corners[(j + 1) % NumMarkerCorners].Y);
				cv::line(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			}
		}

		return true;
	}
	
	// No valid marker found
	return false;
}

/// <summary>
/// Attempts to reorder the corner points so that the non-convex corner comes first,
/// ensuring a consistent winding order for marker decoding.
/// </summary>
/// <param name="corners">Input/output vector of marker corner points</param>
/// <returns>True if reordering succeeded, false otherwise</returns>
bool MarkerDetector::OrderCorners(vector<cv::Point2f> &corners)
{
	vector<int> convexHullIndices;

	// Compute the convex hull indices of the input corners
	cv::convexHull(corners, convexHullIndices);

	// Heuristic check:
	// If one corner is not on the convex hull (e.g., a dented quadrilateral),
	// then the hull will have one less point.
	if (convexHullIndices.size() != corners.size() - 1)
		return false;

	// Find the index of the point that is NOT part of the convex hull
	int concaveIndex = -1;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		bool isOnHull = false;
		for (unsigned int j = 0; j < convexHullIndices.size(); j++)
		{
			if (convexHullIndices[j] == i)
			{
				isOnHull = true;
				break;
			}
		}

		// This is the concave (inner) corner
		if (!isOnHull)
		{
			concaveIndex = i;
			break;
		}
	}

	// Reorder the corners to start from the concave point
	vector<cv::Point2f> reorderedCorners;
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		reorderedCorners.push_back(corners[(concaveIndex + i)%corners.size()]);
	}

	// Update the original vector with reordered points
	corners = reorderedCorners;

	return true;
}

/// <summary>
/// Finds the code of a detected marker by parsing its binary pattern.
/// </summary>
/// <param name="img">Binary thresholded version of the original color frame where the marker was detected</param>
/// <param name="points">Normalized points that make up the shape of the marker</param>
/// <param name="corners">2D coordinates of the corners detected from the color frame</param>
/// <returns>The code of the detected marker if it was successfully found, -1 otherwise.</returns>
int MarkerDetector::GetCode(cv::Mat &img, vector<cv::Point2f> points, vector<cv::Point2f> corners)
{
	cv::Mat H, img2;
	int minX = 0, minY = 0;

	// Determine the width of the inner area of the marker in normalized space
	double markerInterior = NormalizedMarkerSize - 2 * NormalizedMarkerBorderSize;

	// Map normalized points to pixel coordinates in the target image
	// Shift from [-1, 1] to [0, 2], remove border, then scale to pixel resolution
	for (unsigned int i = 0; i < points.size(); i++)
	{
		points[i].x = static_cast<float>((points[i].x - NormalizedMarkerBorderSize + 1) * WarpedMarkerResolutionPerUnit);
		points[i].y = static_cast<float>((points[i].y - NormalizedMarkerBorderSize + 1) * WarpedMarkerResolutionPerUnit);
	}

	// Compute the homography to warp the marker to a front-facing, normalized square
	H = cv::findHomography(corners, points);

	// Warp the marker into a square image (markerInterior x markerInterior units)
	cv::warpPerspective(img, img2, H, cv::Size((int)(WarpedMarkerResolutionPerUnit * markerInterior), (int)(WarpedMarkerResolutionPerUnit * markerInterior)));

	// Divide the marker into a BitGridSize x BitGridSize grid
	int cellWidth = img2.cols / BitGridSize;
	int cellHeight = img2.rows / BitGridSize;
	int cellArea = cellWidth * cellHeight;

	std::vector<int> vals(BitGridSize * BitGridSize);

	// Use integral image for fast region averaging
	cv::Mat integral;
	cv::integral(img2, integral);
	
	// Analyze each grid cell to determine if it is black or white
	for (int i = 0; i < BitGridSize; i++)
	{
		for (int j = 0; j < BitGridSize; j++)
		{
			// Sum the pixel values inside the cell using the integral image
			int temp = integral.at<int>((i + 1) * cellWidth, (j + 1) * cellHeight);
			temp += integral.at<int>(i * cellWidth, j * cellHeight);
			temp -= integral.at<int>((i + 1) * cellWidth, j * cellHeight);
			temp -= integral.at<int>(i * cellWidth, (j + 1) * cellHeight);

			// Compute average intensity
			temp = temp / cellArea;

			// Binarize cell based on threshold
			if (temp < CodeDetectionBitThreshold)
				vals[j + i * BitGridSize] = 0;
			else if (temp >= CodeDetectionBitThreshold)
				vals[j + i * BitGridSize] = 1;
		}
	}


	// Decode the 3x3 marker pattern:
	// Top 4 bits [0-3] encode the marker code (white = 1, black = 0)
	// Bottom 4 bits [4-7] show the same code in inverted colors
	// Last bit [8] used for parity checking
	int ones = 0;
	int code = 0;

	for (int i = 0; i < 4; i++)
	{
		if (vals[i] == vals[i + 4])
			return -1; // Invalid: symmetric pattern (second ID is inverted version of the first)
		else if (vals[i] == 1)
		{
			code += static_cast<int>(pow(2, (double)(3 - i)));
			ones++;
		}
	}
	
	// Parity check using last bit (vals[8])
	bool even = (ones % 2 == 0);
	if ((even && vals[8] == 0) || (!even && vals[8] == 1))
		return -1; // Invalid: parity mismatch
	
	return code;
}

/// <summary>
/// Refines the given corner points by fitting lines to contour segments and computing their intersections.
/// This results in subpixel-accurate corner positions.
/// </summary>
/// <param name="corners">Input/output list of corner points which will be updated with refined positions</param>
/// <param name="contour">The original contour from which the corners were detected</param>
/// <param name="order">If true, assumes corners follow contour order; if false, assumes reversed order</param>
void MarkerDetector::RefineCornerPositions(vector<cv::Point2f> &corners, vector<cv::Point> contour, bool order)
{
	// Find the index in the contour where each corner lies
	int *contourIndices = new int[corners.size()];
	
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		for (unsigned int j = 0; j < contour.size(); j++)
		{
			if (corners[i].x == contour[j].x && corners[i].y == contour[j].y)
			{
				contourIndices[i] = j;
				break;
			}
		}
	}

	// For each edge between corners, extract the contour segment along that edge
	vector<cv::Point> *edgeSegments = new vector<cv::Point>[corners.size()];

	for (unsigned int i = 0; i < corners.size(); i++)
	{
		int startIdx, endIdx;

		if (order)
		{
			startIdx = contourIndices[i];
			endIdx = contourIndices[(i + 1) % corners.size()];
		}
		else
		{
			startIdx = contourIndices[(i + 1) % corners.size()];
			endIdx = contourIndices[i];
		}

		if (startIdx < endIdx)
		{
			// Simple case: extract directly
			edgeSegments[i].resize(endIdx - startIdx);
			copy(contour.begin() + startIdx, contour.begin() + endIdx, edgeSegments[i].begin());
		}
		else
		{
			// Wrap around the contour vector
			edgeSegments[i].resize(endIdx + contour.size() - startIdx);
			copy(contour.begin() + startIdx, contour.end(), edgeSegments[i].begin());
			copy(contour.begin(), contour.begin() + endIdx, edgeSegments[i].end() - endIdx);
		}
	}

	// Fit a line to each contour segment
	cv::Vec4f *fittedLines = new cv::Vec4f[corners.size()];
	
	for (unsigned int i = 0; i < corners.size(); i++)
	{
		cv::fitLine(edgeSegments[i], fittedLines[i], CV_DIST_L2, 0, 0.01, 0.01);
	}

	// Compute the intersection of each pair of fitted lines
	vector<cv::Point2f> refinedCorners;
	for (unsigned int i = corners.size() - 1; i < 2 * corners.size() - 1; i++)
	{
		// Intersect current and next line (wrapped around)
		refinedCorners.push_back(GetIntersection(fittedLines[(i + 1) % corners.size()], fittedLines[i % corners.size()]));
	}

	// Replace original corners with refined subpixel ones
	corners = refinedCorners;

	// Cleanup dynamic memory
	delete[] contourIndices;
	delete[] edgeSegments;
	delete[] fittedLines;
}

/// <summary>
/// Computes the intersection point of two lines represented in the Vec4f format used by cv::fitLine.
/// Each line is defined by a direction vector (dx, dy) and a point on the line (px, py).
/// </summary>
/// <param name="lin1">First line as cv::Vec4f: [dx1, dy1, px1, py1]</param>
/// <param name="lin2">Second line as cv::Vec4f: [dx2, dy2, px2, py2]</param>
/// <returns>The point of intersection as cv::Point2f</returns>
cv::Point2f MarkerDetector::GetIntersection(cv::Vec4f lin1, cv::Vec4f lin2)
{
	// Direction vector of line 1: (a1, a2)
	float a1 = lin1[0];
	float a2 = lin1[1];

	// Direction vector of line 2 (negated for setup): (-b1, -b2)
	float b1 = -lin2[0];
	float b2 = -lin2[1];

	// Difference in points between the lines: (c1, c2)
	float c1 = lin2[2] - lin1[2]; // px2 - px1
	float c2 = lin2[3] - lin1[3]; // py2 - py1

	// Set up system of linear equations: A * t = b
	// Line 1: p1 + t * dir1 == Line 2: p2 + s * dir2
	// Therefore:
	// [a1 b1] [t] = [c1]
	// [a2 b2] [s]   [c2]
	cv::Mat A(2, 2, CV_32F);
	A.at<float>(0, 0) = a1;
	A.at<float>(0, 1) = b1;
	A.at<float>(1, 0) = a2;
	A.at<float>(1, 1) = b2;

	cv::Mat b(2, 1, CV_32F);
	b.at<float>(0, 0) = c1;
	b.at<float>(1, 0) = c2;
	
	cv::Mat dst(2, 1, CV_32F);

	// Solve linear system A * [t, s]^T = [c1, c2]^T
	cv::solve(A, b, dst);

	// Compute intersection point using line 1: point + t * direction
	float t = dst.at<float>(0, 0);
	float intersectionX = t * lin1[0] + lin1[2]; // dx1 * t + px1
	float intersectionY = t * lin1[1] + lin1[3]; // dy1 * t + py1

	return cv::Point2f(intersectionX, intersectionY);
}

/// <summary>
/// Calculates the area of a detected marker using the convex hull of its corners.
/// </summary>
/// <param name="marker">Information of the marker for which we want to calculate the area</param>
/// <returns>The area of the marker</returns>
double MarkerDetector::GetMarkerArea(MarkerInfo &marker)
{
	// Convert the custom Corner type to OpenCV Point2f
	vector<cv::Point2f> cvCorners(NumMarkerCorners);
	for (int i = 0; i < NumMarkerCorners; i++)
	{
		cvCorners[i] = cv::Point2f(marker.Corners[i].X, marker.Corners[i].Y);
	}

	// Compute the convex hull of the marker's corner points
	cv::Mat hull;
	cv::convexHull(cvCorners, hull);

	// Return the area of the convex hull polygon
	return cv::contourArea(hull);
}