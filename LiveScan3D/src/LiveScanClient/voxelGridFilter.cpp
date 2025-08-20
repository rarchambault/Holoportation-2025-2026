/***************************************************************************\

Module Name:  VoxelGridFilter.cpp
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module reduces points according to a voxel grid with customizeable size
such that there remains only one point per grid cell.

\***************************************************************************/

#include "voxelGridFilter.h"
#include <cmath>
#include <stdexcept>

// Constructor for initializing the voxel grid filter
VoxelGridFilter::VoxelGridFilter(float voxelSize, float centerX, float centerY, float centerZ, float halfRange) {
    if (voxelSize <= 0.0f) throw std::invalid_argument("Voxel size must be positive.");

    // Store the inverse of the voxel size for faster computation later
    invVoxelSize = 1.0f / voxelSize;

    // Compute bounds
    minX = centerX - halfRange;
    minY = centerY - halfRange;
    minZ = centerZ - halfRange;

    // Compute the number of voxels in each dimension
    gridSizeX = static_cast<size_t>(std::ceil((halfRange * 2) * invVoxelSize));
    gridSizeY = static_cast<size_t>(std::ceil((halfRange * 2) * invVoxelSize));
    gridSizeZ = static_cast<size_t>(std::ceil((halfRange * 2) * invVoxelSize));

    // Allocate and initialize the voxel grid (as a 1D boolean array)
    size_t totalSize = gridSizeX * gridSizeY * gridSizeZ;
    voxelGrid.resize(totalSize, false);
}

// Clears the voxel grid by setting all entries to false
void VoxelGridFilter::Reset() {
    std::fill(voxelGrid.begin(), voxelGrid.end(), false);
}

// Inserts a point into the voxel grid if it hasn't been inserted before
bool VoxelGridFilter::Insert(float x, float y, float z) {
    // Compute voxel indices for the given point
    int ix = static_cast<int>((x - minX) * invVoxelSize);
    int iy = static_cast<int>((y - minY) * invVoxelSize);
    int iz = static_cast<int>((z - minZ) * invVoxelSize);

    // Check if the point lies outside the voxel grid bounds
    if (ix < 0 || iy < 0 || iz < 0 ||
        ix >= static_cast<int>(gridSizeX) ||
        iy >= static_cast<int>(gridSizeY) ||
        iz >= static_cast<int>(gridSizeZ)) {
        return false;
    }

    // Compute the 1D index for the voxel grid
    size_t idx = VoxelIndex(ix, iy, iz);

    // If the voxel has already been occupied, return false
    if (voxelGrid[idx]) return false;

    // Mark voxel as occupied and return true
    voxelGrid[idx] = true;
    return true;
}

// Converts 3D voxel indices into a 1D index for internal storage
size_t VoxelGridFilter::VoxelIndex(int x, int y, int z) const {
    return static_cast<size_t>(z) * gridSizeY * gridSizeX +
        static_cast<size_t>(y) * gridSizeX +
        static_cast<size_t>(x);
}
