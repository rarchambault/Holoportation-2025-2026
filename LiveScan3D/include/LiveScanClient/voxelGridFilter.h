/***************************************************************************\

Module Name:  VoxelGridFilter.h
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module reduces points according to a voxel grid with customizeable size
such that there remains only one point per grid cell.

\***************************************************************************/

#pragma once

#include <vector>
#include <cstdint>

class VoxelGridFilter {
public:
    VoxelGridFilter(float voxelSize,
        float centerX, float centerY, float centerZ,
        float halfRange);

    void Reset();
    bool Insert(float x, float y, float z);

private:
    size_t gridSizeX, gridSizeY, gridSizeZ;
    float invVoxelSize;

    float minX, minY, minZ;
    std::vector<bool> voxelGrid;

    size_t VoxelIndex(int x, int y, int z) const;
};