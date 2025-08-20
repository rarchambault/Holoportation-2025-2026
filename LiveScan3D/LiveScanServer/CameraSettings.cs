/***************************************************************************\

Module Name:  CameraSettings.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module represents all settings which can be changed by the user to modify
camera data capturing and the point cloud reconstruction in general.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;

namespace LiveScanServer
{
    [Serializable]
    public class CameraSettings
    {
        public float[] MinBounds = new float[3];
        public float[] MaxBounds = new float[3];

        public bool Filter = false;
        public int NumFilterNeighbors = 10;
        public float FilterThreshold = 0.1f;

        public BindingList<MarkerPose> MarkerPoses = new BindingList<MarkerPose>();

        public int NumICPIterations = 10;
        public int NumRefineIterations = 2;

        public bool MergeScansForSave = true;
        public bool SaveAsBinaryPLY = true;

        public bool IsSyncEnabled = false;

        public bool IsAutoExposureEnabled = true;
        public int ExposureStep = 200;

        public CameraSettings()
        {
            MinBounds[0] = -5.0f;
            MinBounds[1] = -5.0f;
            MinBounds[2] = -5.0f;

            MaxBounds[0] = 5.0f;
            MaxBounds[1] = 5.0f;
            MaxBounds[2] = 5.0f;
        }

        /// <summary>
        /// Converts the current C# settings object into a format compatible with C++ for communication with the LiveScanClient processes
        /// </summary>
        /// <param name="markerHandle">A handle to the pinned memory block containing marker pose data</param>
        /// <returns>A populated NativeCameraSettings struct</returns>
        public unsafe NativeCameraSettings ToNative(out GCHandle markerHandle)
        {
            // Populate the easiest to convert parameters of the struct with local data
            NativeCameraSettings native = new NativeCameraSettings
            {
                MinBounds = MinBounds,
                MaxBounds = MaxBounds,
                Filter = Filter,
                NumFilterNeighbors = NumFilterNeighbors,
                FilterThreshold = FilterThreshold,
                NumMarkers = MarkerPoses.Count,
                IsAutoExposureEnabled = IsAutoExposureEnabled,
                ExposureStep = ExposureStep
            };

            // Allocate array for the markers
            var nativeMarkers = new NativeMarkerPose[MarkerPoses.Count];

            for (int i = 0; i < nativeMarkers.Length; i++)
            {
                var src = MarkerPoses[i];

                unsafe
                {
                    fixed (float* rDest = nativeMarkers[i].R)
                    fixed (float* tDest = nativeMarkers[i].T)
                    {
                        for (int j = 0; j < 9; j++)
                            rDest[j] = src.Pose.R.Cast<float>().ToArray()[j];

                        for (int j = 0; j < 3; j++)
                            tDest[j] = src.Pose.T[j];
                    }

                    nativeMarkers[i].MarkerId = src.Id;
                }
            }

            // Pin nativeMarkers
            markerHandle = GCHandle.Alloc(nativeMarkers, GCHandleType.Pinned);
            native.MarkerPoses = markerHandle.AddrOfPinnedObject();

            return native;
        }
    }
}
