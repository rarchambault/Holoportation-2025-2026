/***************************************************************************\

Module Name:  Utils.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module contains some definitions for structs and small data classes which
are used in several other modules of the LiveScanServer project.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;

namespace LiveScanServer
{
    public enum SyncState
    {
        Subordinate,
        Master,
        Standalone,
        Unknown
    }

    public struct Point3f
    {
        public float X;
        public float Y;
        public float Z;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct Point3s
    {
        public short X;
        public short Y;
        public short Z;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct RGB
    {
        public byte Blue;
        public byte Green;
        public byte Red;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct NativeMarkerPose
    {
        public int MarkerId;
        public fixed float R[9];
        public fixed float T[3];
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct NativeCameraSettings
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] MinBounds;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] MaxBounds;

        [MarshalAs(UnmanagedType.I1)]
        public bool Filter;
        public int NumFilterNeighbors;
        public float FilterThreshold;

        public IntPtr MarkerPoses;
        public int NumMarkers;

        [MarshalAs(UnmanagedType.I1)]
        public bool IsAutoExposureEnabled;
        public int ExposureStep;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct NativeAffineTransform
    {
        public fixed float R[9]; // 3x3 matrix
        public fixed float T[3]; // translation vector
    }

    [Serializable]
    public class AffineTransform
    {
        public float[,] R = new float[3, 3];
        public float[] T = new float[3];

        public AffineTransform()
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (i == j)
                        R[i, j] = 1;
                    else
                        R[i, j] = 0;
                }
                T[i] = 0;
            }
        }

        /// <summary>
        /// Converts a C# AffineTransform object into a format compatible with C++ for communication with the LiveScanClient processes
        /// </summary>
        /// <returns>A populated NativeAffineTransform struct</returns>
        public unsafe NativeAffineTransform ToNative()
        {
            NativeAffineTransform native = new NativeAffineTransform();

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    native.R[i * 3 + j] = this.R[i, j];
                }
            }

            for (int i = 0; i < 3; i++)
            {
                native.T[i] = this.T[i];
            }

            return native;
        }
    }

    [Serializable]
    public class MarkerPose
    {
        public AffineTransform Pose = new AffineTransform();
        public int Id = -1;

        public MarkerPose()
        {
            UpdateRotationMatrix();
        }

        public void SetOrientation(float X, float Y, float Z)
        {
            r[0] = X;
            r[1] = Y;
            r[2] = Z;

            UpdateRotationMatrix();
        }

        public void GetOrientation(out float X, out float Y, out float Z)
        {
            X = r[0];
            Y = r[1];
            Z = r[2];
        }

        private void UpdateRotationMatrix()
        {
            float radX = r[0] * (float)Math.PI / 180.0f;
            float radY = r[1] * (float)Math.PI / 180.0f;
            float radZ = r[2] * (float)Math.PI / 180.0f;

            float c1 = (float)Math.Cos(radZ);
            float c2 = (float)Math.Cos(radY);
            float c3 = (float)Math.Cos(radX);
            float s1 = (float)Math.Sin(radZ);
            float s2 = (float)Math.Sin(radY);
            float s3 = (float)Math.Sin(radX);

            // Z Y X rotation
            Pose.R[0, 0] = c1 * c2;
            Pose.R[0, 1] = c1 * s2 * s3 - c3 * s1;
            Pose.R[0, 2] = s1 * s3 + c1 * c3 * s2;
            Pose.R[1, 0] = c2 * s1;
            Pose.R[1, 1] = c1 * c3 + s1 * s2 * s3;
            Pose.R[1, 2] = c3 * s1 * s2 - c1 * s3;
            Pose.R[2, 0] = -s2;
            Pose.R[2, 1] = c2 * s3;
            Pose.R[2, 2] = c2 * c3;
        }

        private float[] r = new float[3];
    }

    public class DocumentInfo
    {
        public List<byte> Data { get; set; } = new List<byte>();
        public float Score { get; set; }
        public float Width { get; set; }
        public float Height { get; set; }
        public bool IsNew { get; set; }
    }

    public class Utils
    {
        public static void saveToPly(string filename, List<Single> vertices, List<byte> colors, bool binary)
        {
            int nVertices = vertices.Count / 3;

            FileStream fileStream = File.Open(filename, FileMode.Create);

            StreamWriter streamWriter = new StreamWriter(fileStream);
            BinaryWriter binaryWriter = new BinaryWriter(fileStream);

            // PLY file header is written here.
            if (binary)
                streamWriter.WriteLine("ply\nformat binary_little_endian 1.0");
            else
                streamWriter.WriteLine("ply\nformat ascii 1.0\n");
            streamWriter.Write("element vertex " + nVertices.ToString() + "\n");
            streamWriter.Write("property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n");
            streamWriter.Flush();

            // Vertex and color data are written here.
            if (binary)
            {
                for (int j = 0; j < vertices.Count / 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                        binaryWriter.Write(vertices[j * 3 + k]);
                    for (int k = 0; k < 3; k++)
                    {
                        byte temp = colors[j * 3 + k];
                        binaryWriter.Write(temp);
                    }
                }
            }
            else
            {
                for (int j = 0; j < vertices.Count / 3; j++)
                {
                    string s = "";
                    for (int k = 0; k < 3; k++)
                        s += vertices[j * 3 + k].ToString(CultureInfo.InvariantCulture) + " ";
                    for (int k = 0; k < 3; k++)
                        s += colors[j * 3 + k].ToString(CultureInfo.InvariantCulture) + " ";
                    streamWriter.WriteLine(s);
                }
            }
            streamWriter.Flush();
            binaryWriter.Flush();
            fileStream.Close();
        }
    }
}