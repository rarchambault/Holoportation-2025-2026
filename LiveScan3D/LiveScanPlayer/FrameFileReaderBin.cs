/***************************************************************************\

Module Name:  FrameFileReaderBin.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module allows reading point cloud frames from .bin files and handles
playback functions.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Text;
using System.IO;

namespace LiveScanPlayer
{
    class FrameFileReaderBin : IFrameFileReader
    {
        private BinaryReader binaryReader;
        private int currentFrameIdx = 0;
        private string filename;

        public int FrameIdx
        {
            get
            {
                return currentFrameIdx;
            }
            set
            {
                JumpToFrame(value);
            }
        }

        public FrameFileReaderBin(string filename)
        {
            this.filename = filename;
            binaryReader = new BinaryReader(File.Open(this.filename, FileMode.Open));
        }

        ~FrameFileReaderBin()
        {
            binaryReader.Dispose();
        }

        public void ReadFrame(List<float> vertices, List<byte> colors)
        {
            if (binaryReader.BaseStream.Position == binaryReader.BaseStream.Length)
                Rewind();

            // Read header lines
            string[] lineParts = ReadLine().Split(' ');
            int pointCount = Int32.Parse(lineParts[1]);

            lineParts = ReadLine().Split(' ');
            int frameTimestamp = Int32.Parse(lineParts[1]); // Currently unused, but parsed

            // Temporary buffers to hold raw frame data
            short[] tempVertices = new short[3 * pointCount];
            byte[] tempColors = new byte[4 * pointCount]; // RGBA, though A is skipped in final output

            int bytesPerVertexPoint = 3 * sizeof(short); // x, y, z
            int bytesPerColorPoint = 4 * sizeof(byte); // r, g, b, a
            int bytesPerPoint = bytesPerVertexPoint + bytesPerColorPoint;

            // Read the entire frame as a block of bytes
            byte[] frameData = binaryReader.ReadBytes(bytesPerPoint * pointCount);

            // Handle incomplete frame: rewind and retry
            if (frameData.Length < bytesPerPoint * pointCount)
            {
                Rewind();
                ReadFrame(vertices, colors);
                return;
            }

            // Split raw byte data into vertex and color buffers
            int vertexDataSize = pointCount * bytesPerVertexPoint;
            int colorDataSize = pointCount * bytesPerColorPoint;

            Buffer.BlockCopy(frameData, 0, tempVertices, 0, vertexDataSize);
            Buffer.BlockCopy(frameData, vertexDataSize, tempColors, 0, colorDataSize);

            // Convert and store vertex and RGB color data
            for (int i = 0; i < pointCount; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    vertices.Add(tempVertices[3 * i + j] / 1000.0f); // Convert to float (meters)
                    colors.Add(tempColors[4 * i + j]); // Skip alpha
                }                    
            }

            // Skip 1 extra byte
            binaryReader.ReadByte();

            currentFrameIdx++;
        }

        public void JumpToFrame(int frameIdx)
        {
            Rewind();

            for (int i = 0; i < frameIdx; i++)
            {
                List<float> vertices = new List<float>();
                List<byte> colors = new List<byte>();
                ReadFrame(vertices, colors);
            }
        }

        public void Rewind()
        {
            currentFrameIdx = 0;
            binaryReader.BaseStream.Seek(0, SeekOrigin.Begin);
        }

        private string ReadLine()
        {
            StringBuilder builder = new StringBuilder();
            byte buffer = binaryReader.ReadByte();

            while (buffer != '\n')
            {
                builder.Append((char)buffer);
                buffer = binaryReader.ReadByte();
            }

            return builder.ToString();
        }
    }
}
