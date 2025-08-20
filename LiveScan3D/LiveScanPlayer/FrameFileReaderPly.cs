/***************************************************************************\

Module Name:  FrameFileReaderPly.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module allows reading point cloud frames from .ply files and handles
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
    class FrameFileReaderPly : IFrameFileReader
    {
        private string[] filenames;
        private int currentFrameIdx = 0;

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

        public FrameFileReaderPly(string[] filenames)
        {
            this.filenames = filenames;
        }

        public void ReadFrame(List<float> vertices, List<byte> colors)
        {
            // Open the current frame file for binary reading
            using (BinaryReader reader = new BinaryReader(new FileStream(filenames[currentFrameIdx], FileMode.Open)))
            {
                bool hasAlphaChannel = false;
                string line = ReadLine(reader);

                // Find the line indicating number of vertices
                while (!line.Contains("element vertex"))
                {
                    line = ReadLine(reader);
                }

                // Parse number of vertices
                string[] tokens = line.Split(' ');
                int vertexCount = Int32.Parse(tokens[2]);

                // Continue parsing the header, checking for alpha channel
                while (!line.Contains("end_header"))
                {
                    if (line.Contains("alpha"))
                        hasAlphaChannel = true;

                    line = ReadLine(reader);
                }

                // Read vertex and color data
                for (int i = 0; i < vertexCount; i++)
                {
                    // Read 3 floats for the vertex position
                    for (int j = 0; j < 3; j++)
                    {
                        vertices.Add(reader.ReadSingle());
                    }

                    // Read 3 bytes for RGB color
                    for (int j = 0; j < 3; j++)
                    {
                        colors.Add(reader.ReadByte());
                    }

                    // Skip alpha byte if present
                    if (hasAlphaChannel)
                        reader.ReadByte();
                }
            }

            // Move to the next frame, looping if at the end
            currentFrameIdx = (currentFrameIdx + 1) % filenames.Length;
        }

        public void JumpToFrame(int frameIdx)
        {
            currentFrameIdx = frameIdx;

            if (currentFrameIdx >= filenames.Length)
                currentFrameIdx = 0;
        }

        public void Rewind()
        {
            currentFrameIdx = 0;
        }

        private string ReadLine(BinaryReader binaryReader)
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
