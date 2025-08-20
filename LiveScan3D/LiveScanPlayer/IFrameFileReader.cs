/***************************************************************************\

Module Name:  IFrameFileReader.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is an interface defining properties and methods which all frame 
file readers must implement.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System.Collections.Generic;

namespace LiveScanPlayer
{
    interface IFrameFileReader
    {
        int FrameIdx
        {
            get;
            set;
        }

        /// <summary>
        /// Reads the next frame's vertex and color data from the current file into
        /// the provided lists.
        /// </summary>
        /// <param name="vertices">List to store the 3D vertex coordinates (x, y, z)</param>
        /// <param name="colors">List to store the RGB color bytes for each vertex</param>
        void ReadFrame(List<float> vertices, List<byte> colors);

        void JumpToFrame(int frameIdx);

        void Rewind();
    }
}
