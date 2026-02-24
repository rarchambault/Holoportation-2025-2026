/***************************************************************************\

Module Name:  PointCloudTransferSocket.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the socket used to send point cloud data to connected clients.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Net.Sockets;

namespace LiveScanServer
{
    public class PointCloudTransferSocket : TransferSocketBase
    {
        // Set the range and determine the minimal precision to make sure position values fit in a byte
        private const float Range = 0.3f; // Range of allowed values for each axis, in meters
        private const float HalfRange = Range / 2.0f;
        private const float MinPrecision = Range / 255; // Min precision (max resolution) with the range and the range of values in a byte (255)

        // Parameters used to find the scale
        private const short MinScale = 400;
        private const short MaxScale = (short)(1 / MinPrecision);
        private const float ScaleFnOffset = 6700.0f;
        private const float ScaleFnFactor = -500.0f;
        private const float xRangeCenter = 0.0f;
        private const float yRangeCenter = 0.0f;
        private const float zRangeCenter = HalfRange;

        public PointCloudTransferSocket(TcpClient clientSocket) : base(clientSocket) { }

        public void SendPointCloud(List<float> vertices, List<byte> colors)
        {
            // Receive 1 byte to check that the receiver has requested a new frame
            byte[] requestBuffer = Receive(1);

            while (requestBuffer.Length != 0)
            {
                if (requestBuffer[0] == 0)
                {
                    // Determine the scale (resolution) dynamically based on the number of points
                    int originalVertexCount = vertices.Count / 3;
                    short scale = DetermineScale(originalVertexCount);

                    // Filter out points which map to the same reduced location once the scale reduction is applied
                    HashSet<(byte, byte, byte)> uniquePoints = new HashSet<(byte, byte, byte)>();
                    List<byte> filteredVertices = new List<byte>();
                    List<byte> filteredColors = new List<byte>();

                    for (int i = 0; i < vertices.Count; i += 3)
                    {
                        float x = vertices[i];
                        float y = vertices[i + 1];
                        float z = vertices[i + 2];

                        // Filter out points which do not fit in the range of values allowed in one byte
                        if (Math.Abs(x - xRangeCenter) > HalfRange || Math.Abs(xRangeCenter - x) > HalfRange
                            || Math.Abs(y - yRangeCenter) > HalfRange || Math.Abs(yRangeCenter - y) > HalfRange
                            || Math.Abs(z - zRangeCenter) > HalfRange || Math.Abs(zRangeCenter - z) > HalfRange)
                        {
                            continue;
                        }

                        // Encode each float position to a byte, using the scale to reduce the resolution
                        byte bx = EncodeFloatToByte(x, xRangeCenter, scale);
                        byte by = EncodeFloatToByte(y, yRangeCenter, scale);
                        byte bz = EncodeFloatToByte(z, zRangeCenter, scale);

                        var point = (bx, by, bz);

                        // If no other point mapped to this reduced position yet, add the point to the filtered result
                        if (uniquePoints.Add(point))
                        {
                            filteredVertices.Add(bx);
                            filteredVertices.Add(by);
                            filteredVertices.Add(bz);

                            // Copy corresponding RGB color
                            int colorIndex = i;
                            filteredColors.Add(colors[colorIndex]);
                            filteredColors.Add(colors[colorIndex + 1]);
                            filteredColors.Add(colors[colorIndex + 2]);
                        }
                    }

                    int numVerticesToSend = filteredVertices.Count / 3;
                    byte[] buffer = new byte[sizeof(byte) * filteredVertices.Count];
                    Buffer.BlockCopy(filteredVertices.ToArray(), 0, buffer, 0, buffer.Length);

                    try
                    {
                        // Send the scale first
                        byte[] scaleBytes = BitConverter.GetBytes(scale);
                        socket.GetStream().Write(scaleBytes, 0, scaleBytes.Length);

                        // Send number of vertices
                        WriteInt(numVerticesToSend);

                        // Send vertices and colors
                        socket.GetStream().Write(buffer, 0, buffer.Length);
                        socket.GetStream().Write(filteredColors.ToArray(), 0, filteredColors.Count);
                    }
                    catch (Exception ex)
                    {
                    }
                }

                // Receive a new request byte to make sure the receiver is ready to receive
                requestBuffer = Receive(1);
            }
        }

        // Determine scale based on number of vertices
        private short DetermineScale(int vertexCount)
        {
            if (vertexCount <= 0) return MaxScale;
            short scale = (short)Math.Truncate(ScaleFnOffset + ScaleFnFactor * Math.Log(vertexCount));
            return Math.Min(MaxScale, Math.Max(scale, MinScale)); // Clamp between min and max acceptable scales
        }

        private byte EncodeFloatToByte(float value, float rangeCenter, float scale)
        {
            float result = (value + HalfRange - rangeCenter) * scale; // Use the computed scale to reduce the resolution
            return (byte)Math.Min(255, Math.Max(result, 0)); // Clamp between 0 and 255 to make sure it fits in a byte
        }
    }
}