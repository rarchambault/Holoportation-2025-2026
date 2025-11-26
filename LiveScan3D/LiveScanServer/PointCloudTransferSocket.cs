/***************************************************************************\

Module Name:  PointCloudTransferSocket.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the socket used to send mesh data (vertices + colors + indices)
to connected clients.

Originally, this class sent a compressed point cloud only (quantized to bytes).
We now send full-precision floats for vertices and ints for triangle indices,
so Unity/HoloLens can reconstruct a proper Mesh.

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
        public PointCloudTransferSocket(TcpClient clientSocket) : base(clientSocket) { }

        /// <summary>
        /// Sends a full mesh (vertices + colors + triangle indices) to the client.
        /// Protocol:
        ///   [int] vertexCount   (number of float3 vertices = vertices.Count / 3)
        ///   [int] colorCount    (number of bytes in colors = colors.Count)
        ///   [int] indexCount    (number of ints in indices = indices.Count)
        ///
        ///   [float] * vertexCount * 3   (X, Y, Z per vertex)
        ///   [byte]  * colorCount        (R, G, B per vertex)
        ///   [int]   * indexCount        (triangle index buffer)
        ///
        /// As before, the receiver must send a 1-byte request (0) to get a new frame.
        /// </summary>
        public void SendPointCloud(List<float> vertices, List<byte> colors, List<int> indices)
        {
            // Wait for the client to request a new frame (1-byte handshake).
            byte[] requestBuffer = Receive(1);

            while (requestBuffer.Length != 0)
            {
                if (requestBuffer[0] == 0)
                {
                    int vertexCount = vertices.Count / 3;
                    int colorCount = colors.Count;
                    int indexCount = indices.Count;

                    // Safety: if colors are less than vertexCount*3, clamp vertexCount
                    if (colorCount < vertexCount * 3)
                    {
                        vertexCount = colorCount / 3;
                    }

                    try
                    {
                        NetworkStream stream = socket.GetStream();

                        // --- HEADER ---
                        // 3 ints: vertexCount, colorCount, indexCount
                        byte[] header = new byte[sizeof(int) * 3];
                        Buffer.BlockCopy(BitConverter.GetBytes(vertexCount), 0, header, 0, 4);
                        Buffer.BlockCopy(BitConverter.GetBytes(colorCount), 0, header, 4, 4);
                        Buffer.BlockCopy(BitConverter.GetBytes(indexCount), 0, header, 8, 4);
                        stream.Write(header, 0, header.Length);

                        // --- VERTICES (floats) ---
                        // Only send the number of vertices that match the clamped vertexCount
                        int vertsToSendCount = vertexCount * 3;
                        float[] vertsArray = vertices.ToArray();
                        byte[] vertsBytes = new byte[vertsToSendCount * sizeof(float)];
                        Buffer.BlockCopy(vertsArray, 0, vertsBytes, 0, vertsBytes.Length);
                        stream.Write(vertsBytes, 0, vertsBytes.Length);

                        // --- COLORS (bytes) ---
                        // We send colorCount bytes (R, G, B per vertex)
                        byte[] colorsArray = colors.ToArray();
                        stream.Write(colorsArray, 0, colorCount);

                        // --- INDICES (ints) ---
                        int[] indexArray = indices.ToArray();
                        byte[] indexBytes = new byte[indexCount * sizeof(int)];
                        Buffer.BlockCopy(indexArray, 0, indexBytes, 0, indexBytes.Length);
                        stream.Write(indexBytes, 0, indexBytes.Length);
                    }
                    catch (Exception)
                    {
                        // You can log if needed
                    }
                }

                // Ask again for the next 1-byte request to know if the client wants another mesh
                requestBuffer = Receive(1);
            }
        }
    }
}
