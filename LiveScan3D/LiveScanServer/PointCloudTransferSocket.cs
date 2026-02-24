using System;
using System.Collections.Generic;
using System.Net.Sockets;

namespace LiveScanServer
{
    public class PointCloudTransferSocket : TransferSocketBase
    {
        // EXPANDED RANGE: 3 meters instead of 30cm! No more clipping the basketball.
        private const float Range = 3.0f;
        private const float HalfRange = Range / 2.0f;

        // 16-BIT PRECISION
        private const float MinPrecision = Range / 65535f;

        private const float MinScale = 400f;
        private const float MaxScale = 1f / MinPrecision;

        private const float xRangeCenter = 0.0f;
        private const float yRangeCenter = 0.0f;
        private const float zRangeCenter = 1.0f; // Camera usually looks forward 1m

        public PointCloudTransferSocket(TcpClient clientSocket) : base(clientSocket) { }

        public void SendPointCloud(List<float> vertices, List<byte> colors, List<int> indices)
        {
            byte[] requestBuffer = Receive(1);

            while (requestBuffer.Length != 0)
            {
                if (requestBuffer[0] == 0)
                {
                    int vertexCount = vertices.Count / 3;
                    float scale = MaxScale; // Always use max 16-bit precision

                    // 1:1 PASS-THROUGH (No deleted vertices, no destroyed triangles!)
                    byte[] vertexBuffer = new byte[vertexCount * 3 * sizeof(ushort)];

                    for (int i = 0; i < vertices.Count; i += 3)
                    {
                        ushort bx = EncodeFloatToUShort(vertices[i], xRangeCenter, scale);
                        ushort by = EncodeFloatToUShort(vertices[i + 1], yRangeCenter, scale);
                        ushort bz = EncodeFloatToUShort(vertices[i + 2], zRangeCenter, scale);

                        int byteIndex = (i / 3) * 6;
                        vertexBuffer[byteIndex] = (byte)(bx & 0xFF);
                        vertexBuffer[byteIndex + 1] = (byte)(bx >> 8);
                        vertexBuffer[byteIndex + 2] = (byte)(by & 0xFF);
                        vertexBuffer[byteIndex + 3] = (byte)(by >> 8);
                        vertexBuffer[byteIndex + 4] = (byte)(bz & 0xFF);
                        vertexBuffer[byteIndex + 5] = (byte)(bz >> 8);
                    }

                    // Because we didn't delete any vertices, we can send the exact C++ triangles!
                    int numTrianglesToSend = indices.Count / 3;
                    byte[] indexBuffer = new byte[indices.Count * sizeof(int)];
                    Buffer.BlockCopy(indices.ToArray(), 0, indexBuffer, 0, indexBuffer.Length);

                    try
                    {
                        byte[] scaleBytes = BitConverter.GetBytes(scale);
                        socket.GetStream().Write(scaleBytes, 0, scaleBytes.Length);

                        WriteInt(vertexCount);
                        socket.GetStream().Write(vertexBuffer, 0, vertexBuffer.Length);
                        socket.GetStream().Write(colors.ToArray(), 0, colors.Count);

                        WriteInt(numTrianglesToSend);
                        socket.GetStream().Write(indexBuffer, 0, indexBuffer.Length);
                    }
                    catch (Exception ex)
                    {
                        // Logger.Log("Error while sending point cloud data: " + ex.Message);
                    }
                }
                requestBuffer = Receive(1);
            }
        }

        private ushort EncodeFloatToUShort(float value, float rangeCenter, float scale)
        {
            float result = (value + HalfRange - rangeCenter) * scale;
            if (result < 0f) result = 0f;
            if (result > 65535f) result = 65535f;
            return (ushort)result;
        }
    }
}