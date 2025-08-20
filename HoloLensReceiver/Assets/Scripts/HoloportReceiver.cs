/***************************************************************************\

Module Name:  HoloportReceiver.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module receives point clouds and documents from a TCP server and sends 
them to the appropriate renderers.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;

public class HoloportReceiver : MonoBehaviour
{
    public string ServerIPAddress = "127.0.0.1";
    public bool IsServerIPAddressSet = false;
    public int PointCloudPort = 48002;
    public int DocumentPort = 48003;
    public float ConnectionRetryInterval = 10.0f;

    // Parameters used to deserialize point clouds
    private const int PointXYZDataSize = 3; // 3 bytes for (x, y, z) positions
    private const int PointRGBDataSize = 3; // 3 bytes for (r, g, b) colors
    private const float Range = 0.3f;
    private const float HalfRange = Range / 2.0f;
    private const float XRangeCenter = 0.0f;
    private const float YRangeCenter = 0.0f;
    private const float ZRangeCenter = HalfRange;

    private TcpClient pointCloudClient;
    private bool isPointCloudClientConnected = false;
    private bool isPointCloudClientConnecting = false;
    private float pointCloudConnectionTimer = 0.0f;

    private TcpClient documentClient;
    private bool isDocumentClientConnected = false;
    private bool isDocumentClientConnecting = false;
    private float documentConnectionTimer = 0.0f;

    private PointCloudRenderer pointCloudRenderer;
    private DocumentRenderer documentRenderer;

    private void Start()
    {
        pointCloudRenderer = GetComponent<PointCloudRenderer>();
        documentRenderer = GetComponent<DocumentRenderer>();
    }

    void Update()
    {
        if (!isPointCloudClientConnecting && IsServerIPAddressSet)
        {
            isPointCloudClientConnecting = true;
            ConnectPointCloudClient();
        }

        if (!isDocumentClientConnecting && IsServerIPAddressSet)
        {
            isDocumentClientConnecting = true;
            ConnectDocumentClient();
        }

        if (isPointCloudClientConnecting && !isPointCloudClientConnected)
        {
            pointCloudConnectionTimer += Time.deltaTime;

            if (pointCloudConnectionTimer >= ConnectionRetryInterval)
            {
                // Retry connecting at regular intervals if connection failed
                ConnectPointCloudClient();
                pointCloudConnectionTimer = 0.0f;
            }
        }

        if (isDocumentClientConnecting && !isDocumentClientConnected)
        {
            documentConnectionTimer += Time.deltaTime;

            if (documentConnectionTimer >= ConnectionRetryInterval)
            {
                // Retry connecting at regular intervals if connection failed
                ConnectDocumentClient();
                documentConnectionTimer = 0.0f;
            }
        }
    }

    private async void ConnectPointCloudClient()
    {
        pointCloudClient = new TcpClient();

        try
        {
            await pointCloudClient.ConnectAsync(ServerIPAddress, PointCloudPort);
            isPointCloudClientConnected = true;
            ReceivePointClouds();
            gameObject.GetComponent<MeshRenderer>().enabled = true;
        }
        catch (Exception e)
        {
            Debug.LogError("Connection to LiveScan3D point cloud server failed: " + e.Message);
        }
    }

    private async void ConnectDocumentClient()
    {
        documentClient = new TcpClient();

        try
        {
            await documentClient.ConnectAsync(ServerIPAddress, DocumentPort);
            isDocumentClientConnected = true;
            ReceiveDocuments();
        }
        catch (Exception e)
        {
            Debug.LogError("Connection to LiveScan3D document server failed: " + e.Message);
        }
    }

    private async void ReceivePointClouds()
    {
        while (isPointCloudClientConnected && pointCloudClient.Connected)
        {
            try
            {
                // Request a new frame
                await pointCloudClient.GetStream().WriteAsync(new byte[] { 0 });

                // Read scale factor (short)
                short scale = await ReadShortAsync(pointCloudClient);

                // Read number of points (4 bytes)
                int numPoints = await ReadIntAsync(pointCloudClient);

                Debug.Log($"Received {numPoints} points with scale {scale}");

                // Initialize arrays for vertices and colors data
                int verticesSize = PointXYZDataSize * numPoints;
                int colorsSize = PointRGBDataSize * numPoints;

                byte[] verticesBytes = new byte[verticesSize];
                byte[] colorsBytes = new byte[colorsSize];

                // Read vertices data
                int numBytesRead = 0;

                while (numBytesRead < verticesSize)
                    numBytesRead += await pointCloudClient.GetStream().ReadAsync(verticesBytes, numBytesRead, Math.Min(verticesSize - numBytesRead, 64000));

                // Read color data
                numBytesRead = 0;

                while (numBytesRead < colorsSize)
                    numBytesRead += await pointCloudClient.GetStream().ReadAsync(colorsBytes, numBytesRead, Math.Min(colorsSize - numBytesRead, 64000));

                Vector3[] vertices;
                Color32[] colors;

                DeserializePointCloud(numPoints, scale, verticesBytes, colorsBytes, out vertices, out colors);
                pointCloudRenderer.EnqueuePointCloud(scale, vertices, colors);
            }
            catch (Exception)
            {
                if (!pointCloudClient.Connected && isPointCloudClientConnected)
                {
                    // The socket was disconnected while trying to receive a point cloud; close the socket and hide the renderer
                    isPointCloudClientConnecting = false;
                    isPointCloudClientConnected = false;
                    pointCloudClient.Close();
                    pointCloudClient.Dispose();
                    gameObject.GetComponent<MeshRenderer>().enabled = false;
                }
            }
        }
    }

    private async void ReceiveDocuments()
    {
        while (isDocumentClientConnected && documentClient.Connected)
        {
            try
            {
                // Request a new frame
                await documentClient.GetStream().WriteAsync(new byte[] { 0 });

                // Read width
                int width = await ReadIntAsync(documentClient);

                // Read height 
                int height = await ReadIntAsync(documentClient);

                int dataSize = await ReadIntAsync(documentClient);

                Debug.Log($"Received document with width {width} and height {height}, size {dataSize}");

                // Initialize array for document data
                byte[] dataBytes = new byte[dataSize];

                // Read document data
                int numBytesRead = 0;

                while (numBytesRead < dataSize)
                    numBytesRead += await documentClient.GetStream().ReadAsync(dataBytes, numBytesRead, Math.Min(dataSize - numBytesRead, 64000));

                documentRenderer.EnqueueDocument(width, height, dataBytes);
            }
            catch (Exception)
            {
                if (!documentClient.Connected && isDocumentClientConnected)
                {
                    // The socket was disconnected while trying to receive a document; close the socket
                    isDocumentClientConnecting = false;
                    isDocumentClientConnected = false;
                    documentClient.Close();
                    documentClient.Dispose();
                }
            }
        }
    }

    private void DeserializePointCloud(int numPoints, float scale, byte[] verticesBytes, byte[] colorsBytes, out Vector3[] vertices, out Color32[] colors)
    {
        vertices = new Vector3[numPoints];
        colors = new Color32[numPoints];

        // Deserialize position data
        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * PointXYZDataSize;
            float x = DecodeByteToFloat(verticesBytes[offset], XRangeCenter, scale);
            float y = -1.0f * DecodeByteToFloat(verticesBytes[offset + 1], YRangeCenter, scale); // Flip Y axis to get the right orientation
            float z = DecodeByteToFloat(verticesBytes[offset + 2], ZRangeCenter, scale);

            vertices[i] = new Vector3(x, y, z);
        }

        // Deserialize color data
        for (int i = 0; i < numPoints; i++)
        {
            int colorOffset = i * PointRGBDataSize;
            byte r = colorsBytes[colorOffset];
            byte g = colorsBytes[colorOffset + 1];
            byte b = colorsBytes[colorOffset + 2];

            colors[i] = new Color32(r, g, b, 255);
        }
    }

    private async Task<short> ReadShortAsync(TcpClient client)
    {
        int numBytesToRead = sizeof(short);
        byte[] buffer = await ReadAsync(client, numBytesToRead);

        return BitConverter.ToInt16(buffer, 0);
    }

    private async Task<int> ReadIntAsync(TcpClient client)
    {
        int numBytesToRead = sizeof(int);
        byte[] buffer = await ReadAsync(client, numBytesToRead);

        return BitConverter.ToInt32(buffer, 0);
    }

    private async Task<byte[]> ReadAsync(TcpClient client, int numBytesToRead)
    {
        byte[] buffer = new byte[numBytesToRead];
        int numBytesRead = 0;

        while (numBytesRead < numBytesToRead)
        {
            numBytesRead += await client.GetStream().ReadAsync(buffer, numBytesRead, numBytesToRead - numBytesRead);
        }

        return buffer;
    }

    private float DecodeByteToFloat(byte encoded, float rangeCenter, float scale)
    {
        return encoded / scale - HalfRange + rangeCenter;
    }

    private void OnDestroy()
    {
        isPointCloudClientConnecting = false;
        isPointCloudClientConnected = false;
        pointCloudClient.Close();
        pointCloudClient.Dispose();

        isDocumentClientConnecting = false;
        isDocumentClientConnected = false;
        documentClient.Close();
        documentClient.Dispose();
    }
}
