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

    public ComputeShader marchingCubesShader;
    private ComputeBuffer positionBuffer;
    private ComputeBuffer colorBuffer;

    // The Grid
    public int gridResolution = 128; // Size of the 3D voxel grid
    private RenderTexture densityGrid;

    // The Triangles
    private ComputeBuffer triangleBuffer;
    public Material renderMaterial; // We will need a shader on this to draw the buffer
    private ComputeBuffer countBuffer; // Used to count how many triangles were generated

    // A struct to hold the triangle data (matches the HLSL side)
    public struct Triangle
    {
        public Vector3 vertexC;
        public Vector3 vertexB;
        public Vector3 vertexA;
        public Color32 color;
    }



    private PointCloudRenderer pointCloudRenderer;
    private DocumentRenderer documentRenderer;

    private void OnDisable()
    {
        positionBuffer?.Release();
        colorBuffer?.Release();
        triangleBuffer?.Release();
        countBuffer?.Release();

        if (densityGrid != null)
        {
            densityGrid.Release();
        }
    }

    private void Start()
    {
        pointCloudRenderer = GetComponent<PointCloudRenderer>();
        documentRenderer = GetComponent<DocumentRenderer>();

        // 1. Create the 3D Density Grid
        densityGrid = new RenderTexture(gridResolution, gridResolution, 0, RenderTextureFormat.RFloat);
        densityGrid.dimension = UnityEngine.Rendering.TextureDimension.Tex3D;
        densityGrid.volumeDepth = gridResolution;
        densityGrid.enableRandomWrite = true; // Crucial for Compute Shaders
        densityGrid.Create();

        // 2. Clear the grid to 0 using the shader (we'll add a Clear kernel to the HLSL next)
        int clearKernel = marchingCubesShader.FindKernel("CSClear");
        marchingCubesShader.SetTexture(clearKernel, "DensityGrid", densityGrid);
        marchingCubesShader.Dispatch(clearKernel, gridResolution / 8, gridResolution / 8, gridResolution / 8);
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

                // 1. Initialize or Resize Buffers if point count changed
                if (positionBuffer == null || positionBuffer.count != numPoints)
                {
                    positionBuffer?.Release();
                    colorBuffer?.Release();
                    positionBuffer = new ComputeBuffer(numPoints, sizeof(float) * 3);
                    colorBuffer = new ComputeBuffer(numPoints, sizeof(byte) * 4);
                }

                // 2. Upload data to GPU
                positionBuffer.SetData(vertices);
                colorBuffer.SetData(colors);

                // 3. Dispatch the Compute Shader (Voxelization & Meshing)
                int kernelHandle = marchingCubesShader.FindKernel("CSMain");
                marchingCubesShader.SetBuffer(kernelHandle, "PositionBuffer", positionBuffer);
                marchingCubesShader.SetBuffer(kernelHandle, "ColorBuffer", colorBuffer);
                marchingCubesShader.SetInt("NumPoints", numPoints);

                marchingCubesShader.SetTexture(kernelHandle, "DensityGrid", densityGrid);

                // Calculate thread groups (assuming 64 threads per group in the shader)
                int groups = Mathf.CeilToInt(numPoints / 64f);
                // --- THE GPU PIPELINE ---

                // 1. CLEAR THE GRID
                int clearKernel = marchingCubesShader.FindKernel("CSClear");
                marchingCubesShader.Dispatch(clearKernel, gridResolution / 8, gridResolution / 8, gridResolution / 8);

                // 2. SPLAT POINTS (Voxelize)
                int mainKernel = marchingCubesShader.FindKernel("CSMain");
                marchingCubesShader.SetBuffer(mainKernel, "PositionBuffer", positionBuffer);
                marchingCubesShader.SetBuffer(mainKernel, "ColorBuffer", colorBuffer);
                marchingCubesShader.SetInt("NumPoints", numPoints);
                marchingCubesShader.SetInt("GridResolution", gridResolution);
                marchingCubesShader.SetTexture(mainKernel, "DensityGrid", densityGrid);

                int splatGroups = Mathf.CeilToInt(numPoints / 64f);
                marchingCubesShader.Dispatch(mainKernel, splatGroups, 1, 1);

                // 3. MARCHING CUBES
                // Set up the Triangle Append Buffer (Max possible triangles = grid^3 * 5)
                int maxTriangles = gridResolution * gridResolution * gridResolution * 5;
                if (triangleBuffer == null)
                {
                    triangleBuffer = new ComputeBuffer(maxTriangles, 40, ComputeBufferType.Append);

                    // Create a buffer for 4 integers (16 bytes)
                    countBuffer = new ComputeBuffer(1, 4 * sizeof(int), ComputeBufferType.IndirectArguments);

                    // Tell it to draw 3 vertices per instance. We will copy the number of instances later!
                    countBuffer.SetData(new int[] { 3, 0, 0, 0 });
                }

                triangleBuffer.SetCounterValue(0); // Reset the append counter
                int marchKernel = marchingCubesShader.FindKernel("CSMarch");
                marchingCubesShader.SetTexture(marchKernel, "DensityGrid", densityGrid);
                marchingCubesShader.SetBuffer(marchKernel, "OutputTriangles", triangleBuffer);
                marchingCubesShader.SetInt("GridResolution", gridResolution);

                marchingCubesShader.Dispatch(marchKernel, gridResolution / 8, gridResolution / 8, gridResolution / 8);
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

    private void OnRenderObject()
    {
        if (triangleBuffer != null && renderMaterial != null)
        {
            renderMaterial.SetPass(0);
            renderMaterial.SetBuffer("TriangleBuffer", triangleBuffer);

            // Shift the copy offset by 4 bytes so it writes into the 'InstanceCount' slot
            ComputeBuffer.CopyCount(triangleBuffer, countBuffer, 4);

            Graphics.DrawProceduralIndirectNow(MeshTopology.Triangles, countBuffer, 0);
        }
    }

    private async void ReceiveDocuments()
    {
        while (isDocumentClientConnected && documentClient.Connected)
        {
            try
            {
                // Read width
                short width = await ReadShortAsync(documentClient);

                // Read height 
                short height = await ReadShortAsync(documentClient);

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
