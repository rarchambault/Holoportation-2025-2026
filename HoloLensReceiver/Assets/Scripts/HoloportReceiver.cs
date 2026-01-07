/***************************************************************************\

Module Name:  HoloportReceiver.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Adapted by:   Mahmoud Amin

<Description>
This module receives mesh data (vertices + colors + triangle indices) and
documents from the LiveScan3D TCP server and forwards them to the appropriate
renderers in Unity.

New mesh protocol (from LiveScan3D TransferServer / PointCloudTransferSocket):
    [int] vertexCount   (number of vertices = vertices.Count / 3)
    [int] colorCount    (number of bytes of color data = colors.Count)
    [int] indexCount    (number of ints in triangle index buffer)

    [float] * vertexCount * 3   (X, Y, Z per vertex)
    [byte]  * colorCount        (R, G, B per vertex)
    [int]   * indexCount        (triangle indices)

The client requests a new frame by sending a single byte: 0

This code was adapted from:
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

    private TcpClient pointCloudClient;
    private bool isPointCloudClientConnected = false;
    private bool isPointCloudClientConnecting = false;
    private float pointCloudConnectionTimer = 0.0f;

    private TcpClient documentClient;
    private bool isDocumentClientConnected = false;
    private bool isDocumentClientConnecting = false;
    private float documentConnectionTimer = 0.0f;

    private const float Range = 0.3f;                  // same as server
    private const float HalfRange = Range / 2.0f;      // 0.15f

    // Same centers as on the sender
    private const float xRangeCenter = 0.0f;
    private const float yRangeCenter = 0.0f;
    private const float zRangeCenter = HalfRange;      // 0.15f

    private StreamingMeshRenderer meshRenderer;
    //private PointCloudRenderer pointCloudRenderer;
    private DocumentRenderer documentRenderer;

    private void Start()
    {
        //pointCloudRenderer = GetComponent<PointCloudRenderer>();
        meshRenderer = GetComponent<StreamingMeshRenderer>();
        documentRenderer = GetComponent<DocumentRenderer>();
    }

    private void Update()
    {
        // Try to connect point cloud client
        if (!isPointCloudClientConnecting && IsServerIPAddressSet)
        {
            isPointCloudClientConnecting = true;
            ConnectPointCloudClient();
        }

        // Try to connect document client
        if (!isDocumentClientConnecting && IsServerIPAddressSet)
        {
            isDocumentClientConnecting = true;
            ConnectDocumentClient();
        }

        // Retry logic for point cloud client
        if (isPointCloudClientConnecting && !isPointCloudClientConnected)
        {
            pointCloudConnectionTimer += Time.deltaTime;

            if (pointCloudConnectionTimer >= ConnectionRetryInterval)
            {
                ConnectPointCloudClient();
                pointCloudConnectionTimer = 0.0f;
            }
        }

        // Retry logic for document client
        if (isDocumentClientConnecting && !isDocumentClientConnected)
        {
            documentConnectionTimer += Time.deltaTime;

            if (documentConnectionTimer >= ConnectionRetryInterval)
            {
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
            Debug.Log("Connected to LiveScan3D point cloud server.");
            ReceivePointClouds();
            var mr = gameObject.GetComponent<MeshRenderer>();
            if (mr != null) mr.enabled = true;
        }
        catch (Exception e)
        {
            Debug.LogError("Connection to LiveScan3D point cloud server failed: " + e.Message);
            isPointCloudClientConnected = false;
            isPointCloudClientConnecting = false;
        }
    }

    private async void ConnectDocumentClient()
    {
        documentClient = new TcpClient();

        try
        {
            await documentClient.ConnectAsync(ServerIPAddress, DocumentPort);
            isDocumentClientConnected = true;
            Debug.Log("Connected to LiveScan3D document server.");
            ReceiveDocuments();
        }
        catch (Exception e)
        {
            Debug.LogError("Connection to LiveScan3D document server failed: " + e.Message);
            isDocumentClientConnected = false;
            isDocumentClientConnecting = false;
        }
    }
    private float DecodeByteToFloat(byte b, float rangeCenter, float scale)
    {
        return (b / scale) - HalfRange + rangeCenter;
    }

    /// <summary>
    /// Receives FULL MESH data from the LiveScan3D server:
    /// header (vertexCount, colorCount, indexCount)
    /// then vertices (float3), colors (bytes), indices (ints).
    /// </summary>
    private async void ReceivePointClouds()
    {
        while (isPointCloudClientConnected && pointCloudClient != null && pointCloudClient.Connected)
        {
            try
            {
                NetworkStream stream = pointCloudClient.GetStream();

                // 1) Request a new frame (1-byte handshake: 0)
                await stream.WriteAsync(new byte[] { 0 }, 0, 1);

                // 2) Read scale (short, 2 bytes)
                byte[] scaleBytes = await ReadAsync(pointCloudClient, sizeof(short));
                short scale = BitConverter.ToInt16(scaleBytes, 0);

                // 3) Read vertex count (int, 4 bytes)
                byte[] vCountBytes = await ReadAsync(pointCloudClient, sizeof(int));
                int vertexCount = BitConverter.ToInt32(vCountBytes, 0);

                // 4) Read compressed vertices (1 byte per coordinate) and colors (RGB)
                int vertexByteCount = vertexCount * 3; // x,y,z as bytes
                int colorByteCount = vertexCount * 3; // r,g,b as bytes

                byte[] verticesBytes = await ReadAsync(pointCloudClient, vertexByteCount);
                byte[] colorsBytes = await ReadAsync(pointCloudClient, colorByteCount);

                // 5) Read triangle count (int, 4 bytes)
                byte[] triCountBytes = await ReadAsync(pointCloudClient, sizeof(int));
                int triangleCount = BitConverter.ToInt32(triCountBytes, 0);
                int indexCount = triangleCount * 3;

                // 6) Read indices (int32)
                int indicesByteCount = indexCount * sizeof(int);
                byte[] indicesBytes = await ReadAsync(pointCloudClient, indicesByteCount);

                int[] meshIndices = new int[indexCount];
                Buffer.BlockCopy(indicesBytes, 0, meshIndices, 0, indicesByteCount);

                // 7) Decompress vertices into Vector3[]
                Vector3[] vertices = new Vector3[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    int b = i * 3;
                    byte bx = verticesBytes[b];
                    byte by = verticesBytes[b + 1];
                    byte bz = verticesBytes[b + 2];

                    float x = DecodeByteToFloat(bx, xRangeCenter, scale);
                    float y = DecodeByteToFloat(by, yRangeCenter, scale);
                    float z = DecodeByteToFloat(bz, zRangeCenter, scale);

                    vertices[i] = new Vector3(x, y, z);
                }

                // 8) Colors
                Color32[] colors = new Color32[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    int b = i * 3;
                    colors[i] = new Color32(
                        colorsBytes[b],
                        colorsBytes[b + 1],
                        colorsBytes[b + 2],
                        255
                    );
                }

                Debug.Log($"Received mesh: {vertexCount} verts, {triangleCount} triangles.");

                if (meshRenderer != null)
                {
                    meshRenderer.EnqueueMesh(vertices, colors, meshIndices);
                }

                // Use indices on your renderer now
                // pointCloudRenderer.EnqueueMesh(vertices, colors, meshIndices);
            }
            catch (Exception e)
            {
                Debug.LogError("Error while receiving mesh: " + e.Message);

                if (!pointCloudClient.Connected && isPointCloudClientConnected)
                {
                    isPointCloudClientConnecting = false;
                    isPointCloudClientConnected = false;

                    try
                    {
                        pointCloudClient.Close();
                        pointCloudClient.Dispose();
                    }
                    catch { }

                    var mr = gameObject.GetComponent<MeshRenderer>();
                    if (mr != null) mr.enabled = false;
                }
            }
        }
    }

    /// <summary>
    /// Receives document images (unchanged protocol).
    /// </summary>
    private async void ReceiveDocuments()
    {
        while (isDocumentClientConnected && documentClient != null && documentClient.Connected)
        {
            try
            {
                // Read width (short)
                short width = await ReadShortAsync(documentClient);

                // Read height (short)
                short height = await ReadShortAsync(documentClient);

                // Read data size (int)
                int dataSize = await ReadIntAsync(documentClient);

                Debug.Log($"Received document with width {width}, height {height}, size {dataSize}");

                // Read document data
                byte[] dataBytes = await ReadAsync(documentClient, dataSize);

                if (documentRenderer != null)
                {
                    documentRenderer.EnqueueDocument(width, height, dataBytes);
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Error while receiving document: " + e.Message);

                if (!documentClient.Connected && isDocumentClientConnected)
                {
                    isDocumentClientConnecting = false;
                    isDocumentClientConnected = false;

                    try
                    {
                        documentClient.Close();
                        documentClient.Dispose();
                    }
                    catch { }
                }
            }
        }
    }

    // ---- Helper methods for reading from TCP ----

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

    /// <summary>
    /// Reads exactly numBytesToRead bytes from the TCP stream.
    /// </summary>
    private async Task<byte[]> ReadAsync(TcpClient client, int numBytesToRead)
    {
        byte[] buffer = new byte[numBytesToRead];
        int numBytesRead = 0;

        NetworkStream stream = client.GetStream();

        while (numBytesRead < numBytesToRead)
        {
            int read = await stream.ReadAsync(buffer, numBytesRead, numBytesToRead - numBytesRead);
            if (read == 0)
            {
                // Connection closed
                throw new Exception("Socket closed while reading.");
            }
            numBytesRead += read;
        }

        return buffer;
    }

    private void OnDestroy()
    {
        isPointCloudClientConnecting = false;
        isPointCloudClientConnected = false;

        if (pointCloudClient != null)
        {
            try
            {
                pointCloudClient.Close();
                pointCloudClient.Dispose();
            }
            catch { }
            pointCloudClient = null;
        }

        isDocumentClientConnecting = false;
        isDocumentClientConnected = false;

        if (documentClient != null)
        {
            try
            {
                documentClient.Close();
                documentClient.Dispose();
            }
            catch { }
            documentClient = null;
        }
    }
}