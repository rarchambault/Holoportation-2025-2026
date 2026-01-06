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

    private PointCloudRenderer pointCloudRenderer;
    private DocumentRenderer documentRenderer;

    private void Start()
    {
        pointCloudRenderer = GetComponent<PointCloudRenderer>();
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
                await stream.WriteAsync(new byte[] { 0 });
                // Header: 2 ints
                byte[] headerBytes = await ReadAsync(pointCloudClient, sizeof(int) * 2);

                int vertexCount = BitConverter.ToInt32(headerBytes, 0);
                int indexCount = BitConverter.ToInt32(headerBytes, 4);

                // Derived color count
                int colorCount = vertexCount * 3;

                // Byte counts
                int verticesByteCount = vertexCount * 3 * sizeof(float);
                int colorsByteCount = colorCount * sizeof(byte);
                int indicesByteCount = indexCount * sizeof(int);

                // Read buffers
                byte[] verticesBytes = await ReadAsync(pointCloudClient, verticesByteCount);
                byte[] colorsBytes = await ReadAsync(pointCloudClient, colorsByteCount);
                byte[] indicesBytes = await ReadAsync(pointCloudClient, indicesByteCount);

                // Deserialize floats
                float[] vertsFloat = new float[vertexCount * 3];
                Buffer.BlockCopy(verticesBytes, 0, vertsFloat, 0, verticesByteCount);

                Vector3[] vertices = new Vector3[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    int b = i * 3;
                    vertices[i] = new Vector3(
                        vertsFloat[b],
                        vertsFloat[b + 1],
                        vertsFloat[b + 2]
                    );
                }

                // Deserialize colors
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

                // Deserialize indices
                int[] meshIndices = new int[indexCount];
                Buffer.BlockCopy(indicesBytes, 0, meshIndices, 0, indicesByteCount);

                Debug.Log($"Received mesh: {vertexCount} verts, {indexCount / 3} triangles.");

                pointCloudRenderer.EnqueueMesh(vertices, colors, meshIndices);

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
