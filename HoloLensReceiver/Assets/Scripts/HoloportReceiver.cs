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

    // EXACT MATCH WITH SENDER
    private const float Range = 3.0f;
    private const float HalfRange = Range / 2.0f;
    private const float xRangeCenter = 0.0f;
    private const float yRangeCenter = 0.0f;
    private const float zRangeCenter = 1.0f;

    private StreamingMeshRenderer meshRenderer;
    private DocumentRenderer documentRenderer;
    private PointCloudRenderer pointCloudRenderer;
        
    private void Start()
    {
        meshRenderer = GetComponent<StreamingMeshRenderer>();
        documentRenderer = GetComponent<DocumentRenderer>();
        pointCloudRenderer = GetComponentInChildren<PointCloudRenderer>();
    }

    private void Update()
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
                ConnectPointCloudClient();
                pointCloudConnectionTimer = 0.0f;
            }
        }

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
            Debug.LogError("CONNECTION FAILED: " + e.Message);
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
            isDocumentClientConnected = false;
            isDocumentClientConnecting = false;
        }
    }

    private float DecodeUShortToFloat(ushort val, float rangeCenter, float scale)
    {
        return (val / scale) - HalfRange + rangeCenter;
    }

    private async void ReceivePointClouds()
    {
        while (isPointCloudClientConnected && pointCloudClient != null && pointCloudClient.Connected)
        {
            try
            {
                NetworkStream stream = pointCloudClient.GetStream();

                await stream.WriteAsync(new byte[] { 0 }, 0, 1);

                byte[] scaleBytes = await ReadAsync(pointCloudClient, sizeof(float));
                float scale = BitConverter.ToSingle(scaleBytes, 0);

                byte[] vCountBytes = await ReadAsync(pointCloudClient, sizeof(int));
                int vertexCount = BitConverter.ToInt32(vCountBytes, 0);

                int vertexByteCount = vertexCount * 3 * sizeof(ushort);
                int colorByteCount = vertexCount * 3;

                byte[] verticesBytes = await ReadAsync(pointCloudClient, vertexByteCount);
                byte[] colorsBytes = await ReadAsync(pointCloudClient, colorByteCount);

                byte[] triCountBytes = await ReadAsync(pointCloudClient, sizeof(int));
                int triangleCount = BitConverter.ToInt32(triCountBytes, 0);
                int indexCount = triangleCount * 3;

                int indicesByteCount = indexCount * sizeof(int);
                byte[] indicesBytes = await ReadAsync(pointCloudClient, indicesByteCount);

                int[] meshIndices = new int[indexCount];
                Buffer.BlockCopy(indicesBytes, 0, meshIndices, 0, indicesByteCount);

                Vector3[] vertices = new Vector3[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    int b = i * 6;
                    ushort ux = BitConverter.ToUInt16(verticesBytes, b);
                    ushort uy = BitConverter.ToUInt16(verticesBytes, b + 2);
                    ushort uz = BitConverter.ToUInt16(verticesBytes, b + 4);

                    float x = DecodeUShortToFloat(ux, xRangeCenter, scale);
                    float y = DecodeUShortToFloat(uy, yRangeCenter, scale);
                    float z = DecodeUShortToFloat(uz, zRangeCenter, scale);

                    vertices[i] = new Vector3(x, y, z);
                }

                Color32[] colors = new Color32[vertexCount];
                for (int i = 0; i < vertexCount; i++)
                {
                    int b = i * 3;
                    colors[i] = new Color32(colorsBytes[b], colorsBytes[b + 1], colorsBytes[b + 2], 255);
                }

                if (meshRenderer != null)
                {
                    meshRenderer.EnqueueMesh(vertices, colors, meshIndices);
                }

                if (pointCloudRenderer != null)
                {
                    pointCloudRenderer.EnqueuePointCloud(scale, vertices, colors);
                }
            }
            catch (Exception e)
            {
                if (!pointCloudClient.Connected && isPointCloudClientConnected)
                {
                    isPointCloudClientConnecting = false;
                    isPointCloudClientConnected = false;
                    try { pointCloudClient.Close(); pointCloudClient.Dispose(); } catch { }
                    var mr = gameObject.GetComponent<MeshRenderer>();
                    if (mr != null) mr.enabled = false;
                }
            }
        }
    }

    private async void ReceiveDocuments()
    {
        while (isDocumentClientConnected && documentClient != null && documentClient.Connected)
        {
            try
            {
                short width = await ReadShortAsync(documentClient);
                short height = await ReadShortAsync(documentClient);
                int dataSize = await ReadIntAsync(documentClient);
                byte[] dataBytes = await ReadAsync(documentClient, dataSize);

                if (documentRenderer != null)
                {
                    documentRenderer.EnqueueDocument(width, height, dataBytes);
                }
            }
            catch (Exception e)
            {
                if (!documentClient.Connected && isDocumentClientConnected)
                {
                    isDocumentClientConnecting = false;
                    isDocumentClientConnected = false;
                    try { documentClient.Close(); documentClient.Dispose(); } catch { }
                }
            }
        }
    }

    private async Task<short> ReadShortAsync(TcpClient client)
    {
        byte[] buffer = await ReadAsync(client, sizeof(short));
        return BitConverter.ToInt16(buffer, 0);
    }

    private async Task<int> ReadIntAsync(TcpClient client)
    {
        byte[] buffer = await ReadAsync(client, sizeof(int));
        return BitConverter.ToInt32(buffer, 0);
    }

    private async Task<byte[]> ReadAsync(TcpClient client, int numBytesToRead)
    {
        byte[] buffer = new byte[numBytesToRead];
        int numBytesRead = 0;
        NetworkStream stream = client.GetStream();

        while (numBytesRead < numBytesToRead)
        {
            int read = await stream.ReadAsync(buffer, numBytesRead, numBytesToRead - numBytesRead);
            if (read == 0) throw new Exception("Socket closed while reading.");
            numBytesRead += read;
        }
        return buffer;
    }

    private void OnDestroy()
    {
        isPointCloudClientConnecting = false;
        isPointCloudClientConnected = false;
        if (pointCloudClient != null) { try { pointCloudClient.Close(); pointCloudClient.Dispose(); } catch { } pointCloudClient = null; }

        isDocumentClientConnecting = false;
        isDocumentClientConnected = false;
        if (documentClient != null) { try { documentClient.Close(); documentClient.Dispose(); } catch { } documentClient = null; }
    }
}