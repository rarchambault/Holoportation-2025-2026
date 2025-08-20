/***************************************************************************\

Module Name:  PointCloudRenderer.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module receives point clouds from the PointCloudReceiver, enqueues them
and renders them.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudRenderer : MonoBehaviour
{
    public Material PointCloudMaterial;

    // Indices representing the corners of each point quad
    private static readonly float[] s_baseOffsetIndices = new float[] { 0, 1, 2, 3, 4, 5 };

    private readonly List<Vector3> Vertices = new();
    private readonly List<Color32> Colors = new();
    private readonly List<Vector2> OffsetIndices = new();
    private readonly List<int> Indices = new();

    private const int MaxQueueSize = 5;

    private const float PointScaleFnA = 170.0f;
    private const float PointScaleFnB = 0.8f;
    private const float PointScaleFnC = 0.002f;

    // Parameters used to calculate and log FPS
    private bool isStarted = false;
    private float timeSinceLastRender = 0.0f;
    private float totalTime = 0.0f;
    private int numFrames = 0;

    private Queue<(float scale, Vector3[] points, Color32[] colors)> pointCloudQueue = new();
    private Mesh mesh;
    private Quaternion rotation = Quaternion.Euler(270.0f, 0f, 0);

    void Start()
    {
        // Initialize point cloud mesh
        this.transform.rotation = rotation;
        mesh = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
        mesh.MarkDynamic(); // Hint for performance

        GetComponent<MeshFilter>().sharedMesh = mesh;
        GetComponent<MeshRenderer>().material = PointCloudMaterial;
    }

    void Update()
    {
        if (isStarted)
        {
            timeSinceLastRender += Time.deltaTime;
        }

        // If the point cloud queue is not empty, render its first entry
        if (pointCloudQueue.Count > 0)
        {
            var (scale, positions, colorData) = pointCloudQueue.Dequeue();
            UpdateMesh(scale, positions, colorData);
        }
    }

    public void EnqueuePointCloud(float scale, Vector3[] positions, Color32[] colors)
    {
        isStarted = true;

        // If the queue is full, dequeue the first entry to add the new one
        if (pointCloudQueue.Count >= MaxQueueSize)
            pointCloudQueue.Dequeue();

        pointCloudQueue.Enqueue((scale, positions, colors));
    }

    private void UpdateMesh(float scale, Vector3[] positions, Color32[] colorData)
    {
        int pointCount = Mathf.Min(positions.Length, colorData.Length);

        if (pointCount == 0)
        {
            return;
        }

        // Find the level of precision of the point cloud from the scale that was sent
        float precision = 1.0f / scale;

        // Make the points slightly larger than the precision to fill holes in the point cloud
        PointCloudMaterial.SetFloat("_PointSize", PointScaleFnA * Mathf.Pow(precision, 2)  + PointScaleFnB * precision + PointScaleFnC);

        // Clear all previous mesh parameters
        mesh.Clear();
        Vertices.Clear();
        OffsetIndices.Clear();
        Colors.Clear();
        Indices.Clear();

        // Add each new received point to global variables
        for (int i = 0; i < pointCount; i++)
        {
            Vector3 pos = positions[i];
            Color32 col = colorData[i];

            for (int j = 0; j < 6; j++)
            {
                Vertices.Add(pos);
                OffsetIndices.Add(new Vector2(s_baseOffsetIndices[j], 0));
                Colors.Add(col);
                Indices.Add(i * 6 + j);
            }
        }

        // Set new mesh parameters
        mesh.SetVertices(Vertices);
        mesh.SetUVs(0, OffsetIndices);
        mesh.SetColors(Colors);
        mesh.SetIndices(Indices, MeshTopology.Triangles, 0);

        // Calculate and log FPS
        totalTime += timeSinceLastRender;
        timeSinceLastRender = 0.0f;
        numFrames++;
        Debug.Log("Average FPS: " + numFrames / totalTime);
    }
}