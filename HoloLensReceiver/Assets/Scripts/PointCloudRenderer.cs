/***************************************************************************\

Module Name:  PointCloudRenderer.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Adapted by:   Mahmoud Amin

<Description>
This module now receives FULL MESHES from the HoloportReceiver:
    - vertices: Vector3[]
    - colors:   Color32[]
    - indices:  int[]   (triangle indices)

It enqueues them and renders them as a standard Unity Mesh.

Originally, this class rendered compressed point clouds as quads using
a custom shader and per-point "_PointSize". With the updated LiveScan3D
pipeline, the mesh is already triangulated on the PC side (C++/PCL),
so we simply display it directly.

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

    private Mesh mesh;

    // Parameters used to calculate and log FPS
    private bool isStarted = false;
    private float timeSinceLastRender = 0.0f;
    private float totalTime = 0.0f;
    private int numFrames = 0;

    // Queue of incoming meshes (vertices + colors + indices)
    private const int MaxQueueSize = 5;
    private readonly Queue<(Vector3[] vertices, Color32[] colors, int[] indices)> meshQueue = new();

    // Optional rotation to match original orientation
    private Quaternion rotation = Quaternion.Euler(270.0f, 0f, 0);

    void Start()
    {
        // Initialize mesh
        this.transform.rotation = rotation;

        mesh = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 // allow large meshes
        };
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

        // If we have a queued mesh, render the most recent one
        if (meshQueue.Count > 0)
        {
            var (vertices, colors, indices) = meshQueue.Dequeue();
            UpdateMesh(vertices, colors, indices);
        }
    }

    /// <summary>
    /// Enqueue a full mesh (vertices + colors + triangle indices).
    /// Called by HoloportReceiver once a frame is decoded.
    /// </summary>
    public void EnqueueMesh(Vector3[] vertices, Color32[] colors, int[] indices)
    {
        isStarted = true;

        if (vertices == null || colors == null || indices == null)
            return;

        // If the queue is full, drop the oldest mesh
        if (meshQueue.Count >= MaxQueueSize)
            meshQueue.Dequeue();

        meshQueue.Enqueue((vertices, colors, indices));
    }

    /// <summary>
    /// Updates the Unity Mesh with the latest frame.
    /// </summary>
    private void UpdateMesh(Vector3[] vertices, Color32[] colors, int[] indices)
    {
        if (mesh == null)
            return;

        int vertexCount = Mathf.Min(vertices.Length, colors.Length);
        if (vertexCount == 0 || indices.Length == 0)
            return;

        // Trim arrays if colors are shorter than vertices
        // (safety in case of small mismatches)
        Vector3[] safeVertices = vertices;
        Color32[] safeColors = colors;

        if (colors.Length != vertices.Length)
        {
            vertexCount = Mathf.Min(vertices.Length, colors.Length);
            safeVertices = new Vector3[vertexCount];
            safeColors = new Color32[vertexCount];

            System.Array.Copy(vertices, safeVertices, vertexCount);
            System.Array.Copy(colors, safeColors, vertexCount);
        }

        mesh.Clear();

        // Assign mesh data
        mesh.SetVertices(safeVertices);
        mesh.SetColors(safeColors);
        mesh.SetTriangles(indices, 0);

        // Optionally recompute normals/bounds if needed
        mesh.RecalculateBounds();
        // mesh.RecalculateNormals(); // uncomment if your material needs normals

        // FPS stats
        totalTime += timeSinceLastRender;
        timeSinceLastRender = 0.0f;
        numFrames++;

        if (totalTime > 0.0f)
        {
            Debug.Log("Average FPS: " + (numFrames / totalTime));
        }
    }
}
