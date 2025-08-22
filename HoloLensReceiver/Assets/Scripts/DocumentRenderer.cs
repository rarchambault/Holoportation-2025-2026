/***************************************************************************\

Module Name:  DocumentRenderer.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module renders document images on a plane renderer.

\***************************************************************************/

using System.Collections.Generic;
using UnityEngine;

public class DocumentRenderer : MonoBehaviour
{
    public float MaxImageSize = 3.0f;
    public float MinImageSize = 2.0f;
    public Renderer TargetRenderer;

    private const float ImageTimeout = 30.0f;
    private const float PixelToMeter = 0.26f / 1000f; // Convert pixels to meters
    private const int MaxQueueSize = 5;

    private float xScaleUnitWidth;
    private float zScaleUnitHeight;

    private bool isStarted = false;
    private float timeSinceLastRender = 0.0f;

    private Queue<(short width, short height, byte[] data)> documentQueue = new();

    public void Start()
    {
        if (TargetRenderer == null)
        {
            Debug.LogError("Target Renderer is not assigned!");
            return;
        }

        float currentWorldWidth = TargetRenderer.localBounds.size.x;
        float currentWorldHeight = TargetRenderer.localBounds.size.z;

        Vector3 localScale = TargetRenderer.transform.localScale;

        // Compute the unit local x-scale and z-scale
        xScaleUnitWidth = (1.0f * localScale.x) / currentWorldWidth;
        zScaleUnitHeight = (1.0f * localScale.z) / currentWorldHeight;

        TargetRenderer.material.color = Color.white * 2.0f; // brighten
        TargetRenderer.enabled = false; // Initially hide the renderer
    }

    void Update()
    {
        if (isStarted)
        {
            timeSinceLastRender += Time.deltaTime;
        }

        // If the document queue is not empty, render its first entry
        if (documentQueue.Count > 0)
        {
            var (width, height, data) = documentQueue.Dequeue();
            UpdateMesh(width, height, data);
        }

        // Hide renderer if no new image has been received in the timeout period
        if (TargetRenderer.enabled && timeSinceLastRender > ImageTimeout)
        {
            TargetRenderer.enabled = false;
        }
    }

    public void EnqueueDocument(short width, short height, byte[] data)
    {
        isStarted = true;

        // If the queue is full, dequeue the first entry to add the new one
        if (documentQueue.Count >= MaxQueueSize)
            documentQueue.Dequeue();

        documentQueue.Enqueue((width, height, data));
    }

    public void UpdateMesh(short width, short height, byte[] data)
    {
        if (data == null || data.Length == 0)
        {
            return;
        }

        // Load the received image into a 2D Texture
        Texture2D texture = new Texture2D(2, 2, TextureFormat.RGB24, false, true);

        if (texture.LoadImage(data))
        {
            texture.Apply();

            // Apply the texture on the renderer
            TargetRenderer.material.mainTexture = texture;
            TargetRenderer.enabled = true;

            // Scale the renderer to match the aspect ratio
            AdjustRendererScale(width, height);

            timeSinceLastRender = 0.0f;
        }
        else
        {
            Debug.LogError("Failed to load image data into texture");
        }
    }

    private void AdjustRendererScale(short width, short height)
    {
        float aspectRatio = (float)width / (float)height;

        // Calculate image dimensions in world space based on pixel dimensions
        float realWidth = (float)width * PixelToMeter;
        float realHeight = (float)height * PixelToMeter;

        // Clamp width and height to fit within allowed dimensions
        realWidth = Mathf.Clamp(realWidth, MinImageSize, MaxImageSize);
        realHeight = realWidth / aspectRatio;

        realHeight = Mathf.Clamp(realHeight, MinImageSize, MaxImageSize);
        realWidth = realHeight * aspectRatio;

        // Calculate the new x and z scale of the renderer
        Vector3 newScale = TargetRenderer.transform.localScale;
        newScale.x = realWidth * xScaleUnitWidth;  // Width
        newScale.z = realHeight * zScaleUnitHeight; // Height

        TargetRenderer.transform.localScale = newScale;
    }
}