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
using UnityEngine.UI;
using System.IO;

public class DocumentRenderer : MonoBehaviour
{
    private int debugSaveCount = 0;
    private RawImage DebugRawImage;
    public float MaxImageSize = 1.1f;  // 80 cm
    public float MinImageSize = 0.4f;  // 20 cm
    
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
        if (data == null || data.Length != width * height * 3)
        {
            Debug.LogWarning($"Invalid document data. width={width}, height={height}, bytes={(data == null ? 0 : data.Length)}");
            return;
        }

        Debug.Log($"Received document with width {width} and height {height}, size {data.Length}");

        byte[] correctedData = ImageCorrection(width, height, data);

        Texture2D texture = new Texture2D(
            width,
            height,
            TextureFormat.RGB24,
            mipChain: false,
            linear: false
        );

        texture.LoadRawTextureData(correctedData);
        texture.filterMode = FilterMode.Point;
        texture.wrapMode = TextureWrapMode.Clamp;
        texture.anisoLevel = 1;
        texture.Apply(false, false);

        // Show on the 3D plane as before
        TargetRenderer.material.mainTexture = texture;
        TargetRenderer.material.color = Color.white;
        Debug.Log("Runtime material name: " + TargetRenderer.material.name);
        Debug.Log("Runtime shader: " + TargetRenderer.material.shader.name);
        TargetRenderer.enabled = true;

        // ALSO show directly in UI for comparison
        if (DebugRawImage != null)
        {
            DebugRawImage.texture = texture;
            DebugRawImage.color = Color.white;

            // Keep aspect ratio
            RectTransform rt = DebugRawImage.rectTransform;
            float aspect = (float)width / height;
            float maxWidth = 1000f;
            float maxHeight = 700f;

            float uiWidth = maxWidth;
            float uiHeight = uiWidth / aspect;

            if (uiHeight > maxHeight)
            {
                uiHeight = maxHeight;
                uiWidth = uiHeight * aspect;
            }

            rt.sizeDelta = new Vector2(uiWidth, uiHeight);
        }

        AdjustRendererScale(width, height);
        timeSinceLastRender = 0.0f;
    }

    private byte[] ImageCorrection(int width, int height, byte[] src)
    {
        byte[] dst = new byte[src.Length];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int srcIndex = (y * width + x) * 3;

                // Vertical flip only
                int dstX = x;
                int dstY = height - 1 - y;
                int dstIndex = (dstY * width + dstX) * 3;

                // BGR -> RGB
                dst[dstIndex + 0] = src[srcIndex + 2]; // R
                dst[dstIndex + 1] = src[srcIndex + 1]; // G
                dst[dstIndex + 2] = src[srcIndex + 0]; // B
            }
        }

        return dst;
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