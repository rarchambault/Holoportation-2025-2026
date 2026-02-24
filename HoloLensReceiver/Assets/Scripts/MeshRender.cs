/*using System.Collections.Generic;
using System.IO;
using UnityEngine;

[System.Serializable]
public class Vertex
{
    public float x, y, z;
}

[System.Serializable]
public class ColorRGB
{
    public float r, g, b;
}

[System.Serializable]
public class TriangleMesh
{
    public Vertex[] vertices;
    public ColorRGB[] colors;
    public int[] triangles; // flattened
}

public class MeshRender : MonoBehaviour
{
    public string jsonFilePath = "Assets/frame_cam2.json";
    public float scale = 1f;
    public bool flipTriangles = false;
    public bool doubleSided = true;

    void Start()
    {
        string jsonText = File.ReadAllText(jsonFilePath);
        TriangleMesh meshData = JsonUtility.FromJson<TriangleMesh>(jsonText);

        // Vertices
        Vector3[] vertices = new Vector3[meshData.vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = new Vector3(
                meshData.vertices[i].x,
                meshData.vertices[i].y,
                meshData.vertices[i].z
            ) * scale;
        }

        // Colors
        Color[] colors = new Color[meshData.colors.Length];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = new Color(
                meshData.colors[i].r / 255f,
                meshData.colors[i].g / 255f,
                meshData.colors[i].b / 255f
            );
        }

        // Triangles
        int[] triangles = meshData.triangles;

        if (flipTriangles)
        {
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int temp = triangles[i];
                triangles[i] = triangles[i + 1];
                triangles[i + 1] = temp;
            }
        }

        // Build mesh
        GameObject go = new GameObject("MeshFromJSON");
        Mesh mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.colors = colors;

        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        mesh.RecalculateBounds();

        MeshFilter mf = go.AddComponent<MeshFilter>();
        mf.mesh = mesh;

        MeshRenderer mr = go.AddComponent<MeshRenderer>();

        // Use vertex-color-compatible material
        mr.material = new Material(Shader.Find("Particles/Standard Unlit"));

        if (doubleSided)
            mr.material.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
    }
}*/
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[System.Serializable]
public class Vertex
{
    public float x, y, z;
}

[System.Serializable]
public class ColorRGB
{
    public float r, g, b;
}

[System.Serializable]
public class TriangleMesh
{
    public Vertex[] vertices;
    public ColorRGB[] colors;
    public int[] triangles; // flattened
}

public class MeshRender : MonoBehaviour
{
    public string jsonFilePath = "Assets/frame_cam2.json";
    public float scale = 1f;
    public bool flipTriangles = false;
    public bool doubleSided = true;

    void Start()
    {
        if (!File.Exists(jsonFilePath))
        {
            Debug.LogError("File not found: " + jsonFilePath);
            return;
        }

        string jsonText = File.ReadAllText(jsonFilePath);
        TriangleMesh meshData = JsonUtility.FromJson<TriangleMesh>(jsonText);

        if (meshData == null || meshData.vertices == null)
        {
            Debug.LogError("JSON data is empty or invalid.");
            return;
        }

        // Vertices
        Vector3[] vertices = new Vector3[meshData.vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            // Note: Unity is Left-Handed (Y-up), PCL is usually Right-Handed. 
            // You might need to flip X or Z depending on your calibration.
            // Usually simply negating X is enough for PCL -> Unity.
            vertices[i] = new Vector3(
                -meshData.vertices[i].x, // Try negating X if model is mirrored
                meshData.vertices[i].y,
                meshData.vertices[i].z
            ) * scale;
        }

        // Colors
        Color[] colors = new Color[meshData.colors.Length];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = new Color(
                meshData.colors[i].r / 255f,
                meshData.colors[i].g / 255f,
                meshData.colors[i].b / 255f
            );
        }

        // Triangles
        int[] triangles = meshData.triangles;

        if (flipTriangles)
        {
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int temp = triangles[i];
                triangles[i] = triangles[i + 1];
                triangles[i + 1] = temp;
            }
        }

        // Build mesh
        GameObject go = new GameObject("MeshFromJSON_" + System.DateTime.Now.Ticks);
        Mesh mesh = new Mesh();

        // *** CRITICAL FIX FOR MARCHING CUBES ***
        // Allows meshes larger than 65k vertices
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.colors = colors;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        MeshFilter mf = go.AddComponent<MeshFilter>();
        mf.mesh = mesh;

        MeshRenderer mr = go.AddComponent<MeshRenderer>();

        // Ensure the shader handles vertex colors
        mr.material = new Material(Shader.Find("Particles/Standard Unlit"));

        if (doubleSided)
        {
            mr.material.SetFloat("_Cull", (float)UnityEngine.Rendering.CullMode.Off);
        }

        Debug.Log($"Loaded Mesh: {vertices.Length} vertices, {triangles.Length / 3} triangles.");
    }
}
