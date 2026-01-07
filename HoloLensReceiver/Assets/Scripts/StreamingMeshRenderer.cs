using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class StreamingMeshRenderer : MonoBehaviour
{
    [Tooltip("Scale applied to incoming vertices (optional).")]
    public float scale = 1f;

    [Tooltip("If true, disable back-face culling so mesh is double-sided.")]
    public bool doubleSided = true;

    [Tooltip("Optional offset to move the whole mesh in front of the camera.")]
    public Vector3 positionOffset = new Vector3(0, 0, 2f);

    [Tooltip("Flip X coordinate (PCL -> Unity mirroring).")]
    public bool flipX = true;

    private Mesh mesh;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        // Create the mesh once, reuse it for streaming updates
        mesh = new Mesh();
        // *** CRITICAL FIX FOR MARCHING CUBES ***
        // Allows meshes larger than 65k vertices
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        meshFilter.mesh = mesh;

        // Ensure the shader handles vertex colors
        var shader = Shader.Find("Particles/Standard Unlit");
        if (meshRenderer.sharedMaterial == null)
        {
            if (shader != null)
            {
                meshRenderer.material = new Material(shader);
            }
            else
            {
                // Fallback so we at least see *something* even if shader name is wrong
                Debug.LogWarning("StreamingMeshRenderer: Shader 'Particles/Standard Unlit' not found. Using default material.");
                meshRenderer.material = new Material(Shader.Find("Standard"));
            }
        }

        if (doubleSided && meshRenderer.material != null)
        {
            meshRenderer.material.SetFloat("_Cull", (float)UnityEngine.Rendering.CullMode.Off);
        }

        Debug.Log("StreamingMeshRenderer Awake: mesh + material initialized.");
    }

    /// <summary>
    /// Called by HoloportReceiver each time a new mesh frame arrives.
    /// </summary>
    public void EnqueueMesh(Vector3[] vertices, Color32[] colors32, int[] triangles)
    {
        if (vertices == null || colors32 == null || triangles == null ||
            vertices.Length == 0 || triangles.Length == 0)
        {
            Debug.LogWarning("StreamingMeshRenderer: empty mesh data, skipping.");
            return;
        }

        // Optional scale & coordinate tweaks, mirroring your old MeshRender logic
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 v = vertices[i];

            if (flipX)
                v.x = -v.x;              // PCL -> Unity mirror like your old script

            if (!Mathf.Approximately(scale, 1f))
                v *= scale;

            v += positionOffset;          // Move mesh in front of camera

            vertices[i] = v;
        }

        // Convert Color32[] to Color[] because your old script used mesh.colors
        Color[] colors = new Color[colors32.Length];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = colors32[i];
        }

        // *** This block mirrors your old MeshRender logic as closely as possible ***
        mesh.Clear();

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.colors = colors;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        Debug.Log($"StreamingMeshRenderer: updated mesh " +
                  $"(verts={mesh.vertexCount}, tris={mesh.triangles.Length / 3}, " +
                  $"bounds center={mesh.bounds.center}, size={mesh.bounds.size})");

    }
}
