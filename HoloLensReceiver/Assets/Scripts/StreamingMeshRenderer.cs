using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class StreamingMeshRenderer : MonoBehaviour
{
    [Header("Settings")]
    public float scale = 1f;
    public Vector3 positionOffset = new Vector3(0, 0, 2f);
    public bool flipX = true;

    [Header("Performance & Looks")]
    [Tooltip("Required if using the Surface shader to calculate lighting and shadows.")]
    public bool calculateNormals = true;

    private Mesh mesh;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    // --- TRUE FPS TRACKING ---
    private int networkPackets = 0;
    private int uniqueFrames = 0;
    private float lastLogTime = 0f;

    // --- FILTER MEMORY ---
    private int lastVertexCount = -1;
    private Vector3 lastFirstVertex = Vector3.zero;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        mesh = new Mesh();
        // Allow meshes larger than 65k vertices
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        // Optimize the mesh for frequent frame-by-frame updates
        mesh.MarkDynamic();
        meshFilter.mesh = mesh;

        // Apply the upgraded Surface shader for better 3D depth and lighting
        var shader = Shader.Find("Particles/Standard Surface");
        if (meshRenderer.sharedMaterial == null)
        {
            if (shader != null) meshRenderer.material = new Material(shader);
            else meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        // Apply Transforms (Hardware acceleration instead of C# loops)
        transform.localPosition = positionOffset;
        transform.localScale = new Vector3(flipX ? -scale : scale, scale, scale);

        // Initialize the timer
        lastLogTime = Time.realtimeSinceStartup;
    }

    public void EnqueueMesh(Vector3[] vertices, Color32[] colors, int[] triangles)
    {
        // 1. ALWAYS Count the network packet
        networkPackets++;

        // 2. CHECK FOR DUPLICATES
        // If the vertex count AND the exact position of the first vertex match the last frame,
        // it is a ghost frame sent by the server. 
        bool isDuplicate = false;
        if (vertices != null && vertices.Length == lastVertexCount && vertices.Length > 0)
        {
            if (vertices[0] == lastFirstVertex)
            {
                isDuplicate = true;
            }
        }

        // 3. TRUE LOGGING (Based on real-world time)
        float currentTime = Time.realtimeSinceStartup;
        float timeElapsed = currentTime - lastLogTime;

        if (timeElapsed >= 1.0f)
        {
            // Calculate FPS based on exactly how much real time passed
            float netFPS = networkPackets / timeElapsed;
            float realFPS = uniqueFrames / timeElapsed;

            Debug.Log($"[True System FPS] Network Receives: {netFPS:F1}/sec | New Meshes Rendered: {realFPS:F1}/sec");

            // Reset counters
            networkPackets = 0;
            uniqueFrames = 0;
            lastLogTime = currentTime;
        }

        // 4. IF DUPLICATE, STOP HERE
        if (isDuplicate) return;

        // 5. UPDATE MESH (Only for brand new data)
        lastVertexCount = vertices.Length;
        if (vertices.Length > 0) lastFirstVertex = vertices[0];
        uniqueFrames++;

        // False keeps the memory layout, which is slightly faster for dynamic meshes
        mesh.Clear(false);

        mesh.SetVertices(vertices);
        mesh.SetColors(colors);
        mesh.SetTriangles(triangles, 0);

        mesh.RecalculateBounds();

        // Calculate normals so the "Standard Surface" shader can draw shadows
        if (calculateNormals)
        {
            mesh.RecalculateNormals();
        }
    }
}