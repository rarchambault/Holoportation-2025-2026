using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class StreamingMeshRenderer : MonoBehaviour
{
    [Header("Settings")]
    public float scale = 1f;
    public Vector3 positionOffset = new Vector3(0, 0, 2f);
    public bool flipX = true;

    [Header("Performance")]
    public bool calculateNormals = false;

    private Mesh mesh;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    // --- FPS & FILTER VARIABLES ---
    private int framesReceived = 0;
    private float fpsTimer = 0f;

    // We use these to detect if the frame is identical to the last one
    private int lastVertexCount = -1;
    private Vector3 lastFirstVertex = Vector3.zero;
    // ------------------------------

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        mesh = new Mesh();
        // Allow large meshes (essential for point clouds)
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.MarkDynamic(); // Optimize for frequent updates

        meshFilter.mesh = mesh;

        // Apply Shader (Unlit is faster)
        var shader = Shader.Find("Particles/Standard Unlit");
        if (meshRenderer.sharedMaterial == null)
        {
            if (shader != null) meshRenderer.material = new Material(shader);
            else meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        // Apply Transform once (Hardware acceleration)
        transform.localPosition = positionOffset;
        transform.localScale = new Vector3(flipX ? -scale : scale, scale, scale);
    }

    public void EnqueueMesh(Vector3[] vertices, Color32[] colors, int[] triangles)
    {
        // 1. SAFETY CHECKS
        if (vertices == null || vertices.Length == 0) return;

        // 2. DUPLICATE FRAME FILTER
        // If the vertex count AND the first vertex are exactly the same, 
        // it's a duplicate frame. We ignore it.
        if (vertices.Length == lastVertexCount && vertices[0] == lastFirstVertex)
        {
            // Do NOT update mesh. Do NOT increment FPS. Just exit.
            return;
        }

        // Update our "Last Frame" memory
        lastVertexCount = vertices.Length;
        lastFirstVertex = vertices[0];

        // 3. UPDATE MESH (Only runs if data is NEW)
        mesh.Clear(false);
        mesh.SetVertices(vertices);
        mesh.SetColors(colors);
        mesh.SetTriangles(triangles, 0);

        mesh.RecalculateBounds();

        if (calculateNormals)
            mesh.RecalculateNormals();

        // 4. TRUE FPS CALCULATION
        // This will now only count FRAMES THAT ACTUALLY CHANGED.
        fpsTimer += Time.deltaTime;
        framesReceived++;

        if (fpsTimer >= 1.0f)
        {
            Debug.Log($"[TRUE FPS] {framesReceived} fps | Verts: {mesh.vertexCount}");

            framesReceived = 0;
            fpsTimer = 0f;
        }
    }
}