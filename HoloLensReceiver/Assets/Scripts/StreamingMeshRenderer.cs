using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class StreamingMeshRenderer : MonoBehaviour
{
    [Header("Settings")]
    public float scale = 1f;
    public Vector3 positionOffset = new Vector3(0, 0, 2f);
    public bool flipX = true;

    [Tooltip("If true, turns off backface culling to prevent holes in the mesh.")]
    public bool doubleSided = true;

    [Header("Performance & Looks")]
    [Tooltip("MUST be checked if using a Lit/Standard shader so light bounces correctly!")]
    public bool calculateNormals = true; 

    private Mesh mesh;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    private int networkPackets = 0;
    private int uniqueFrames = 0;
    private float lastLogTime = 0f;

    private int lastVertexCount = -1;
    private Vector3 lastFirstVertex = Vector3.zero;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        mesh = new Mesh();

        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        mesh.MarkDynamic();
        meshFilter.mesh = mesh;

        if (meshRenderer.sharedMaterial == null)
        {

            Shader shader = Shader.Find("Standard");
            if (shader != null)
            {
                meshRenderer.material = new Material(shader);
            }
        }


        if (doubleSided && meshRenderer.material.HasProperty("_Cull"))
        {
            meshRenderer.material.SetFloat("_Cull", (float)UnityEngine.Rendering.CullMode.Off);
        }

        //transform.localPosition = positionOffset;
        //transform.localScale = new Vector3(flipX ? -scale : scale, scale, scale);

        lastLogTime = Time.realtimeSinceStartup;
    }

    public void EnqueueMesh(Vector3[] vertices, Color32[] colors, int[] triangles)
    {
        networkPackets++;

        bool isDuplicate = false;
        if (vertices != null && vertices.Length == lastVertexCount && vertices.Length > 0)
        {
            if (vertices[0] == lastFirstVertex)
            {
                isDuplicate = true;
            }
        }

        float currentTime = Time.realtimeSinceStartup;
        float timeElapsed = currentTime - lastLogTime;

        if (timeElapsed >= 1.0f)
        {
            float netFPS = networkPackets / timeElapsed;
            float realFPS = uniqueFrames / timeElapsed;

            Debug.Log($"[FPS] Network Receives: {netFPS:F1} fps");

            networkPackets = 0;
            uniqueFrames = 0;
            lastLogTime = currentTime;
        }

        if (isDuplicate) return;

        lastVertexCount = vertices.Length;
        if (vertices.Length > 0) lastFirstVertex = vertices[0];
        uniqueFrames++;

        mesh.Clear(false);

        mesh.SetVertices(vertices);
        mesh.SetColors(colors);

        // Guard: drop any triangle that references an out-of-bounds vertex.
        // This can happen when vertices and indices arrive from different frames
        // due to a race condition on the server side (frame N vertices + frame N+1 indices).
        int vc = vertices.Length;
        bool anyBad = false;
        for (int i = 0; i + 2 < triangles.Length; i += 3)
        {
            if (triangles[i] >= vc || triangles[i + 1] >= vc || triangles[i + 2] >= vc)
            {
                anyBad = true;
                break;
            }
        }

        int[] safeTriangles = triangles;
        if (anyBad)
        {
            var safe = new System.Collections.Generic.List<int>(triangles.Length);
            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                if (triangles[i] < vc && triangles[i + 1] < vc && triangles[i + 2] < vc)
                {
                    safe.Add(triangles[i]);
                    safe.Add(triangles[i + 1]);
                    safe.Add(triangles[i + 2]);
                }
            }
            safeTriangles = safe.ToArray();
        }

        mesh.SetTriangles(safeTriangles, 0);

        mesh.RecalculateBounds();

        if (calculateNormals)
        {
            mesh.RecalculateNormals();
        }
    }
}