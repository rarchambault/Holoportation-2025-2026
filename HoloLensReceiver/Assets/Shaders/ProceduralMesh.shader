Shader "Custom/ProceduralMarchingCubes"
{
    Properties
    {
        _Color ("Main Color", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Cull Off // Double-sided rendering to avoid invisible backfaces

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            // Must match the struct in your C# and Compute Shader
            struct Triangle {
                float3 vertexC;
                float3 vertexB;
                float3 vertexA;
                float4 color; // Use float4 for colors in shaders
            };

            // The buffer passed from C#
            StructuredBuffer<Triangle> TriangleBuffer;
            float4 _Color;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float3 worldNormal : TEXCOORD0;
                float4 color : COLOR;
            };

            // The Vertex Shader runs once for every single vertex in the TriangleBuffer
            v2f vert (uint vertexID : SV_VertexID)
            {
                v2f o;
                
                // 1 Triangle = 3 Vertices. Find which triangle and which point we are on.
                int triIndex = vertexID / 3;
                int vertIndex = vertexID % 3;
                
                Triangle tri = TriangleBuffer[triIndex];
                
                float3 vPos;
                if (vertIndex == 0) vPos = tri.vertexA;
                else if (vertIndex == 1) vPos = tri.vertexB;
                else vPos = tri.vertexC;

                // Transform from World space to Screen space
                o.pos = UnityObjectToClipPos(float4(vPos, 1.0));
                
                // Calculate face normal for basic lighting (Cross product of edges)
                float3 edge1 = tri.vertexB - tri.vertexA;
                float3 edge2 = tri.vertexC - tri.vertexA;
                o.worldNormal = normalize(cross(edge1, edge2));

                o.color = tri.color; // Pass the point cloud color to the fragment shader
                return o;
            }

            // The Fragment Shader runs for every pixel on the screen that the triangle covers
            fixed4 frag (v2f i) : SV_Target
            {
                // Basic directional lighting so the mesh doesn't look completely flat
                float3 lightDir = normalize(float3(0.5, 1.0, 0.5));
                float NdotL = max(0.1, dot(i.worldNormal, lightDir)); // 0.1 is ambient light
                
                // Multiply the struct color by the lighting
                return i.color * NdotL * _Color;
            }
            ENDCG
        }
    }
}