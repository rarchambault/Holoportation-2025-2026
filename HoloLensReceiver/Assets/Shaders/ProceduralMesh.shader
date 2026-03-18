Shader "Custom/ProceduralMarchingCubes"
{
    Properties
    {
        _Color ("Main Color Multiplier", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Cull Off 

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct Triangle {
                float3 vertexA;
                float3 vertexB;
                float3 vertexC;
                float3 padding;
                float4 color;
            };

            StructuredBuffer<Triangle> TriangleBuffer;
            float4 _Color;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float3 worldNormal : TEXCOORD0;
                float4 color : COLOR;
            };

            v2f vert (uint vertexID : SV_VertexID, uint instanceID : SV_InstanceID)
            {
                v2f o;
                Triangle tri = TriangleBuffer[instanceID];
    
                float3 vPos;
                if (vertexID == 0) vPos = tri.vertexA;
                else if (vertexID == 1) vPos = tri.vertexB;
                else vPos = tri.vertexC;

                // Removed the *10.0 scale for final render
                o.pos = mul(UNITY_MATRIX_VP, float4(vPos, 1.0));
    
                float3 edge1 = tri.vertexB - tri.vertexA;
                float3 edge2 = tri.vertexC - tri.vertexA;
                o.worldNormal = normalize(cross(edge1, edge2));

                o.color = tri.color; 
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float3 lightDir = normalize(float3(0.5, 1.0, 0.5));
                float NdotL = max(0.3, dot(i.worldNormal, lightDir)); // 0.3 ambient
                
                // Final color is the point cloud color * lighting * inspector multiplier
                return i.color * NdotL * _Color;
            }
            ENDCG
        }
    }
}