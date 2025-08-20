/***************************************************************************\

Module Name:  PointCloudBillboard.shader
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This shader makes every point cloud vertex into a small quad plane which 
rotates to face the camera.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

Shader "Custom/PointCloudBillboard"
{
    Properties
    {
        _PointSize("Point Size", Float) = 0.02
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }
        Pass
        {
            Cull Off ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile _ UNITY_SINGLE_PASS_STEREO

            #include "UnityCG.cginc"

            // Size of each point (in world units)
            half _PointSize;

            // Input struct from the mesh
            struct VertexInput
            {
                float3 position : POSITION;
                half3 color : COLOR; // RGB only
                float2 uv : TEXCOORD0; // uv.x stores corner index (0–5)
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            // Output struct from the vertex shader to the fragment shader
            struct VertexOutput
            {
                float4 position : SV_POSITION;
                half3 color : COLOR;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            // Maps a quad corner index (0–5) to a normalized screen-space offset.
            // This defines a triangle strip forming a square particle billboard.
            float2 GetQuadCornerOffset(float cornerIndex)
            {
                if (cornerIndex == 0) return float2(-0.5, -0.5);
                if (cornerIndex == 1) return float2( 0.5, -0.5);
                if (cornerIndex == 2) return float2(-0.5,  0.5);
                if (cornerIndex == 3) return float2( 0.5, -0.5);
                if (cornerIndex == 4) return float2( 0.5,  0.5);
                return float2(-0.5, 0.5); // case 5
            }

            // Vertex shader: expands each point into a camera-facing quad in view space
            VertexOutput vert(VertexInput input)
            {
                VertexOutput output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                // Transform the vertex position from object space to view space
                // This accounts for both object rotation and camera view
                float4 viewPos = mul(UNITY_MATRIX_MV, float4(input.position, 1.0));

                // Get quad corner offset (e.g., -0.5 to +0.5 range)
                float2 baseOffset = GetQuadCornerOffset(input.uv.x);

                // Apply the 2D billboard offset in view space (camera-facing XY plane)
                // _PointSize is in world units but works in view space scale since projection handles perspective
                viewPos.xy += baseOffset * _PointSize;

                // Transform final view-space position to clip space (for rasterization)
                output.position = mul(UNITY_MATRIX_P, viewPos);

                // Pass vertex color through to fragment shader
                output.color = input.color;

                return output;
            }

            // Fragment shader: outputs point color (with forced full alpha).
            half4 frag(VertexOutput input) : SV_Target
            {
                return half4(input.color, 1.0); // Fully opaque
            }

            ENDCG
        }
    }
}