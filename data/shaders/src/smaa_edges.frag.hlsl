Texture2D colorTex : register(t0, space2);
SamplerState colorTexSampler : register(s0, space2);

#include "SMAACommon.hlsli"

struct PSInput
{
    float4 color : COLOR0;
    float2 uv : TEXCOORD0;
};

float4 main(PSInput input) : SV_Target0
{
    float4 offsets[3];
    SMAAEdgeDetectionVS(input.uv, offsets);
    SMAATexture color = { colorTex, colorTexSampler };
    return float4(SMAAColorEdgeDetectionPS(input.uv, offsets, color), 0.0, 1.0);
}
