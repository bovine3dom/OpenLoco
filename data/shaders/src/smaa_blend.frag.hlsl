Texture2D colorTex : register(t0, space2);
SamplerState colorTexSampler : register(s0, space2);
Texture2D blendTex : register(t1, space2);
SamplerState blendTexSampler : register(s1, space2);

#include "SMAACommon.hlsli"

struct PSInput
{
    float4 color : COLOR0;
    float2 uv : TEXCOORD0;
};

float4 main(PSInput input) : SV_Target0
{
    float4 offset;
    SMAANeighborhoodBlendingVS(input.uv, offset);
    SMAATexture color = { colorTex, colorTexSampler };
    SMAATexture blend = { blendTex, blendTexSampler };
    return SMAANeighborhoodBlendingPS(input.uv, offset, color, blend) * input.color;
}
