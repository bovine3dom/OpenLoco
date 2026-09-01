Texture2D edgesTex : register(t0, space2);
SamplerState edgesTexSampler : register(s0, space2);
Texture2D areaTex : register(t1, space2);
SamplerState areaTexSampler : register(s1, space2);
Texture2D searchTex : register(t2, space2);
SamplerState searchTexSampler : register(s2, space2);

#include "SMAACommon.hlsli"

struct PSInput
{
    float4 color : COLOR0;
    float2 uv : TEXCOORD0;
};

float4 main(PSInput input) : SV_Target0
{
    float2 pixelCoordinate;
    float4 offsets[3];
    SMAABlendingWeightCalculationVS(input.uv, pixelCoordinate, offsets);
    SMAATexture edges = { edgesTex, edgesTexSampler };
    SMAATexture area = { areaTex, areaTexSampler };
    SMAATexture search = { searchTex, searchTexSampler };
    return SMAABlendingWeightCalculationPS(
        input.uv,
        pixelCoordinate,
        offsets,
        edges,
        area,
        search,
        0.0);
}
