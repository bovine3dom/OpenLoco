#define FXAA_PC 1
#define FXAA_HLSL_5 1
#define FXAA_GREEN_AS_LUMA 1
#define FXAA_QUALITY__PRESET 12

#include "Fxaa3_11.h"

Texture2D colorTexture : register(t0, space2);
SamplerState colorSampler : register(s0, space2);

cbuffer Parameters : register(b0, space3)
{
    float2 reciprocalFrame;
    float2 padding;
};

struct PSInput
{
    float4 color : COLOR0;
    float2 uv : TEXCOORD0;
};

float4 main(PSInput input) : SV_Target0
{
    FxaaTex texture;
    texture.tex = colorTexture;
    texture.smpl = colorSampler;

    float4 color = FxaaPixelShader(
        input.uv,
        0.0,
        texture,
        texture,
        texture,
        texture,
        reciprocalFrame,
        0.0,
        0.0,
        0.0,
        0.75,
        0.125,
        0.0,
        8.0,
        0.125,
        0.05,
        0.0);
    color.a = 1.0;
    return color * input.color;
}
