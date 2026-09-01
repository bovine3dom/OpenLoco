#define SMAA_PRESET_HIGH
#define SMAA_CUSTOM_SL 1

cbuffer Parameters : register(b0, space3)
{
    float4 renderTargetMetrics;
};

#define SMAA_RT_METRICS renderTargetMetrics
struct SMAATexture
{
    Texture2D texture;
    SamplerState sampler;
};

#define SMAATexture2D(tex) SMAATexture tex
#define SMAATexturePass2D(tex) tex
#define SMAASampleLevelZero(tex, coord) tex.texture.SampleLevel(tex.sampler, coord, 0)
#define SMAASampleLevelZeroPoint(tex, coord) tex.texture.SampleLevel(tex.sampler, coord, 0)
#define SMAASampleLevelZeroOffset(tex, coord, offset) tex.texture.SampleLevel(tex.sampler, coord, 0, offset)
#define SMAASample(tex, coord) tex.texture.Sample(tex.sampler, coord)
#define SMAASamplePoint(tex, coord) tex.texture.Sample(tex.sampler, coord)
#define SMAASampleOffset(tex, coord, offset) tex.texture.Sample(tex.sampler, coord, offset)
#define SMAA_FLATTEN [flatten]
#define SMAA_BRANCH [branch]

#include "SMAA.hlsl"
