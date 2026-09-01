#include "PostProcessor.h"
#include "Environment.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <array>
#include <fstream>
#include <string>
#include <vector>

using namespace OpenLoco::Diagnostics;

namespace OpenLoco::Gfx
{
    namespace
    {
        struct ShaderAsset
        {
            SDL_GPUShaderFormat format;
            const char* extension;
            const char* entrypoint;
        };

        std::vector<uint8_t> readFile(const fs::path& path)
        {
            std::ifstream stream(path, std::ios::binary | std::ios::ate);
            if (!stream)
            {
                return {};
            }

            const auto size = stream.tellg();
            if (size <= 0)
            {
                return {};
            }

            std::vector<uint8_t> data(static_cast<size_t>(size));
            stream.seekg(0);
            stream.read(reinterpret_cast<char*>(data.data()), size);
            return stream ? data : std::vector<uint8_t>{};
        }
    }

    PostProcessor::~PostProcessor()
    {
        releaseRendererResources();
    }

    bool PostProcessor::isSupported(SDL_Renderer* renderer)
    {
        if (renderer == nullptr)
        {
            return false;
        }

        const auto* name = SDL_GetRendererName(renderer);
        if (name == nullptr || std::string_view(name) != SDL_GPU_RENDERER)
        {
            return false;
        }

        auto* device = SDL_GetGPURendererDevice(renderer);
        return device != nullptr
            && (SDL_GetGPUShaderFormats(device) & (SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL)) != 0;
    }

    bool PostProcessor::configure(SDL_Renderer* renderer, int32_t width, int32_t height, Config::AntiAliasing mode)
    {
        auto* device = isSupported(renderer) ? SDL_GetGPURendererDevice(renderer) : nullptr;
        if (renderer != _renderer || device != _device)
        {
            releaseRendererResources();
            _renderer = renderer;
            _device = device;
        }
        else
        {
            reset();
        }

        if (mode == Config::AntiAliasing::none)
        {
            return true;
        }
        if (width <= 0 || height <= 0 || _device == nullptr)
        {
            return false;
        }

        _colourTexture = createTarget(width, height);
        if (_colourTexture == nullptr)
        {
            reset();
            return false;
        }

        bool success = false;
        switch (mode)
        {
            case Config::AntiAliasing::fxaa:
                success = createFxaaResources(width, height);
                break;

            case Config::AntiAliasing::smaa:
                success = createSmaaResources(width, height);
                break;

            case Config::AntiAliasing::none:
                break;
        }

        if (!success)
        {
            Logging::warn("Unable to initialise anti-aliasing: {}", SDL_GetError());
            reset();
            return false;
        }

        _mode = mode;
        Logging::info("Using {} anti-aliasing", mode == Config::AntiAliasing::fxaa ? "FXAA" : "SMAA");
        return true;
    }

    SDL_GPUShader* PostProcessor::createShader(std::string_view name, uint32_t samplerCount)
    {
        const auto formats = SDL_GetGPUShaderFormats(_device);
        constexpr std::array kAssets = {
            ShaderAsset{ SDL_GPU_SHADERFORMAT_SPIRV, ".frag.spv", "main" },
            ShaderAsset{ SDL_GPU_SHADERFORMAT_DXIL, ".frag.dxil", "main" },
            ShaderAsset{ SDL_GPU_SHADERFORMAT_MSL, ".frag.msl", "main0" },
        };

        for (const auto& asset : kAssets)
        {
            if ((formats & asset.format) == 0)
            {
                continue;
            }

            const auto path = Environment::getPathNoWarning(Environment::PathId::shaders) / (std::string(name) + asset.extension);
            const auto code = readFile(path);
            if (code.empty())
            {
                Logging::warn("Unable to load anti-aliasing shader {}", path.u8string());
                continue;
            }

            SDL_GPUShaderCreateInfo info{};
            info.code = code.data();
            info.code_size = code.size();
            info.entrypoint = asset.entrypoint;
            info.format = asset.format;
            info.stage = SDL_GPU_SHADERSTAGE_FRAGMENT;
            info.num_samplers = samplerCount;
            info.num_uniform_buffers = 1;
            if (auto* shader = SDL_CreateGPUShader(_device, &info); shader != nullptr)
            {
                return shader;
            }
        }
        return nullptr;
    }

    SDL_GPURenderState* PostProcessor::createState(SDL_GPUShader* shader, std::span<SDL_Texture* const> additionalTextures)
    {
        if (shader == nullptr)
        {
            return nullptr;
        }

        std::vector<SDL_GPUTextureSamplerBinding> bindings;
        bindings.reserve(additionalTextures.size());
        for (auto* texture : additionalTextures)
        {
            auto* gpuTexture = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(
                SDL_GetTextureProperties(texture), SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
            if (gpuTexture == nullptr)
            {
                return nullptr;
            }
            bindings.push_back({ gpuTexture, _linearSampler });
        }

        SDL_GPURenderStateCreateInfo info{};
        info.fragment_shader = shader;
        info.num_sampler_bindings = static_cast<int32_t>(bindings.size());
        info.sampler_bindings = bindings.data();
        return SDL_CreateGPURenderState(_renderer, &info);
    }

    SDL_Texture* PostProcessor::createTarget(int32_t width, int32_t height)
    {
        auto* texture = SDL_CreateTexture(_renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_TARGET, width, height);
        if (texture == nullptr
            || !SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE)
            || !SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_LINEAR))
        {
            SDL_DestroyTexture(texture);
            return nullptr;
        }
        return texture;
    }

    SDL_Texture* PostProcessor::createLookupTexture(std::string_view filename, int32_t width, int32_t height, int32_t channels)
    {
        const auto path = Environment::getPathNoWarning(Environment::PathId::shaders) / filename;
        const auto source = readFile(path);
        if (source.size() != static_cast<size_t>(width * height * channels))
        {
            Logging::warn("Invalid anti-aliasing lookup texture {}", path.u8string());
            return nullptr;
        }

        std::vector<SDL_Color> pixels(static_cast<size_t>(width * height));
        for (size_t i = 0; i < pixels.size(); ++i)
        {
            pixels[i] = {
                source[i * channels],
                channels > 1 ? source[i * channels + 1] : uint8_t{},
                0,
                255,
            };
        }

        auto* texture = SDL_CreateTexture(_renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, width, height);
        if (texture == nullptr
            || !SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE)
            || !SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_LINEAR)
            || !SDL_UpdateTexture(texture, nullptr, pixels.data(), width * sizeof(SDL_Color)))
        {
            SDL_DestroyTexture(texture);
            return nullptr;
        }
        return texture;
    }

    bool PostProcessor::createFxaaResources(int32_t width, int32_t height)
    {
        if (_fxaaShader == nullptr)
        {
            _fxaaShader = createShader("fxaa", 1);
        }
        _fxaaState = createState(_fxaaShader);
        if (_fxaaState == nullptr)
        {
            return false;
        }

        const std::array metrics = { 1.0F / width, 1.0F / height, static_cast<float>(width), static_cast<float>(height) };
        return SDL_SetGPURenderStateFragmentUniforms(_fxaaState, 0, metrics.data(), sizeof(metrics));
    }

    bool PostProcessor::createSmaaResources(int32_t width, int32_t height)
    {
        _edgesTexture = createTarget(width, height);
        _weightsTexture = createTarget(width, height);
        _areaTexture = createLookupTexture("smaa_area.raw", 160, 560, 2);
        _searchTexture = createLookupTexture("smaa_search.raw", 64, 16, 1);

        if (_linearSampler == nullptr)
        {
            SDL_GPUSamplerCreateInfo samplerInfo{};
            samplerInfo.min_filter = SDL_GPU_FILTER_LINEAR;
            samplerInfo.mag_filter = SDL_GPU_FILTER_LINEAR;
            samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
            samplerInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            samplerInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            _linearSampler = SDL_CreateGPUSampler(_device, &samplerInfo);
        }

        if (_edgesTexture == nullptr || _weightsTexture == nullptr || _areaTexture == nullptr || _searchTexture == nullptr || _linearSampler == nullptr)
        {
            return false;
        }

        if (_smaaEdgesShader == nullptr)
        {
            _smaaEdgesShader = createShader("smaa_edges", 1);
        }
        if (_smaaWeightsShader == nullptr)
        {
            _smaaWeightsShader = createShader("smaa_weights", 3);
        }
        if (_smaaBlendShader == nullptr)
        {
            _smaaBlendShader = createShader("smaa_blend", 2);
        }
        _smaaEdgesState = createState(_smaaEdgesShader);
        const std::array weightTextures = { _areaTexture, _searchTexture };
        _smaaWeightsState = createState(_smaaWeightsShader, weightTextures);
        const std::array blendTextures = { _weightsTexture };
        _smaaBlendState = createState(_smaaBlendShader, blendTextures);

        if (_smaaEdgesState == nullptr || _smaaWeightsState == nullptr || _smaaBlendState == nullptr)
        {
            return false;
        }

        const std::array metrics = { 1.0F / width, 1.0F / height, static_cast<float>(width), static_cast<float>(height) };
        return SDL_SetGPURenderStateFragmentUniforms(_smaaEdgesState, 0, metrics.data(), sizeof(metrics))
            && SDL_SetGPURenderStateFragmentUniforms(_smaaWeightsState, 0, metrics.data(), sizeof(metrics))
            && SDL_SetGPURenderStateFragmentUniforms(_smaaBlendState, 0, metrics.data(), sizeof(metrics));
    }

    bool PostProcessor::render(SDL_Texture* source)
    {
        switch (_mode)
        {
            case Config::AntiAliasing::fxaa:
                return renderFxaa(source);

            case Config::AntiAliasing::smaa:
                return renderSmaa(source);

            case Config::AntiAliasing::none:
                return SDL_RenderTexture(_renderer, source, nullptr, nullptr);
        }
        return false;
    }

    bool PostProcessor::renderFxaa(SDL_Texture* source)
    {
        if (!SDL_SetRenderTarget(_renderer, _colourTexture)
            || !SDL_SetGPURenderState(_renderer, nullptr)
            || !SDL_RenderTexture(_renderer, source, nullptr, nullptr)
            || !SDL_SetRenderTarget(_renderer, nullptr)
            || !SDL_SetGPURenderState(_renderer, _fxaaState)
            || !SDL_RenderTexture(_renderer, _colourTexture, nullptr, nullptr)
            || !SDL_SetGPURenderState(_renderer, nullptr))
        {
            SDL_SetGPURenderState(_renderer, nullptr);
            SDL_SetRenderTarget(_renderer, nullptr);
            return false;
        }
        return true;
    }

    bool PostProcessor::renderSmaa(SDL_Texture* source)
    {
        float previousR{};
        float previousG{};
        float previousB{};
        float previousA{};
        SDL_GetRenderDrawColorFloat(_renderer, &previousR, &previousG, &previousB, &previousA);

        bool success = SDL_SetRenderTarget(_renderer, _colourTexture)
            && SDL_SetGPURenderState(_renderer, nullptr)
            && SDL_RenderTexture(_renderer, source, nullptr, nullptr)
            && SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 0)
            && SDL_SetRenderTarget(_renderer, _edgesTexture)
            && SDL_RenderClear(_renderer)
            && SDL_SetTextureScaleMode(_colourTexture, SDL_SCALEMODE_NEAREST)
            && SDL_SetGPURenderState(_renderer, _smaaEdgesState)
            && SDL_RenderTexture(_renderer, _colourTexture, nullptr, nullptr)
            && SDL_SetGPURenderState(_renderer, nullptr)
            && SDL_SetTextureScaleMode(_colourTexture, SDL_SCALEMODE_LINEAR)
            && SDL_SetRenderTarget(_renderer, _weightsTexture)
            && SDL_RenderClear(_renderer)
            && SDL_SetGPURenderState(_renderer, _smaaWeightsState)
            && SDL_RenderTexture(_renderer, _edgesTexture, nullptr, nullptr)
            && SDL_SetGPURenderState(_renderer, nullptr)
            && SDL_SetRenderTarget(_renderer, nullptr)
            && SDL_SetGPURenderState(_renderer, _smaaBlendState)
            && SDL_RenderTexture(_renderer, _colourTexture, nullptr, nullptr)
            && SDL_SetGPURenderState(_renderer, nullptr);

        SDL_SetGPURenderState(_renderer, nullptr);
        SDL_SetRenderTarget(_renderer, nullptr);
        SDL_SetRenderDrawColorFloat(_renderer, previousR, previousG, previousB, previousA);
        return success;
    }

    void PostProcessor::reset()
    {
        if (_renderer != nullptr && _device != nullptr)
        {
            SDL_SetGPURenderState(_renderer, nullptr);
        }

        SDL_DestroyGPURenderState(_fxaaState);
        SDL_DestroyGPURenderState(_smaaEdgesState);
        SDL_DestroyGPURenderState(_smaaWeightsState);
        SDL_DestroyGPURenderState(_smaaBlendState);
        SDL_DestroyTexture(_colourTexture);
        SDL_DestroyTexture(_edgesTexture);
        SDL_DestroyTexture(_weightsTexture);
        SDL_DestroyTexture(_areaTexture);
        SDL_DestroyTexture(_searchTexture);

        _mode = Config::AntiAliasing::none;
        _colourTexture = nullptr;
        _edgesTexture = nullptr;
        _weightsTexture = nullptr;
        _areaTexture = nullptr;
        _searchTexture = nullptr;
        _fxaaState = nullptr;
        _smaaEdgesState = nullptr;
        _smaaWeightsState = nullptr;
        _smaaBlendState = nullptr;
    }

    void PostProcessor::releaseRendererResources()
    {
        reset();
        if (_device != nullptr)
        {
            SDL_ReleaseGPUShader(_device, _fxaaShader);
            SDL_ReleaseGPUShader(_device, _smaaEdgesShader);
            SDL_ReleaseGPUShader(_device, _smaaWeightsShader);
            SDL_ReleaseGPUShader(_device, _smaaBlendShader);
            SDL_ReleaseGPUSampler(_device, _linearSampler);
        }

        _renderer = nullptr;
        _device = nullptr;
        _linearSampler = nullptr;
        _fxaaShader = nullptr;
        _smaaEdgesShader = nullptr;
        _smaaWeightsShader = nullptr;
        _smaaBlendShader = nullptr;
    }
}
