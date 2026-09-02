#pragma once

#include "Config.h"
#include <SDL3/SDL.h>
#include <cstdint>
#include <span>
#include <string_view>

namespace OpenLoco::Gfx
{
    class PostProcessor
    {
    public:
        PostProcessor() = default;
        ~PostProcessor();

        PostProcessor(const PostProcessor&) = delete;
        PostProcessor& operator=(const PostProcessor&) = delete;

        static bool isSupported(SDL_Renderer* renderer);

        bool configure(SDL_Renderer* renderer, int32_t width, int32_t height, Config::AntiAliasing mode);
        SDL_Texture* process(SDL_Texture* source, const SDL_FRect* sourceRect);
        void reset();

        Config::AntiAliasing getMode() const { return _mode; }

    private:
        void releaseRendererResources();
        SDL_GPUShader* createShader(std::string_view name, uint32_t samplerCount);
        SDL_GPURenderState* createState(SDL_GPUShader* shader, std::span<SDL_Texture* const> additionalTextures = {});
        SDL_Texture* createTarget(int32_t width, int32_t height);
        SDL_Texture* createLookupTexture(std::string_view filename, int32_t width, int32_t height, int32_t channels);
        bool createFxaaResources(int32_t width, int32_t height);
        bool createSmaaResources(int32_t width, int32_t height);
        bool processFxaa(SDL_Texture* source, const SDL_FRect* sourceRect);
        bool processSmaa(SDL_Texture* source, const SDL_FRect* sourceRect);

        SDL_Renderer* _renderer{};
        SDL_GPUDevice* _device{};
        Config::AntiAliasing _mode = Config::AntiAliasing::none;
        SDL_Texture* _colourTexture{};
        SDL_Texture* _outputTexture{};
        SDL_Texture* _edgesTexture{};
        SDL_Texture* _weightsTexture{};
        SDL_Texture* _areaTexture{};
        SDL_Texture* _searchTexture{};
        SDL_GPUSampler* _linearSampler{};
        SDL_GPUShader* _fxaaShader{};
        SDL_GPUShader* _smaaEdgesShader{};
        SDL_GPUShader* _smaaWeightsShader{};
        SDL_GPUShader* _smaaBlendShader{};
        SDL_GPURenderState* _fxaaState{};
        SDL_GPURenderState* _smaaEdgesState{};
        SDL_GPURenderState* _smaaWeightsState{};
        SDL_GPURenderState* _smaaBlendState{};
    };
}
