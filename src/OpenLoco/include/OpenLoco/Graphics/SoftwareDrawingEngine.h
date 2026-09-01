#pragma once

#include "Graphics/Gfx.h"
#include "InvalidationGrid.h"
#include "SoftwareDrawingContext.h"
#include <OpenLoco/Engine/Ui/Rect.hpp>
#include <SDL3/SDL_pixels.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

struct SDL_Palette;
struct SDL_Surface;
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

namespace OpenLoco::Ui
{
    struct ScreenInfo;
}

namespace OpenLoco::Config
{
    enum class AntiAliasing : uint8_t;
}

namespace OpenLoco::Gfx
{
    class PostProcessor;
    struct RenderTarget;

    struct RenderFrameStats
    {
        uint64_t dirtyRenderNs{};
        uint64_t uiCompositionNs{};
        uint64_t paletteConversionNs{};
        uint64_t textureUploadNs{};
        uint64_t composePresentNs{};
        uint64_t screenUploadBytes{};
        uint64_t worldUploadBytes{};
        uint64_t uiUploadBytes{};
        uint64_t paletteChangeBytes{};
        uint64_t textureUploadCount{};
        bool screenTextureIndexed{};
        bool worldTextureIndexed{};
    };

    class SoftwareDrawingEngine
    {
    public:
        SoftwareDrawingEngine();
        ~SoftwareDrawingEngine();

        void initialize(SDL_Window* window);

        bool resize(int32_t width, int32_t height);

        // Renders all invalidated regions.
        void render();

        // Renders a specific region.
        void render(const Ui::Rect& rect);

        // Presents the final image to the screen.
        bool present();

        // Invalidates a region, this forces it to be rendered next frame.
        void invalidateRegion(int32_t left, int32_t top, int32_t right, int32_t bottom);
        void invalidateUiRegion(int32_t left, int32_t top, int32_t right, int32_t bottom);

        void createPalette();
        SDL_Palette* getPalette() { return _palette; }
        void updatePalette(const PaletteEntry* entries, int32_t index, int32_t count);

        DrawingContext& getDrawingContext();

        const RenderTarget& getScreenRT();
        const RenderTarget& getScreenshotRT();

        // Moves the pixels in the specified render target.
        void movePixels(
            const RenderTarget& rt,
            int16_t dstX,
            int16_t dstY,
            int16_t width,
            int16_t height,
            int16_t srcX,
            int16_t srcY);

        const Ui::ScreenInfo& getScreenInfo() const;
        Ui::Size getOutputSize() const;
        Ui::Size getPresentationSize() const;
        bool hasSeparateWorldResources() const;
        bool shouldUseSeparateWorld() const;

        bool setVSync(bool state);
        bool isVSyncDisabled() const;
        bool isGpuPaletteEnabled() const;
        bool supportsAntiAliasing() const;
        Config::AntiAliasing getActiveAntiAliasing() const;

        void setFrameStatsEnabled(bool enabled);
        const RenderFrameStats& getLastFrameStats() const;
        void setPresentationReadbackEnabled(bool enabled);
        uint64_t getLastPresentationHash() const;
        std::string_view getRendererName() const;

    private:
        void destroyScaledScreenResources();
        void destroySeparateWorldResources();
        void destroyScreenResources();
        void renderSeparateWorld(const Ui::Rect& rect);
        void renderDirtyWorldRegions();
        void renderSeparateUi();
        bool capturePresentationHash();
        bool presentStandard();
        bool presentSeparate();
        void renderDirtyRegions();

        SDL_Renderer* _renderer{};
        SDL_Window* _window{};
        SDL_Palette* _palette{};
        SDL_Surface* _screenSurface{};
        SDL_Surface* _screenRGBASurface{};
        SDL_Surface* _worldSurface{};
        SDL_Surface* _worldRGBASurface{};
        SDL_Surface* _uiRGBASurface{};

        SDL_Texture* _screenTexture{};
        SDL_Texture* _scaledScreenTexture{};
        SDL_Texture* _worldTexture{};
        SDL_Texture* _uiTexture{};
        std::unique_ptr<PostProcessor> _postProcessor;

        std::vector<PaletteIndex_t> _uiBase;
        std::vector<uint8_t> _uiCoverage;
        std::vector<int32_t> _uiToWorldX;
        std::vector<int32_t> _uiToWorldY;

        uint8_t _pixelScaleFactor = 1;

        SoftwareDrawingContext _ctx;
        InvalidationGrid _invalidationGrid;
        InvalidationGrid _screenUploadGrid;
        InvalidationGrid _worldUploadGrid;

        bool _screenTextureDirty = true;
        bool _worldTextureDirty = true;
        bool _uiTextureDirty = true;
        bool _uiTextureUploadPending = false;
        bool _screenTextureIndexed = false;
        bool _worldTextureIndexed = false;
        bool _frameStatsEnabled = false;
        bool _presentationReadbackEnabled = false;
        uint64_t _pendingPaletteChangeBytes{};
        uint64_t _lastPresentationHash{};
        RenderFrameStats _frameStats{};
        RenderFrameStats _lastFrameStats{};
        int32_t _outputWidth{};
        int32_t _outputHeight{};
    };
}
