#pragma once

#include "Graphics/Gfx.h"
#include "InvalidationGrid.h"
#include "SoftwareDrawingContext.h"
#include <OpenLoco/Engine/Ui/Rect.hpp>
#include <SDL3/SDL_pixels.h>
#include <algorithm>
#include <cstddef>
#include <memory>
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

namespace OpenLoco::Gfx
{
    struct RenderTarget;

    class SoftwareDrawingEngine
    {
    public:
        SoftwareDrawingEngine();
        ~SoftwareDrawingEngine();

        void initialize(SDL_Window* window);

        void resize(int32_t width, int32_t height);

        // Renders all invalidated regions.
        void render();

        // Renders a specific region.
        void render(const Ui::Rect& rect);

        // Presents the final image to the screen.
        void present();

        // Invalidates a region, this forces it to be rendered next frame.
        void invalidateRegion(int32_t left, int32_t top, int32_t right, int32_t bottom);

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
        bool hasSeparateWorldResources() const;
        bool shouldUseSeparateWorld() const;

        bool setVSync(bool state);

    private:
        void destroyScaledScreenResources();
        void destroySeparateWorldResources();
        void destroyScreenResources();
        void renderSeparateWorld();
        void renderSeparateUi();
        void presentSeparate();
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

        std::vector<PaletteIndex_t> _uiBase;
        std::vector<uint8_t> _uiCoverage;

        uint8_t _pixelScaleFactor = 1;

        SoftwareDrawingContext _ctx;
        InvalidationGrid _invalidationGrid;

        bool _vsync = false;
        int32_t _outputWidth{};
        int32_t _outputHeight{};
    };
}
