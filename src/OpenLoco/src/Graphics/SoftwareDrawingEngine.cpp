#include "Graphics/SoftwareDrawingEngine.h"
#include "Config.h"
#include "Graphics/FPSCounter.h"
#include "Graphics/RenderTarget.h"
#include "Logging.h"
#include "SceneManager.h"
#include "Ui.h"
#include "Ui/Window.h"
#include "Ui/WindowManager.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

using namespace OpenLoco::Gfx;
using namespace OpenLoco::Ui;
using namespace OpenLoco::Diagnostics;

namespace OpenLoco::Gfx
{
    using SetPaletteFunc = void (*)(const PaletteEntry* palette, int32_t index, int32_t count);

    // TODO: Move into the renderer.
    // 0x0050B884
    static RenderTarget _screenRT{};
    static RenderTarget _worldRT{};
    static RenderTarget _screenshotRT{};
    static std::vector<PaletteIndex_t> _screenshotBuffer;
    // 0x0050B894
    static Ui::ScreenInfo _screenInfo;

    static int32_t sampleNearest(int32_t position, int32_t sourceSize, int32_t destinationSize)
    {
        return static_cast<int32_t>((static_cast<int64_t>(position) * 2 + 1) * sourceSize / (static_cast<int64_t>(destinationSize) * 2));
    }

    SoftwareDrawingEngine::SoftwareDrawingEngine()
    {
        RenderTarget rtDummy{};
        _ctx.pushRenderTarget(rtDummy);
    }

    SoftwareDrawingEngine::~SoftwareDrawingEngine()
    {
        destroyScreenResources();

        if (_palette != nullptr)
        {
            SDL_DestroyPalette(_palette);
            _palette = nullptr;
        }
        if (_renderer != nullptr)
        {
            SDL_DestroyRenderer(_renderer);
            _renderer = nullptr;
        }

        delete[] _screenRT.bits;
        _screenRT = {};
        delete[] _worldRT.bits;
        _worldRT = {};
    }

    void SoftwareDrawingEngine::destroyScaledScreenResources()
    {
        if (_scaledScreenTexture != nullptr)
        {
            SDL_DestroyTexture(_scaledScreenTexture);
            _scaledScreenTexture = nullptr;
        }
        _pixelScaleFactor = 1;
    }

    void SoftwareDrawingEngine::destroySeparateWorldResources()
    {
        if (_uiTexture != nullptr)
        {
            SDL_DestroyTexture(_uiTexture);
            _uiTexture = nullptr;
        }
        if (_worldTexture != nullptr)
        {
            SDL_DestroyTexture(_worldTexture);
            _worldTexture = nullptr;
        }
        if (_uiRGBASurface != nullptr)
        {
            SDL_DestroySurface(_uiRGBASurface);
            _uiRGBASurface = nullptr;
        }
        if (_worldRGBASurface != nullptr)
        {
            SDL_DestroySurface(_worldRGBASurface);
            _worldRGBASurface = nullptr;
        }
        if (_worldSurface != nullptr)
        {
            SDL_DestroySurface(_worldSurface);
            _worldSurface = nullptr;
        }
        delete[] _worldRT.bits;
        _worldRT = {};
    }

    void SoftwareDrawingEngine::destroyScreenResources()
    {
        destroyScaledScreenResources();
        destroySeparateWorldResources();
        if (_screenTexture != nullptr)
        {
            SDL_DestroyTexture(_screenTexture);
            _screenTexture = nullptr;
        }
        if (_screenSurface != nullptr)
        {
            SDL_DestroySurface(_screenSurface);
            _screenSurface = nullptr;
        }
        if (_screenRGBASurface != nullptr)
        {
            SDL_DestroySurface(_screenRGBASurface);
            _screenRGBASurface = nullptr;
        }
    }

    void SoftwareDrawingEngine::initialize(SDL_Window* window)
    {
        _renderer = SDL_CreateRenderer(window, nullptr);
        if (_renderer == nullptr)
        {
            // Try to fallback to software renderer.
            Logging::warn("Hardware acceleration not available, falling back to software renderer.");

            _renderer = SDL_CreateRenderer(window, "software");
            if (_renderer == nullptr)
            {
                Logging::error("Unable to create software renderer: {}", SDL_GetError());
                std::abort();
            }
        }

        _window = window;
        createPalette();
    }

    void SoftwareDrawingEngine::resize(const int32_t width, const int32_t height)
    {
        _outputWidth = width;
        _outputHeight = height;

        // Scale the width and height by configured scale factor
        const auto scaleFactor = Config::get().scaleFactor;
        const auto scaledWidth = std::max(1, static_cast<int32_t>(width / scaleFactor));
        const auto scaledHeight = std::max(1, static_cast<int32_t>(height / scaleFactor));

        // Release old resources.
        destroyScreenResources();

        constexpr auto outputFormat = SDL_PIXELFORMAT_ARGB8888;

        // Surfaces.
        _screenSurface = SDL_CreateSurface(scaledWidth, scaledHeight, SDL_PIXELFORMAT_INDEX8);
        if (_screenSurface == nullptr)
        {
            Logging::error("SDL_CreateSurface (_screenSurface) failed: {}", SDL_GetError());
            return;
        }

        _screenRGBASurface = SDL_CreateSurface(scaledWidth, scaledHeight, outputFormat);
        if (_screenRGBASurface == nullptr)
        {
            Logging::error("SDL_CreateSurface (_screenRGBASurface) failed: {}", SDL_GetError());
            return;
        }

        if (!SDL_SetSurfaceBlendMode(_screenRGBASurface, SDL_BLENDMODE_NONE))
        {
            Logging::error("SDL_SetSurfaceBlendMode (_screenRGBASurface) failed: {}", SDL_GetError());
        }
        if (!SDL_SetSurfacePalette(_screenSurface, _palette))
        {
            Logging::error("SDL_SetSurfacePalette (_screenSurface) failed: {}", SDL_GetError());
        }

        _screenTexture = SDL_CreateTexture(_renderer, outputFormat, SDL_TEXTUREACCESS_STREAMING, scaledWidth, scaledHeight);
        if (_screenTexture == nullptr)
        {
            Logging::error("SDL_CreateTexture (_screenTexture) failed: {}", SDL_GetError());
            return;
        }

        // Keep the palette buffer crisp when scaling the software-rendered frame.
        if (!SDL_SetTextureScaleMode(_screenTexture, SDL_SCALEMODE_NEAREST))
        {
            Logging::error("SDL_SetTextureScaleMode (_screenTexture) failed: {}", SDL_GetError());
        }

        if (Config::get().nativeViewportRendering && scaleFactor > 1.0f)
        {
            _worldSurface = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_INDEX8);
            _worldRGBASurface = SDL_CreateSurface(width, height, outputFormat);
            _uiRGBASurface = SDL_CreateSurface(scaledWidth, scaledHeight, outputFormat);
            _worldTexture = SDL_CreateTexture(_renderer, outputFormat, SDL_TEXTUREACCESS_STREAMING, width, height);
            _uiTexture = SDL_CreateTexture(_renderer, outputFormat, SDL_TEXTUREACCESS_STREAMING, scaledWidth, scaledHeight);
            if (_worldSurface == nullptr || _worldRGBASurface == nullptr || _uiRGBASurface == nullptr || _worldTexture == nullptr || _uiTexture == nullptr)
            {
                Logging::error("Unable to create separate world rendering resources: {}", SDL_GetError());
                destroySeparateWorldResources();
            }
            else
            {
                bool configured = true;
                configured &= SDL_SetSurfacePalette(_worldSurface, _palette);
                configured &= SDL_SetSurfaceBlendMode(_worldRGBASurface, SDL_BLENDMODE_NONE);
                configured &= SDL_SetSurfaceBlendMode(_uiRGBASurface, SDL_BLENDMODE_NONE);
                configured &= SDL_SetTextureScaleMode(_worldTexture, SDL_SCALEMODE_LINEAR);
                configured &= SDL_SetTextureScaleMode(_uiTexture, SDL_SCALEMODE_NEAREST);
                configured &= SDL_SetTextureBlendMode(_uiTexture, SDL_BLENDMODE_BLEND);
                if (!configured)
                {
                    Logging::error("Unable to configure separate world rendering resources: {}", SDL_GetError());
                    destroySeparateWorldResources();
                }
            }
        }

        if (scaleFactor > 1.0f)
        {
            const auto scale = std::clamp(static_cast<int32_t>(std::ceil(scaleFactor)), 2, 4);
            if (scaledWidth > std::numeric_limits<int32_t>::max() / scale || scaledHeight > std::numeric_limits<int32_t>::max() / scale)
            {
                Logging::error("Scaled screen dimensions are too large");
            }
            else
            {
                const auto outputWidth = scaledWidth * scale;
                const auto outputHeight = scaledHeight * scale;
                _scaledScreenTexture = SDL_CreateTexture(_renderer, outputFormat, SDL_TEXTUREACCESS_TARGET, outputWidth, outputHeight);
                if (_scaledScreenTexture == nullptr)
                {
                    Logging::error("SDL_CreateTexture (_scaledScreenTexture) failed: {}", SDL_GetError());
                }
                else
                {
                    _pixelScaleFactor = static_cast<uint8_t>(scale);
                    if (!SDL_SetTextureScaleMode(_scaledScreenTexture, SDL_SCALEMODE_LINEAR))
                    {
                        Logging::error("SDL_SetTextureScaleMode (_scaledScreenTexture) failed: {}", SDL_GetError());
                    }
                }
            }
        }

        int32_t pitch = _screenSurface->pitch;

        RenderTarget& rt = _screenRT;
        if (rt.bits != nullptr)
        {
            delete[] rt.bits;
        }
        rt.bits = new uint8_t[pitch * scaledHeight];
        rt.width = scaledWidth;
        rt.height = scaledHeight;
        rt.pitch = pitch - scaledWidth;

        if (_worldSurface != nullptr)
        {
            const auto worldPitch = _worldSurface->pitch;
            delete[] _worldRT.bits;
            _worldRT.bits = new uint8_t[static_cast<size_t>(worldPitch) * height];
            _worldRT.x = 0;
            _worldRT.y = 0;
            _worldRT.width = width;
            _worldRT.height = height;
            _worldRT.pitch = worldPitch - width;
            std::fill_n(_worldRT.bits, static_cast<size_t>(worldPitch) * height, PaletteIndex::black0);
        }

        if (_worldSurface != nullptr)
        {
            _uiBase.resize(static_cast<size_t>(pitch) * scaledHeight);
            _uiCoverage.resize(static_cast<size_t>(pitch) * scaledHeight);
        }
        else
        {
            _uiBase.clear();
            _uiCoverage.clear();
        }

        _screenInfo.width = scaledWidth;
        _screenInfo.height = scaledHeight;
        _screenInfo.width_2 = scaledWidth;
        _screenInfo.height_2 = scaledHeight;
        _screenInfo.width_3 = scaledWidth;
        _screenInfo.height_3 = scaledHeight;

        int32_t widthShift = 6;
        int16_t blockWidth = 1 << widthShift;
        int32_t heightShift = 3;
        int16_t blockHeight = 1 << heightShift;

        _invalidationGrid.reset(scaledWidth, scaledHeight, blockWidth, blockHeight);

        // Reset the drawing context, this holds the old screen render target.
        _ctx.reset();

        // Push the screen render target so that by default we render to that.
        _ctx.pushRenderTarget(rt);

        // Set the normal background colour.
        _ctx.clearSingle(PaletteIndex::black0);
    }

    /**
     * 0x004C5C69
     *
     * @param left @<ax>
     * @param top @<bx>
     * @param right @<dx>
     * @param bottom @<bp>
     */
    void SoftwareDrawingEngine::invalidateRegion(int32_t left, int32_t top, int32_t right, int32_t bottom)
    {
        _invalidationGrid.invalidate(left, top, right, bottom);
    }

    void SoftwareDrawingEngine::createPalette()
    {
        // Create a palette for the window
        _palette = SDL_CreatePalette(256);
        if (_palette == nullptr)
        {
            Logging::error("SDL_CreatePalette failed: {}", SDL_GetError());
            return;
        }
    }

    void SoftwareDrawingEngine::updatePalette(const PaletteEntry* entries, int32_t index, int32_t count)
    {
        assert(index + count < 256);

        if (_palette == nullptr)
        {
            // In headless mode, the palette is not created, so we cannot update it.
            return;
        }

        SDL_Color base[256]{};
        SDL_Color* basePtr = &base[index];
        auto* entryPtr = &entries[index];
        for (int i = 0; i < count; ++i, basePtr++, entryPtr++)
        {
            basePtr->r = entryPtr->r;
            basePtr->g = entryPtr->g;
            basePtr->b = entryPtr->b;
            basePtr->a = 255;
        }

        if (!SDL_SetPaletteColors(_palette, &base[index], index, count))
        {
            Logging::error("SDL_SetPaletteColors failed: {}", SDL_GetError());
        }
    }

    // 0x004C5CFA
    void SoftwareDrawingEngine::render()
    {
        if (shouldUseSeparateWorld())
        {
            if (!SceneManager::isProgressBarActive())
            {
                WindowManager::updateViewports();
                renderSeparateWorld();
            }
            renderSeparateUi();
            return;
        }

        // Need to first render the current dirty regions before updating the viewports.
        // This is needed to ensure it will copy the correct pixels when the viewport will be moved.
        renderDirtyRegions();

        // Updating the viewports will potentially move pixels and mark previously invisible regions as dirty.
        WindowManager::updateViewports();

        // Render the uncovered regions.
        renderDirtyRegions();

        // Draw FPS counter.
        if (Config::get().showFPS)
        {
            Gfx::drawFPS(_ctx);
        }
    }

    bool SoftwareDrawingEngine::hasSeparateWorldResources() const
    {
        return Config::get().nativeViewportRendering
            && _worldRT.bits != nullptr
            && _worldSurface != nullptr
            && _worldRGBASurface != nullptr
            && _uiRGBASurface != nullptr
            && _worldTexture != nullptr
            && _uiTexture != nullptr;
    }

    bool SoftwareDrawingEngine::shouldUseSeparateWorld() const
    {
        return hasSeparateWorldResources()
            && WindowManager::getMainWindow() != nullptr
            && SceneManager::getCurrentScene() != SceneManager::SceneId::intro;
    }

    void SoftwareDrawingEngine::renderSeparateWorld()
    {
        auto* mainWindow = WindowManager::getMainWindow();
        if (mainWindow == nullptr || mainWindow->viewports[0] == nullptr)
        {
            return;
        }

        SoftwareDrawingContext worldContext;
        worldContext.pushRenderTarget(_worldRT);

        auto viewport = *mainWindow->viewports[0];
        viewport.x = 0;
        viewport.y = 0;
        viewport.setDimensions({ _worldRT.width, _worldRT.height }, { _worldRT.width, _worldRT.height });
        viewport.render(worldContext, false);
    }

    void SoftwareDrawingEngine::renderSeparateUi()
    {
        const auto uiStride = _screenRT.width + _screenRT.pitch;
        const auto isProgressBarActive = SceneManager::isProgressBarActive();
        if (!isProgressBarActive)
        {
            const auto worldStride = _worldRT.width + _worldRT.pitch;
            for (int32_t y = 0; y < _screenRT.height; ++y)
            {
                const auto worldY = sampleNearest(y, _worldRT.height, _screenRT.height);
                for (int32_t x = 0; x < _screenRT.width; ++x)
                {
                    const auto worldX = sampleNearest(x, _worldRT.width, _screenRT.width);
                    _uiBase[static_cast<size_t>(y) * uiStride + x] = _worldRT.bits[static_cast<size_t>(worldY) * worldStride + worldX];
                }
            }

            std::copy(_uiBase.begin(), _uiBase.end(), _screenRT.bits);
            std::fill(_uiCoverage.begin(), _uiCoverage.end(), 0);
        }

        const auto screenRect = Ui::Rect(0, 0, _screenRT.width, _screenRT.height);
        _ctx.pushRenderTarget(_screenRT);
        if (!isProgressBarActive)
        {
            if (auto* mainWindow = WindowManager::getMainWindow(); mainWindow != nullptr && mainWindow->viewports[0] != nullptr)
            {
                mainWindow->viewports[0]->renderUiOverlays(_ctx);
            }
        }
        WindowManager::renderUi(_ctx, screenRect);
        if (Config::get().showFPS)
        {
            Gfx::drawFPS(_ctx);
        }
        _ctx.popRenderTarget();

        if (!isProgressBarActive)
        {
            std::fill(_uiCoverage.begin(), _uiCoverage.end(), 0xFF);
            auto coverageTarget = _screenRT;
            coverageTarget.bits = _uiCoverage.data();
            _ctx.pushRenderTarget(coverageTarget);
            if (auto* mainWindow = WindowManager::getMainWindow(); mainWindow != nullptr && mainWindow->viewports[0] != nullptr)
            {
                mainWindow->viewports[0]->renderUiOverlays(_ctx);
            }
            WindowManager::renderUi(_ctx, screenRect, true);
            if (Config::get().showFPS)
            {
                Gfx::drawFPS(_ctx, false);
            }
            _ctx.popRenderTarget();

            for (int32_t y = 0; y < _screenRT.height; ++y)
            {
                for (int32_t x = 0; x < _screenRT.width; ++x)
                {
                    const auto offset = static_cast<size_t>(y) * uiStride + x;
                    _uiCoverage[offset] = _uiCoverage[offset] != 0xFF ? 0xFF : 0;
                }
            }
        }

        for (size_t i = 0; i < WindowManager::count(); ++i)
        {
            auto* window = WindowManager::get(i);
            if (window->type == WindowType::main || !window->isVisible() || window->hasFlags(WindowFlags::noBackground))
            {
                continue;
            }
            if (isProgressBarActive && window->type != WindowType::progressBar)
            {
                continue;
            }

            const auto rect = screenRect.intersection(Ui::Rect(window->x, window->y, window->width, window->height));
            if (rect.width() <= 0 || rect.height() <= 0)
            {
                continue;
            }
            for (int32_t y = rect.top(); y < rect.bottom(); ++y)
            {
                std::fill_n(_uiCoverage.data() + static_cast<size_t>(y) * uiStride + rect.left(), rect.width(), 0xFF);
            }
        }

        const auto palette = Gfx::getRgbaPalette();
        for (int32_t y = 0; y < _screenRT.height; ++y)
        {
            auto* output = static_cast<uint32_t*>(_uiRGBASurface->pixels) + static_cast<size_t>(y) * _uiRGBASurface->pitch / sizeof(uint32_t);
            for (int32_t x = 0; x < _screenRT.width; ++x)
            {
                const auto offset = static_cast<size_t>(y) * uiStride + x;
                if (_screenRT.bits[offset] != _uiBase[offset])
                {
                    _uiCoverage[offset] = 0xFF;
                }

                if (_uiCoverage[offset] == 0)
                {
                    output[x] = 0;
                    continue;
                }

                const auto& colour = palette[_screenRT.bits[offset]];
                output[x] = 0xFF000000U | static_cast<uint32_t>(colour.r) << 16 | static_cast<uint32_t>(colour.g) << 8 | colour.b;
            }
        }
    }

    void SoftwareDrawingEngine::renderDirtyRegions()
    {
        _invalidationGrid.traverseDirtyCells([this](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            this->render(Rect::fromLTRB(left, top, right, bottom));
        });
    }

    void SoftwareDrawingEngine::render(const Rect& _rect)
    {
        auto max = Rect(0, 0, Ui::width(), Ui::height());
        auto rect = _rect.intersection(max);

        RenderTarget rt;
        rt.width = rect.width();
        rt.height = rect.height();
        rt.x = rect.left();
        rt.y = rect.top();
        rt.bits = _screenRT.bits + rect.left() + ((_screenRT.width + _screenRT.pitch) * rect.top());
        rt.pitch = _screenRT.width + _screenRT.pitch - rect.width();

        // Set the render target to the screen rt.
        _ctx.pushRenderTarget(rt);

        // TODO: Remove main window and draw that independent from UI.

        // Draw UI.
        Ui::WindowManager::render(_ctx, rect);

        // Restore state.
        _ctx.popRenderTarget();
    }

    void SoftwareDrawingEngine::present()
    {
        if (shouldUseSeparateWorld())
        {
            presentSeparate();
            return;
        }

        if (_screenSurface == nullptr || _screenRGBASurface == nullptr || _screenTexture == nullptr)
        {
            return;
        }

        // Lock the surface before setting its pixels
        if (SDL_MUSTLOCK(_screenSurface))
        {
            if (!SDL_LockSurface(_screenSurface))
            {
                return;
            }
        }

        // Copy pixels from the virtual screen buffer to the surface
        auto& rt = getScreenRT();
        if (rt.bits != nullptr)
        {
            std::memcpy(_screenSurface->pixels, rt.bits, _screenSurface->pitch * _screenSurface->h);
        }

        // Unlock the surface
        if (SDL_MUSTLOCK(_screenSurface))
        {
            SDL_UnlockSurface(_screenSurface);
        }

        // Convert colours via palette mapping onto the RGBA surface.
        if (!SDL_BlitSurface(_screenSurface, nullptr, _screenRGBASurface, nullptr))
        {
            Logging::error("SDL_BlitSurface {}", SDL_GetError());
            return;
        }

        // Copy the RGBA pixels into screen texture.
        if (!SDL_UpdateTexture(_screenTexture, nullptr, _screenRGBASurface->pixels, _screenRGBASurface->pitch))
        {
            Logging::error("SDL_UpdateTexture {}", SDL_GetError());
            return;
        }

        auto* displayTexture = _screenTexture;
        if (_scaledScreenTexture != nullptr && _pixelScaleFactor > 1)
        {
            // Integer nearest-neighbour prescale followed by linear downsampling.
            if (!SDL_SetRenderTarget(_renderer, _scaledScreenTexture))
            {
                Logging::error("SDL_SetRenderTarget (_scaledScreenTexture) failed: {}", SDL_GetError());
                return;
            }
            if (!SDL_RenderTexture(_renderer, _screenTexture, nullptr, nullptr))
            {
                Logging::error("SDL_RenderTexture (_screenTexture) failed: {}", SDL_GetError());
                SDL_SetRenderTarget(_renderer, nullptr);
                return;
            }

            if (!SDL_SetRenderTarget(_renderer, nullptr))
            {
                Logging::error("SDL_SetRenderTarget (nullptr) failed: {}", SDL_GetError());
                return;
            }
            displayTexture = _scaledScreenTexture;
        }

        if (!SDL_RenderTexture(_renderer, displayTexture, nullptr, nullptr))
        {
            Logging::error("SDL_RenderTexture failed: {}", SDL_GetError());
            return;
        }

        // Display buffers.
        if (!SDL_RenderPresent(_renderer))
        {
            Logging::error("SDL_RenderPresent failed: {}", SDL_GetError());
        }
    }

    void SoftwareDrawingEngine::presentSeparate()
    {
        const auto worldStride = _worldRT.width + _worldRT.pitch;
        std::memcpy(_worldSurface->pixels, _worldRT.bits, static_cast<size_t>(worldStride) * _worldRT.height);
        if (!SDL_BlitSurface(_worldSurface, nullptr, _worldRGBASurface, nullptr)
            || !SDL_UpdateTexture(_worldTexture, nullptr, _worldRGBASurface->pixels, _worldRGBASurface->pitch)
            || !SDL_UpdateTexture(_uiTexture, nullptr, _uiRGBASurface->pixels, _uiRGBASurface->pitch))
        {
            Logging::error("Unable to upload separate world rendering textures: {}", SDL_GetError());
            return;
        }

        if (!SDL_RenderTexture(_renderer, _worldTexture, nullptr, nullptr)
            || !SDL_RenderTexture(_renderer, _uiTexture, nullptr, nullptr)
            || !SDL_RenderPresent(_renderer))
        {
            Logging::error("Unable to present separate world rendering textures: {}", SDL_GetError());
        }
    }

    DrawingContext& SoftwareDrawingEngine::getDrawingContext()
    {
        return _ctx;
    }

    const RenderTarget& SoftwareDrawingEngine::getScreenRT()
    {
        return _screenRT;
    }

    const RenderTarget& SoftwareDrawingEngine::getScreenshotRT()
    {
        if (!shouldUseSeparateWorld())
        {
            return _screenRT;
        }

        const auto worldStride = _worldRT.width + _worldRT.pitch;
        const auto uiStride = _screenRT.width + _screenRT.pitch;
        _screenshotBuffer.resize(static_cast<size_t>(_worldRT.width) * _worldRT.height);
        for (int32_t y = 0; y < _worldRT.height; ++y)
        {
            const auto uiY = sampleNearest(y, _screenRT.height, _worldRT.height);
            for (int32_t x = 0; x < _worldRT.width; ++x)
            {
                const auto uiX = sampleNearest(x, _screenRT.width, _worldRT.width);
                const auto uiOffset = static_cast<size_t>(uiY) * uiStride + uiX;
                _screenshotBuffer[static_cast<size_t>(y) * _worldRT.width + x] = _uiCoverage[uiOffset] != 0
                    ? _screenRT.bits[uiOffset]
                    : _worldRT.bits[static_cast<size_t>(y) * worldStride + x];
            }
        }

        _screenshotRT = RenderTarget{ _screenshotBuffer.data(), 0, 0, _worldRT.width, _worldRT.height, 0 };
        return _screenshotRT;
    }

    void SoftwareDrawingEngine::movePixels(
        const RenderTarget& rt,
        int16_t dstX,
        int16_t dstY,
        int16_t width,
        int16_t height,
        int16_t srcX,
        int16_t srcY)
    {
        if (dstX == 0 && dstY == 0)
        {
            return;
        }

        // Adjust for move off canvas.
        // NOTE: when zooming, there can be x, y, dx, dy combinations that go off the
        // canvas; hence the checks. This code should ultimately not be called when
        // zooming because this function is specific to updating the screen on move
        int32_t lmargin = std::min(dstX - srcX, 0);
        int32_t rmargin = std::min((int32_t)rt.width - (dstX - srcX + width), 0);
        int32_t tmargin = std::min(dstY - srcY, 0);
        int32_t bmargin = std::min((int32_t)rt.height - (dstY - srcY + height), 0);

        dstX -= lmargin;
        dstY -= tmargin;
        width += lmargin + rmargin;
        height += tmargin + bmargin;

        int32_t stride = rt.width + rt.pitch;
        uint8_t* to = rt.bits + dstY * stride + dstX;
        uint8_t* from = rt.bits + (dstY - srcY) * stride + dstX - srcX;

        if (srcY > 0)
        {
            // If positive dy, reverse directions
            to += (height - 1) * stride;
            from += (height - 1) * stride;
            stride = -stride;
        }

        // Move bytes
        for (int32_t i = 0; i < height; i++)
        {
            std::memmove(to, from, width);
            to += stride;
            from += stride;
        }
    }

    const Ui::ScreenInfo& SoftwareDrawingEngine::getScreenInfo() const
    {
        return _screenInfo;
    }

    Ui::Size SoftwareDrawingEngine::getOutputSize() const
    {
        return { _outputWidth, _outputHeight };
    }

    bool SoftwareDrawingEngine::setVSync(bool state)
    {
        if (_vsync == state)
        {
            return true;
        }

        if (SDL_SetRenderVSync(_renderer, state ? 1 : 0))
        {
            _vsync = state;
            return true;
        }

        return false;
    }
}
