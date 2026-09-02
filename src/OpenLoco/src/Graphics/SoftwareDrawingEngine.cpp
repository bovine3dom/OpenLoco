#include "Graphics/SoftwareDrawingEngine.h"
#include "Config.h"
#include "Graphics/FPSCounter.h"
#include "Graphics/PostProcessor.h"
#include "Graphics/RenderTarget.h"
#include "Logging.h"
#include "SceneManager.h"
#include "Ui.h"
#include "Ui/Window.h"
#include "Ui/WindowManager.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <execution>
#include <limits>
#include <ranges>
#include <string>
#include <utility>

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

    static std::pair<int32_t, int32_t> divideWithPositiveRemainder(int32_t value, int32_t divisor)
    {
        auto quotient = value / divisor;
        auto remainder = value % divisor;
        if (remainder < 0)
        {
            --quotient;
            remainder += divisor;
        }
        return { quotient, remainder };
    }

    static int32_t getNativeRenderDimension(int32_t outputDimension, int32_t scale)
    {
        // Keep a border texel for AA sampling and phase shifts at the far edge.
        const auto scaledDimension = (static_cast<int64_t>(outputDimension) + scale - 1) / scale + 1;
        return static_cast<int32_t>(std::min<int64_t>(outputDimension, scaledDimension));
    }

    template<typename F>
    static void measure(bool enabled, uint64_t& elapsedNanoseconds, F&& func)
    {
        if (!enabled)
        {
            func();
            return;
        }

        const auto start = std::chrono::steady_clock::now();
        func();
        elapsedNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
    }

    template<typename F>
    static bool measureResult(bool enabled, uint64_t& elapsedNanoseconds, F&& func)
    {
        if (!enabled)
        {
            return func();
        }

        const auto start = std::chrono::steady_clock::now();
        const auto result = func();
        elapsedNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
        return result;
    }

    static SDL_Texture* createPresentationTexture(SDL_Renderer* renderer, SDL_Palette* palette, int32_t width, int32_t height, bool& isIndexed)
    {
#if SDL_VERSION_ATLEAST(3, 4, 4)
        if (std::string_view(SDL_GetRendererName(renderer)) == SDL_GPU_RENDERER
            && SDL_GetVersion() >= SDL_VERSIONNUM(3, 4, 4))
        {
            auto* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_INDEX8, SDL_TEXTUREACCESS_STREAMING, width, height);
            if (texture != nullptr && SDL_SetTexturePalette(texture, palette))
            {
                isIndexed = true;
                return texture;
            }

            const auto indexedError = std::string(SDL_GetError());
            if (texture != nullptr)
            {
                SDL_DestroyTexture(texture);
            }
            Logging::warn("Unable to create indexed presentation texture: {}. Falling back to ARGB8888.", indexedError);
        }
#endif

        isIndexed = false;
        return SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);
    }

    SoftwareDrawingEngine::SoftwareDrawingEngine()
        : _postProcessor(std::make_unique<PostProcessor>())
    {
        RenderTarget rtDummy{};
        _ctx.pushRenderTarget(rtDummy);
    }

    SoftwareDrawingEngine::~SoftwareDrawingEngine()
    {
        destroyScreenResources();
        _postProcessor.reset();

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
        _worldTextureDirty = true;
        _uiTextureDirty = true;
        _uiTextureUploadPending = false;
        _worldTextureIndexed = false;
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
        _uiBase.clear();
        _uiCoverage.clear();
        _uiToWorldX.clear();
        _uiToWorldY.clear();
        _worldRT = {};
        _worldPresentationScale = 1;
        _worldPresentationPhase = {};
        _worldRenderOrigin = {};
        _worldTransformValid = false;
    }

    void SoftwareDrawingEngine::destroyScreenResources()
    {
        _postProcessor->reset();
        _screenTextureDirty = true;
        _screenTextureIndexed = false;
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
        const auto* requestedRenderer = SDL_GetHint(SDL_HINT_RENDER_DRIVER);
        if (requestedRenderer != nullptr && requestedRenderer[0] != '\0')
        {
            _renderer = SDL_CreateRenderer(window, nullptr);
        }
        else
        {
            _renderer = SDL_CreateRenderer(window, SDL_GPU_RENDERER);
            if (_renderer == nullptr)
            {
                Logging::warn("SDL GPU renderer is unavailable: {}. Falling back to the default renderer.", SDL_GetError());
                _renderer = SDL_CreateRenderer(window, nullptr);
            }
        }

        if (_renderer == nullptr)
        {
            Logging::warn("Hardware acceleration is unavailable, falling back to the software renderer.");
            _renderer = SDL_CreateRenderer(window, SDL_SOFTWARE_RENDERER);
            if (_renderer == nullptr)
            {
                Logging::error("Unable to create software renderer: {}", SDL_GetError());
                std::abort();
            }
        }

        Logging::info("Using SDL '{}' renderer", SDL_GetRendererName(_renderer));
        _window = window;
        createPalette();
    }

    bool SoftwareDrawingEngine::resize(const int32_t width, const int32_t height)
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
            return false;
        }

        if (!SDL_SetSurfacePalette(_screenSurface, _palette))
        {
            Logging::error("SDL_SetSurfacePalette (_screenSurface) failed: {}", SDL_GetError());
            return false;
        }

        _screenTexture = createPresentationTexture(_renderer, _palette, scaledWidth, scaledHeight, _screenTextureIndexed);
        if (_screenTexture == nullptr)
        {
            Logging::error("SDL_CreateTexture (_screenTexture) failed: {}", SDL_GetError());
            return false;
        }
        if (!_screenTextureIndexed)
        {
            _screenRGBASurface = SDL_CreateSurface(scaledWidth, scaledHeight, outputFormat);
            if (_screenRGBASurface == nullptr || !SDL_SetSurfaceBlendMode(_screenRGBASurface, SDL_BLENDMODE_NONE))
            {
                Logging::error("Unable to create fallback screen surface: {}", SDL_GetError());
                return false;
            }
        }

        // Keep the palette buffer crisp when scaling the software-rendered frame.
        if (!SDL_SetTextureScaleMode(_screenTexture, SDL_SCALEMODE_NEAREST))
        {
            Logging::error("SDL_SetTextureScaleMode (_screenTexture) failed: {}", SDL_GetError());
        }

        const auto requestedAntiAliasing = Config::get().display.antiAliasing;
        const auto canUseAntiAliasing = requestedAntiAliasing != Config::AntiAliasing::none
            && PostProcessor::isSupported(_renderer);
        if (Config::get().nativeViewportRendering && (scaleFactor > 1.0f || canUseAntiAliasing))
        {
            _worldSurface = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_INDEX8);
            _uiRGBASurface = SDL_CreateSurface(scaledWidth, scaledHeight, outputFormat);
            _worldTexture = createPresentationTexture(_renderer, _palette, width, height, _worldTextureIndexed);
            _uiTexture = SDL_CreateTexture(_renderer, outputFormat, SDL_TEXTUREACCESS_STREAMING, scaledWidth, scaledHeight);
            if (!_worldTextureIndexed)
            {
                _worldRGBASurface = SDL_CreateSurface(width, height, outputFormat);
            }
            if (_worldSurface == nullptr || (!_worldTextureIndexed && _worldRGBASurface == nullptr) || _uiRGBASurface == nullptr || _worldTexture == nullptr || _uiTexture == nullptr)
            {
                Logging::error("Unable to create separate world rendering resources: {}", SDL_GetError());
                destroySeparateWorldResources();
            }
            else
            {
                bool configured = true;
                configured &= !SDL_MUSTLOCK(_worldSurface);
                configured &= SDL_SetSurfacePalette(_worldSurface, _palette);
                configured &= _worldTextureIndexed || SDL_SetSurfaceBlendMode(_worldRGBASurface, SDL_BLENDMODE_NONE);
                configured &= SDL_SetSurfaceBlendMode(_uiRGBASurface, SDL_BLENDMODE_NONE);
                configured &= SDL_SetTextureScaleMode(_worldTexture, SDL_SCALEMODE_NEAREST);
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
            _worldRT.bits = static_cast<uint8_t*>(_worldSurface->pixels);
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
            updateUiToWorldMap();
        }
        else
        {
            _uiBase.clear();
            _uiCoverage.clear();
            _uiToWorldX.clear();
            _uiToWorldY.clear();
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
        _screenUploadGrid.reset(scaledWidth, scaledHeight, blockWidth, blockHeight);
        _worldUploadGrid.reset(width, height, blockWidth, blockHeight);

        _screenTextureDirty = true;
        _worldTextureDirty = true;

        // Reset the drawing context, this holds the old screen render target.
        _ctx.reset();

        // Push the screen render target so that by default we render to that.
        _ctx.pushRenderTarget(rt);

        // Set the normal background colour.
        _ctx.clearSingle(PaletteIndex::black0);

        const auto antiAliasing = hasSeparateWorldResources() && canUseAntiAliasing
            ? requestedAntiAliasing
            : Config::AntiAliasing::none;
        if (!_postProcessor->configure(_renderer, width, height, antiAliasing) && antiAliasing != Config::AntiAliasing::none)
        {
            Logging::warn("Anti-aliasing is unavailable; using unfiltered presentation.");
            if (scaleFactor <= 1.0F)
            {
                destroySeparateWorldResources();
            }
        }
        return true;
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
        _uiTextureDirty = true;
    }

    void SoftwareDrawingEngine::invalidateUiRegion(int32_t left, int32_t top, int32_t right, int32_t bottom)
    {
        if (!shouldUseSeparateWorld())
        {
            _invalidationGrid.invalidate(left, top, right, bottom);
        }
        _uiTextureDirty = true;
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
        assert(index >= 0 && count >= 0 && index + count <= 256);
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
            return;
        }

        if (!_screenTextureIndexed)
        {
            _screenTextureDirty = true;
        }
        if (!_worldTextureIndexed)
        {
            _worldTextureDirty = true;
        }
        _uiTextureDirty = true;
        _pendingPaletteChangeBytes += static_cast<uint64_t>(count) * sizeof(SDL_Color);
    }

    // 0x004C5CFA
    void SoftwareDrawingEngine::render()
    {
        _frameStats = {};
        _frameStats.paletteChangeBytes = std::exchange(_pendingPaletteChangeBytes, 0);
        _frameStats.screenTextureIndexed = _screenTextureIndexed;
        _frameStats.worldTextureIndexed = _worldTextureIndexed;

        if (shouldUseSeparateWorld())
        {
            if (!SceneManager::isProgressBarActive())
            {
                measure(_frameStatsEnabled, _frameStats.dirtyRenderNs, [&] {
                    WindowManager::updateViewports();
                    updateWorldRenderTarget();
                    renderDirtyWorldRegions();
                });
            }

            if (_uiTextureDirty)
            {
                _uiTextureDirty = false;
                const auto conversionBefore = _frameStats.paletteConversionNs;
                measure(_frameStatsEnabled, _frameStats.uiCompositionNs, [&] {
                    renderSeparateUi();
                });
                const auto conversionTime = _frameStats.paletteConversionNs - conversionBefore;
                _frameStats.uiCompositionNs -= std::min(_frameStats.uiCompositionNs, conversionTime);
                _uiTextureUploadPending = true;
            }
            return;
        }

        measure(_frameStatsEnabled, _frameStats.dirtyRenderNs, [&] {
            // Render before moving viewport pixels, then render regions exposed by the move.
            renderDirtyRegions();
            WindowManager::updateViewports();
            renderDirtyRegions();

            if (Config::get().showFPS)
            {
                const auto rect = Gfx::drawFPS(_ctx);
                _screenUploadGrid.invalidate(rect.left(), rect.top(), rect.right(), rect.bottom());
            }
        });
    }

    bool SoftwareDrawingEngine::hasSeparateWorldResources() const
    {
        return Config::get().nativeViewportRendering
            && _worldRT.bits != nullptr
            && _worldSurface != nullptr
            && (_worldTextureIndexed || _worldRGBASurface != nullptr)
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

    void SoftwareDrawingEngine::updateWorldRenderTarget()
    {
        auto* mainWindow = WindowManager::getMainWindow();
        if (!hasSeparateWorldResources() || mainWindow == nullptr || mainWindow->viewports[0] == nullptr)
        {
            return;
        }

        const auto& viewport = *mainWindow->viewports[0];
        auto scale = 1;
        Point phase{};
        Point origin{ viewport.viewX, viewport.viewY };
        auto renderWidth = _outputWidth;
        auto renderHeight = _outputHeight;
        const auto antiAliasing = _postProcessor->getMode();

        if (antiAliasing != Config::AntiAliasing::none && viewport.zoom < ZoomLevel::full)
        {
            // Render at zoom 0, retaining the magnified viewport's subpixel camera phase for presentation.
            scale = viewport.zoom.applyInversedTo(1);
            const auto [originOffsetX, remainderX] = divideWithPositiveRemainder(viewport.viewRasterOffset.x, scale);
            const auto [originOffsetY, remainderY] = divideWithPositiveRemainder(viewport.viewRasterOffset.y, scale);
            origin += Point{ originOffsetX, originOffsetY };
            phase = Point{ remainderX, remainderY };
            renderWidth = getNativeRenderDimension(_outputWidth, scale);
            renderHeight = getNativeRenderDimension(_outputHeight, scale);
        }

        if ((renderWidth != _worldRT.width || renderHeight != _worldRT.height)
            && antiAliasing != Config::AntiAliasing::none
            && !_postProcessor->configure(_renderer, renderWidth, renderHeight, antiAliasing))
        {
            Logging::warn("Unable to resize anti-aliasing resources; using the magnified unfiltered world raster.");
            scale = 1;
            phase = {};
            origin = { viewport.viewX, viewport.viewY };
            renderWidth = _outputWidth;
            renderHeight = _outputHeight;
        }

        const auto dimensionsChanged = renderWidth != _worldRT.width || renderHeight != _worldRT.height;
        const auto scaleChanged = scale != _worldPresentationScale;
        const auto phaseChanged = phase != _worldPresentationPhase;
        const auto originChanged = origin != _worldRenderOrigin;

        if (dimensionsChanged)
        {
            _worldRT.width = renderWidth;
            _worldRT.height = renderHeight;
            _worldRT.pitch = _worldSurface->pitch - renderWidth;
            _worldUploadGrid.reset(renderWidth, renderHeight, 64, 8);
            _worldTextureDirty = true;
        }

        if (!_worldTransformValid || dimensionsChanged || scaleChanged || originChanged)
        {
            _invalidationGrid.invalidate(0, 0, _screenRT.width, _screenRT.height);
            _uiTextureDirty = true;
        }

        _worldPresentationScale = scale;
        _worldPresentationPhase = phase;
        _worldRenderOrigin = origin;
        _worldTransformValid = true;

        if (dimensionsChanged || scaleChanged || phaseChanged)
        {
            updateUiToWorldMap();
            _uiTextureDirty = true;
        }
    }

    void SoftwareDrawingEngine::updateUiToWorldMap()
    {
        if (_worldRT.width <= 0 || _worldRT.height <= 0 || _screenRT.width <= 0 || _screenRT.height <= 0)
        {
            _uiToWorldX.clear();
            _uiToWorldY.clear();
            return;
        }

        _uiToWorldX.resize(_screenRT.width);
        _uiToWorldY.resize(_screenRT.height);
        for (int32_t x = 0; x < _screenRT.width; ++x)
        {
            const auto outputX = sampleNearest(x, _outputWidth, _screenRT.width);
            _uiToWorldX[x] = std::min((outputX + _worldPresentationPhase.x) / _worldPresentationScale, _worldRT.width - 1);
        }
        for (int32_t y = 0; y < _screenRT.height; ++y)
        {
            const auto outputY = sampleNearest(y, _outputHeight, _screenRT.height);
            _uiToWorldY[y] = std::min((outputY + _worldPresentationPhase.y) / _worldPresentationScale, _worldRT.height - 1);
        }
    }

    void SoftwareDrawingEngine::renderDirtyWorldRegions()
    {
        const auto uiWidth = _screenRT.width;
        const auto uiHeight = _screenRT.height;
        _invalidationGrid.traverseDirtyCells([&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            const auto outputLeft = static_cast<int32_t>(static_cast<int64_t>(left) * _outputWidth / uiWidth);
            const auto outputTop = static_cast<int32_t>(static_cast<int64_t>(top) * _outputHeight / uiHeight);
            const auto outputRight = static_cast<int32_t>((static_cast<int64_t>(right) * _outputWidth + uiWidth - 1) / uiWidth);
            const auto outputBottom = static_cast<int32_t>((static_cast<int64_t>(bottom) * _outputHeight + uiHeight - 1) / uiHeight);
            // Refresh neighbouring source pixels read by the post-process passes.
            const auto worldLeft = std::max(0, (outputLeft + _worldPresentationPhase.x) / _worldPresentationScale - 1);
            const auto worldTop = std::max(0, (outputTop + _worldPresentationPhase.y) / _worldPresentationScale - 1);
            const auto worldRight = std::min(_worldRT.width, (outputRight + _worldPresentationPhase.x + _worldPresentationScale - 1) / _worldPresentationScale + 1);
            const auto worldBottom = std::min(_worldRT.height, (outputBottom + _worldPresentationPhase.y + _worldPresentationScale - 1) / _worldPresentationScale + 1);
            renderSeparateWorld(Rect::fromLTRB(worldLeft, worldTop, worldRight, worldBottom));
            _worldUploadGrid.invalidate(worldLeft, worldTop, worldRight, worldBottom);
        });
    }

    void SoftwareDrawingEngine::renderSeparateWorld(const Rect& rect)
    {
        auto* mainWindow = WindowManager::getMainWindow();
        if (mainWindow == nullptr || mainWindow->viewports[0] == nullptr)
        {
            return;
        }

        SoftwareDrawingContext worldContext;
        auto worldTarget = _worldRT;
        const auto worldStride = worldTarget.width + worldTarget.pitch;
        worldTarget.bits += rect.left() + static_cast<size_t>(rect.top()) * worldStride;
        worldTarget.x = rect.left();
        worldTarget.y = rect.top();
        worldTarget.width = rect.width();
        worldTarget.height = rect.height();
        worldTarget.pitch = worldStride - rect.width();
        worldContext.pushRenderTarget(worldTarget);

        auto viewport = *mainWindow->viewports[0];
        viewport.x = 0;
        viewport.y = 0;
        if (_worldPresentationScale > 1)
        {
            viewport.zoom = ZoomLevel::full;
            viewport.viewX = _worldRenderOrigin.x;
            viewport.viewY = _worldRenderOrigin.y;
            viewport.viewRasterOffset = {};
        }
        viewport.setDimensions({ _worldRT.width, _worldRT.height }, { _worldRT.width, _worldRT.height });
        viewport.render(worldContext, false, true);
    }

    void SoftwareDrawingEngine::renderSeparateUi()
    {
        _screenTextureDirty = true;
        const auto uiStride = _screenRT.width + _screenRT.pitch;
        const auto isProgressBarActive = SceneManager::isProgressBarActive();
        if (!isProgressBarActive)
        {
            const auto worldStride = _worldRT.width + _worldRT.pitch;
            const auto rows = std::views::iota(0, _screenRT.height);
            std::for_each(std::execution::par, rows.begin(), rows.end(), [&](int32_t y) {
                const auto worldY = _uiToWorldY[y];
                for (int32_t x = 0; x < _screenRT.width; ++x)
                {
                    const auto worldX = _uiToWorldX[x];
                    _uiBase[static_cast<size_t>(y) * uiStride + x] = _worldRT.bits[static_cast<size_t>(worldY) * worldStride + worldX];
                }
            });

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
                std::fill_n(_uiCoverage.data() + static_cast<size_t>(y) * uiStride + rect.left(), rect.width(), isProgressBarActive ? 0xFF : 0);
            }
        }

        measure(_frameStatsEnabled, _frameStats.paletteConversionNs, [&] {
            const auto palette = Gfx::getRgbaPalette();
            const auto rows = std::views::iota(0, _screenRT.height);
            std::for_each(std::execution::par, rows.begin(), rows.end(), [&](int32_t y) {
                auto* output = static_cast<uint32_t*>(_uiRGBASurface->pixels) + static_cast<size_t>(y) * _uiRGBASurface->pitch / sizeof(uint32_t);
                for (int32_t x = 0; x < _screenRT.width; ++x)
                {
                    const auto offset = static_cast<size_t>(y) * uiStride + x;
                    if (!isProgressBarActive)
                    {
                        const auto isCovered = _uiCoverage[offset] != 0xFF || _screenRT.bits[offset] != _uiBase[offset];
                        _uiCoverage[offset] = isCovered ? 0xFF : 0;
                    }
                    else if (_screenRT.bits[offset] != _uiBase[offset])
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
            });
        });
    }

    void SoftwareDrawingEngine::renderDirtyRegions()
    {
        _invalidationGrid.traverseDirtyCells([this](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            this->render(Rect::fromLTRB(left, top, right, bottom));
        });
    }

    void SoftwareDrawingEngine::render(const Rect& _rect)
    {
        _uiTextureDirty = true;
        auto max = Rect(0, 0, Ui::width(), Ui::height());
        auto rect = _rect.intersection(max);
        if (rect.width() <= 0 || rect.height() <= 0)
        {
            return;
        }

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

        _screenUploadGrid.invalidate(rect.left(), rect.top(), rect.right(), rect.bottom());
    }

    bool SoftwareDrawingEngine::present()
    {
        _lastPresentationHash = 0;
        const auto result = shouldUseSeparateWorld() ? presentSeparate() : presentStandard();
        _lastFrameStats = _frameStats;
        return result;
    }

    bool SoftwareDrawingEngine::capturePresentationHash()
    {
        if (!_presentationReadbackEnabled)
        {
            return true;
        }

        auto* surface = SDL_RenderReadPixels(_renderer, nullptr);
        if (surface == nullptr)
        {
            Logging::error("Unable to read presentation pixels: {}", SDL_GetError());
            return false;
        }

        if (surface->format != SDL_PIXELFORMAT_RGBA32)
        {
            auto* convertedSurface = SDL_ConvertSurface(surface, SDL_PIXELFORMAT_RGBA32);
            SDL_DestroySurface(surface);
            surface = convertedSurface;
            if (surface == nullptr)
            {
                Logging::error("Unable to convert presentation pixels: {}", SDL_GetError());
                return false;
            }
        }

        uint64_t hash = 14695981039346656037ULL;
        for (int32_t y = 0; y < surface->h; ++y)
        {
            const auto* pixels = static_cast<const uint8_t*>(surface->pixels) + static_cast<size_t>(y) * surface->pitch;
            for (int32_t x = 0; x < surface->w * 4; ++x)
            {
                hash ^= pixels[x];
                hash *= 1099511628211ULL;
            }
        }
        SDL_DestroySurface(surface);
        _lastPresentationHash = hash;
        return true;
    }

    bool SoftwareDrawingEngine::presentStandard()
    {
        if (_screenSurface == nullptr || (!_screenTextureIndexed && _screenRGBASurface == nullptr) || _screenTexture == nullptr)
        {
            return false;
        }

        const auto& rt = getScreenRT();
        const auto indexedPitch = rt.width + rt.pitch;
        const auto uploadRegion = [&](const SDL_Rect& rect) {
            const void* pixels = rt.bits + static_cast<size_t>(rect.y) * indexedPitch + rect.x;
            auto pitch = indexedPitch;
            auto bytesPerPixel = sizeof(PaletteIndex_t);

            if (!_screenTextureIndexed)
            {
                const auto converted = measureResult(_frameStatsEnabled, _frameStats.paletteConversionNs, [&] {
                    if (SDL_MUSTLOCK(_screenSurface) && !SDL_LockSurface(_screenSurface))
                    {
                        return false;
                    }

                    for (int32_t y = rect.y; y < rect.y + rect.h; ++y)
                    {
                        auto* destination = static_cast<uint8_t*>(_screenSurface->pixels) + static_cast<size_t>(y) * _screenSurface->pitch + rect.x;
                        const auto* source = rt.bits + static_cast<size_t>(y) * indexedPitch + rect.x;
                        std::memcpy(destination, source, rect.w);
                    }

                    if (SDL_MUSTLOCK(_screenSurface))
                    {
                        SDL_UnlockSurface(_screenSurface);
                    }
                    return SDL_BlitSurface(_screenSurface, &rect, _screenRGBASurface, &rect);
                });
                if (!converted)
                {
                    return false;
                }

                pixels = static_cast<uint8_t*>(_screenRGBASurface->pixels)
                    + static_cast<size_t>(rect.y) * _screenRGBASurface->pitch
                    + static_cast<size_t>(rect.x) * sizeof(uint32_t);
                pitch = _screenRGBASurface->pitch;
                bytesPerPixel = sizeof(uint32_t);
            }

            const auto uploaded = measureResult(_frameStatsEnabled, _frameStats.textureUploadNs, [&] {
                return SDL_UpdateTexture(_screenTexture, &rect, pixels, pitch);
            });
            if (uploaded)
            {
                _frameStats.screenUploadBytes += static_cast<uint64_t>(rect.w) * rect.h * bytesPerPixel;
                _frameStats.textureUploadCount++;
            }
            return uploaded;
        };

        const auto uploaded = Detail::uploadDirtyRegions(_screenUploadGrid, _screenTextureDirty, rt.width, rt.height, [&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            return uploadRegion(SDL_Rect{ left, top, right - left, bottom - top });
        });
        _screenTextureDirty = !uploaded;

        if (!uploaded)
        {
            Logging::error("Unable to update screen texture: {}", SDL_GetError());
            return false;
        }

        return measureResult(_frameStatsEnabled, _frameStats.composePresentNs, [&] {
            auto* displayTexture = _screenTexture;
            if (_scaledScreenTexture != nullptr && _pixelScaleFactor > 1)
            {
                // Integer nearest-neighbour prescale followed by linear downsampling.
                if (!SDL_SetRenderTarget(_renderer, _scaledScreenTexture))
                {
                    Logging::error("SDL_SetRenderTarget (_scaledScreenTexture) failed: {}", SDL_GetError());
                    return false;
                }
                if (!SDL_RenderTexture(_renderer, _screenTexture, nullptr, nullptr))
                {
                    Logging::error("SDL_RenderTexture (_screenTexture) failed: {}", SDL_GetError());
                    SDL_SetRenderTarget(_renderer, nullptr);
                    return false;
                }

                if (!SDL_SetRenderTarget(_renderer, nullptr))
                {
                    Logging::error("SDL_SetRenderTarget (nullptr) failed: {}", SDL_GetError());
                    return false;
                }
                displayTexture = _scaledScreenTexture;
            }

            if (!SDL_RenderTexture(_renderer, displayTexture, nullptr, nullptr))
            {
                Logging::error("SDL_RenderTexture failed: {}", SDL_GetError());
                return false;
            }

            if (!capturePresentationHash())
            {
                return false;
            }
            if (!SDL_RenderPresent(_renderer))
            {
                Logging::error("SDL_RenderPresent failed: {}", SDL_GetError());
                return false;
            }
            return true;
        });
    }

    bool SoftwareDrawingEngine::presentSeparate()
    {
        const auto worldPitch = _worldRT.width + _worldRT.pitch;
        const auto uploadWorldRegion = [&](const SDL_Rect& rect) {
            const void* pixels = _worldRT.bits + static_cast<size_t>(rect.y) * worldPitch + rect.x;
            auto pitch = worldPitch;
            auto bytesPerPixel = sizeof(PaletteIndex_t);

            if (!_worldTextureIndexed)
            {
                const auto converted = measureResult(_frameStatsEnabled, _frameStats.paletteConversionNs, [&] {
                    return SDL_BlitSurface(_worldSurface, &rect, _worldRGBASurface, &rect);
                });
                if (!converted)
                {
                    return false;
                }

                pixels = static_cast<uint8_t*>(_worldRGBASurface->pixels)
                    + static_cast<size_t>(rect.y) * _worldRGBASurface->pitch
                    + static_cast<size_t>(rect.x) * sizeof(uint32_t);
                pitch = _worldRGBASurface->pitch;
                bytesPerPixel = sizeof(uint32_t);
            }

            const auto uploaded = measureResult(_frameStatsEnabled, _frameStats.textureUploadNs, [&] {
                return SDL_UpdateTexture(_worldTexture, &rect, pixels, pitch);
            });
            if (uploaded)
            {
                _frameStats.worldUploadBytes += static_cast<uint64_t>(rect.w) * rect.h * bytesPerPixel;
                _frameStats.textureUploadCount++;
            }
            return uploaded;
        };

        const auto worldUploaded = Detail::uploadDirtyRegions(_worldUploadGrid, _worldTextureDirty, _worldRT.width, _worldRT.height, [&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            return uploadWorldRegion(SDL_Rect{ left, top, right - left, bottom - top });
        });
        _worldTextureDirty = !worldUploaded;

        auto uiUploaded = true;
        if (_uiTextureUploadPending)
        {
            uiUploaded = measureResult(_frameStatsEnabled, _frameStats.textureUploadNs, [&] {
                return SDL_UpdateTexture(_uiTexture, nullptr, _uiRGBASurface->pixels, _uiRGBASurface->pitch);
            });
            if (uiUploaded)
            {
                _frameStats.uiUploadBytes += static_cast<uint64_t>(_uiRGBASurface->w) * _uiRGBASurface->h * sizeof(uint32_t);
                _frameStats.textureUploadCount++;
            }
        }
        _uiTextureUploadPending = !uiUploaded;

        if (!worldUploaded || !uiUploaded)
        {
            Logging::error("Unable to upload separate world rendering textures: {}", SDL_GetError());
            return false;
        }

        const auto presented = measureResult(_frameStatsEnabled, _frameStats.composePresentNs, [&] {
            const SDL_FRect sourceRect{ 0, 0, static_cast<float>(_worldRT.width), static_cast<float>(_worldRT.height) };
            const SDL_FRect destinationRect{
                static_cast<float>(-_worldPresentationPhase.x),
                static_cast<float>(-_worldPresentationPhase.y),
                static_cast<float>(_worldRT.width) * _worldPresentationScale,
                static_cast<float>(_worldRT.height) * _worldPresentationScale,
            };

            auto* worldTexture = _worldTexture;
            const SDL_FRect* worldSourceRect = &sourceRect;
            if (_postProcessor->getMode() != Config::AntiAliasing::none)
            {
                worldTexture = _postProcessor->process(_worldTexture, &sourceRect);
                if (worldTexture != nullptr)
                {
                    worldSourceRect = nullptr;
                }
                else
                {
                    Logging::error("Anti-aliasing failed during presentation: {}. Disabling it.", SDL_GetError());
                    _postProcessor->reset();
                    worldTexture = _worldTexture;
                }
            }

            auto worldPresented = SDL_RenderTexture(_renderer, worldTexture, worldSourceRect, &destinationRect);
            if (!worldPresented && worldTexture != _worldTexture)
            {
                Logging::error("Unable to draw the anti-aliased world: {}. Disabling it.", SDL_GetError());
                _postProcessor->reset();
                worldPresented = SDL_RenderTexture(_renderer, _worldTexture, &sourceRect, &destinationRect);
            }
            return worldPresented
                && SDL_RenderTexture(_renderer, _uiTexture, nullptr, nullptr)
                && capturePresentationHash()
                && SDL_RenderPresent(_renderer);
        });
        if (!presented)
        {
            Logging::error("Unable to present separate world rendering textures: {}", SDL_GetError());
        }
        return presented;
    }

    DrawingContext& SoftwareDrawingEngine::getDrawingContext()
    {
        // Callers can mutate the default screen target directly.
        _screenTextureDirty = true;
        _uiTextureDirty = true;
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
        _screenshotBuffer.resize(static_cast<size_t>(_outputWidth) * _outputHeight);
        for (int32_t y = 0; y < _outputHeight; ++y)
        {
            const auto uiY = sampleNearest(y, _screenRT.height, _outputHeight);
            const auto worldY = std::min((y + _worldPresentationPhase.y) / _worldPresentationScale, _worldRT.height - 1);
            for (int32_t x = 0; x < _outputWidth; ++x)
            {
                const auto uiX = sampleNearest(x, _screenRT.width, _outputWidth);
                const auto worldX = std::min((x + _worldPresentationPhase.x) / _worldPresentationScale, _worldRT.width - 1);
                const auto uiOffset = static_cast<size_t>(uiY) * uiStride + uiX;
                _screenshotBuffer[static_cast<size_t>(y) * _outputWidth + x] = _uiCoverage[uiOffset] != 0
                    ? _screenRT.bits[uiOffset]
                    : _worldRT.bits[static_cast<size_t>(worldY) * worldStride + worldX];
            }
        }

        _screenshotRT = RenderTarget{ _screenshotBuffer.data(), 0, 0, _outputWidth, _outputHeight, 0 };
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
        _uiTextureDirty = true;
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
        if (width <= 0 || height <= 0)
        {
            return;
        }

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
        _screenUploadGrid.invalidate(dstX, dstY, dstX + width, dstY + height);
    }

    const Ui::ScreenInfo& SoftwareDrawingEngine::getScreenInfo() const
    {
        return _screenInfo;
    }

    Ui::Size SoftwareDrawingEngine::getOutputSize() const
    {
        return { _outputWidth, _outputHeight };
    }

    Ui::Size SoftwareDrawingEngine::getPresentationSize() const
    {
        int32_t width{};
        int32_t height{};
        return SDL_GetRenderOutputSize(_renderer, &width, &height) ? Ui::Size{ width, height } : Ui::Size{};
    }

    bool SoftwareDrawingEngine::setVSync(bool state)
    {
        return SDL_SetRenderVSync(_renderer, state ? 1 : SDL_RENDERER_VSYNC_DISABLED);
    }

    bool SoftwareDrawingEngine::isVSyncDisabled() const
    {
        int vsync{};
        return _renderer != nullptr
            && SDL_GetRenderVSync(_renderer, &vsync)
            && vsync == SDL_RENDERER_VSYNC_DISABLED;
    }

    bool SoftwareDrawingEngine::isGpuPaletteEnabled() const
    {
        return getRendererName() == SDL_GPU_RENDERER
            && _screenTextureIndexed
            && (!hasSeparateWorldResources() || _worldTextureIndexed);
    }

    bool SoftwareDrawingEngine::supportsAntiAliasing() const
    {
        return PostProcessor::isSupported(_renderer);
    }

    Config::AntiAliasing SoftwareDrawingEngine::getActiveAntiAliasing() const
    {
        return _postProcessor->getMode();
    }

    void SoftwareDrawingEngine::setFrameStatsEnabled(bool enabled)
    {
        _frameStatsEnabled = enabled;
    }

    const RenderFrameStats& SoftwareDrawingEngine::getLastFrameStats() const
    {
        return _lastFrameStats;
    }

    void SoftwareDrawingEngine::setPresentationReadbackEnabled(bool enabled)
    {
        _presentationReadbackEnabled = enabled;
    }

    uint64_t SoftwareDrawingEngine::getLastPresentationHash() const
    {
        return _lastPresentationHash;
    }

    std::string_view SoftwareDrawingEngine::getRendererName() const
    {
        const auto* name = _renderer == nullptr ? nullptr : SDL_GetRendererName(_renderer);
        return name == nullptr ? std::string_view{} : name;
    }
}
