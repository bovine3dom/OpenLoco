#include "Scenario/Scenario.h"
#include <SDL3/SDL_events.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
// timeGetTime is unavailable if we use lean and mean
// #define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <objbase.h>
#include <windows.h>

// `small` is used as a type in `windows.h`
#undef small
#endif

#include "Audio/Audio.h"
#include "Config.h"
#include "Entities/EntityManager.h"
#include "Entities/EntityTweener.h"
#include "Environment.h"
#include "GameCommands/GameCommands.h"
#include "GameState.h"
#include "Graphics/Colour.h"
#include "Graphics/Gfx.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/SoftwareDrawingEngine.h"
#include "Gui.h"
#include "Input.h"
#include "Input/Shortcuts.h"
#include "Localisation/Formatting.h"
#include "Localisation/LanguageFiles.h"
#include "Localisation/Languages.h"
#include "Localisation/StringIds.h"
#include "Logging.h"
#include "Map/TileManager.h"
#include "MessageManager.h"
#include "MultiPlayer.h"
#include "Network/Network.h"
#include "Objects/ObjectIndex.h"
#include "Objects/ObjectManager.h"
#include "OpenLoco.h"
#include "Scenario/ScenarioManager.h"
#include "SceneManager.h"
#include "Scenes/BootScene.h"
#include "Scenes/GameScene.h"
#include "Tutorial.h"
#include "Ui.h"
#include "Ui/ProgressBar.h"
#include "Ui/ToolTip.h"
#include "Ui/WindowManager.h"
#include "Vehicles/Vehicle.h"
#include "ViewportManager.h"
#include "World/CompanyManager.h"
#include <OpenLoco/Core/Numerics.hpp>
#include <OpenLoco/Platform/Crash.h>
#include <OpenLoco/Platform/Platform.h>
#include <OpenLoco/Version.hpp>

using namespace OpenLoco::Ui;
using namespace OpenLoco::Input;
using namespace OpenLoco::Diagnostics;

namespace OpenLoco
{
    using Clock = std::chrono::high_resolution_clock;
    using Timepoint = Clock::time_point;

    static double _accumulator = 0.0;
    static Timepoint _lastUpdate = Clock::now();
    static CrashHandler::Handle _exHandler = nullptr;

    static uint32_t _time_since_last_tick; // 0x0050C19C
    static uint32_t _last_tick_time;       // 0x0050C19E
    static uint16_t _numFrameUpdates;      // 0x00F253A0

    // 0x004BE621
    [[noreturn]] void exitWithError(StringId titleStringId, StringId messageStringId)
    {
        char titleBuffer[256] = { 0 };
        char messageBuffer[256] = { 0 };
        StringManager::formatString(titleBuffer, 255, titleStringId);
        StringManager::formatString(messageBuffer, 255, messageStringId);
        Ui::showMessageBox(titleBuffer, messageBuffer);

        exitCleanly();
    }

    // 0x004BE65E
    [[noreturn]] void exitCleanly()
    {
        Audio::close();
        Audio::disposeDSound();
        Ui::disposeCursors();
        Localisation::unloadLanguageFile();

        auto tempFilePath = Environment::getPathNoWarning(Environment::PathId::_1tmp);
        if (fs::exists(tempFilePath))
        {
            auto path8 = tempFilePath.u8string();
            Logging::info("Removing temp file '{}'", path8.c_str());
            fs::remove(tempFilePath);
        }
        CrashHandler::shutdown(_exHandler);

        // Logging should be the last before terminating.
        Logging::shutdown();

        // SDL_Quit();
        exit(0);
    }

    // 0x00441400
    static void startupChecks()
    {
        const auto& config = Config::get();
        if (!config.allowMultipleInstances && !Platform::lockSingleInstance())
        {
            exitWithError(StringIds::game_init_failure, StringIds::loco_already_running);
        }

        // Originally the game would check that all the game
        // files exist are some have the correct checksum. We
        // do not need to do this anymore, the game should work
        // with g1 alone and some objects?
    }

    // 0x004C57C0
    void resetSubsystems()
    {
        Ui::Windows::MapToolTip::reset();

        Colours::initColourMap();
        Ui::WindowManager::init();
        Ui::ViewportManager::init();

        Input::init();
        Input::initMouse();

        // tooltip-related
        Ui::ToolTip::set_52336E(false);

        Ui::Windows::TextInput::cancel();

        // TODO Move this to a more generic, initialise game state function when
        //      we have one hooked / implemented.
        Scenes::GameScene::autosaveReset();
    }

    void initialise()
    {
        _last_tick_time = Platform::getTime();

        std::srand(std::time(nullptr));

        // Do this first since some shutdown logic might otherwise read bad data.
        EntityManager::reset();

        Localisation::enumerateLanguages();
        Localisation::loadLanguageFile();

        startupChecks();

        Input::Shortcuts::initialize();
        World::TileManager::allocateMapElements();

        Gfx::loadG1();
        Gfx::initialise();

        Ui::initialise();
        resetSubsystems();
        Gui::init();

        MessageManager::reset();
        Scenario::reset();

        ObjectManager::loadIndex();
        ScenarioManager::loadIndex();
    }

    void sub_431695(uint16_t var_F253A0)
    {
        GameCommands::setUpdatingCompanyId(CompanyManager::getControllingId());
        for (auto i = 0; i < var_F253A0; i++)
        {
            MessageManager::sub_428E47();
            WindowManager::dispatchUpdateAll();
        }

        Input::processKeyboardInput();
        WindowManager::tick();
        Ui::handleInput();
        CompanyManager::updateOwnerStatus();
    }

    // The remainder of a tick is abandoned when a scene transition was requested, the game
    // state it was operating on has been replaced by that point.
    static void applyPendingScene()
    {
        if (!SceneManager::applySceneTransition())
        {
            return;
        }

        EntityTweener::get().reset();
    }

    // Decides how many ticks the world is advanced by this fixed update.
    static uint32_t calculateNumTicks()
    {
        uint32_t numUpdates = std::clamp<uint32_t>(_time_since_last_tick / 31U, 1, 3);

        if (WindowManager::find(Ui::WindowType::multiplayer, 0) != nullptr)
        {
            numUpdates = 1;
        }
        if (SceneManager::isNetworked())
        {
            numUpdates = 1;
        }

        if (Input::hasPendingMouseInputUpdate())
        {
            Input::clearPendingMouseInputUpdate();
            numUpdates = 1;
        }
        else
        {
            switch (Input::state())
            {
                case State::reset:
                case State::normal:
                case State::dropdownActive:
                    if (Input::hasFlag(Flags::viewportScrolling))
                    {
                        Input::resetFlag(Flags::viewportScrolling);
                        numUpdates = 1;
                    }
                    break;
                case State::widgetPressed: break;
                case State::positioningWindow: break;
                case State::viewportRight: break;
                case State::viewportLeft: break;
                case State::scrollLeft: break;
                case State::resizing: break;
                case State::scrollRight: break;
            }
        }

        Ui::WindowManager::setVehiclePreviewRotationFrame(Ui::WindowManager::getVehiclePreviewRotationFrame() + numUpdates);

        if (SceneManager::isPaused())
        {
            numUpdates = 0;
        }

        _numFrameUpdates = std::max<uint16_t>(1, numUpdates);
        SceneManager::setSceneAge(std::min(0xFFFF, (int32_t)SceneManager::getSceneAge() + _numFrameUpdates));

        if (SceneManager::getGameSpeed() != GameSpeed::Normal)
        {
            numUpdates *= 3;
            if (SceneManager::getGameSpeed() != GameSpeed::FastForward)
            {
                numUpdates *= 3;
            }
        }

        // Catch up to server (usually after we have just joined the game)
        auto numTicksBehind = Network::getServerTick() - ScenarioManager::getScenarioTicks();
        if (numTicksBehind > 4)
        {
            numUpdates = 4;
        }

        return numUpdates;
    }

    // 0x0046A794
    static void tick()
    {
        uint32_t time = Platform::getTime();
        _time_since_last_tick = (uint16_t)std::min(time - _last_tick_time, 500U);
        _last_tick_time = time;

        if (Tutorial::state() != Tutorial::State::none)
        {
            _time_since_last_tick = 31;
        }

        GameCommands::resetCommandNestLevel();
        Ui::tick();

        // Original called 0x00440DEC here which handled legacy cmd line options
        // like installing scenarios and handling multiplayer.

        Input::handleKeyboard();
        Input::processMouseMovement();
        Audio::tick();

        // Network messages are handled outside of the scenes, they can request a scene transition.
        Network::tick();

        applyPendingScene();

        const auto numTicks = calculateNumTicks();
        for (auto i = 0U; i < numTicks; i++)
        {
            SceneManager::tick();
        }

        SceneManager::tickInterface();

        applyPendingScene();
    }

    static void tickWait()
    {
        // Idle loop for a 40 FPS
        do
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (Platform::getTime() - _last_tick_time < Engine::UpdateRateInMs);
    }

    bool promptTickLoop(std::function<bool()> tickAction)
    {
        while (true)
        {
            _last_tick_time = Platform::getTime();
            _time_since_last_tick = 31;
            if (!Input::processMessages())
            {
                return false;
            }
            if (!tickAction())
            {
                break;
            }
            Ui::render();
            tickWait();
        }
        return true;
    }

    constexpr auto MaxUpdateTime = static_cast<double>(Engine::MaxTimeDeltaMs) / 1000.0;
    constexpr auto UpdateTime = static_cast<double>(Engine::UpdateRateInMs) / 1000.0;
    constexpr auto TimeScale = 1.0;

    static void variableUpdate()
    {
        auto& tweener = EntityTweener::get();

        const auto alpha = std::min<float>(_accumulator / UpdateTime, 1.0);

        while (_accumulator > UpdateTime)
        {
            tweener.preTick();

            tick();
            _accumulator -= UpdateTime;

            tweener.postTick();
        }

        tweener.tween(alpha);

        SceneManager::update();

        Ui::render();
    }

    static void fixedUpdate()
    {
        auto& tweener = EntityTweener::get();
        tweener.reset();

        if (_accumulator < UpdateTime)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else
        {
            tick();
            _accumulator -= UpdateTime;

            SceneManager::update();

            Ui::render();
        }
    }

    void update()
    {
        auto timeNow = Clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timeNow - _lastUpdate).count() / 1'000'000.0;

        elapsed *= TimeScale;

        _accumulator = std::min(_accumulator + elapsed, MaxUpdateTime);
        _lastUpdate = timeNow;

        if (Config::get().uncapFPS)
        {
            variableUpdate();
        }
        else
        {
            fixedUpdate();
        }
    }

    uint16_t getTimeSinceLastTick()
    {
        return _time_since_last_tick;
    }

    uint16_t getNumFrameUpdates()
    {
        return _numFrameUpdates;
    }

    void simulateGame(const fs::path& savePath, int32_t ticks)
    {
        try
        {
            initialise();
            Scenes::BootScene::loadFile(savePath);

            // The load itself is performed as part of the scene transition.
            SceneManager::applySceneTransition();
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to simulate park: {}", e.what());
        }

        if (SceneManager::getCurrentScene() != SceneManager::SceneId::gameplay)
        {
            Logging::error("Unable to simulate park!");
            return;
        }

        Logging::info("File loaded. Starting simulation.");

        for (int32_t i = 0; i < ticks; i++)
        {
            if (SceneManager::isSceneTransitionPending())
            {
                break;
            }

            Scenes::GameScene::tick();
        }
    }

    bool runRenderBenchmark(const fs::path& savePath, int32_t warmupFrames, int32_t frames, int32_t width, int32_t height, float scaleFactor)
    {
        auto& config = Config::get();
        struct BenchmarkStateGuard
        {
            Config::Config& config;
            Config::Config originalConfig;
            bool writesEnabled;
            bool audioEnabled;

            ~BenchmarkStateGuard()
            {
                if (Audio::isAudioEnabled() != audioEnabled)
                {
                    Audio::toggleSound();
                }
                config = std::move(originalConfig);
                Config::setWriteEnabled(writesEnabled);
            }
        } stateGuard{ config, config, Config::setWriteEnabled(false), Audio::isAudioEnabled() };

        config.display.mode = Config::ScreenMode::window;
        config.display.windowResolution = { width, height };
        config.display.vsync = false;
        config.scaleFactor = scaleFactor;
        config.nativeViewportRendering = true;
        config.showFPS = false;
        config.allowMultipleInstances = true;

        Ui::createWindow(config.display);
        initialise();
        if (Audio::isAudioEnabled())
        {
            Audio::toggleSound();
        }
        Gfx::loadDefaultPalette();
        Gfx::invalidateScreen();
        SceneManager::addSceneFlags(SceneManager::Flags::initialised);
        const auto loadRequested = Scenes::BootScene::loadFile(savePath);
        const auto sceneChanged = SceneManager::applySceneTransition();

        auto& drawingEngine = Gfx::getDrawingEngine();
        drawingEngine.resize(width, height);
        Gui::resize();
        Gfx::invalidateScreen();
        const auto outputSize = drawingEngine.getOutputSize();
        const auto presentationSize = drawingEngine.getPresentationSize();
        if (!loadRequested
            || !sceneChanged
            || SceneManager::isSceneTransitionPending()
            || SceneManager::getCurrentScene() != SceneManager::SceneId::gameplay
            || !drawingEngine.shouldUseSeparateWorld()
            || !drawingEngine.setVSync(false)
            || WindowManager::find(WindowType::topToolbar) == nullptr
            || presentationSize.width <= 0
            || presentationSize.height <= 0
            || outputSize.width != width
            || outputSize.height != height)
        {
            Logging::error(
                "Render benchmark setup failed (gameplay: {}, pending transition: {}, native renderer: {}, requested framebuffer: {}x{}, actual: {}x{})",
                SceneManager::getCurrentScene() == SceneManager::SceneId::gameplay,
                SceneManager::isSceneTransitionPending(),
                drawingEngine.shouldUseSeparateWorld(),
                width,
                height,
                outputSize.width,
                outputSize.height);
            return false;
        }

        _numFrameUpdates = 1;
        _time_since_last_tick = Engine::UpdateRateInMs;
        const auto processEvents = [&] {
            SDL_Event event{};
            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED)
                {
                    throw std::runtime_error("Render benchmark window was closed");
                }
            }

            const auto currentSize = drawingEngine.getPresentationSize();
            if (currentSize.width != presentationSize.width || currentSize.height != presentationSize.height)
            {
                throw std::runtime_error("Presentation size changed during render benchmark");
            }
        };
        constexpr std::array<float, 4> kTweenFrames = { 0.0F, 0.25F, 0.5F, 0.75F };
        auto& tweener = EntityTweener::get();
        auto prepareFrame = [&](int32_t frame) {
            const auto tweenFrame = static_cast<size_t>(frame) % kTweenFrames.size();
            if (tweenFrame == 0)
            {
                tweener.preTick();
                Scenes::GameScene::tick();
                Scenes::GameScene::tickInterface();
                tweener.postTick();
                if (SceneManager::isSceneTransitionPending())
                {
                    throw std::runtime_error("Scene transition requested during render benchmark");
                }
            }
            tweener.tween(kTweenFrames[tweenFrame]);
            SceneManager::update();
        };

        for (int32_t frame = 0; frame < warmupFrames; ++frame)
        {
            processEvents();
            prepareFrame(frame);
            drawingEngine.render();
            if (!drawingEngine.present())
            {
                throw std::runtime_error("Renderer presentation failed during benchmark warmup");
            }
        }

        using BenchmarkClock = std::chrono::steady_clock;
        std::vector<uint64_t> frameTotals(static_cast<size_t>(frames));
        std::vector<uint64_t> renderTotals(static_cast<size_t>(frames));
        std::vector<uint64_t> presentTotals(static_cast<size_t>(frames));
        for (int32_t frame = 0; frame < frames; ++frame)
        {
            processEvents();
            prepareFrame(frame + warmupFrames);

            const auto index = static_cast<size_t>(frame);
            const auto renderStart = BenchmarkClock::now();
            drawingEngine.render();
            renderTotals[index] = std::chrono::duration_cast<std::chrono::nanoseconds>(BenchmarkClock::now() - renderStart).count();

            const auto presentStart = BenchmarkClock::now();
            if (!drawingEngine.present())
            {
                throw std::runtime_error("Renderer presentation failed during benchmark");
            }
            presentTotals[index] = std::chrono::duration_cast<std::chrono::nanoseconds>(BenchmarkClock::now() - presentStart).count();
            frameTotals[index] = renderTotals[index] + presentTotals[index];
        }

        const auto logMetric = [](std::string_view name, std::vector<uint64_t> values) {
            std::ranges::sort(values);
            const auto median = values[values.size() / 2];
            const auto p95 = values[(values.size() * 95 + 99) / 100 - 1];
            const auto toMilliseconds = [](uint64_t nanoseconds) { return static_cast<double>(nanoseconds) / 1'000'000.0; };
            Logging::info("  {:<18} median={:>8.3f} ms  p95={:>8.3f} ms", name, toMilliseconds(median), toMilliseconds(p95));
        };

        const auto hashFrame = [&] {
            const auto& screenshot = drawingEngine.getScreenshotRT();
            uint64_t hash = 14695981039346656037ULL;
            const auto screenshotStride = screenshot.width + screenshot.pitch;
            for (int32_t y = 0; y < screenshot.height; ++y)
            {
                for (int32_t x = 0; x < screenshot.width; ++x)
                {
                    hash ^= screenshot.bits[static_cast<size_t>(y) * screenshotStride + x];
                    hash *= 1099511628211ULL;
                }
            }
            return hash;
        };
        const auto frameHash = hashFrame();
        Gfx::invalidateScreen();
        drawingEngine.render();
        const auto fullRenderHash = hashFrame();
        tweener.restore();
        if (frameHash != fullRenderHash)
        {
            Logging::error("Incremental render hash 0x{:016X} differs from full render hash 0x{:016X}", frameHash, fullRenderHash);
            return false;
        }

        Logging::info("--------------------------------");
        Logging::info("- Render benchmark");
        Logging::info("--------------------------------");
        Logging::info("  path:               {}", savePath.u8string());
        Logging::info("  framebuffer:        {}x{} at {:.2f}x", width, height, scaleFactor);
        Logging::info("  presentation:       {}x{}", presentationSize.width, presentationSize.height);
        Logging::info("  warmup / measured:  {} / {} frames", warmupFrames, frames);
        Logging::info("  scenario ticks:     {}", getGameState().scenarioTicks);
        Logging::info("  framebuffer hash:   0x{:016X}", frameHash);
        logMetric("frame total", std::move(frameTotals));
        logMetric("render total", std::move(renderTotals));
        logMetric("present total", std::move(presentTotals));
        return true;
    }

}
