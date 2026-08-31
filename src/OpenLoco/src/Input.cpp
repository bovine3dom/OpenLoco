#include "Input.h"
#include "Audio/Audio.h"
#include "Config.h"
#include "Localisation/StringIds.h"
#include "Logging.h"
#include "Ui.h"
#include "Ui/ScrollView.h"
#include "Ui/Window.h"
#include <SDL3/SDL.h>
#include <map>

namespace OpenLoco::Input
{
    static Flags _flags;
    static State _state;
    static Ui::Point _cursorDragStart;
    static Ui::Point _cursorDragStartOutput;
    static uint32_t _cursorDragState;
    static bool _exitRequested = false;
    static bool _fatalError = false;

    static bool tryResizeWindow()
    {
        if (Ui::triggerResize())
        {
            return true;
        }

        Diagnostics::Logging::error("Unable to recreate rendering resources");
        _fatalError = true;
        _exitRequested = true;
        return false;
    }

    void init()
    {
        _flags = Flags::none;
        _state = State::reset;
    }

    bool hasFatalError()
    {
        return _fatalError;
    }

    bool hasFlag(Flags value)
    {
        return (_flags & value) != Flags::none;
    }

    void setFlag(Flags value)
    {
        _flags |= value;
    }

    void resetFlag(Flags value)
    {
        _flags &= ~value;
    }

    State state()
    {
        return _state;
    }

    void state(State state)
    {
        _state = state;
    }

    // 0x00407218
    void startCursorDrag()
    {
        if (_cursorDragState == 0)
        {
            _cursorDragState = 1;
            auto cursor = Ui::getCursorPos();
            _cursorDragStart = cursor;
            _cursorDragStartOutput = Ui::getCursorPosOutput();
            Ui::hideCursor();
        }
    }

    // 0x00407231
    void stopCursorDrag()
    {
        if (_cursorDragState != 0)
        {
            _cursorDragState = 0;
            Ui::setCursorPos(_cursorDragStart.x, _cursorDragStart.y);
            Ui::showCursor();
        }
    }

    Ui::Point getNextDragOffset()
    {
        auto current = Ui::getCursorPos();

        auto delta = Ui::windowToUi(current) - Ui::windowToUi(_cursorDragStart);

        Ui::setCursorPos(_cursorDragStart.x, _cursorDragStart.y);

        return { static_cast<int16_t>(delta.x), static_cast<int16_t>(delta.y) };
    }

    Ui::Point getNextViewportDragOffset(const Ui::Viewport& viewport)
    {
        const auto current = Ui::getCursorPos();
        const auto usesOutputCoordinates = viewport.rasterWidth != viewport.width || viewport.rasterHeight != viewport.height;
        Ui::Point delta;
        if (usesOutputCoordinates)
        {
            delta = Ui::getCursorPosOutput() - _cursorDragStartOutput;
        }
        else
        {
            delta = viewport.uiToRaster(Ui::windowToUi(current) - Ui::windowToUi(_cursorDragStart));
        }

        Ui::setCursorPos(_cursorDragStart.x, _cursorDragStart.y);
        if (usesOutputCoordinates)
        {
            _cursorDragStartOutput = Ui::windowToOutput(static_cast<float>(_cursorDragStart.x), static_cast<float>(_cursorDragStart.y));
        }
        return delta;
    }

    // 0x004072EC
    bool processMessagesMini()
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
                case SDL_EVENT_QUIT:
                    return false;
                case SDL_EVENT_WINDOW_MOVED:
                    Ui::windowPositionChanged(e.window.data1, e.window.data2);
                    break;
                case SDL_EVENT_WINDOW_RESIZED:
                case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
                    if (!tryResizeWindow())
                    {
                        return false;
                    }
                    break;
                case SDL_EVENT_RENDER_TARGETS_RESET:
                case SDL_EVENT_RENDER_DEVICE_RESET:
                    if (!tryResizeWindow())
                    {
                        return false;
                    }
                    break;
                case SDL_EVENT_RENDER_DEVICE_LOST:
                    Diagnostics::Logging::error("The rendering device was lost and cannot be recovered");
                    _fatalError = true;
                    _exitRequested = true;
                    return false;
            }
        }
        return false;
    }

    // 0x0040726D
    bool processMessages()
    {
        // The game has more than one loop for processing messages, if the secondary loop receives
        // SDL_QUIT then the message would be lost for the primary loop so we have to keep track of it.
        if (_exitRequested)
        {
            return false;
        }

        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
                case SDL_EVENT_QUIT:
                    _exitRequested = true;
                    return false;

                case SDL_EVENT_WINDOW_MOVED:
                    Ui::windowPositionChanged(e.window.data1, e.window.data2);
                    break;
                case SDL_EVENT_WINDOW_RESIZED:
                case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
                    if (!tryResizeWindow())
                    {
                        return false;
                    }
                    break;
                case SDL_EVENT_RENDER_TARGETS_RESET:
                case SDL_EVENT_RENDER_DEVICE_RESET:
                    if (!tryResizeWindow())
                    {
                        return false;
                    }
                    break;
                case SDL_EVENT_RENDER_DEVICE_LOST:
                    Diagnostics::Logging::error("The rendering device was lost and cannot be recovered");
                    _fatalError = true;
                    _exitRequested = true;
                    return false;

                case SDL_EVENT_MOUSE_MOTION:
                {
                    const auto position = Ui::windowToUi(e.motion.x, e.motion.y);
                    const auto previous = Ui::windowToUi(e.motion.x - e.motion.xrel, e.motion.y - e.motion.yrel);
                    moveMouse(position.x, position.y, position.x - previous.x, position.y - previous.y, Ui::windowToOutput(e.motion.x, e.motion.y));
                    break;
                }
                case SDL_EVENT_MOUSE_WHEEL:
                    mouseWheel(e.wheel.y);
                    break;
                case SDL_EVENT_MOUSE_BUTTON_DOWN:
                {
                    const auto position = Ui::windowToUi(e.button.x, e.button.y);
                    const auto outputPosition = Ui::windowToOutput(e.button.x, e.button.y);
                    setPendingMouseInputUpdate();
                    switch (e.button.button)
                    {
                        case SDL_BUTTON_LEFT:
                            enqueueMouseButton({ position, outputPosition, 1 });
                            break;
                        case SDL_BUTTON_RIGHT:
                            enqueueMouseButton({ position, outputPosition, 2 });
                            setRightMouseButtonDown(true);
                            break;
                    }
                    break;
                }
                case SDL_EVENT_MOUSE_BUTTON_UP:
                {
                    const auto position = Ui::windowToUi(e.button.x, e.button.y);
                    const auto outputPosition = Ui::windowToOutput(e.button.x, e.button.y);
                    setPendingMouseInputUpdate();
                    switch (e.button.button)
                    {
                        case SDL_BUTTON_LEFT:
                            enqueueMouseButton({ position, outputPosition, 3 });
                            break;
                        case SDL_BUTTON_RIGHT:
                            enqueueMouseButton({ position, outputPosition, 4 });
                            setRightMouseButtonDown(false);
                            break;
                    }
                    break;
                }
                case SDL_EVENT_KEY_DOWN:
                {
                    auto keycode = e.key.key;

#if !(defined(__APPLE__) && defined(__MACH__))
                    // Toggle fullscreen when ALT+RETURN is pressed
                    if (keycode == SDLK_RETURN)
                    {
                        if ((e.key.mod & SDL_KMOD_LALT) || (e.key.mod & SDL_KMOD_RALT))
                        {
                            Ui::toggleFullscreenDesktop();
                        }
                    }
#endif

                    handleKeyInput(keycode);
                    break;
                }
                case SDL_EVENT_KEY_UP:
                    break;
                case SDL_EVENT_TEXT_INPUT:
                    enqueueText(e.text.text);
                    break;
            }
        }

        readKeyboardState();
        return true;
    }
}
