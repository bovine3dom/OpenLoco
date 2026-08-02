#include "Gui.h"
#include "Config.h"
#include "Graphics/Colour.h"
#include "Graphics/Gfx.h"
#include "Graphics/SoftwareDrawingEngine.h"
#include "Map/Tile.h"
#include "SceneManager.h"
#include "Tutorial.h"
#include "Ui.h"
#include "Ui/Widget.h"
#include "Ui/Window.h"
#include "Ui/WindowManager.h"
#include "ViewportManager.h"

using namespace OpenLoco::Ui;

namespace OpenLoco::Gui
{
    // 0x00438A6C
    void init()
    {
        Windows::Main::open();

        Windows::Terraform::setAdjustLandToolSize(1);
        Windows::Terraform::setAdjustWaterToolSize(1);
        Windows::Terraform::setClearAreaToolSize(2);

        if (SceneManager::isTitleMode())
        {
            Ui::Windows::TitleMenu::open();
            Ui::Windows::TitleExit::open();
            Ui::Windows::TitleLogo::open();
            Ui::Windows::TitleVersion::open();
            Ui::Windows::TitleOptions::open();
        }
        else if (SceneManager::isPlayMode())
        {
            Windows::ToolbarTop::Game::open();

            Windows::PlayerInfoPanel::open();
            Windows::TimePanel::open();

            if (OpenLoco::Tutorial::state() != Tutorial::State::none)
            {
                Windows::Tutorial::open();
            }
        }

        resize();
    }

    // 0x004392BD
    void resize()
    {
        const int32_t uiWidth = Ui::width();
        const int32_t uiHeight = Ui::height();

        auto window = WindowManager::getMainWindow();
        if (window)
        {
            window->width = uiWidth;
            window->height = uiHeight;
            if (!window->widgets.empty())
            {
                window->widgets[0].right = uiWidth;
                window->widgets[0].bottom = uiHeight;
            }
            if (window->viewports[0])
            {
                auto& viewport = *window->viewports[0];
                const auto centre = viewport.getCentre();
                auto& drawingEngine = Gfx::getDrawingEngine();
                const auto outputSize = drawingEngine.getOutputSize();
                const auto rasterSize = drawingEngine.shouldUseSeparateWorld() ? outputSize : Ui::Size{ uiWidth, uiHeight };
                viewport.setDimensions({ uiWidth, uiHeight }, rasterSize);
                viewport.viewX = centre.x - viewport.viewWidth / 2;
                viewport.viewY = centre.y - viewport.viewHeight / 2;
                window->viewportConfigurations[0].savedViewX = viewport.viewX;
                window->viewportConfigurations[0].savedViewY = viewport.viewY;
            }
        }

        window = WindowManager::find(WindowType::topToolbar);
        if (window)
        {
            window->width = std::max(uiWidth, 640);
        }

        window = WindowManager::find(WindowType::playerInfoToolbar);
        if (window)
        {
            window->y = uiHeight - window->height;
        }

        window = WindowManager::find(WindowType::timeToolbar);
        if (window)
        {
            window->y = uiHeight - window->height;
            window->x = std::max(uiWidth, 640) - window->width;
        }

        window = WindowManager::find(WindowType::editorToolbar);
        if (window)
        {
            window->y = uiHeight - window->height;
            window->width = std::max(uiWidth, 640);
        }

        window = WindowManager::find(WindowType::titleMenu);
        if (window)
        {
            window->x = uiWidth / 2 - 148;
            window->y = uiHeight - 117;
        }

        window = WindowManager::find(WindowType::titleExit);
        if (window)
        {
            window->x = uiWidth - 40;
            window->y = uiHeight - 28;
        }

        window = WindowManager::find(WindowType::openLocoVersion);
        if (window)
        {
            window->y = uiHeight - window->height;
        }

        window = WindowManager::find(WindowType::titleOptions);
        if (window)
        {
            window->x = uiWidth - window->width;
        }

        window = WindowManager::find(WindowType::tutorial);
        if (window)
        {
            if (Tutorial::state() == Tutorial::State::none)
            {
                WindowManager::close(window);
            }
        }

        window = WindowManager::find(WindowType::options);
        if (window != nullptr)
        {
            window->moveToCentre();
        }
    }
}
