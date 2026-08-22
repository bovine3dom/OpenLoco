#include "GameCommands/General/SetVehiclesNeverExpire.h"

#include "GameRules.h"
#include "SceneManager.h"
#include "Ui/WindowManager.h"
#include "Ui/WindowType.h"

namespace OpenLoco::GameCommands
{
    static uint32_t setVehiclesNeverExpire(const SetVehiclesNeverExpireArgs& args, const uint8_t flags)
    {
        if (args.enabled > 1)
        {
            return kFailure;
        }

        const auto enabled = args.enabled != 0;
        if ((flags & Flags::apply) != 0 && GameRules::vehiclesNeverExpire() != enabled)
        {
            GameRules::setVehiclesNeverExpire(enabled);
            Ui::WindowManager::invalidate(Ui::WindowType::scenarioOptions);
            Ui::WindowManager::invalidate(Ui::WindowType::cheats);
            if (SceneManager::isSceneInitialised() && SceneManager::isPlayMode())
            {
                Ui::Windows::Construction::updateAvailableRoadAndRailOptions();
                Ui::Windows::Construction::updateAvailableAirportAndDockOptions();
            }
            Ui::WindowManager::close(Ui::WindowType::buildVehicle);
            Ui::WindowManager::invalidate(Ui::WindowType::construction);
        }
        return 0;
    }

    void setVehiclesNeverExpire(registers& regs, const uint8_t flags)
    {
        regs.ebx = setVehiclesNeverExpire(SetVehiclesNeverExpireArgs(regs), flags);
    }
}
