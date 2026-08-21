#include "GameCommands/General/SetCargoDistMode.h"
#include "Objects/CargoObject.h"
#include "Objects/ObjectManager.h"
#include "Ui/WindowManager.h"
#include "Ui/WindowType.h"
#include <OpenLoco/CargoDist/Simulation.h>

namespace OpenLoco::GameCommands
{
    static uint32_t setCargoDistMode(const SetCargoDistModeArgs& args, uint8_t flags)
    {
        if ((args.cargo != kAllCargo
             && (args.cargo >= CargoDist::getStateConst().settings.modes.size() || ObjectManager::get<CargoObject>(args.cargo) == nullptr))
            || (args.mode != CargoDist::DistributionMode::manual && args.mode != CargoDist::DistributionMode::asymmetric))
        {
            return kFailure;
        }

        if (args.mode == CargoDist::DistributionMode::manual)
        {
            if (args.cargo == kAllCargo)
            {
                for (uint8_t cargo = 0; cargo < CargoDist::getStateConst().settings.modes.size(); ++cargo)
                {
                    if (CargoDist::getMode(cargo) != CargoDist::DistributionMode::manual && CargoDist::hasOutstandingTransferCredits(cargo))
                    {
                        return kFailure;
                    }
                }
            }
            else if (CargoDist::getMode(args.cargo) != CargoDist::DistributionMode::manual && CargoDist::hasOutstandingTransferCredits(args.cargo))
            {
                return kFailure;
            }
        }

        if ((flags & Flags::apply) != 0)
        {
            bool changed = false;
            if (args.cargo == kAllCargo)
            {
                for (uint8_t cargo = 0; cargo < CargoDist::getStateConst().settings.modes.size(); ++cargo)
                {
                    const auto mode = ObjectManager::get<CargoObject>(cargo) == nullptr ? CargoDist::DistributionMode::manual : args.mode;
                    changed |= CargoDist::getMode(cargo) != mode;
                    CargoDist::setMode(cargo, mode);
                }
            }
            else
            {
                changed = CargoDist::getMode(args.cargo) != args.mode;
                CargoDist::setMode(args.cargo, args.mode);
            }
            if (changed)
            {
                CargoDist::recalculateNow();
                Ui::WindowManager::invalidate(Ui::WindowType::options);
            }
        }
        return 0;
    }

    void setCargoDistMode(registers& regs, uint8_t flags)
    {
        regs.ebx = setCargoDistMode(SetCargoDistModeArgs(regs), flags);
    }
}
