#pragma once

#include "GameCommands/GameCommands.h"
#include <OpenLoco/CargoDist/CargoDist.h>

namespace OpenLoco::GameCommands
{
    constexpr uint8_t kAllCargo = 0xFF;

    struct SetCargoDistModeArgs
    {
        static constexpr auto command = GameCommand::setCargoDistMode;

        SetCargoDistModeArgs() = default;
        explicit SetCargoDistModeArgs(const registers& regs)
            : cargo(static_cast<uint8_t>(regs.al))
            , mode(static_cast<CargoDist::DistributionMode>(regs.ah))
        {
        }

        SetCargoDistModeArgs(uint8_t cargo, CargoDist::DistributionMode mode)
            : cargo(cargo)
            , mode(mode)
        {
        }

        uint8_t cargo{};
        CargoDist::DistributionMode mode{};

        explicit operator registers() const
        {
            registers regs;
            regs.al = static_cast<int8_t>(cargo);
            regs.ah = static_cast<int8_t>(mode);
            return regs;
        }
    };

    void setCargoDistMode(registers& regs, uint8_t flags);
}
