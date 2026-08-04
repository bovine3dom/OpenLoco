#pragma once

#include "GameCommands/GameCommands.h"
#include <cstdint>

namespace OpenLoco::GameCommands
{
    struct SetVehicleAutoRenewalArgs
    {
        static constexpr auto command = GameCommand::setVehicleAutoRenewal;

        SetVehicleAutoRenewalArgs() = default;
        explicit SetVehicleAutoRenewalArgs(const registers& regs)
            : enabled(static_cast<uint8_t>(regs.al))
            , reliabilityThreshold(static_cast<uint8_t>(regs.ah))
        {
        }

        SetVehicleAutoRenewalArgs(const uint8_t enabled, const uint8_t reliabilityThreshold)
            : enabled(enabled)
            , reliabilityThreshold(reliabilityThreshold)
        {
        }

        uint8_t enabled{};
        uint8_t reliabilityThreshold{};

        explicit operator registers() const
        {
            registers regs;
            regs.al = static_cast<int8_t>(enabled);
            regs.ah = static_cast<int8_t>(reliabilityThreshold);
            return regs;
        }
    };

    void setVehicleAutoRenewal(registers& regs, uint8_t flags);
}
