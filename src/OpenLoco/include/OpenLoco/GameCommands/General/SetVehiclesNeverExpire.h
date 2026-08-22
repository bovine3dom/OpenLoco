#pragma once

#include "GameCommands/GameCommands.h"
#include <cstdint>

namespace OpenLoco::GameCommands
{
    struct SetVehiclesNeverExpireArgs
    {
        static constexpr auto command = GameCommand::setVehiclesNeverExpire;

        SetVehiclesNeverExpireArgs() = default;
        explicit SetVehiclesNeverExpireArgs(const registers& regs)
            : enabled(static_cast<uint8_t>(regs.al))
        {
        }

        explicit SetVehiclesNeverExpireArgs(const uint8_t enabled)
            : enabled(enabled)
        {
        }

        uint8_t enabled{};

        explicit operator registers() const
        {
            registers regs;
            regs.al = static_cast<int8_t>(enabled);
            return regs;
        }
    };

    void setVehiclesNeverExpire(registers& regs, uint8_t flags);
}
