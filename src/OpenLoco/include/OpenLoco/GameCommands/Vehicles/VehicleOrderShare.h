#pragma once

#include "GameCommands/GameCommands.h"

namespace OpenLoco::GameCommands
{
    struct VehicleOrderShareArgs
    {
        enum class Mode : uint8_t
        {
            joinSource,
            leave,
            joinAllMatching,
        };

        static constexpr auto command = GameCommand::vehicleOrderShare;

        VehicleOrderShareArgs() = default;
        explicit VehicleOrderShareArgs(const registers& regs)
            : target(EntityId(regs.di))
            , source(EntityId(regs.dx))
            , mode(static_cast<Mode>(regs.al))
        {
        }

        EntityId target = EntityId::null;
        EntityId source = EntityId::null;
        Mode mode = Mode::joinSource;

        explicit operator registers() const
        {
            registers regs;
            regs.di = enumValue(target);
            regs.dx = enumValue(source);
            regs.al = enumValue(mode);
            return regs;
        }
    };

    void vehicleOrderShare(registers& regs, const uint8_t flags);
}
