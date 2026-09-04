#pragma once

#include "GameCommands/GameCommands.h"

namespace OpenLoco::GameCommands
{
    struct VehicleOrderSetCrushLoadingArgs
    {
        static constexpr auto command = GameCommand::vehicleOrderSetCrushLoading;

        VehicleOrderSetCrushLoadingArgs() = default;
        explicit VehicleOrderSetCrushLoadingArgs(const registers& regs)
            : head(EntityId(regs.di))
            , orderOffset(regs.edx)
            , enabled(static_cast<uint8_t>(regs.al))
        {
        }

        EntityId head = EntityId::null;
        uint32_t orderOffset{};
        uint8_t enabled{};

        explicit operator registers() const
        {
            registers regs;
            regs.di = enumValue(head);
            regs.edx = orderOffset;
            regs.al = static_cast<int8_t>(enabled);
            return regs;
        }
    };

    void vehicleOrderSetCrushLoading(registers& regs, uint8_t flags);
}
