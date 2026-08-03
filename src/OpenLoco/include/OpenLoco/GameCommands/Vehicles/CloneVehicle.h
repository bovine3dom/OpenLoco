#pragma once

#include "GameCommands/GameCommands.h"

namespace OpenLoco::GameCommands
{
    struct VehicleCloneArgs
    {
        static constexpr auto command = GameCommand::vehicleClone;

        VehicleCloneArgs() = default;
        explicit VehicleCloneArgs(const registers& regs)
            : vehicleHeadId(static_cast<EntityId>(regs.ax))
            , shareOrders(regs.dh == 1)
        {
        }

        EntityId vehicleHeadId = EntityId::null;
        bool shareOrders = false;

        explicit operator registers() const
        {
            registers regs;
            regs.ax = enumValue(vehicleHeadId);
            regs.dh = shareOrders ? 1 : 0;
            return regs;
        }
    };

    void cloneVehicle(registers& regs, const uint8_t flags);
}
