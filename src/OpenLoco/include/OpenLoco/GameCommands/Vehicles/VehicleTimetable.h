#pragma once

#include "GameCommands/GameCommands.h"
#include <limits>

namespace OpenLoco::GameCommands
{
    struct VehicleTimetableArgs
    {
        enum class Action : uint8_t
        {
            setEnabled,
            setTravelMinutes,
            setDwellMinutes,
            setDispatchPeriod,
            setDispatchPhase,
            setDispatchMaxDelay,
            addDispatchSlot,
            removeDispatchSlot,
            clearDispatch,
            setClockRate,
            resetDispatch,
        };

        static constexpr auto command = GameCommand::vehicleTimetable;
        static constexpr uint32_t kClearValue = std::numeric_limits<uint32_t>::max();

        VehicleTimetableArgs() = default;
        explicit VehicleTimetableArgs(const registers& regs)
            : head(EntityId(regs.di))
            , action(static_cast<Action>(regs.bh))
            , orderIndex(regs.dx)
            , value(regs.eax)
        {
        }

        EntityId head = EntityId::null;
        Action action = Action::setEnabled;
        uint16_t orderIndex{};
        uint32_t value{};

        explicit operator registers() const
        {
            registers regs;
            regs.di = enumValue(head);
            regs.bh = enumValue(action);
            regs.dx = orderIndex;
            regs.eax = value;
            return regs;
        }
    };

    void vehicleTimetable(registers& regs, uint8_t flags);
}
