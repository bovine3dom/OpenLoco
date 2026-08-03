#pragma once

#include "GameCommands/GameCommands.h"

namespace OpenLoco::GameCommands::Undo
{
    enum class Result
    {
        success,
        unavailable,
        stateChanged,
    };

    void prepare(GameCommand command, CompanyId company, const registers& regs, uint8_t flags);
    void commit(currency32_t cost, ExpenditureType expenditureType, const World::Pos3& position);
    void cancel();
    void clear();
    bool isAvailable();
    Result apply();
}
