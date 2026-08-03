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

    class Group
    {
    public:
        Group();
        ~Group();

        Group(const Group&) = delete;
        Group& operator=(const Group&) = delete;
    };

    void clear();
    bool isAvailable();
    Result apply();
}
