#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <OpenLoco/Vehicles/SignalFuzzer.h>
#include <optional>
#include <vector>

namespace OpenLoco::Vehicles::SignalFuzzer::Layouts
{
    struct PreparedLayout
    {
        World::Pos2 centre;
        int32_t radius;
        std::vector<EntityId> vehicles;
    };

    std::optional<PreparedLayout> generate(Layout layout);
}
