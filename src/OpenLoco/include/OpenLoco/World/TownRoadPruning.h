#pragma once

#include "GameCommands/Road/RemoveRoad.h"
#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenLoco::World
{
    struct RoadElement;
}

namespace OpenLoco::TownRoadPruning
{
    struct RoadPieceId
    {
        World::Pos3 pos;
        uint8_t roadId;
        uint8_t rotation;
        uint8_t objectId;

        constexpr bool operator==(const RoadPieceId&) const = default;
    };

    struct Plan
    {
        std::vector<RoadPieceId> roads;
        std::vector<World::Pos3> buildings;
    };

    std::optional<RoadPieceId> getRoadPieceId(const World::Pos2& pos, const World::RoadElement& road);
    bool matchesWaypoint(const RoadPieceId& road, const World::Pos3& waypointPos, uint16_t waypointTad);
    bool isPotentialBlocker(const World::RoadElement& road);
    std::optional<Plan> plan(TownId townId, const World::Pos3& buildingPos, int16_t clearHeight, bool isLarge);
    bool contains(const Plan& plan, const World::Pos2& pos, const World::RoadElement& road);
    GameCommands::RoadRemovalArgs makeRemovalArgs(const RoadPieceId& road);
}
