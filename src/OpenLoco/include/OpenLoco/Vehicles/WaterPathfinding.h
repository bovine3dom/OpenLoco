#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <cstdint>
#include <span>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::WaterPathfinding
{
    enum class RouteStatus : uint8_t
    {
        found,
        arrived,
        temporarilyBlocked,
        unreachable,
    };

    struct SearchResult
    {
        RouteStatus status;
        World::TilePos2 nextTile;
        uint16_t goal;
        uint32_t remainingDistance;
    };

    struct PathingResult
    {
        RouteStatus status;
        World::Pos2 headTarget;
        StationId stationId;
        World::Pos3 stationPos;
    };

    bool isNavigable(World::TilePos2 tilePos, World::MicroZ waterLevel);
    SearchResult findNextTile(
        World::TilePos2 start,
        World::MicroZ waterLevel,
        std::span<const World::TilePos2> goals,
        std::span<const World::TilePos2> blockedTiles,
        uint8_t currentDirection);
    PathingResult getNextTarget(const VehicleHead& head, bool isLeavingDock);
}
