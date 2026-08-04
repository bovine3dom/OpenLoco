#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <OpenLoco/Vehicles/Routing.h>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::PathSignals
{
    struct ClaimedResource
    {
        EntityId vehicle;
        RoutingHandle handle;
        World::Pos3 pos;
        uint32_t conflictMask; // Four occupied-quarter bits per track connection flag.
        bool occupied;
    };

    bool tryReservePath(VehicleHead& head, const World::Pos3& firstPos, uint16_t firstRouting);
    std::optional<uint16_t> tryReservePath(VehicleHead& head, const World::Pos3& firstPos, uint16_t preferredRouting, std::span<const uint16_t> firstRoutings);
    bool isPathReserved(const World::Pos3& pos, uint16_t routing);
    bool hasPathReservationConflict(EntityId vehicle, const World::Pos3& pos, uint16_t routing);
    std::vector<ClaimedResource> getClaimedResources();
}
