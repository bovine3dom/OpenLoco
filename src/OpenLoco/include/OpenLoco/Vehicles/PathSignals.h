#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <OpenLoco/Vehicles/Routing.h>
#include <cstdint>
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
        World::Pos3 routePos;
        World::Pos3 pos;
        uint16_t routing;
        uint32_t conflictMask; // Four occupied-quarter bits per track connection flag.
        bool occupied;
    };

    using ReservationCallback = void (*)(EntityId vehicle, RoutingHandle handle, World::Pos3 pos, uint16_t routing);

    bool tryReservePath(VehicleHead& head, const World::Pos3& firstPos, uint16_t firstRouting);
    bool isPathReserved(const World::Pos3& pos, uint16_t routing);
    std::vector<ClaimedResource> getClaimedResources();
    void setReservationCallback(ReservationCallback callback);
}
