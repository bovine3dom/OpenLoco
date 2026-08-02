#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::PathSignals
{
    bool tryReservePath(VehicleHead& head, const World::Pos3& firstPos, uint16_t firstRouting);
    bool isPathReserved(const World::Pos3& pos, uint16_t routing);
}
