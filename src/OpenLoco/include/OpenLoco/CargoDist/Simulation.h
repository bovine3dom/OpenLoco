// SPDX-License-Identifier: MIT
#pragma once

#include "CargoDist.h"

namespace OpenLoco
{
    struct StationCargoStats;

    namespace Vehicles
    {
        struct VehicleCargo;
        struct VehicleHead;
    }
}

namespace OpenLoco::CargoDist
{
    struct UnloadResult
    {
        PacketList delivered;
        uint16_t transferred{};

        uint32_t quantity() const { return delivered.quantity() + transferred; }
    };

    void synchroniseStationCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo);
    void synchroniseVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo);

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity);
    void updateStationCargoDaily(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantityBeforeUpdate);
    void updateVehicleCargoDaily(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo);

    uint32_t getLoadableQuantity(StationId station, uint8_t cargo, StationId nextStop);
    uint16_t loadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, StationId nextStop);
    UnloadResult unloadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, std::span<const StationId> remainingStops, bool forceUnload);

    StationId getNextStop(const Vehicles::VehicleHead& head);
    void enableAll();
    void updateDaily();

    void removeStation(StationId station);
    void eraseVehicleCargoForComponent(EntityId component);
    void moveVehicleCargo(VehicleCargoKey source, VehicleCargoKey destination);
}
