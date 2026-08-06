// SPDX-License-Identifier: MIT
#pragma once

#include "CargoDist.h"

namespace OpenLoco
{
    struct GameState;
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
    void setStationAttraction(StationId station, uint8_t cargo, uint32_t attraction);

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity);
    void updateStationCargoDaily(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantityBeforeUpdate);
    void updateVehicleCargoDaily(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo);

    uint32_t getLoadableQuantity(StationId station, uint8_t cargo, const VehicleServiceLeg& serviceLeg);
    std::map<ServiceEdgeKey, CommittedServiceDemand> getCommittedServiceDemands(uint8_t cargo);
    uint16_t loadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, const VehicleServiceLeg& serviceLeg);
    UnloadResult unloadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, std::span<const StationId> remainingStops, bool forceUnload, std::optional<VehicleServiceLeg> onwardLeg);

    std::optional<VehicleServiceLeg> getCurrentServiceLeg(const Vehicles::VehicleHead& head);
    StationId getNextStop(const Vehicles::VehicleHead& head);
    void recalculateNow();
    void validateState(const State& state, const GameState& gameState);
    void restoreState(State state);
    void updateDaily();

    void removeStation(StationId station);
    void removeVehicleService(EntityId vehicle);
    void eraseVehicleCargoForComponent(EntityId component);
    void moveVehicleCargo(VehicleCargoKey source, VehicleCargoKey destination);
}
