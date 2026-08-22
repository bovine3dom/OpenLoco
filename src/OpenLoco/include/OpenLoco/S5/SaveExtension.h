// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/Core/BitSet.hpp>
#include <OpenLoco/Engine/Limits.h>
#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/GameRules.h>
#include <OpenLoco/Objects/Object.h>
#include <OpenLoco/S5/Limits.h>
#include <OpenLoco/Types.hpp>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/RoutingManager.h>
#include <OpenLoco/Vehicles/SharedOrderManager.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <OpenLoco/Vehicles/VehicleReplacement.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::S5::SaveExtension
{
    constexpr size_t kMaxDataSize = CargoDist::kMaxSaveDataSize + Vehicles::RailTraffic::kMaxSaveDataSize + Vehicles::RoutingManager::kMaxSaveDataSize + 64 * 1024;
    constexpr size_t kExtendedVehicleObjectStart = OpenLoco::S5::Limits::kMaxVehicleObjects;
    constexpr size_t kExtendedVehicleObjectCount = OpenLoco::Limits::kMaxVehicleObjects - kExtendedVehicleObjectStart;

    struct VehicleObjectSlot
    {
        uint16_t slot{};
        ObjectHeader header{};

        bool operator==(const VehicleObjectSlot& rhs) const;
    };

    struct VehicleObjectState
    {
        std::vector<VehicleObjectSlot> objects;
        std::array<BitSet<kExtendedVehicleObjectCount>, OpenLoco::S5::Limits::kMaxCompanies> companyUnlocks;

        bool operator==(const VehicleObjectState& rhs) const;
    };

    struct StationTileOverflow
    {
        StationId station = StationId::null;
        uint16_t stationTileSize = 0;
        std::vector<World::Pos3> stationTiles;
    };

    struct State
    {
        std::optional<CargoDist::State> cargoDistState;
        std::optional<Vehicles::SharedOrderManager::State> sharedOrderState;
        std::optional<Vehicles::RoutingManager::State> pathReservationState;
        bool discardPathReservationsOnLoad{};
        std::optional<Vehicles::VehicleAutoRenewal::State> vehicleAutoRenewalState;
        std::optional<Vehicles::VehicleReplacement::State> vehicleReplacementState;
        std::optional<Vehicles::RailTraffic::State> railTrafficState;
        std::optional<std::vector<StationTileOverflow>> stationTileOverflowState;
        std::optional<GameRules::State> gameRulesState;
        std::optional<VehicleObjectState> vehicleObjectState;
    };

    struct StateView
    {
        const CargoDist::State* cargoDistState{};
        const Vehicles::SharedOrderManager::State* sharedOrderState{};
        const Vehicles::RoutingManager::State* pathReservationState{};
        const Vehicles::VehicleAutoRenewal::State* vehicleAutoRenewalState{};
        const Vehicles::VehicleReplacement::State* vehicleReplacementState{};
        bool discardPathReservationsOnLoad{};
        const Vehicles::RailTraffic::State* railTrafficState{};
        const std::vector<StationTileOverflow>* stationTileOverflowState{};
        const GameRules::State* gameRulesState{};
        const VehicleObjectState* vehicleObjectState{};
    };

    std::vector<std::byte> encode(const State& state);
    std::vector<std::byte> encode(StateView state);
    State decode(std::span<const std::byte> data);
}
