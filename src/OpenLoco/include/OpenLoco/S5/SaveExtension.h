// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/RoutingManager.h>
#include <OpenLoco/Vehicles/SharedOrderManager.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <OpenLoco/Vehicles/VehicleReplacement.h>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::S5::SaveExtension
{
    constexpr size_t kMaxDataSize = CargoDist::kMaxSaveDataSize + Vehicles::RailTraffic::kMaxSaveDataSize + Vehicles::RoutingManager::kMaxSaveDataSize + 64 * 1024;

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
    };

    std::vector<std::byte> encode(const State& state);
    std::vector<std::byte> encode(StateView state);
    State decode(std::span<const std::byte> data);
}
