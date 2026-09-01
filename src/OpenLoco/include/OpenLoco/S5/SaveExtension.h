// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/CargoDist/FlowAnalytics.h>
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
#include <OpenLoco/Vehicles/TimetableManager.h>
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
    constexpr size_t kMaxTimetableDataSize = sizeof(uint16_t) + sizeof(uint64_t) + sizeof(uint32_t) * 3
        + OpenLoco::S5::Limits::kMaxVehicles * (sizeof(uint32_t) * 2 + sizeof(uint16_t))
        + OpenLoco::S5::Limits::kMaxVehicles * OpenLoco::S5::Limits::kMaxOrdersPerVehicle
            * (sizeof(uint32_t) + sizeof(uint8_t) * 3 + sizeof(uint16_t) + sizeof(uint32_t) * 2
               + sizeof(uint8_t) + sizeof(uint32_t) * 3 + sizeof(uint16_t) + sizeof(uint64_t) + Vehicles::TimetableManager::kMaxSlots * sizeof(uint32_t))
        + sizeof(uint32_t) + OpenLoco::S5::Limits::kMaxEntities * (sizeof(uint16_t) + sizeof(uint32_t))
        + sizeof(uint32_t) + OpenLoco::S5::Limits::kMaxEntities * (sizeof(uint16_t) + sizeof(uint32_t) * 3 + sizeof(uint64_t) * 4 + sizeof(uint8_t));
    constexpr size_t kMaxDataSize = CargoDist::kMaxSaveDataSize + CargoDist::FlowAnalytics::kMaxSaveDataSize + Vehicles::RailTraffic::kMaxSaveDataSize + Vehicles::RoutingManager::kMaxSaveDataSize + kMaxTimetableDataSize + 64 * 1024;
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
        std::optional<Vehicles::TimetableManager::State> timetableState;
        std::optional<CargoDist::FlowAnalytics::State> cargoFlowHistoryState;
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
        const Vehicles::TimetableManager::State* timetableState{};
        const CargoDist::FlowAnalytics::State* cargoFlowHistoryState{};
    };

    std::vector<std::byte> encode(const State& state);
    std::vector<std::byte> encode(StateView state);
    State decode(std::span<const std::byte> data);
}
