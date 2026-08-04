// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/RoutingManager.h>
#include <OpenLoco/Vehicles/SharedOrderManager.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::S5::SaveExtension
{
    constexpr size_t kMaxDataSize = CargoDist::kMaxSaveDataSize + Vehicles::RailTraffic::kMaxSaveDataSize + 64 * 1024;

    struct State
    {
        std::optional<CargoDist::State> cargoDistState;
        std::optional<Vehicles::SharedOrderManager::State> sharedOrderState;
        std::optional<Vehicles::RoutingManager::State> pathReservationState;
        bool discardPathReservationsOnLoad{};
        std::optional<Vehicles::VehicleAutoRenewal::State> vehicleAutoRenewalState;
        std::optional<Vehicles::RailTraffic::State> railTrafficState;
    };

    struct StateView
    {
        const CargoDist::State* cargoDistState{};
        const Vehicles::SharedOrderManager::State* sharedOrderState{};
        const Vehicles::RoutingManager::State* pathReservationState{};
        const Vehicles::VehicleAutoRenewal::State* vehicleAutoRenewalState{};
        bool discardPathReservationsOnLoad{};
        const Vehicles::RailTraffic::State* railTrafficState{};
    };

    std::vector<std::byte> encode(const State& state);
    std::vector<std::byte> encode(StateView state);
    State decode(std::span<const std::byte> data);
}
