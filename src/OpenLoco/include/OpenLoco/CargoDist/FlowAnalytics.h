// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Types.hpp>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace OpenLoco::CargoDist::FlowAnalytics
{
    constexpr uint16_t kMaximumHorizonDays = 365;
    constexpr size_t kMaxDailyRecords = 3000;
    constexpr size_t kMaxSaveDataSize = 48 * 1024 * 1024;

    struct ServiceKey
    {
        uint8_t cargo{};
        StationId from = StationId::null;
        StationId to = StationId::null;

        auto operator<=>(const ServiceKey&) const = default;
    };

    struct ServiceMetric
    {
        ServiceKey key;
        uint64_t throughput{};
        uint64_t observedThroughput{};
        uint64_t offeredCapacity{};
        uint64_t plannedDemand{};
        uint64_t capacityQ16{};

        auto operator<=>(const ServiceMetric&) const = default;
    };

    struct DailyRecord
    {
        uint32_t day{};
        std::vector<ServiceMetric> services;

        auto operator<=>(const DailyRecord&) const = default;
    };

    struct State
    {
        std::vector<DailyRecord> days;

        auto operator<=>(const State&) const = default;
    };

    struct ServiceSummary
    {
        StationId from = StationId::null;
        StationId to = StationId::null;
        uint64_t throughput{};
        uint64_t observedThroughput{};
        uint64_t offeredCapacity{};
        uint64_t plannedDemand{};
        uint64_t capacityQ16{};

        auto operator<=>(const ServiceSummary&) const = default;
    };

    enum class EndpointKind : uint8_t
    {
        town,
        industry,
        station,
    };

    struct EndpointKey
    {
        EndpointKind kind{};
        uint16_t id{};

        auto operator<=>(const EndpointKey&) const = default;
    };

    struct Endpoint
    {
        EndpointKey key;
        World::Pos2 position;
        StringId name{};
        uint64_t supply{};
        uint64_t attraction{};
        uint64_t localDemand{};
        TownId town = TownId::null;
        int16_t z{};

        auto operator<=>(const Endpoint&) const = default;
    };

    enum class GapReason : uint8_t
    {
        served,
        capacityShortfall,
        noRoute,
        noStation,
    };

    struct DestinationFlow
    {
        EndpointKey origin;
        EndpointKey destination;
        uint64_t demand{};
        uint64_t capacity{};
        GapReason gap{};

        auto operator<=>(const DestinationFlow&) const = default;
    };

    struct DestinationModel
    {
        std::vector<Endpoint> endpoints;
        std::vector<DestinationFlow> flows;
    };

    void reset();
    void recordDeparture(uint8_t cargo, StationId from, StationId to, uint32_t quantity, uint32_t capacity);
    void updateDaily();
    uint32_t getRevision();

    std::vector<ServiceSummary> getServiceSummaries(uint8_t cargo, uint16_t horizonDays);
    std::vector<ServiceSummary> summarise(const State& state, uint8_t cargo, uint16_t horizonDays, uint32_t currentDay);
    uint64_t roundCapacity(uint64_t capacityQ16);
    std::vector<DestinationFlow> allocateLatentDemand(std::vector<Endpoint>& endpoints, uint8_t distanceEffect);
    DestinationModel getDestinationModel(uint8_t cargo, uint16_t horizonDays);

    State captureState();
    bool validateState(const State& state);
    bool restoreState(const State& state);
    bool isDefault(const State& state);
}
