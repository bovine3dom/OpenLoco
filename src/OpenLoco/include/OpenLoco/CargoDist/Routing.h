// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/Types.hpp>
#include <compare>
#include <cstdint>
#include <limits>
#include <vector>

namespace OpenLoco::CargoDist
{
    enum class ServiceId : uint16_t
    {
        null = std::numeric_limits<uint16_t>::max(),
    };

    inline constexpr uint16_t kNoServiceOccurrence = std::numeric_limits<uint16_t>::max();

    struct ServicePoint
    {
        ServiceId service = ServiceId::null;
        uint16_t occurrence = kNoServiceOccurrence;

        auto operator<=>(const ServicePoint&) const = default;

        [[nodiscard]] constexpr bool empty() const
        {
            return service == ServiceId::null && occurrence == kNoServiceOccurrence;
        }
    };

    struct RoutingNode
    {
        StationId station;
        int16_t x;
        int16_t y;
        uint32_t supply;
        bool accepts;
        uint32_t attraction = 1;
    };

    struct RoutingEdge
    {
        StationId from;
        StationId to;
        uint32_t capacity;
        uint32_t travelTime;
        ServicePoint departure{};
        ServicePoint arrival{};
        uint32_t waitTime{};
        uint32_t headway{};
    };

    struct RoutingDemand
    {
        StationId source;
        StationId origin;
        uint32_t amount;
        ServicePoint incoming{};
        StationId destination = StationId::null;
    };

    struct RoutingGraph
    {
        std::vector<RoutingNode> nodes;
        std::vector<RoutingEdge> edges;
        bool timeSensitive;
        std::vector<RoutingDemand> demands;
    };

    struct RoutingSettings
    {
        uint8_t distanceEffect = 100;
        uint8_t saturation = 80;
        uint8_t accuracy = 16;
    };

    struct FlowShare
    {
        StationId station;
        StationId origin;
        StationId via;
        uint32_t amount;
        ServicePoint incoming{};
        ServicePoint departure{};
        ServicePoint arrival{};
        StationId destination = StationId::null;
    };

    inline constexpr uint64_t kUnreachableJourneyCost = std::numeric_limits<uint64_t>::max();

    struct StationJourneyCost
    {
        StationId destination = StationId::null;
        uint64_t cost = kUnreachableJourneyCost;
    };

    std::vector<FlowShare> calculateAsymmetricFlows(const RoutingGraph& graph, const RoutingSettings& settings = {});
    std::vector<StationJourneyCost> calculateJourneyCosts(const RoutingGraph& graph, StationId source, ServicePoint departure = {});
    uint64_t calculateJourneyCost(const RoutingGraph& graph, StationId source, StationId destination, ServicePoint departure = {});
}
