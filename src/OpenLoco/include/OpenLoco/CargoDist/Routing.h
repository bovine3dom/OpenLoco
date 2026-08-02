// SPDX-License-Identifier: MIT
#pragma once

#include <OpenLoco/Types.hpp>
#include <cstdint>
#include <vector>

namespace OpenLoco::CargoDist
{
    struct RoutingNode
    {
        StationId station;
        int16_t x;
        int16_t y;
        uint32_t supply;
        bool accepts;
    };

    struct RoutingEdge
    {
        StationId from;
        StationId to;
        uint32_t capacity;
        uint32_t travelTime;
    };

    struct RoutingDemand
    {
        StationId source;
        StationId origin;
        uint32_t amount;
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
    };

    std::vector<FlowShare> calculateAsymmetricFlows(const RoutingGraph& graph, const RoutingSettings& settings = {});
}
