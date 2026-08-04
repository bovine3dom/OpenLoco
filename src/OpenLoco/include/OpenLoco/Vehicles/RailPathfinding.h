#pragma once

#include "Types.hpp"
#include "Vehicles/RailTraffic.h"
#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <limits>

namespace OpenLoco::Vehicles::RailPathfinding
{
    // In order of preference when finding a route.
    enum class SignalState : uint32_t
    {
        noSignals = 1,
        signalClear = 2,
        signalBlockedOneWay = 3,
        signalBlockedTwoWay = 4,
        signalNoRoute = 6,
        null = std::numeric_limits<uint32_t>::max(),
    };

    struct RouteResult
    {
        RailTraffic::TravelTime bestDistToTarget;
        RailTraffic::TravelTime bestTrackWeighting;
        SignalState signalState;
        uint8_t numTargetsReached{}; // Current order, then optional lookahead order.
        bool searchExhausted{};
    };

    struct Target
    {
        StationId stationId = StationId::null;
        World::Pos3 pos{};
        uint16_t tad{};
        World::Pos3 reversePos{};
        uint16_t reverseTad{};
    };

    RouteResult findRoute(
        World::Pos3 pos,
        uint16_t tad,
        CompanyId company,
        uint8_t trackType,
        uint8_t requiredMods,
        uint8_t queryMods,
        const Target& target,
        const Target* nextTarget = nullptr);
    RouteResult findRoute(
        World::Pos3 pos,
        uint16_t tad,
        CompanyId company,
        uint8_t trackType,
        uint8_t requiredMods,
        uint8_t queryMods,
        const RailTraffic::SpeedProfile& speedProfile,
        const Target& target,
        const Target* nextTarget = nullptr);
    bool isBetterRoute(const RouteResult& base, const RouteResult& candidate);
}
