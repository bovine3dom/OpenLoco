#pragma once

#include "Station.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace OpenLoco::TownGrowth
{
    struct RoadTraversalState
    {
        World::Pos3 pos;
        uint16_t tad;
        bool isBridge;
        uint16_t depth;
    };

    constexpr uint32_t kCentreWeight = 256;
    constexpr uint32_t kMinimumStationAccessibility = 16;
    constexpr uint32_t kMaximumStationWeight = 4096;

    constexpr bool isGrowthStationCandidate(const TownId targetTown, const TownId stationTown, const uint16_t stationTileSize, const StationFlags flags, const uint32_t accessibility)
    {
        return targetTown == stationTown
            && stationTileSize != 0
            && (flags & StationFlags::flag_5) == StationFlags::none
            && accessibility >= kMinimumStationAccessibility;
    }

    constexpr bool shouldUseStation(const uint32_t bestAccessibility, const uint16_t roll)
    {
        const auto transitWeight = std::min<uint64_t>(kCentreWeight, static_cast<uint64_t>(bestAccessibility) * 2);
        return static_cast<uint64_t>(roll) * (kCentreWeight + transitWeight) / 65536 >= kCentreWeight;
    }

    inline bool wasRoadStateVisited(const std::span<const RoadTraversalState> visited, const World::Pos3& pos, const uint16_t tad)
    {
        return std::any_of(visited.begin(), visited.end(), [&](const auto& state) {
            return state.pos == pos && state.tad == tad;
        });
    }

    inline std::optional<size_t> selectStation(const std::span<const uint32_t> accessibilities, const uint16_t roll)
    {
        uint64_t totalWeight = 0;
        for (const auto accessibility : accessibilities)
        {
            totalWeight += std::min(accessibility, kMaximumStationWeight);
        }
        if (totalWeight == 0)
        {
            return std::nullopt;
        }

        const auto choice = static_cast<uint64_t>(roll) * totalWeight / 65536;
        uint64_t cumulativeWeight = 0;
        for (size_t index = 0; index < accessibilities.size(); ++index)
        {
            cumulativeWeight += std::min(accessibilities[index], kMaximumStationWeight);
            if (choice < cumulativeWeight)
            {
                return index;
            }
        }
        return accessibilities.size() - 1;
    }
}
