#include "Vehicles/RailPathfinding.h"

#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Vehicles/Vehicle.h"
#include <algorithm>
#include <cstdlib>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace OpenLoco::Vehicles::RailPathfinding
{
    using namespace World;

    // Bound work at complex junctions while allowing routes across the full map.
    static constexpr size_t kMaxSearchNodes = 4096;
    static constexpr uint32_t kMaxSignalAvoidanceWeighting = 320;

    struct SearchNode
    {
        Pos3 pos;
        uint16_t routing;
        StationId stationId;
        uint32_t weighting;
        SignalState signalState;
    };

    struct PendingNode
    {
        uint32_t estimatedCost;
        uint32_t weighting;
        uint16_t index;
    };

    static uint32_t addWeighting(const uint32_t lhs, const uint32_t rhs)
    {
        return static_cast<uint32_t>(std::min<uint64_t>(static_cast<uint64_t>(lhs) + rhs, std::numeric_limits<uint32_t>::max()));
    }

    static uint16_t getDistanceToTarget(const Pos3& pos, const Target& target)
    {
        const auto xDiff = static_cast<uint32_t>(std::abs(pos.x - target.pos.x));
        const auto yDiff = static_cast<uint32_t>(std::abs(pos.y - target.pos.y));
        const auto zDiff = static_cast<uint32_t>(std::abs(pos.z - target.pos.z));
        const auto distance = std::min(xDiff, yDiff) / 4 + std::max(xDiff, yDiff) + zDiff;
        return static_cast<uint16_t>(std::min<uint32_t>(distance, std::numeric_limits<uint16_t>::max()));
    }

    static bool hasReachedTarget(const SearchNode& node, const Target& target)
    {
        if (target.stationId != StationId::null)
        {
            return node.stationId == target.stationId;
        }

        const auto tad = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
        return (node.pos == target.pos && tad == target.tad)
            || (node.pos == target.reversePos && tad == target.reverseTad);
    }

    static bool canPassSignal(SearchNode& node, const uint8_t trackType)
    {
        if ((node.routing & Track::AdditionalTaDFlags::hasSignal) == 0)
        {
            return true;
        }

        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
        const auto signal = Vehicles::getSignalState(node.pos, tad, trackType, 0);
        if ((signal & SignalStateFlags::blockedNoRoute) != SignalStateFlags::none)
        {
            if (node.signalState == SignalState::null)
            {
                node.signalState = SignalState::signalNoRoute;
            }
            return false;
        }
        if (node.signalState == SignalState::null)
        {
            if ((signal & SignalStateFlags::occupied) != SignalStateFlags::none)
            {
                node.signalState = (signal & SignalStateFlags::occupiedOneWay) != SignalStateFlags::none
                    ? SignalState::signalBlockedOneWay
                    : SignalState::signalBlockedTwoWay;
            }
            else
            {
                node.signalState = SignalState::signalClear;
            }
        }
        return true;
    }

    static uint64_t getRouteKey(const SearchNode& node)
    {
        const auto signal = node.signalState == SignalState::null ? 7U : enumValue(node.signalState);
        return static_cast<uint16_t>(node.pos.x)
            | (static_cast<uint64_t>(static_cast<uint16_t>(node.pos.y)) << 16)
            | (static_cast<uint64_t>(static_cast<uint16_t>(node.pos.z)) << 32)
            | (static_cast<uint64_t>(node.routing & Track::AdditionalTaDFlags::basicTaDMask) << 48)
            | (static_cast<uint64_t>(signal) << 57);
    }

    bool isBetterRoute(const RouteResult& base, const RouteResult& candidate)
    {
        if (base.signalState == SignalState::null)
        {
            return true;
        }
        const bool candidateReachedTarget = candidate.bestDistToTarget == 0;
        const bool baseReachedTarget = base.bestDistToTarget == 0;
        if (candidateReachedTarget && baseReachedTarget
            && candidate.signalState != base.signalState)
        {
            if (addWeighting(candidate.bestTrackWeighting, kMaxSignalAvoidanceWeighting) < base.bestTrackWeighting)
            {
                return true;
            }
            if (addWeighting(base.bestTrackWeighting, kMaxSignalAvoidanceWeighting) < candidate.bestTrackWeighting)
            {
                return false;
            }
        }
        if (candidateReachedTarget && candidate.signalState == SignalState::signalBlockedOneWay)
        {
            if (base.signalState <= SignalState::signalClear && base.bestTrackWeighting > 288)
            {
                const auto adjustedWeighting = candidate.bestTrackWeighting * 5 / 4;
                if (adjustedWeighting <= base.bestTrackWeighting)
                {
                    return true;
                }
            }
        }
        if (baseReachedTarget && base.signalState == SignalState::signalBlockedOneWay)
        {
            if (candidate.signalState <= SignalState::signalClear && candidate.bestTrackWeighting > 288)
            {
                const auto adjustedWeighting = base.bestTrackWeighting * 5 / 4;
                if (adjustedWeighting <= candidate.bestTrackWeighting)
                {
                    return false;
                }
            }
        }
        if (!candidateReachedTarget && candidate.signalState == SignalState::signalBlockedOneWay)
        {
            if (!baseReachedTarget && base.signalState == SignalState::signalClear)
            {
                const auto adjustedDist = candidate.bestDistToTarget * 5 / 4;
                if (adjustedDist <= base.bestDistToTarget
                    || candidate.bestDistToTarget + 320 <= base.bestDistToTarget)
                {
                    return true;
                }
            }
        }
        if (!baseReachedTarget && base.signalState == SignalState::signalBlockedOneWay)
        {
            if (!candidateReachedTarget && candidate.signalState == SignalState::signalClear)
            {
                const auto adjustedDist = base.bestDistToTarget * 5 / 4;
                if (adjustedDist <= candidate.bestDistToTarget
                    || base.bestDistToTarget + 320 <= candidate.bestDistToTarget)
                {
                    return false;
                }
            }
        }
        if (candidateReachedTarget != baseReachedTarget)
        {
            return candidateReachedTarget;
        }
        if (candidate.signalState != base.signalState)
        {
            return candidate.signalState < base.signalState;
        }
        if (candidate.bestDistToTarget != base.bestDistToTarget)
        {
            return candidate.bestDistToTarget < base.bestDistToTarget;
        }
        return candidate.bestTrackWeighting <= base.bestTrackWeighting;
    }

    RouteResult findRoute(
        const Pos3 pos,
        const uint16_t tad,
        const CompanyId company,
        const uint8_t trackType,
        const uint8_t requiredMods,
        const uint8_t queryMods,
        const Target& target)
    {
        const auto comparePending = [](const PendingNode& lhs, const PendingNode& rhs) {
            return std::tie(lhs.estimatedCost, lhs.weighting, lhs.index)
                > std::tie(rhs.estimatedCost, rhs.weighting, rhs.index);
        };

        std::vector<SearchNode> nodes;
        nodes.reserve(kMaxSearchNodes);
        nodes.push_back({ pos, tad, StationId::null, 0, SignalState::null });

        std::priority_queue<PendingNode, std::vector<PendingNode>, decltype(comparePending)> pending(comparePending);
        pending.push({ getDistanceToTarget(pos, target), 0, 0 });

        std::unordered_map<uint64_t, uint32_t> bestWeightingByRoute;
        bestWeightingByRoute.reserve(kMaxSearchNodes);
        bestWeightingByRoute[getRouteKey(nodes.front())] = 0;

        RouteResult bestReached{ std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint32_t>::max(), SignalState::null };
        RouteResult bestFallback{ std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint32_t>::max(), SignalState::null };
        while (!pending.empty())
        {
            const auto pendingNode = pending.top();
            pending.pop();
            auto node = nodes[pendingNode.index];
            if (bestWeightingByRoute[getRouteKey(node)] != node.weighting)
            {
                continue;
            }

            if (hasReachedTarget(node, target))
            {
                const RouteResult result{ 0, node.weighting, node.signalState == SignalState::null ? SignalState::noSignals : node.signalState };
                if (isBetterRoute(bestReached, result))
                {
                    bestReached = result;
                }
                continue;
            }

            const auto canContinue = canPassSignal(node, trackType);
            const RouteResult fallback{
                getDistanceToTarget(node.pos, target),
                node.weighting,
                node.signalState == SignalState::null ? SignalState::signalClear : node.signalState,
            };
            if (isBetterRoute(bestFallback, fallback))
            {
                bestFallback = fallback;
            }
            if (!canContinue)
            {
                continue;
            }

            TrackAndDirection::_TrackAndDirection currentTad{ 0, 0 };
            currentTad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
            const auto weighting = addWeighting(node.weighting, TrackData::getTrackMiscData(currentTad.id()).unkWeighting);
            const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, currentTad._data);
            const auto connections = Track::getTrackConnections(nextPos, nextRotation, company, trackType, requiredMods, queryMods);

            for (const auto routing : connections.connections)
            {
                if (nodes.size() >= kMaxSearchNodes)
                {
                    break;
                }

                SearchNode child{ nextPos, routing, connections.stationId, weighting, node.signalState };
                const auto key = getRouteKey(child);
                const auto previous = bestWeightingByRoute.find(key);
                if (previous != bestWeightingByRoute.end() && previous->second <= weighting)
                {
                    continue;
                }
                bestWeightingByRoute[key] = weighting;

                nodes.push_back(child);
                const auto index = static_cast<uint16_t>(nodes.size() - 1);
                const auto estimatedCost = addWeighting(weighting, getDistanceToTarget(nextPos, target));
                pending.push({ estimatedCost, weighting, index });
            }
        }

        return bestReached.signalState != SignalState::null ? bestReached : bestFallback;
    }
}
