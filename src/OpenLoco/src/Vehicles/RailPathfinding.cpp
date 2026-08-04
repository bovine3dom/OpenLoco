#include "Vehicles/RailPathfinding.h"

#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Map/StationElement.h"
#include "Map/TileManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/RailTraffic.h"
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
    static constexpr size_t kMaxSearchNodes = 16384;
    struct SearchNode
    {
        Pos3 pos;
        uint16_t routing;
        RailTraffic::TravelTime weighting;
        SignalState signalState;
        uint8_t numTargetsReached;
    };

    struct PendingNode
    {
        RailTraffic::TravelTime estimatedCost;
        RailTraffic::TravelTime weighting;
        uint16_t index;
    };

    static RailTraffic::TravelTime addWeighting(const RailTraffic::TravelTime lhs, const RailTraffic::TravelTime rhs)
    {
        return rhs > std::numeric_limits<RailTraffic::TravelTime>::max() - lhs
            ? std::numeric_limits<RailTraffic::TravelTime>::max()
            : lhs + rhs;
    }

    static uint32_t getDistanceToTarget(const Pos3& pos, const Target& target)
    {
        if (target.stationId != StationId::null)
        {
            // A station coordinate need not identify the platform reached by this route.
            return 0;
        }
        const auto getDistance = [&pos](const Pos3& destination) {
            const auto xDiff = static_cast<uint32_t>(std::abs(pos.x - destination.x));
            const auto yDiff = static_cast<uint32_t>(std::abs(pos.y - destination.y));
            const auto zDiff = static_cast<uint32_t>(std::abs(pos.z - destination.z));
            return std::min(xDiff, yDiff) / 4 + std::max(xDiff, yDiff) + zDiff;
        };
        // The cheapest track pieces cost at least two-fifths of this geometric metric.
        return std::min(getDistance(target.pos), getDistance(target.reversePos)) * 2 / 5;
    }

    static RailTraffic::TravelTime getEstimatedDistanceToTargets(const Pos3& pos, const uint8_t numTargetsReached, const RailTraffic::SpeedProfile& speedProfile, const Target& target, const Target* nextTarget)
    {
        if (numTargetsReached != 0)
        {
            return RailTraffic::getHeuristicTime(speedProfile, getDistanceToTarget(pos, *nextTarget));
        }

        if (target.stationId != StationId::null)
        {
            return 0;
        }

        const auto distanceToTarget = getDistanceToTarget(pos, target);
        const auto distanceToNext = nextTarget == nullptr
            ? 0
            : std::min(getDistanceToTarget(target.pos, *nextTarget), getDistanceToTarget(target.reversePos, *nextTarget));
        return nextTarget == nullptr
            ? RailTraffic::getHeuristicTime(speedProfile, distanceToTarget)
            : RailTraffic::getHeuristicTime(speedProfile, distanceToTarget + distanceToNext);
    }

    static Pos3 getTrackStart(Pos3 pos, const TrackAndDirection::_TrackAndDirection tad)
    {
        if (tad.isReversed())
        {
            const auto& trackSize = TrackData::getUnkTrack(tad._data);
            pos += trackSize.pos;
            if (trackSize.rotationEnd < 12)
            {
                pos -= Pos3{ kRotationOffset[trackSize.rotationEnd], 0 };
            }
        }
        return pos;
    }

    static StationId getStationId(const Pos3& pos, const uint16_t routing)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
        const auto trackStart = getTrackStart(pos, tad);
        const auto* station = TileManager::get(trackStart).trainStation(tad.id(), tad.cardinalDirection(), trackStart.z / kSmallZStep);
        return station != nullptr && !station->isGhost() && !station->isAiAllocated() ? station->stationId() : StationId::null;
    }

    static bool hasReachedTarget(const SearchNode& node, const Target& target)
    {
        if (target.stationId != StationId::null)
        {
            return getStationId(node.pos, node.routing) == target.stationId;
        }

        const auto tad = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
        return (node.pos == target.pos && tad == target.tad)
            || (node.pos == target.reversePos && tad == target.reverseTad);
    }

    static bool canPassSignal(SearchNode& node, const uint8_t trackType, const RailTraffic::SpeedProfile& speedProfile)
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
                node.weighting = addWeighting(node.weighting, RailTraffic::getLiveSignalPenalty(speedProfile, node.routing, trackType));
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
        const auto hasSeenSignal = node.signalState == SignalState::null ? 0U : 1U;
        return static_cast<uint16_t>(node.pos.x)
            | (static_cast<uint64_t>(static_cast<uint16_t>(node.pos.y)) << 16)
            | (static_cast<uint64_t>(static_cast<uint16_t>(node.pos.z)) << 32)
            | (static_cast<uint64_t>(node.routing & Track::AdditionalTaDFlags::basicTaDMask) << 48)
            | (static_cast<uint64_t>(hasSeenSignal) << 57)
            | (static_cast<uint64_t>(node.numTargetsReached) << 60);
    }

    bool isBetterRoute(const RouteResult& base, const RouteResult& candidate)
    {
        if (base.signalState == SignalState::null)
        {
            return true;
        }
        if (candidate.numTargetsReached != base.numTargetsReached)
        {
            return candidate.numTargetsReached > base.numTargetsReached;
        }
        const auto candidateTime = addWeighting(candidate.bestTrackWeighting, candidate.bestDistToTarget);
        const auto baseTime = addWeighting(base.bestTrackWeighting, base.bestDistToTarget);
        return std::tie(candidateTime, candidate.bestDistToTarget, candidate.bestTrackWeighting)
            <= std::tie(baseTime, base.bestDistToTarget, base.bestTrackWeighting);
    }

    RouteResult findRoute(
        const Pos3 pos,
        const uint16_t tad,
        const CompanyId company,
        const uint8_t trackType,
        const uint8_t requiredMods,
        const uint8_t queryMods,
        const Target& target,
        const Target* nextTarget)
    {
        return findRoute(pos, tad, company, trackType, requiredMods, queryMods, RailTraffic::SpeedProfile{}, target, nextTarget);
    }

    RouteResult findRoute(
        const Pos3 pos,
        const uint16_t tad,
        const CompanyId company,
        const uint8_t trackType,
        const uint8_t requiredMods,
        const uint8_t queryMods,
        const RailTraffic::SpeedProfile& speedProfile,
        const Target& target,
        const Target* nextTarget)
    {
        const auto comparePending = [](const PendingNode& lhs, const PendingNode& rhs) {
            return std::tie(lhs.estimatedCost, lhs.weighting, lhs.index)
                > std::tie(rhs.estimatedCost, rhs.weighting, rhs.index);
        };

        std::vector<SearchNode> nodes;
        nodes.reserve(kMaxSearchNodes);
        nodes.push_back({ pos, tad, 0, SignalState::null, 0 });

        std::priority_queue<PendingNode, std::vector<PendingNode>, decltype(comparePending)> pending(comparePending);
        if (canPassSignal(nodes.front(), trackType, speedProfile))
        {
            pending.push({ addWeighting(nodes.front().weighting, getEstimatedDistanceToTargets(pos, 0, speedProfile, target, nextTarget)), nodes.front().weighting, 0 });
        }

        std::unordered_map<uint64_t, RailTraffic::TravelTime> bestWeightingByRoute;
        bestWeightingByRoute.reserve(kMaxSearchNodes);
        bestWeightingByRoute[getRouteKey(nodes.front())] = nodes.front().weighting;

        RouteResult bestFallback{ std::numeric_limits<RailTraffic::TravelTime>::max(), std::numeric_limits<RailTraffic::TravelTime>::max(), SignalState::null };
        bool searchExhausted = false;
        while (!pending.empty())
        {
            const auto pendingNode = pending.top();
            pending.pop();
            auto node = nodes[pendingNode.index];
            if (bestWeightingByRoute[getRouteKey(node)] != node.weighting)
            {
                continue;
            }

            auto advancedTarget = false;
            while (node.numTargetsReached < 2)
            {
                const auto* activeTarget = node.numTargetsReached == 0 ? &target : nextTarget;
                if (activeTarget == nullptr || !hasReachedTarget(node, *activeTarget))
                {
                    break;
                }
                node.numTargetsReached++;
                advancedTarget = true;
            }
            if (advancedTarget)
            {
                if (nextTarget == nullptr || node.numTargetsReached == 2)
                {
                    return { 0, node.weighting, node.signalState == SignalState::null ? SignalState::noSignals : node.signalState, node.numTargetsReached, searchExhausted };
                }

                const auto key = getRouteKey(node);
                const auto previous = bestWeightingByRoute.find(key);
                if (previous != bestWeightingByRoute.end() && previous->second <= node.weighting)
                {
                    continue;
                }
                bestWeightingByRoute[key] = node.weighting;
            }

            const auto& searchTarget = node.numTargetsReached == 0 ? target : *nextTarget;
            const RouteResult fallback{
                RailTraffic::getHeuristicTime(speedProfile, getDistanceToTarget(node.pos, searchTarget)),
                node.weighting,
                node.signalState == SignalState::null ? SignalState::signalClear : node.signalState,
                node.numTargetsReached,
            };
            if (isBetterRoute(bestFallback, fallback))
            {
                bestFallback = fallback;
            }
            TrackAndDirection::_TrackAndDirection currentTad{ 0, 0 };
            currentTad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
            const auto weighting = addWeighting(node.weighting, RailTraffic::getTravelTime(speedProfile, node.pos, node.routing, trackType));
            const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, currentTad._data);
            const auto connections = Track::getTrackConnections(nextPos, nextRotation, company, trackType, requiredMods, queryMods);

            for (const auto routing : connections.connections)
            {
                SearchNode child{ nextPos, routing, weighting, node.signalState, node.numTargetsReached };
                if (!canPassSignal(child, trackType, speedProfile))
                {
                    continue;
                }
                const auto key = getRouteKey(child);
                const auto previous = bestWeightingByRoute.find(key);
                if (previous != bestWeightingByRoute.end() && previous->second <= child.weighting)
                {
                    continue;
                }
                if (nodes.size() >= kMaxSearchNodes)
                {
                    searchExhausted = true;
                    break;
                }
                bestWeightingByRoute[key] = child.weighting;

                nodes.push_back(child);
                const auto index = static_cast<uint16_t>(nodes.size() - 1);
                const auto estimatedCost = addWeighting(child.weighting, getEstimatedDistanceToTargets(nextPos, child.numTargetsReached, speedProfile, target, nextTarget));
                pending.push({ estimatedCost, child.weighting, index });
            }
        }

        bestFallback.searchExhausted = searchExhausted;
        return bestFallback;
    }
}
