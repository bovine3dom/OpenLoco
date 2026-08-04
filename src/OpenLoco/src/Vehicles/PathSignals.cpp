#include "Vehicles/PathSignals.h"

#include "Map/QuarterTile.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/RoutingManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleTail.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace OpenLoco::Vehicles::PathSignals
{
    using namespace World;

    static constexpr size_t kMaxSearchNodes = 4096;
    static constexpr size_t kMaxReservationCandidates = 16;
    static constexpr size_t kMaxLookaheadNodes = 512;
    static constexpr uint16_t kMaxLookaheadDepth = 256;
    static constexpr uint32_t kMaxReservationDetourWeighting = 320;
    // Prefer a clear detour of up to roughly sixteen straight track pieces.
    static constexpr uint32_t kClaimedRoutingPenalty = 512;
    static constexpr uint16_t kNoParent = std::numeric_limits<uint16_t>::max();

    struct Resource
    {
        Pos3 pos;
        uint32_t conflictMask;
    };

    using ResourceMasks = std::unordered_map<uint64_t, uint32_t>;

    struct SearchNode
    {
        Pos3 pos;
        uint16_t routing;
        uint16_t parent;
        uint8_t depth;
        uint32_t weighting;
        uint8_t numTargetsReached;
    };

    struct Target
    {
        Pos3 pos{};
        Pos3 reversePos{};
        uint16_t tad{};
        uint16_t reverseTad{};
        StationId stationId{ StationId::null };
        bool hasTarget{};
    };

    struct Candidate
    {
        std::vector<uint16_t> routings;
        uint8_t numTargetsReached;
        uint32_t distance;
        uint32_t weighting;
        std::optional<std::pair<Pos3, uint16_t>> continuation;
        uint8_t baselineTargetsReached;
        uint32_t baselineDistance;
        uint32_t baselineWeighting;
        bool reservationConflict;
    };

    struct LookaheadNode
    {
        Pos3 pos;
        uint16_t routing;
        uint16_t depth;
        uint32_t weighting;
        uint8_t numTargetsReached;
    };

    struct RouteScore
    {
        uint8_t numTargetsReached;
        uint32_t distance;
        uint32_t weighting;
    };

    static uint64_t getResourceKey(const Pos3& pos)
    {
        return static_cast<uint16_t>(pos.x)
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.y)) << 16)
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.z)) << 32);
    }

    // Pack quarters by connection flag so adjacent diagonal tracks remain distinct.
    static uint32_t getConflictMask(const uint8_t quarters, const uint8_t connectFlags)
    {
        uint32_t mask = 0;
        for (uint8_t connection = 0; connection < 8; ++connection)
        {
            if ((connectFlags & (1U << connection)) != 0)
            {
                mask |= static_cast<uint32_t>(quarters) << (connection * 4);
            }
        }
        return mask;
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

    template<typename TFunc>
    static bool forEachResource(const Pos3& pos, const uint16_t routing, TFunc&& func)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
        const auto trackStart = getTrackStart(pos, tad);
        for (const auto& piece : TrackData::getTrackPiece(tad.id()))
        {
            const auto offset = Math::Vector::rotate(Pos2{ piece.x, piece.y }, tad.cardinalDirection());
            const auto piecePos = trackStart + Pos3{ offset, piece.z };
            auto quarters = piece.subTileClearance.rotate(tad.cardinalDirection()).getBaseQuarterOccupied();
            if (quarters == 0)
            {
                quarters = 0xF;
            }
            const auto conflictMask = getConflictMask(quarters, piece.connectFlags[tad.cardinalDirection()]);
            if (!func(Resource{ piecePos, conflictMask }))
            {
                return false;
            }
        }
        return true;
    }

    static void appendResources(std::vector<Resource>& resources, const Pos3& pos, const uint16_t routing)
    {
        forEachResource(pos, routing, [&resources](const auto& resource) {
            resources.push_back(resource);
            return true;
        });
    }

    template<typename TFunc>
    static void forEachClaimedResource(TFunc&& func)
    {
        std::vector<Resource> resources;
        for (const auto* head : VehicleManager::VehicleList())
        {
            if (head->mode != TransportMode::rail || head->tileX == -1)
            {
                continue;
            }

            const Vehicle train(*head);
            auto pos = Pos3{ train.tail->tileX, train.tail->tileY, train.tail->tileBaseZ * kSmallZStep };
            auto occupied = true;
            for (const auto handle : RoutingManager::RingView(train.tail->routingHandle))
            {
                const auto routing = RoutingManager::getRouting(handle);
                resources.clear();
                appendResources(resources, pos, routing);
                for (const auto& resource : resources)
                {
                    func(*head, handle, resource, occupied);
                }
                pos += TrackData::getUnkTrack(routing & Track::AdditionalTaDFlags::basicTaDMask).pos;
                if (handle == head->routingHandle)
                {
                    occupied = false;
                }
            }
        }
    }

    static ResourceMasks getClaimedResourceMask()
    {
        ResourceMasks claimed;
        forEachClaimedResource([&claimed](const auto&, const auto, const auto& resource, const bool) {
            claimed[getResourceKey(resource.pos)] |= resource.conflictMask;
        });
        return claimed;
    }

    std::vector<ClaimedResource> getClaimedResources()
    {
        std::vector<ClaimedResource> result;
        forEachClaimedResource([&result](const auto& head, const auto handle, const auto& resource, const bool occupied) {
            result.push_back({ head.id, handle, resource.pos, resource.conflictMask, occupied });
        });
        return result;
    }

    static Target makeTarget(const Order& order)
    {
        Target target{};
        if (const auto* stationOrder = order.as<OrderStation>(); stationOrder != nullptr)
        {
            target.stationId = stationOrder->getStation();
            const auto* station = StationManager::get(target.stationId);
            target.pos = { station->x, station->y, station->z };
            target.hasTarget = true;
        }
        else if (const auto* waypointOrder = order.as<OrderRouteWaypoint>(); waypointOrder != nullptr)
        {
            target.pos = waypointOrder->getWaypoint();
            target.tad = (waypointOrder->getTrackId() << 3) | waypointOrder->getDirection();
            const auto& trackSize = TrackData::getUnkTrack(target.tad);
            target.reversePos = target.pos + trackSize.pos;
            if (trackSize.rotationEnd < 12)
            {
                target.reversePos -= Pos3{ kRotationOffset[trackSize.rotationEnd], 0 };
            }
            target.reverseTad = target.tad ^ (1U << 2);
            target.hasTarget = true;
        }
        return target;
    }

    static std::pair<Target, std::optional<Target>> getTargets(const VehicleHead& head)
    {
        const auto orders = head.getCurrentOrders();
        const auto currentOrder = orders.begin();
        auto target = makeTarget(*currentOrder);
        std::optional<Target> nextTarget;
        for (auto nextOrder = std::next(currentOrder); target.hasTarget && nextOrder != orders.end(); ++nextOrder)
        {
            auto candidate = makeTarget(*nextOrder);
            if (candidate.hasTarget)
            {
                nextTarget = candidate;
                break;
            }
        }
        return { target, nextTarget };
    }

    static bool reachesWaypoint(const Pos3& pos, const uint16_t routing, const Target& target)
    {
        const auto tad = routing & Track::AdditionalTaDFlags::basicTaDMask;
        return (pos == target.pos && tad == target.tad)
            || (pos == target.reversePos && tad == target.reverseTad);
    }

    static uint32_t getDistanceToTarget(const Pos3& pos, const Target& target)
    {
        if (!target.hasTarget)
        {
            return 0;
        }
        const auto xDiff = static_cast<uint32_t>(std::abs(pos.x - target.pos.x));
        const auto yDiff = static_cast<uint32_t>(std::abs(pos.y - target.pos.y));
        const auto zDiff = static_cast<uint32_t>(std::abs(pos.z - target.pos.z));
        return std::min(xDiff, yDiff) / 4 + std::max(xDiff, yDiff) + zDiff;
    }

    static const Target* getActiveTarget(const Target& target, const Target* nextTarget, const uint8_t numTargetsReached)
    {
        if (!target.hasTarget)
        {
            return nullptr;
        }
        if (numTargetsReached == 0)
        {
            return &target;
        }
        return numTargetsReached == 1 ? nextTarget : nullptr;
    }

    static uint32_t getDistanceToActiveTarget(const Pos3& pos, const Target& target, const Target* nextTarget, const uint8_t numTargetsReached)
    {
        const auto* activeTarget = getActiveTarget(target, nextTarget, numTargetsReached);
        return activeTarget == nullptr ? 0 : getDistanceToTarget(pos, *activeTarget);
    }

    static uint8_t advanceWaypointTarget(const Pos3& pos, const uint16_t routing, const Target& target, const Target* nextTarget, const uint8_t numTargetsReached)
    {
        const auto* activeTarget = getActiveTarget(target, nextTarget, numTargetsReached);
        return activeTarget != nullptr && activeTarget->stationId == StationId::null && reachesWaypoint(pos, routing, *activeTarget)
            ? numTargetsReached + 1
            : numTargetsReached;
    }

    static uint8_t advanceStationTarget(const StationId stationId, const Target& target, const Target* nextTarget, const uint8_t numTargetsReached)
    {
        const auto* activeTarget = getActiveTarget(target, nextTarget, numTargetsReached);
        return activeTarget != nullptr && activeTarget->stationId != StationId::null && activeTarget->stationId == stationId
            ? numTargetsReached + 1
            : numTargetsReached;
    }

    static std::vector<uint16_t> getPath(const std::vector<SearchNode>& nodes, uint16_t index)
    {
        std::vector<uint16_t> path;
        while (index != kNoParent)
        {
            path.push_back(nodes[index].routing);
            index = nodes[index].parent;
        }
        std::ranges::reverse(path);
        return path;
    }

    static bool hasConflict(const std::vector<uint16_t>& path, const Pos3& firstPos, const ResourceMasks& claimed)
    {
        ResourceMasks pathResources;
        std::vector<Resource> resources;
        auto pos = firstPos;
        for (const auto routing : path)
        {
            resources.clear();
            appendResources(resources, pos, routing);
            for (const auto& resource : resources)
            {
                const auto key = getResourceKey(resource.pos);
                const auto existing = claimed.find(key);
                if (existing != claimed.end() && (existing->second & resource.conflictMask) != 0)
                {
                    return true;
                }
                if ((pathResources[key] & resource.conflictMask) != 0)
                {
                    return true;
                }
                pathResources[key] |= resource.conflictMask;
            }
            pos += TrackData::getUnkTrack(routing & Track::AdditionalTaDFlags::basicTaDMask).pos;
        }
        return false;
    }

    static bool isClaimed(const Pos3& pos, const uint16_t routing, const ResourceMasks& claimed)
    {
        auto hasClaim = false;
        forEachResource(pos, routing, [&claimed, &hasClaim](const auto& resource) {
            const auto existing = claimed.find(getResourceKey(resource.pos));
            hasClaim = existing != claimed.end() && (existing->second & resource.conflictMask) != 0;
            return !hasClaim;
        });
        return hasClaim;
    }

    static uint32_t addWeighting(const uint32_t lhs, const uint32_t rhs)
    {
        return static_cast<uint32_t>(std::min<uint64_t>(static_cast<uint64_t>(lhs) + rhs, std::numeric_limits<uint32_t>::max()));
    }

    static uint32_t getRoutingWeighting(const Pos3& pos, const uint16_t routing, const uint8_t trackType, const bool includeTrackWeighting, const ResourceMasks* claimed)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
        auto weighting = includeTrackWeighting ? static_cast<uint32_t>(TrackData::getTrackMiscData(tad.id()).unkWeighting) : 0;
        if (claimed == nullptr)
        {
            return weighting;
        }
        if (isClaimed(pos, routing, *claimed))
        {
            return addWeighting(weighting, kClaimedRoutingPenalty);
        }
        if ((routing & Track::AdditionalTaDFlags::hasSignal) != 0
            && (getSignalState(pos, tad, trackType, 0) & SignalStateFlags::occupied) != SignalStateFlags::none)
        {
            weighting = addWeighting(weighting, kClaimedRoutingPenalty);
        }
        return weighting;
    }

    bool isPathReserved(const Pos3& pos, const uint16_t routing)
    {
        std::vector<Resource> targetResources;
        std::vector<Resource> routeResources;
        appendResources(targetResources, pos, routing);

        for (const auto* head : VehicleManager::VehicleList())
        {
            if (head->mode != TransportMode::rail || head->tileX == -1)
            {
                continue;
            }

            auto routePos = Pos3{ head->tileX, head->tileY, head->tileBaseZ * kSmallZStep };
            auto previousRouting = RoutingManager::getRouting(head->routingHandle);
            auto handle = head->routingHandle;
            for (size_t i = 1; i < Limits::kMaxRoutingsPerVehicle; ++i)
            {
                handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
                const auto nextRouting = RoutingManager::getRouting(handle);
                if (nextRouting == RoutingManager::kAllocatedButFreeRouting)
                {
                    break;
                }

                routePos += TrackData::getUnkTrack(previousRouting & Track::AdditionalTaDFlags::basicTaDMask).pos;
                routeResources.clear();
                appendResources(routeResources, routePos, nextRouting);
                for (const auto& target : targetResources)
                {
                    const auto conflict = std::ranges::find_if(routeResources, [&target](const auto& resource) {
                        return resource.pos == target.pos
                            && (resource.conflictMask & target.conflictMask) != 0;
                    });
                    if (conflict != routeResources.end())
                    {
                        return true;
                    }
                }
                previousRouting = nextRouting;
            }
        }
        return false;
    }

    bool hasPathReservationConflict(const EntityId vehicle, const Pos3& pos, const uint16_t routing)
    {
        if (!RoutingManager::hasPathReservations())
        {
            return false;
        }
        std::vector<Resource> targetResources;
        appendResources(targetResources, pos, routing);
        auto hasConflict = false;
        for (const auto* head : VehicleManager::VehicleList())
        {
            if (head->id == vehicle || head->mode != TransportMode::rail || head->tileX == -1
                || !RoutingManager::hasPathReservations(head->routingHandle))
            {
                continue;
            }

            const Vehicle train(*head);
            auto routePos = Pos3{ train.tail->tileX, train.tail->tileY, train.tail->tileBaseZ * kSmallZStep };
            for (const auto handle : RoutingManager::RingView(train.tail->routingHandle))
            {
                const auto claimedRouting = RoutingManager::getRouting(handle);
                if (RoutingManager::isPathReserved(handle))
                {
                    forEachResource(routePos, claimedRouting, [&](const auto& resource) {
                        hasConflict = std::ranges::any_of(targetResources, [&resource](const auto& target) {
                            return resource.pos == target.pos && (resource.conflictMask & target.conflictMask) != 0;
                        });
                        return !hasConflict;
                    });
                    if (hasConflict)
                    {
                        return true;
                    }
                }
                routePos += TrackData::getUnkTrack(claimedRouting & Track::AdditionalTaDFlags::basicTaDMask).pos;
            }
        }
        return hasConflict;
    }

    static bool isBetterCandidate(const Candidate& candidate, const Candidate& current)
    {
        if (candidate.numTargetsReached != current.numTargetsReached)
        {
            return candidate.numTargetsReached > current.numTargetsReached;
        }
        const auto candidateCost = addWeighting(candidate.weighting, candidate.distance);
        const auto currentCost = addWeighting(current.weighting, current.distance);
        return std::tie(candidateCost, candidate.distance, candidate.weighting, candidate.routings)
            < std::tie(currentCost, current.distance, current.weighting, current.routings);
    }

    static bool isBetterBaselineCandidate(const Candidate& candidate, const Candidate& current)
    {
        if (candidate.baselineTargetsReached != current.baselineTargetsReached)
        {
            return candidate.baselineTargetsReached > current.baselineTargetsReached;
        }
        const auto candidateCost = addWeighting(candidate.baselineWeighting, candidate.baselineDistance);
        const auto currentCost = addWeighting(current.baselineWeighting, current.baselineDistance);
        return std::tie(candidateCost, candidate.baselineDistance, candidate.baselineWeighting, candidate.routings)
            < std::tie(currentCost, current.baselineDistance, current.baselineWeighting, current.routings);
    }

    static bool containsAncestor(const std::vector<SearchNode>& nodes, uint16_t parent, const Pos3& pos, const uint16_t routing)
    {
        const auto basicRouting = routing & Track::AdditionalTaDFlags::basicTaDMask;
        while (parent != kNoParent)
        {
            const auto& node = nodes[parent];
            if (node.pos == pos && (node.routing & Track::AdditionalTaDFlags::basicTaDMask) == basicRouting)
            {
                return true;
            }
            parent = node.parent;
        }
        return false;
    }

    static RouteScore getLookaheadScore(
        const Pos3& firstPos,
        const uint16_t firstRouting,
        const VehicleHead& head,
        const uint8_t requiredMods,
        const uint8_t queryMods,
        const Target& target,
        const Target* nextTarget,
        const uint8_t initialTargetsReached,
        const ResourceMasks* claimed)
    {
        struct PendingNode
        {
            uint32_t estimatedCost;
            uint16_t index;
        };
        const auto comparePending = [](const PendingNode& lhs, const PendingNode& rhs) {
            return std::tie(lhs.estimatedCost, lhs.index) > std::tie(rhs.estimatedCost, rhs.index);
        };

        const auto seedProgress = advanceWaypointTarget(firstPos, firstRouting, target, nextTarget, initialTargetsReached);
        const auto seedWeighting = getRoutingWeighting(firstPos, firstRouting, head.trackType, target.hasTarget, claimed);
        const auto seedDistance = getDistanceToActiveTarget(firstPos, target, nextTarget, seedProgress);
        std::vector<LookaheadNode> nodes;
        nodes.reserve(kMaxLookaheadNodes);
        nodes.push_back({ firstPos, firstRouting, 1, seedWeighting, seedProgress });
        std::priority_queue<PendingNode, std::vector<PendingNode>, decltype(comparePending)> pending(comparePending);
        pending.push({ addWeighting(seedWeighting, seedDistance), 0 });

        std::unordered_map<uint64_t, uint32_t> bestWeightingByRoute;
        const auto getRouteKey = [](const Pos3& pos, const uint16_t routing, const uint8_t numTargetsReached) {
            return getResourceKey(pos)
                | (static_cast<uint64_t>(routing & Track::AdditionalTaDFlags::basicTaDMask) << 48)
                | (static_cast<uint64_t>(numTargetsReached) << 57);
        };
        bestWeightingByRoute[getRouteKey(firstPos, firstRouting, seedProgress)] = seedWeighting;

        std::optional<RouteScore> bestReached;
        std::optional<RouteScore> bestFallback;
        uint16_t bestFallbackDepth = 0;
        const auto isBetterScore = [](const RouteScore& candidate, const RouteScore& current) {
            if (candidate.numTargetsReached != current.numTargetsReached)
            {
                return candidate.numTargetsReached > current.numTargetsReached;
            }
            const auto candidateCost = addWeighting(candidate.weighting, candidate.distance);
            const auto currentCost = addWeighting(current.weighting, current.distance);
            return std::tie(candidateCost, candidate.distance, candidate.weighting)
                < std::tie(currentCost, current.distance, current.weighting);
        };
        const auto considerScore = [&](const RouteScore& score, const uint16_t depth) {
            if (target.hasTarget && getActiveTarget(target, nextTarget, score.numTargetsReached) == nullptr)
            {
                if (!bestReached.has_value() || isBetterScore(score, *bestReached))
                {
                    bestReached = score;
                }
                return;
            }
            if (!bestFallback.has_value()
                || score.numTargetsReached > bestFallback->numTargetsReached
                || (score.numTargetsReached == bestFallback->numTargetsReached && depth > bestFallbackDepth))
            {
                bestFallback.reset();
                bestFallbackDepth = depth;
            }
            if (depth == bestFallbackDepth
                && (!bestFallback.has_value() || isBetterScore(score, *bestFallback)))
            {
                bestFallback = score;
            }
        };

        while (!pending.empty())
        {
            const auto pendingNode = pending.top();
            pending.pop();
            const auto node = nodes[pendingNode.index];
            const auto routeKey = getRouteKey(node.pos, node.routing, node.numTargetsReached);
            if (bestWeightingByRoute[routeKey] != node.weighting)
            {
                continue;
            }

            TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
            tad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
            const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, tad._data);
            const auto connections = Track::getTrackConnections(nextPos, nextRotation, head.owner, head.trackType, requiredMods, queryMods);
            const auto numTargetsReached = advanceStationTarget(connections.stationId, target, nextTarget, node.numTargetsReached);
            const auto distance = getDistanceToActiveTarget(nextPos, target, nextTarget, numTargetsReached);
            const auto reachedAllTargets = target.hasTarget && getActiveTarget(target, nextTarget, numTargetsReached) == nullptr;

            if (reachedAllTargets || connections.connections.empty() || node.depth >= kMaxLookaheadDepth)
            {
                considerScore({ numTargetsReached, distance, node.weighting }, node.depth);
                continue;
            }
            considerScore({ numTargetsReached, distance, node.weighting }, node.depth);
            if (nodes.size() >= kMaxLookaheadNodes)
            {
                continue;
            }

            for (const auto routing : connections.connections)
            {
                TrackAndDirection::_TrackAndDirection nextTad{ 0, 0 };
                nextTad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
                const auto nextSignalMode = getSignalMode(nextPos, nextTad, head.trackType, 0);
                if (!nextSignalMode.has_value() && getSignalState(nextPos, nextTad, head.trackType, 0) != SignalStateFlags::none)
                {
                    continue;
                }

                const auto childWeighting = addWeighting(node.weighting, getRoutingWeighting(nextPos, routing, head.trackType, target.hasTarget, claimed));
                const auto childProgress = advanceWaypointTarget(nextPos, routing, target, nextTarget, numTargetsReached);
                const auto childKey = getRouteKey(nextPos, routing, childProgress);
                const auto previousWeighting = bestWeightingByRoute.find(childKey);
                if (previousWeighting != bestWeightingByRoute.end() && previousWeighting->second <= childWeighting)
                {
                    continue;
                }
                bestWeightingByRoute[childKey] = childWeighting;

                const auto childDistance = getDistanceToActiveTarget(nextPos, target, nextTarget, childProgress);
                nodes.push_back({ nextPos, routing, static_cast<uint16_t>(node.depth + 1), childWeighting, childProgress });
                const auto childIndex = static_cast<uint16_t>(nodes.size() - 1);
                pending.push({ addWeighting(childWeighting, childDistance), childIndex });
                if (nodes.size() == kMaxLookaheadNodes)
                {
                    break;
                }
            }
        }

        if (bestReached.has_value())
        {
            return *bestReached;
        }
        return bestFallback.value_or(RouteScore{ seedProgress, seedDistance, seedWeighting });
    }

    static size_t getAvailableRoutingSlots(const VehicleHead& head)
    {
        auto handle = head.routingHandle;
        size_t count = 0;
        for (size_t i = 1; i < Limits::kMaxRoutingsPerVehicle; ++i)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            if (RoutingManager::getRouting(handle) != RoutingManager::kAllocatedButFreeRouting)
            {
                break;
            }
            ++count;
        }
        constexpr size_t kRequiredFreeSlots = 3;
        return count > kRequiredFreeSlots ? count - kRequiredFreeSlots : 0;
    }

    bool tryReservePath(VehicleHead& head, const Pos3& firstPos, const uint16_t firstRouting)
    {
        const auto maxPathLength = getAvailableRoutingSlots(head);
        if (maxPathLength == 0)
        {
            return false;
        }

        const Vehicle train(head);
        const auto requiredMods = head.var_53;
        const auto queryMods = train.veh1->var_49;
        const auto [target, nextTarget] = getTargets(head);
        const auto* nextTargetPtr = nextTarget.has_value() ? &*nextTarget : nullptr;
        const auto claimed = getClaimedResourceMask();
        const ResourceMasks noClaims;

        std::vector<SearchNode> nodes;
        nodes.reserve(kMaxSearchNodes);
        nodes.push_back({ firstPos, firstRouting, kNoParent, 1, 0, 0 });
        std::vector<uint16_t> pending{ 0 };
        std::vector<Candidate> candidates;

        const auto considerCandidate = [&](const uint16_t tailIndex, const Pos3& endpoint, const uint8_t numTargetsReached, const uint32_t weighting, std::optional<std::pair<Pos3, uint16_t>> continuation) {
            auto path = getPath(nodes, tailIndex);
            if (path.empty() || path.size() > maxPathLength || hasConflict(path, firstPos, noClaims))
            {
                return;
            }
            const auto distance = getDistanceToActiveTarget(endpoint, target, nextTargetPtr, numTargetsReached);
            const auto conflictsWithClaim = hasConflict(path, firstPos, claimed)
                || (continuation.has_value() && isClaimed(continuation->first, continuation->second, claimed));
            candidates.push_back({ std::move(path), numTargetsReached, distance, weighting, continuation, numTargetsReached, distance, weighting, conflictsWithClaim });
        };

        while (!pending.empty())
        {
            const auto index = pending.back();
            pending.pop_back();
            auto node = nodes[index];

            TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
            tad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
            if (index != 0 && getSignalMode(node.pos, tad, head.trackType, 0).has_value())
            {
                considerCandidate(node.parent, node.pos, node.numTargetsReached, nodes[node.parent].weighting, std::pair{ node.pos, node.routing });
                continue;
            }

            node.numTargetsReached = advanceWaypointTarget(node.pos, node.routing, target, nextTargetPtr, node.numTargetsReached);
            node.weighting += TrackData::getTrackMiscData(tad.id()).unkWeighting;
            nodes[index] = node;

            const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, tad._data);
            const auto connections = Track::getTrackConnections(nextPos, nextRotation, head.owner, head.trackType, requiredMods, queryMods);
            const auto numTargetsReached = advanceStationTarget(connections.stationId, target, nextTargetPtr, node.numTargetsReached);
            if (connections.connections.empty())
            {
                considerCandidate(index, nextPos, numTargetsReached, node.weighting, std::nullopt);
                continue;
            }
            // The terminating signal is not part of the reserved path, so inspect one extra connection.
            if (node.depth > maxPathLength || nodes.size() >= kMaxSearchNodes)
            {
                continue;
            }

            for (auto iter = connections.connections.rbegin(); iter != connections.connections.rend(); ++iter)
            {
                const auto routing = *iter;
                TrackAndDirection::_TrackAndDirection nextTad{ 0, 0 };
                nextTad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
                const auto nextSignalMode = getSignalMode(nextPos, nextTad, head.trackType, 0);
                if ((!nextSignalMode.has_value() && getSignalState(nextPos, nextTad, head.trackType, 0) != SignalStateFlags::none)
                    || containsAncestor(nodes, index, nextPos, routing))
                {
                    continue;
                }
                nodes.push_back({ nextPos, routing, index, static_cast<uint8_t>(node.depth + 1), node.weighting, numTargetsReached });
                pending.push_back(static_cast<uint16_t>(nodes.size() - 1));
                if (nodes.size() == kMaxSearchNodes)
                {
                    break;
                }
            }
        }

        if (candidates.empty())
        {
            return false;
        }

        std::ranges::sort(candidates, isBetterCandidate);
        if (candidates.size() > kMaxReservationCandidates)
        {
            candidates.resize(kMaxReservationCandidates);
        }

        for (auto& candidate : candidates)
        {
            if (!candidate.continuation.has_value())
            {
                continue;
            }

            const auto& [continuationPos, continuationRouting] = *candidate.continuation;
            // Compare routes to the current order, or to the next order after entering a station.
            const auto* baselineNextTarget = candidate.baselineTargetsReached == 0 ? nullptr : nextTargetPtr;
            if (getActiveTarget(target, baselineNextTarget, candidate.baselineTargetsReached) != nullptr || !target.hasTarget)
            {
                const auto baselineScore = getLookaheadScore(continuationPos, continuationRouting, head, requiredMods, queryMods, target, baselineNextTarget, candidate.baselineTargetsReached, nullptr);
                candidate.baselineTargetsReached = baselineScore.numTargetsReached;
                candidate.baselineDistance = baselineScore.distance;
                candidate.baselineWeighting = addWeighting(candidate.baselineWeighting, baselineScore.weighting);
            }
            if (!candidate.reservationConflict
                && (getActiveTarget(target, nextTargetPtr, candidate.numTargetsReached) != nullptr || !target.hasTarget))
            {
                const auto score = getLookaheadScore(continuationPos, continuationRouting, head, requiredMods, queryMods, target, nextTargetPtr, candidate.numTargetsReached, &claimed);
                candidate.numTargetsReached = score.numTargetsReached;
                candidate.distance = score.distance;
                candidate.weighting = addWeighting(candidate.weighting, score.weighting);
            }
        }
        const auto& preferred = *std::ranges::min_element(candidates, isBetterBaselineCandidate);
        const auto preferredCost = addWeighting(preferred.baselineWeighting, preferred.baselineDistance);
        const Candidate* best = nullptr;
        for (const auto& candidate : candidates)
        {
            const auto candidateCost = addWeighting(candidate.baselineWeighting, candidate.baselineDistance);
            if (candidate.reservationConflict
                || candidate.baselineTargetsReached != preferred.baselineTargetsReached
                || candidateCost > addWeighting(preferredCost, kMaxReservationDetourWeighting))
            {
                continue;
            }
            if (best == nullptr || isBetterCandidate(candidate, *best))
            {
                best = &candidate;
            }
        }
        if (best == nullptr)
        {
            return false;
        }

        auto handle = head.routingHandle;
        for (size_t i = 0; i < best->routings.size(); ++i)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            if (RoutingManager::getRouting(handle) != RoutingManager::kAllocatedButFreeRouting)
            {
                return false;
            }
        }
        handle = head.routingHandle;
        for (const auto routing : best->routings)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            RoutingManager::setRouting(handle, routing);
            RoutingManager::markPathReserved(handle);
        }
        return true;
    }
}
