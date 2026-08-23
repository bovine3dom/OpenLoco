#include "Vehicles/PathSignals.h"

#include "Entities/EntityManager.h"
#include "Map/QuarterTile.h"
#include "Map/StationElement.h"
#include "Map/TileManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/RailTraffic.h"
#include "Vehicles/RoutingManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleTail.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include <OpenLoco/Engine/Limits.h>
#include <algorithm>
#include <array>
#include <bit>
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
    static constexpr uint32_t kClaimedRoutingPenalty = 512;
    static constexpr uint16_t kNoParent = std::numeric_limits<uint16_t>::max();

    struct Resource
    {
        Pos3 pos;
        uint32_t conflictMask;
    };

    using ResourceMasks = std::unordered_map<uint64_t, uint32_t>;

    using ResourceClaimCounts = std::array<uint32_t, 32>;

    struct ReservationClaim
    {
        EntityId vehicle;
        uint32_t mask;
    };

    struct VehicleClaim
    {
        Resource resource;
        bool pathReserved;
    };

    struct ClaimedResourceCache
    {
        bool active{};
        ResourceMasks masks;
        std::unordered_map<uint64_t, ResourceClaimCounts> counts;
        std::unordered_map<EntityId, std::vector<VehicleClaim>> byVehicle;
        std::unordered_map<uint64_t, std::vector<ReservationClaim>> reservations;
        ResourceMasks fallback;
        std::array<bool, Limits::kMaxVehicles> dirty{};
        std::array<EntityId, Limits::kMaxVehicles> owners{};
        std::vector<uint16_t> dirtyVehicles;
    };

    static ClaimedResourceCache _claimedResourceCache;

    struct SearchNode
    {
        Pos3 pos;
        uint16_t routing;
        uint16_t parent;
        uint16_t depth;
        RailTraffic::TravelTime weighting;
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
        RailTraffic::TravelTime distance;
        RailTraffic::TravelTime weighting;
        std::optional<std::pair<Pos3, uint16_t>> continuation;
        uint8_t baselineTargetsReached;
        RailTraffic::TravelTime baselineDistance;
        RailTraffic::TravelTime baselineWeighting;
        bool reservationConflict;
    };

    struct LookaheadNode
    {
        Pos3 pos;
        uint16_t routing;
        uint16_t depth;
        RailTraffic::TravelTime weighting;
        uint8_t numTargetsReached;
    };

    struct RouteScore
    {
        uint8_t numTargetsReached;
        RailTraffic::TravelTime distance;
        RailTraffic::TravelTime weighting;
        bool searchExhausted{};
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

    static StationId getStationId(const Pos3& pos, const uint16_t routing)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
        const auto trackStart = getTrackStart(pos, tad);
        const auto* station = TileManager::get(trackStart).trainStation(tad.id(), tad.cardinalDirection(), trackStart.z / kSmallZStep);
        return station != nullptr && !station->isGhost() && !station->isAiAllocated() ? station->stationId() : StationId::null;
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

    static bool hasResourceConflict(const std::span<const Resource> targets, const Pos3& pos, const uint16_t routing)
    {
        auto hasConflict = false;
        forEachResource(pos, routing, [&](const auto& resource) {
            hasConflict = std::ranges::any_of(targets, [&resource](const auto& target) {
                return resource.pos == target.pos && (resource.conflictMask & target.conflictMask) != 0;
            });
            return !hasConflict;
        });
        return hasConflict;
    }

    template<typename TFunc>
    static void forEachClaimedResource(const VehicleHead& head, TFunc&& func)
    {
        if (head.mode != TransportMode::rail || head.tileX == -1)
        {
            return;
        }

        const Vehicle train(head);
        auto pos = Pos3{ train.tail->tileX, train.tail->tileY, train.tail->tileBaseZ * kSmallZStep };
        auto occupied = true;
        for (const auto handle : RoutingManager::RingView(train.tail->routingHandle))
        {
            const auto routing = RoutingManager::getRouting(handle);
            forEachResource(pos, routing, [&](const auto& resource) {
                func(head, resource, occupied, RoutingManager::isPathReserved(handle));
                return true;
            });
            pos += TrackData::getUnkTrack(routing & Track::AdditionalTaDFlags::basicTaDMask).pos;
            if (handle == head.routingHandle)
            {
                occupied = false;
            }
        }
        for (const auto routing : RoutingManager::getReservedContinuation(head.routingHandle))
        {
            forEachResource(pos, routing, [&](const auto& resource) {
                func(head, resource, false, true);
                return true;
            });
            pos += TrackData::getUnkTrack(routing & Track::AdditionalTaDFlags::basicTaDMask).pos;
        }
    }

    template<typename TFunc>
    static void forEachClaimedResource(TFunc&& func)
    {
        for (const auto* head : VehicleManager::VehicleList())
        {
            forEachClaimedResource(*head, func);
        }
    }

    static void addCachedResource(const Resource& resource)
    {
        const auto key = getResourceKey(resource.pos);
        auto& counts = _claimedResourceCache.counts[key];
        auto bits = resource.conflictMask;
        while (bits != 0)
        {
            const auto bit = std::countr_zero(bits);
            if (++counts[bit] == 1)
            {
                _claimedResourceCache.masks[key] |= 1U << bit;
            }
            bits &= bits - 1;
        }
    }

    static void removeCachedResource(const Resource& resource)
    {
        const auto key = getResourceKey(resource.pos);
        auto counts = _claimedResourceCache.counts.find(key);
        auto mask = _claimedResourceCache.masks.find(key);
        if (counts == _claimedResourceCache.counts.end() || mask == _claimedResourceCache.masks.end())
        {
            return;
        }
        auto bits = resource.conflictMask;
        while (bits != 0)
        {
            const auto bit = std::countr_zero(bits);
            if (--counts->second[bit] == 0)
            {
                mask->second &= ~(1U << bit);
            }
            bits &= bits - 1;
        }
        if (mask->second == 0)
        {
            _claimedResourceCache.masks.erase(mask);
            _claimedResourceCache.counts.erase(counts);
        }
    }

    static void addCachedReservation(const EntityId vehicle, const Resource& resource)
    {
        auto& claims = _claimedResourceCache.reservations[getResourceKey(resource.pos)];
        const auto existing = std::ranges::find(claims, vehicle, &ReservationClaim::vehicle);
        if (existing != claims.end())
        {
            existing->mask |= resource.conflictMask;
        }
        else
        {
            claims.push_back({ vehicle, resource.conflictMask });
        }
    }

    static void removeCachedReservation(const EntityId vehicle, const Resource& resource)
    {
        const auto key = getResourceKey(resource.pos);
        const auto claims = _claimedResourceCache.reservations.find(key);
        if (claims == _claimedResourceCache.reservations.end())
        {
            return;
        }
        std::erase_if(claims->second, [vehicle](const auto& claim) { return claim.vehicle == vehicle; });
        if (claims->second.empty())
        {
            _claimedResourceCache.reservations.erase(claims);
        }
    }

    static void refreshVehicleClaims(const VehicleHead& head, bool force = false)
    {
        if (!_claimedResourceCache.active)
        {
            return;
        }
        const auto vehicleRef = head.routingHandle.getVehicleRef();
        if (!force && !_claimedResourceCache.dirty[vehicleRef])
        {
            return;
        }
        auto existing = _claimedResourceCache.byVehicle.find(head.id);
        std::vector<VehicleClaim> claims;
        if (existing != _claimedResourceCache.byVehicle.end())
        {
            for (const auto& claim : existing->second)
            {
                removeCachedResource(claim.resource);
                if (claim.pathReserved)
                {
                    removeCachedReservation(head.id, claim.resource);
                }
            }
            claims = std::move(existing->second);
            claims.clear();
        }
        else
        {
            claims.reserve(32);
        }

        forEachClaimedResource(head, [&claims](const auto&, const auto& resource, const bool, const bool pathReserved) {
            claims.push_back({ resource, pathReserved });
        });
        for (const auto& claim : claims)
        {
            addCachedResource(claim.resource);
            if (claim.pathReserved)
            {
                addCachedReservation(head.id, claim.resource);
            }
        }
        if (existing != _claimedResourceCache.byVehicle.end())
        {
            existing->second = std::move(claims);
        }
        else if (!claims.empty())
        {
            _claimedResourceCache.byVehicle.emplace(head.id, std::move(claims));
        }
        _claimedResourceCache.dirty[vehicleRef] = false;
    }

    static void refreshDirtyVehicleClaims()
    {
        for (const auto vehicleRef : _claimedResourceCache.dirtyVehicles)
        {
            if (_claimedResourceCache.dirty[vehicleRef])
            {
                refreshVehicleClaims(_claimedResourceCache.owners[vehicleRef], vehicleRef);
            }
        }
        _claimedResourceCache.dirtyVehicles.clear();
    }

    static const ResourceMasks& getClaimedResourceMask()
    {
        if (_claimedResourceCache.active)
        {
            refreshDirtyVehicleClaims();
            return _claimedResourceCache.masks;
        }
        auto& claimed = _claimedResourceCache.fallback;
        claimed.clear();
        forEachClaimedResource([&](const auto&, const auto& resource, const bool, const bool) {
            claimed[getResourceKey(resource.pos)] |= resource.conflictMask;
        });
        return claimed;
    }

    void beginTick()
    {
        _claimedResourceCache.active = true;
        _claimedResourceCache.masks.clear();
        _claimedResourceCache.counts.clear();
        _claimedResourceCache.reservations.clear();
        _claimedResourceCache.masks.reserve(4096);
        _claimedResourceCache.counts.reserve(4096);
        _claimedResourceCache.byVehicle.reserve(256);
        _claimedResourceCache.reservations.reserve(4096);
        for (auto& [vehicle, claims] : _claimedResourceCache.byVehicle)
        {
            claims.clear();
        }
        _claimedResourceCache.dirty.fill(false);
        _claimedResourceCache.owners.fill(EntityId::null);
        _claimedResourceCache.dirtyVehicles.clear();
        _claimedResourceCache.dirtyVehicles.reserve(256);
        for (const auto* head : VehicleManager::VehicleList())
        {
            _claimedResourceCache.owners[head->routingHandle.getVehicleRef()] = head->id;
            auto& claims = _claimedResourceCache.byVehicle[head->id];
            claims.reserve(32);
            forEachClaimedResource(*head, [&claims](const auto&, const auto& resource, const bool, const bool pathReserved) {
                claims.push_back({ resource, pathReserved });
            });
            for (const auto& claim : claims)
            {
                addCachedResource(claim.resource);
                if (claim.pathReserved)
                {
                    addCachedReservation(head->id, claim.resource);
                }
            }
        }
        std::erase_if(_claimedResourceCache.byVehicle, [](const auto& item) { return item.second.empty(); });
    }

    void markVehicleClaimsDirty(const uint16_t vehicleRef)
    {
        if (_claimedResourceCache.active && vehicleRef < _claimedResourceCache.dirty.size())
        {
            if (!_claimedResourceCache.dirty[vehicleRef])
            {
                _claimedResourceCache.dirty[vehicleRef] = true;
                _claimedResourceCache.dirtyVehicles.push_back(vehicleRef);
            }
        }
    }

    void refreshVehicleClaims(const EntityId vehicle, const uint16_t vehicleRef)
    {
        if (!_claimedResourceCache.active || vehicleRef >= _claimedResourceCache.dirty.size() || !_claimedResourceCache.dirty[vehicleRef])
        {
            return;
        }
        const auto* head = EntityManager::get<VehicleHead>(vehicle);
        if (head != nullptr)
        {
            const auto currentVehicleRef = head->routingHandle.getVehicleRef();
            _claimedResourceCache.owners[currentVehicleRef] = vehicle;
            refreshVehicleClaims(*head, currentVehicleRef != vehicleRef);
            _claimedResourceCache.dirty[vehicleRef] = false;
            return;
        }
        const auto existing = _claimedResourceCache.byVehicle.find(vehicle);
        if (existing != _claimedResourceCache.byVehicle.end())
        {
            for (const auto& claim : existing->second)
            {
                removeCachedResource(claim.resource);
                if (claim.pathReserved)
                {
                    removeCachedReservation(vehicle, claim.resource);
                }
            }
            _claimedResourceCache.byVehicle.erase(existing);
        }
        _claimedResourceCache.dirty[vehicleRef] = false;
    }

    void endTick()
    {
        _claimedResourceCache.active = false;
    }

    std::vector<ClaimedResource> getClaimedResources()
    {
        std::vector<ClaimedResource> result;
        forEachClaimedResource([&result](const auto& head, const auto& resource, const bool occupied, const bool pathReserved) {
            result.push_back({ head.id, resource.pos, resource.conflictMask, occupied, pathReserved });
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

    static RailTraffic::TravelTime getDistanceToTarget(const Pos3& pos, const Target& target, const RailTraffic::SpeedProfile& speedProfile)
    {
        if (!target.hasTarget)
        {
            return 0;
        }
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
        return RailTraffic::getHeuristicTime(speedProfile, std::min(getDistance(target.pos), getDistance(target.reversePos)) * 2 / 5);
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

    static RailTraffic::TravelTime getDistanceToActiveTarget(const Pos3& pos, const Target& target, const Target* nextTarget, const uint8_t numTargetsReached, const RailTraffic::SpeedProfile& speedProfile)
    {
        const auto* activeTarget = getActiveTarget(target, nextTarget, numTargetsReached);
        return activeTarget == nullptr ? 0 : getDistanceToTarget(pos, *activeTarget, speedProfile);
    }

    static uint8_t advanceTargets(const Pos3& pos, const uint16_t routing, const Target& target, const Target* nextTarget, const uint8_t numTargetsReached)
    {
        auto progress = numTargetsReached;
        while (const auto* activeTarget = getActiveTarget(target, nextTarget, progress))
        {
            const auto reached = activeTarget->stationId == StationId::null
                ? reachesWaypoint(pos, routing, *activeTarget)
                : activeTarget->stationId == getStationId(pos, routing);
            if (!reached)
            {
                break;
            }
            progress++;
        }
        return progress;
    }

    static std::vector<uint16_t> getPath(const std::vector<SearchNode>& nodes, uint16_t index)
    {
        std::vector<uint16_t> path;
        path.reserve(nodes[index].depth);
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
        pathResources.reserve(path.size() * 2);
        std::vector<Resource> resources;
        resources.reserve(4);
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

    static RailTraffic::TravelTime addWeighting(const RailTraffic::TravelTime lhs, const RailTraffic::TravelTime rhs)
    {
        return rhs > std::numeric_limits<RailTraffic::TravelTime>::max() - lhs
            ? std::numeric_limits<RailTraffic::TravelTime>::max()
            : lhs + rhs;
    }

    static RailTraffic::TravelTime getRoutingWeighting(const Pos3& pos, const uint16_t routing, const uint8_t trackType, const bool includeTrackWeighting, const ResourceMasks* claimed, const RailTraffic::SpeedProfile& speedProfile)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
        auto weighting = includeTrackWeighting ? RailTraffic::getTravelTime(speedProfile, pos, routing, trackType) : 0;
        if (claimed == nullptr)
        {
            return weighting;
        }
        if (isClaimed(pos, routing, *claimed))
        {
            return addWeighting(weighting, RailTraffic::getHeuristicTime(speedProfile, kClaimedRoutingPenalty));
        }
        if ((routing & Track::AdditionalTaDFlags::hasSignal) != 0
            && (getSignalState(pos, tad, trackType, 0) & SignalStateFlags::occupied) != SignalStateFlags::none)
        {
            weighting = addWeighting(weighting, RailTraffic::getLiveSignalPenalty(speedProfile, routing, trackType));
        }
        return weighting;
    }

    bool isPathReserved(const Pos3& pos, const uint16_t routing)
    {
        std::vector<Resource> targetResources;
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
                if (hasResourceConflict(targetResources, routePos, nextRouting))
                {
                    return true;
                }
                previousRouting = nextRouting;
            }
            routePos += TrackData::getUnkTrack(previousRouting & Track::AdditionalTaDFlags::basicTaDMask).pos;
            for (const auto routing : RoutingManager::getReservedContinuation(head->routingHandle))
            {
                if (hasResourceConflict(targetResources, routePos, routing))
                {
                    return true;
                }
                routePos += TrackData::getUnkTrack(routing & Track::AdditionalTaDFlags::basicTaDMask).pos;
            }
        }
        return false;
    }

    bool hasPathReservationConflict(const EntityId vehicle, const Pos3& pos, const uint16_t routing)
    {
        if (_claimedResourceCache.active)
        {
            refreshDirtyVehicleClaims();
            auto hasConflict = false;
            forEachResource(pos, routing, [&](const auto& resource) {
                const auto claims = _claimedResourceCache.reservations.find(getResourceKey(resource.pos));
                hasConflict = claims != _claimedResourceCache.reservations.end()
                    && std::ranges::any_of(claims->second, [&](const auto& claim) {
                                  return claim.vehicle != vehicle && (claim.mask & resource.conflictMask) != 0;
                              });
                return !hasConflict;
            });
            return hasConflict;
        }
        if (!RoutingManager::hasPathReservations())
        {
            return false;
        }
        std::vector<Resource> targetResources;
        appendResources(targetResources, pos, routing);
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
                if (RoutingManager::isPathReserved(handle) && hasResourceConflict(targetResources, routePos, claimedRouting))
                {
                    return true;
                }
                routePos += TrackData::getUnkTrack(claimedRouting & Track::AdditionalTaDFlags::basicTaDMask).pos;
            }
            for (const auto continuationRouting : RoutingManager::getReservedContinuation(head->routingHandle))
            {
                if (hasResourceConflict(targetResources, routePos, continuationRouting))
                {
                    return true;
                }
                routePos += TrackData::getUnkTrack(continuationRouting & Track::AdditionalTaDFlags::basicTaDMask).pos;
            }
        }
        return false;
    }

    static bool isBetterCandidate(const Candidate& candidate, const Candidate& current)
    {
        if (candidate.reservationConflict != current.reservationConflict)
        {
            return !candidate.reservationConflict;
        }
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

    static std::optional<RouteScore> getLookaheadScore(
        const Pos3& firstPos,
        const uint16_t firstRouting,
        const VehicleHead& head,
        const uint8_t requiredMods,
        const uint8_t queryMods,
        const Target& target,
        const Target* nextTarget,
        const uint8_t initialTargetsReached,
        const ResourceMasks* claimed,
        const RailTraffic::SpeedProfile& speedProfile)
    {
        struct PendingNode
        {
            RailTraffic::TravelTime estimatedCost;
            uint16_t index;
        };
        const auto comparePending = [](const PendingNode& lhs, const PendingNode& rhs) {
            return std::tie(lhs.estimatedCost, lhs.index) > std::tie(rhs.estimatedCost, rhs.index);
        };

        const auto seedProgress = advanceTargets(firstPos, firstRouting, target, nextTarget, initialTargetsReached);
        const auto seedWeighting = getRoutingWeighting(firstPos, firstRouting, head.trackType, target.hasTarget, claimed, speedProfile);
        const auto seedDistance = getDistanceToActiveTarget(firstPos, target, nextTarget, seedProgress, speedProfile);
        std::vector<LookaheadNode> nodes;
        nodes.reserve(kMaxLookaheadNodes);
        nodes.push_back({ firstPos, firstRouting, 1, seedWeighting, seedProgress });
        std::vector<PendingNode> pendingStorage;
        pendingStorage.reserve(kMaxLookaheadNodes);
        std::priority_queue<PendingNode, std::vector<PendingNode>, decltype(comparePending)> pending(comparePending, std::move(pendingStorage));
        pending.push({ addWeighting(seedWeighting, seedDistance), 0 });

        std::unordered_map<uint64_t, RailTraffic::TravelTime> bestWeightingByRoute;
        bestWeightingByRoute.reserve(kMaxLookaheadNodes);
        const auto getRouteKey = [](const Pos3& pos, const uint16_t routing, const uint8_t numTargetsReached) {
            return getResourceKey(pos)
                | (static_cast<uint64_t>(routing & Track::AdditionalTaDFlags::basicTaDMask) << 48)
                | (static_cast<uint64_t>(numTargetsReached) << 57);
        };
        bestWeightingByRoute[getRouteKey(firstPos, firstRouting, seedProgress)] = seedWeighting;

        std::optional<RouteScore> bestReached;
        std::optional<RouteScore> bestFallback;
        uint16_t bestFallbackDepth = 0;
        bool searchExhausted = false;
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
            if (target.hasTarget && getActiveTarget(target, nextTarget, node.numTargetsReached) == nullptr)
            {
                return RouteScore{ node.numTargetsReached, 0, node.weighting, searchExhausted };
            }

            TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
            tad._data = node.routing & Track::AdditionalTaDFlags::basicTaDMask;
            const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, tad._data);
            const auto connections = Track::getTrackConnections(nextPos, nextRotation, head.owner, head.trackType, requiredMods, queryMods);
            const auto numTargetsReached = node.numTargetsReached;
            const auto distance = getDistanceToActiveTarget(nextPos, target, nextTarget, numTargetsReached, speedProfile);
            if (connections.connections.empty())
            {
                considerScore({ numTargetsReached, distance, node.weighting }, node.depth);
                continue;
            }
            considerScore({ numTargetsReached, distance, node.weighting }, node.depth);

            for (const auto routing : connections.connections)
            {
                TrackAndDirection::_TrackAndDirection nextTad{ 0, 0 };
                nextTad._data = routing & Track::AdditionalTaDFlags::basicTaDMask;
                const auto nextSignalMode = getSignalMode(nextPos, nextTad, head.trackType, 0);
                if (!nextSignalMode.has_value() && getSignalState(nextPos, nextTad, head.trackType, 0) != SignalStateFlags::none)
                {
                    continue;
                }

                const auto childWeighting = addWeighting(node.weighting, getRoutingWeighting(nextPos, routing, head.trackType, target.hasTarget, claimed, speedProfile));
                const auto childProgress = advanceTargets(nextPos, routing, target, nextTarget, numTargetsReached);
                const auto childKey = getRouteKey(nextPos, routing, childProgress);
                const auto previousWeighting = bestWeightingByRoute.find(childKey);
                if (previousWeighting != bestWeightingByRoute.end() && previousWeighting->second <= childWeighting)
                {
                    continue;
                }
                if (node.depth >= kMaxLookaheadDepth || nodes.size() >= kMaxLookaheadNodes)
                {
                    searchExhausted = true;
                    continue;
                }
                bestWeightingByRoute[childKey] = childWeighting;

                const auto childDistance = getDistanceToActiveTarget(nextPos, target, nextTarget, childProgress, speedProfile);
                nodes.push_back({ nextPos, routing, static_cast<uint16_t>(node.depth + 1), childWeighting, childProgress });
                const auto childIndex = static_cast<uint16_t>(nodes.size() - 1);
                pending.push({ addWeighting(childWeighting, childDistance), childIndex });
            }
        }

        auto score = bestReached.has_value()
            ? *bestReached
            : bestFallback.value_or(RouteScore{ seedProgress, seedDistance, seedWeighting });
        if (target.hasTarget && initialTargetsReached == 0 && score.numTargetsReached == 0 && !searchExhausted)
        {
            return std::nullopt;
        }
        score.searchExhausted = searchExhausted;
        return score;
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
        return count > RoutingManager::kRequiredFreeRoutingSlots ? count - RoutingManager::kRequiredFreeRoutingSlots : 0;
    }

    std::optional<uint16_t> tryReservePath(VehicleHead& head, const Pos3& firstPos, const uint16_t preferredRouting, const std::span<const uint16_t> firstRoutings)
    {
        const Vehicle train(head);
        // Placement and reversal advance the head while laying out the consist.
        if (head.var_52 != 1 && train.veh1->routingHandle != head.routingHandle)
        {
            return std::nullopt;
        }
        const auto maxPathLength = getAvailableRoutingSlots(head);
        if (maxPathLength == 0)
        {
            return std::nullopt;
        }
        if (!RoutingManager::getReservedContinuation(head.routingHandle).empty())
        {
            return std::nullopt;
        }

        refreshVehicleClaims(head);
        const auto requiredMods = head.var_53;
        const auto queryMods = train.veh1->var_49;
        const auto [target, nextTarget] = getTargets(head);
        const auto* nextTargetPtr = nextTarget.has_value() ? &*nextTarget : nullptr;
        const auto speedProfile = RailTraffic::getSpeedProfile(head);
        const auto& claimed = getClaimedResourceMask();
        const ResourceMasks noClaims;

        std::vector<Candidate> candidates;
        candidates.reserve(kMaxReservationCandidates);
        const auto appendCandidates = [&](const uint16_t firstRouting) {
            auto searchTruncated = false;
            std::vector<SearchNode> nodes;
            nodes.reserve(kMaxSearchNodes);
            nodes.push_back({ firstPos, firstRouting, kNoParent, 1, 0, 0 });
            std::vector<uint16_t> pending{ 0 };
            pending.reserve(kMaxSearchNodes);
            const auto firstCandidate = candidates.size();

            const auto considerCandidate = [&](const uint16_t tailIndex, const Pos3& endpoint, const uint8_t numTargetsReached, const RailTraffic::TravelTime weighting, std::optional<std::pair<Pos3, uint16_t>> continuation) {
                auto path = getPath(nodes, tailIndex);
                if (path.empty() || hasConflict(path, firstPos, noClaims))
                {
                    return;
                }
                if (target.hasTarget && numTargetsReached == 0 && !continuation.has_value())
                {
                    return;
                }
                const auto distance = getDistanceToActiveTarget(endpoint, target, nextTargetPtr, numTargetsReached, speedProfile);
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
                const auto signalMode = getSignalMode(node.pos, tad, head.trackType, 0);
                if (index != 0 && signalMode.has_value())
                {
                    considerCandidate(node.parent, node.pos, node.numTargetsReached, nodes[node.parent].weighting, std::pair{ node.pos, node.routing });
                    continue;
                }
                if (index == 0 && !signalMode.has_value()
                    && getSignalState(node.pos, tad, head.trackType, 0) != SignalStateFlags::none)
                {
                    continue;
                }

                node.numTargetsReached = advanceTargets(node.pos, node.routing, target, nextTargetPtr, node.numTargetsReached);
                node.weighting = addWeighting(node.weighting, RailTraffic::getTravelTime(speedProfile, node.pos, node.routing, head.trackType));
                nodes[index] = node;

                const auto [nextPos, nextRotation] = Track::getTrackConnectionEnd(node.pos, tad._data);
                const auto connections = Track::getTrackConnections(nextPos, nextRotation, head.owner, head.trackType, requiredMods, queryMods);
                const auto numTargetsReached = node.numTargetsReached;
                if (connections.connections.empty())
                {
                    considerCandidate(index, nextPos, numTargetsReached, node.weighting, std::nullopt);
                    continue;
                }
                if (nodes.size() >= kMaxSearchNodes)
                {
                    searchTruncated = true;
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
                    nodes.push_back({ nextPos, routing, index, static_cast<uint16_t>(node.depth + 1), node.weighting, numTargetsReached });
                    pending.push_back(static_cast<uint16_t>(nodes.size() - 1));
                    if (nodes.size() == kMaxSearchNodes)
                    {
                        searchTruncated = true;
                        break;
                    }
                }
            }

            const auto getContinuationKey = [](const Candidate& candidate) {
                const auto pos = candidate.continuation.has_value() ? candidate.continuation->first : Pos3{};
                const auto routing = candidate.continuation.has_value() ? candidate.continuation->second : 0;
                return std::tuple{ candidate.continuation.has_value(), pos.x, pos.y, pos.z, routing, candidate.numTargetsReached, candidate.reservationConflict };
            };
            const auto hasSameContinuation = [&getContinuationKey](const Candidate& lhs, const Candidate& rhs) {
                return getContinuationKey(lhs) == getContinuationKey(rhs);
            };
            std::sort(candidates.begin() + firstCandidate, candidates.end(), [&](const Candidate& lhs, const Candidate& rhs) {
                if (!hasSameContinuation(lhs, rhs))
                {
                    return getContinuationKey(lhs) < getContinuationKey(rhs);
                }
                return isBetterCandidate(lhs, rhs);
            });
            candidates.erase(std::unique(candidates.begin() + firstCandidate, candidates.end(), hasSameContinuation), candidates.end());
            return searchTruncated;
        };

        const auto scoreBaselineCandidates = [&](const size_t firstCandidate) {
            for (auto& candidate : std::span{ candidates }.subspan(firstCandidate))
            {
                if (!candidate.continuation.has_value())
                {
                    continue;
                }

                const auto& [continuationPos, continuationRouting] = *candidate.continuation;
                // Compare unclaimed and claimed lookahead over the same targets.
                const auto* baselineNextTarget = nextTargetPtr;
                if (getActiveTarget(target, baselineNextTarget, candidate.baselineTargetsReached) != nullptr || !target.hasTarget)
                {
                    const auto baselineScore = getLookaheadScore(continuationPos, continuationRouting, head, requiredMods, queryMods, target, baselineNextTarget, candidate.baselineTargetsReached, nullptr, speedProfile);
                    if (!baselineScore.has_value())
                    {
                        candidate.baselineDistance = std::numeric_limits<RailTraffic::TravelTime>::max();
                        candidate.baselineWeighting = std::numeric_limits<RailTraffic::TravelTime>::max();
                        candidate.reservationConflict = true;
                        continue;
                    }
                    candidate.baselineTargetsReached = baselineScore->numTargetsReached;
                    candidate.baselineDistance = baselineScore->distance;
                    candidate.baselineWeighting = addWeighting(candidate.baselineWeighting, baselineScore->weighting);
                }
            }
        };

        const auto truncateCandidates = [&](const size_t firstCandidate) {
            auto candidateRange = std::span{ candidates }.subspan(firstCandidate);
            if (candidateRange.size() <= kMaxReservationCandidates)
            {
                return;
            }

            const auto baseline = std::ranges::min_element(candidateRange, isBetterBaselineCandidate);
            const auto numAvailable = static_cast<size_t>(std::ranges::count(candidateRange, false, &Candidate::reservationConflict));
            const auto conflictedBaseline = baseline->reservationConflict && numAvailable >= kMaxReservationCandidates
                ? std::optional<Candidate>{ *baseline }
                : std::nullopt;
            std::sort(candidates.begin() + firstCandidate, candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
                if (lhs.reservationConflict != rhs.reservationConflict)
                {
                    return !lhs.reservationConflict;
                }
                return isBetterBaselineCandidate(lhs, rhs);
            });
            candidates.erase(candidates.begin() + firstCandidate + kMaxReservationCandidates, candidates.end());
            if (conflictedBaseline.has_value())
            {
                candidates.push_back(*conflictedBaseline);
            }
        };

        const auto scoreClaimedCandidates = [&](const size_t firstCandidate) {
            for (auto& candidate : std::span{ candidates }.subspan(firstCandidate))
            {
                if (!candidate.continuation.has_value() || candidate.reservationConflict)
                {
                    continue;
                }

                const auto& [continuationPos, continuationRouting] = *candidate.continuation;
                if (getActiveTarget(target, nextTargetPtr, candidate.numTargetsReached) != nullptr || !target.hasTarget)
                {
                    const auto score = getLookaheadScore(continuationPos, continuationRouting, head, requiredMods, queryMods, target, nextTargetPtr, candidate.numTargetsReached, &claimed, speedProfile);
                    if (!score.has_value())
                    {
                        candidate.reservationConflict = true;
                        continue;
                    }
                    candidate.numTargetsReached = score->numTargetsReached;
                    candidate.distance = score->distance;
                    candidate.weighting = addWeighting(candidate.weighting, score->weighting);
                }
            }
        };

        bool mustReachCurrentTarget = false;
        const auto requireTargetIfSearchTruncated = [&](const bool searchTruncated, const uint16_t firstRouting, const size_t firstCandidate) {
            if (!searchTruncated || !target.hasTarget
                || std::ranges::any_of(std::span{ candidates }.subspan(firstCandidate), [](const auto& candidate) { return candidate.baselineTargetsReached != 0; }))
            {
                return;
            }
            const auto unrestrictedScore = getLookaheadScore(firstPos, firstRouting, head, requiredMods, queryMods, target, nullptr, 0, nullptr, speedProfile);
            mustReachCurrentTarget |= unrestrictedScore.has_value()
                && (unrestrictedScore->numTargetsReached != 0 || unrestrictedScore->searchExhausted);
        };
        const auto findBestCandidate = [&]() -> const Candidate* {
            if (candidates.empty())
            {
                return nullptr;
            }

            const auto& preferred = *std::ranges::min_element(candidates, isBetterBaselineCandidate);
            if (mustReachCurrentTarget && preferred.baselineTargetsReached == 0)
            {
                return nullptr;
            }
            const auto preferredCost = addWeighting(preferred.baselineWeighting, preferred.baselineDistance);
            const auto waitAllowance = RailTraffic::getHeuristicTime(speedProfile, kMaxReservationDetourWeighting);
            const Candidate* best = nullptr;
            for (const auto& candidate : candidates)
            {
                const auto candidateCost = addWeighting(candidate.baselineWeighting, candidate.baselineDistance);
                if (candidate.reservationConflict
                    || candidate.baselineTargetsReached != preferred.baselineTargetsReached
                    || candidateCost > addWeighting(preferredCost, waitAllowance))
                {
                    continue;
                }
                if (best == nullptr || isBetterCandidate(candidate, *best))
                {
                    best = &candidate;
                }
            }
            return best;
        };

        const auto preferredSearchTruncated = appendCandidates(preferredRouting);
        scoreBaselineCandidates(0);
        truncateCandidates(0);
        scoreClaimedCandidates(0);
        // A search limit must not make a shorter route that misses the current target preferable.
        requireTargetIfSearchTruncated(preferredSearchTruncated, preferredRouting, 0);
        auto* best = findBestCandidate();
        if (best == nullptr)
        {
            const auto preferredBasicTaD = preferredRouting & Track::AdditionalTaDFlags::basicTaDMask;
            for (const auto firstRouting : firstRoutings)
            {
                if ((firstRouting & Track::AdditionalTaDFlags::basicTaDMask) != preferredBasicTaD)
                {
                    const auto firstCandidate = candidates.size();
                    const auto searchTruncated = appendCandidates(firstRouting);
                    scoreBaselineCandidates(firstCandidate);
                    truncateCandidates(firstCandidate);
                    scoreClaimedCandidates(firstCandidate);
                    requireTargetIfSearchTruncated(searchTruncated, firstRouting, firstCandidate);
                }
            }
            best = findBestCandidate();
            if (best == nullptr)
            {
                return std::nullopt;
            }
        }

        const auto materializedSize = std::min(maxPathLength, best->routings.size());
        auto handle = head.routingHandle;
        for (size_t i = 0; i < materializedSize; ++i)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            if (RoutingManager::getRouting(handle) != RoutingManager::kAllocatedButFreeRouting)
            {
                return std::nullopt;
            }
        }

        std::vector<uint16_t> continuation(best->routings.begin() + materializedSize, best->routings.end());
        RoutingManager::setReservedContinuation(head.routingHandle, std::move(continuation));

        handle = head.routingHandle;
        for (size_t i = 0; i < materializedSize; ++i)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            RoutingManager::setRouting(handle, best->routings[i]);
            RoutingManager::markPathReserved(handle);
        }
        refreshVehicleClaims(head);
        return best->routings.front();
    }

    bool tryReservePath(VehicleHead& head, const Pos3& firstPos, const uint16_t firstRouting)
    {
        return tryReservePath(head, firstPos, firstRouting, {}).has_value();
    }
}
