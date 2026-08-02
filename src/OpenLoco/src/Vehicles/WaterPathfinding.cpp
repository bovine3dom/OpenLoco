#include "Vehicles/WaterPathfinding.h"

#include "Entities/EntityManager.h"
#include "Map/StationElement.h"
#include "Map/SurfaceElement.h"
#include "Map/Tile.h"
#include "Map/TileManager.h"
#include "Objects/DockObject.h"
#include "Objects/ObjectManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleHead.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include <algorithm>
#include <array>
#include <deque>
#include <iterator>
#include <limits>
#include <optional>
#include <vector>

using namespace OpenLoco::World;

namespace OpenLoco::Vehicles::WaterPathfinding
{
    static constexpr uint32_t kUnreachable = std::numeric_limits<uint32_t>::max();
    static constexpr uint16_t kNoGoal = std::numeric_limits<uint16_t>::max();
    static constexpr size_t kMaxCachedRoutes = 8;

    static constexpr size_t getTileIndex(const TilePos2 pos)
    {
        return static_cast<size_t>(pos.y) * kMapColumns + pos.x;
    }

    static constexpr TilePos2 getTilePos(const size_t index)
    {
        return {
            static_cast<tile_coord_t>(index % kMapColumns),
            static_cast<tile_coord_t>(index / kMapColumns),
        };
    }

    bool isNavigable(const TilePos2 tilePos, const MicroZ waterLevel)
    {
        if (waterLevel == 0 || !validCoords(tilePos))
        {
            return false;
        }

        const auto tile = TileManager::get(tilePos);
        const auto* surfaceEntry = tile.surfaceEntry();
        const auto* surface = surfaceEntry == nullptr ? nullptr : surfaceEntry->as<SurfaceElement>();
        if (surface == nullptr || surface->water() != waterLevel)
        {
            return false;
        }

        for (auto* element = surfaceEntry; !element->isLast();)
        {
            element = element->next();
            if (element->isGhost() || element->isAiAllocated())
            {
                continue;
            }

            const auto clearance = static_cast<int16_t>(element->baseZ()) / kMicroToSmallZStep - waterLevel;
            return clearance >= 1;
        }
        return true;
    }

    static bool isBlocked(const TilePos2 tilePos, std::span<const TilePos2> blockedTiles)
    {
        return std::ranges::find(blockedTiles, tilePos) != blockedTiles.end();
    }

    static constexpr std::array<uint8_t, 4> getDirectionOrder(const uint8_t currentDirection)
    {
        return {
            static_cast<uint8_t>(currentDirection & 3),
            static_cast<uint8_t>((currentDirection + 1) & 3),
            static_cast<uint8_t>((currentDirection + 3) & 3),
            static_cast<uint8_t>((currentDirection + 2) & 3),
        };
    }

    // Reverse BFS fields are expanded lazily and shared by ships with the same destination.
    struct RouteCache
    {
        MicroZ waterLevel;
        std::vector<TilePos2> goals;
        std::vector<TilePos2> blockedTiles;
        std::vector<uint32_t> distances;
        std::vector<uint16_t> goalIndices;
        std::deque<size_t> frontier;
        uint64_t lastUse{};

        RouteCache(const MicroZ level, std::span<const TilePos2> routeGoals, std::span<const TilePos2> routeBlockedTiles = {})
            : distances(kMapSize, kUnreachable)
            , goalIndices(kMapSize, kNoGoal)
        {
            initialise(level, routeGoals, routeBlockedTiles);
        }

        void initialise(const MicroZ level, std::span<const TilePos2> routeGoals, std::span<const TilePos2> routeBlockedTiles = {})
        {
            waterLevel = level;
            goals.assign(routeGoals.begin(), routeGoals.end());
            blockedTiles.assign(routeBlockedTiles.begin(), routeBlockedTiles.end());
            std::ranges::fill(distances, kUnreachable);
            std::ranges::fill(goalIndices, kNoGoal);
            frontier.clear();

            for (size_t i = 0; i < goals.size(); ++i)
            {
                const auto goal = goals[i];
                if (!isNavigable(goal, waterLevel) || isBlocked(goal, blockedTiles))
                {
                    continue;
                }

                const auto index = getTileIndex(goal);
                if (distances[index] != kUnreachable)
                {
                    continue;
                }

                distances[index] = 0;
                goalIndices[index] = static_cast<uint16_t>(i);
                frontier.push_back(index);
            }
        }

        void expandTo(const TilePos2 destination)
        {
            const auto destinationIndex = getTileIndex(destination);
            while (distances[destinationIndex] == kUnreachable && !frontier.empty())
            {
                const auto index = frontier.front();
                frontier.pop_front();

                const auto tilePos = getTilePos(index);
                for (auto direction = 0U; direction < 4; ++direction)
                {
                    const auto nextPos = tilePos + toTileSpace(kRotationOffset[direction]);
                    if (!validCoords(nextPos))
                    {
                        continue;
                    }

                    const auto nextIndex = getTileIndex(nextPos);
                    if (distances[nextIndex] != kUnreachable)
                    {
                        continue;
                    }
                    if (isBlocked(nextPos, blockedTiles) || !isNavigable(nextPos, waterLevel))
                    {
                        continue;
                    }

                    distances[nextIndex] = distances[index] + 1;
                    goalIndices[nextIndex] = goalIndices[index];
                    frontier.push_back(nextIndex);
                }
            }
        }
    };

    static std::vector<RouteCache> _routeCache;
    static std::optional<RouteCache> _detourCache;
    static uint32_t _mapRevision{};
    static uint64_t _useCounter{};

    static void reset()
    {
        _routeCache.clear();
        _detourCache.reset();
        _mapRevision = TileManager::getMapRevision();
        _useCounter = 0;
    }

    static RouteCache& getRoute(const MicroZ waterLevel, std::span<const TilePos2> goals)
    {
        const auto revision = TileManager::getMapRevision();
        if (_mapRevision != revision)
        {
            reset();
        }

        const auto matches = [waterLevel, goals](const RouteCache& route) {
            return route.waterLevel == waterLevel && std::ranges::equal(route.goals, goals);
        };
        auto route = std::ranges::find_if(_routeCache, matches);
        if (route == _routeCache.end())
        {
            if (_routeCache.size() == kMaxCachedRoutes)
            {
                route = std::ranges::min_element(_routeCache, {}, &RouteCache::lastUse);
                route->initialise(waterLevel, goals);
            }
            else
            {
                _routeCache.emplace_back(waterLevel, goals);
                route = std::prev(_routeCache.end());
            }
        }
        route->lastUse = ++_useCounter;
        return *route;
    }

    static RouteCache& getDetourRoute(const MicroZ waterLevel, std::span<const TilePos2> goals, std::span<const TilePos2> blockedTiles)
    {
        const auto matches = _detourCache.has_value()
            && _detourCache->waterLevel == waterLevel
            && std::ranges::equal(_detourCache->goals, goals)
            && std::ranges::equal(_detourCache->blockedTiles, blockedTiles);
        if (!matches)
        {
            if (_detourCache.has_value())
            {
                _detourCache->initialise(waterLevel, goals, blockedTiles);
            }
            else
            {
                _detourCache.emplace(waterLevel, goals, blockedTiles);
            }
        }
        return *_detourCache;
    }

    static SearchResult selectNextTile(
        const RouteCache& route,
        const TilePos2 start,
        std::span<const TilePos2> blockedTiles,
        const uint8_t currentDirection,
        bool& hasStaleRoute)
    {
        SearchResult result{ RouteStatus::unreachable, start, kNoGoal, kUnreachable };
        const auto startIndex = getTileIndex(start);
        const auto distance = route.distances[startIndex];
        if (distance == kUnreachable)
        {
            return result;
        }
        if (distance == 0)
        {
            return { RouteStatus::arrived, start, route.goalIndices[startIndex], 0 };
        }

        bool hasBlockedRoute = false;
        for (const auto direction : getDirectionOrder(currentDirection))
        {
            const auto nextPos = start + toTileSpace(kRotationOffset[direction]);
            if (!validCoords(nextPos))
            {
                continue;
            }

            const auto nextIndex = getTileIndex(nextPos);
            const auto nextDistance = route.distances[nextIndex];
            if (nextDistance == kUnreachable || nextDistance + 1 != distance)
            {
                continue;
            }
            if (!isNavigable(nextPos, route.waterLevel))
            {
                hasStaleRoute = true;
                continue;
            }
            if (isBlocked(nextPos, blockedTiles))
            {
                hasBlockedRoute = true;
                continue;
            }

            return { RouteStatus::found, nextPos, route.goalIndices[nextIndex], distance };
        }

        result.status = hasBlockedRoute ? RouteStatus::temporarilyBlocked : RouteStatus::unreachable;
        result.remainingDistance = distance;
        result.goal = route.goalIndices[startIndex];
        return result;
    }

    static SearchResult findNextTileImpl(
        const TilePos2 start,
        const MicroZ waterLevel,
        std::span<const TilePos2> goals,
        std::span<const TilePos2> blockedTiles,
        const uint8_t currentDirection,
        const bool canRefresh)
    {
        SearchResult result{ RouteStatus::unreachable, start, kNoGoal, kUnreachable };
        if (!isNavigable(start, waterLevel) || goals.empty() || goals.size() >= kNoGoal)
        {
            return result;
        }

        auto& route = getRoute(waterLevel, goals);
        route.expandTo(start);

        bool hasStaleRoute = false;
        result = selectNextTile(route, start, blockedTiles, currentDirection, hasStaleRoute);
        if (hasStaleRoute && canRefresh)
        {
            _routeCache.clear();
            _detourCache.reset();
            return findNextTileImpl(start, waterLevel, goals, blockedTiles, currentDirection, false);
        }
        if (result.status != RouteStatus::temporarilyBlocked)
        {
            return result;
        }

        // Static routes ignore traffic. Build a reusable traffic-aware field only when the
        // shortest-path gradient is blocked.
        auto& detour = getDetourRoute(waterLevel, goals, blockedTiles);
        detour.expandTo(start);
        bool hasStaleDetour = false;
        const auto detourResult = selectNextTile(detour, start, {}, currentDirection, hasStaleDetour);
        if (hasStaleDetour && canRefresh)
        {
            _detourCache.reset();
            return findNextTileImpl(start, waterLevel, goals, blockedTiles, currentDirection, false);
        }
        return detourResult.status == RouteStatus::found ? detourResult : result;
    }

    SearchResult findNextTile(
        const TilePos2 start,
        const MicroZ waterLevel,
        std::span<const TilePos2> goals,
        std::span<const TilePos2> blockedTiles,
        const uint8_t currentDirection)
    {
        return findNextTileImpl(start, waterLevel, goals, blockedTiles, currentDirection, true);
    }

    struct DockTarget
    {
        TilePos2 tile;
        Pos2 headTarget;
        StationId stationId;
        Pos3 stationPos;
        bool occupied;
    };

    static std::vector<DockTarget> getDockTargets(const StationId stationId)
    {
        std::vector<DockTarget> targets;
        const auto* station = StationManager::get(stationId);
        for (auto i = 0U; i < station->stationTileSize; ++i)
        {
            const auto& pos = station->stationTiles[i];
            const auto tile = TileManager::get(pos);
            for (const auto& element : tile)
            {
                const auto* stationElement = element.as<StationElement>();
                if (stationElement == nullptr || stationElement->isGhost() || stationElement->isAiAllocated())
                {
                    continue;
                }
                if (stationElement->baseZ() != pos.z / kSmallZStep
                    || stationElement->stationType() != StationType::docks
                    || stationElement->stationId() != stationId)
                {
                    continue;
                }

                const auto* dockObject = ObjectManager::get<DockObject>(stationElement->objectId());
                const auto boatPos = Math::Vector::rotate(dockObject->boatPosition, stationElement->rotation()) + pos;
                const auto headTarget = boatPos + Pos2{ 32, 32 };
                targets.push_back({
                    toTileSpace(headTarget),
                    headTarget,
                    stationId,
                    Pos3{ pos.x, pos.y, Numerics::floor2(pos.z, 4) },
                    stationElement->isFlag6(),
                });
                break;
            }
        }
        return targets;
    }

    static std::vector<TilePos2> findBlockedNeighbours(const VehicleHead& head, const TilePos2 start)
    {
        std::vector<TilePos2> blockedTiles;
        for (auto direction = 0U; direction < 4; ++direction)
        {
            const auto tilePos = start + toTileSpace(kRotationOffset[direction]);
            if (!validCoords(tilePos))
            {
                continue;
            }

            for (const auto* entity : EntityManager::EntityTileList(toWorldSpace(tilePos)))
            {
                const auto* vehicle = entity->asBase<VehicleBase>();
                if (vehicle == nullptr
                    || vehicle->getTransportMode() != TransportMode::water
                    || (vehicle->getSubType() != VehicleEntityType::body_start && vehicle->getSubType() != VehicleEntityType::head)
                    || vehicle->has38Flags(Flags38::isGhost)
                    || vehicle->head == head.id)
                {
                    continue;
                }
                blockedTiles.push_back(tilePos);
                break;
            }
        }
        return blockedTiles;
    }

    static PathingResult makeResult(const RouteStatus status, const Pos2 target)
    {
        return { status, target, StationId::null, {} };
    }

    static PathingResult findEgressTarget(
        const TilePos2 start,
        const MicroZ waterLevel,
        const uint8_t currentDirection,
        std::span<const TilePos2> blockedTiles)
    {
        bool hasBlockedTarget = false;
        for (const auto direction : getDirectionOrder(currentDirection))
        {
            const auto target = start + toTileSpace(kRotationOffset[direction]);
            if (!isNavigable(target, waterLevel))
            {
                continue;
            }
            if (isBlocked(target, blockedTiles))
            {
                hasBlockedTarget = true;
                continue;
            }
            return makeResult(RouteStatus::found, toWorldSpace(target) + Pos2{ 16, 16 });
        }
        return makeResult(hasBlockedTarget ? RouteStatus::temporarilyBlocked : RouteStatus::unreachable, toWorldSpace(start) + Pos2{ 16, 16 });
    }

    static PathingResult findDockRoute(
        const TilePos2 start,
        const MicroZ waterLevel,
        const uint8_t currentDirection,
        std::span<const TilePos2> blockedTiles,
        std::span<const DockTarget> targets,
        const bool isLeavingDock)
    {
        std::vector<TilePos2> goals;
        goals.reserve(targets.size());
        for (const auto& target : targets)
        {
            goals.push_back(target.tile);
        }

        const auto route = findNextTile(start, waterLevel, goals, blockedTiles, currentDirection);
        if (route.status != RouteStatus::found && route.status != RouteStatus::arrived)
        {
            return makeResult(route.status, toWorldSpace(start) + Pos2{ 16, 16 });
        }
        if (route.status == RouteStatus::arrived && isLeavingDock)
        {
            return findEgressTarget(start, waterLevel, currentDirection, blockedTiles);
        }

        const auto& target = targets[route.goal];
        if ((route.status == RouteStatus::arrived || route.remainingDistance == 1) && target.occupied)
        {
            return makeResult(RouteStatus::temporarilyBlocked, toWorldSpace(start) + Pos2{ 16, 16 });
        }
        if (route.status == RouteStatus::arrived || route.remainingDistance == 1)
        {
            return { RouteStatus::found, target.headTarget, target.stationId, target.stationPos };
        }
        return makeResult(RouteStatus::found, toWorldSpace(route.nextTile) + Pos2{ 16, 16 });
    }

    PathingResult getNextTarget(const VehicleHead& head, const bool isLeavingDock)
    {
        const auto start = toTileSpace(head.position);
        const auto currentTarget = toWorldSpace(start) + Pos2{ 16, 16 };

        const Vehicle train(head);
        const auto& vehicle2 = *train.veh2;
        const auto waterLevel = static_cast<MicroZ>(vehicle2.position.z / kMicroZStep);
        const auto currentDirection = static_cast<uint8_t>(((vehicle2.spriteYaw + 7) >> 4) & 3);
        const auto blockedTiles = findBlockedNeighbours(head, start);

        const auto orders = head.getCurrentOrders();
        const auto currentOrder = orders.begin();
        if (const auto* waypointOrder = currentOrder->as<OrderRouteWaypoint>(); waypointOrder != nullptr)
        {
            const std::array goals = { toTileSpace(waypointOrder->getWaypoint()) };
            const auto route = findNextTile(start, waterLevel, goals, blockedTiles, currentDirection);
            if (route.status == RouteStatus::found)
            {
                return makeResult(RouteStatus::found, toWorldSpace(route.nextTile) + Pos2{ 16, 16 });
            }
            if (route.status == RouteStatus::arrived)
            {
                return findEgressTarget(start, waterLevel, currentDirection, blockedTiles);
            }
            return makeResult(route.status, currentTarget);
        }

        const auto* stationOrder = currentOrder->as<OrderStation>();
        if (stationOrder == nullptr)
        {
            if (isLeavingDock)
            {
                return findEgressTarget(start, waterLevel, currentDirection, blockedTiles);
            }
            return makeResult(RouteStatus::unreachable, currentTarget);
        }

        const auto targets = getDockTargets(stationOrder->getStation());
        if (targets.empty())
        {
            return makeResult(RouteStatus::unreachable, currentTarget);
        }

        std::vector<DockTarget> freeTargets;
        std::ranges::copy_if(targets, std::back_inserter(freeTargets), [](const auto& target) { return !target.occupied; });
        if (!freeTargets.empty())
        {
            const auto result = findDockRoute(start, waterLevel, currentDirection, blockedTiles, freeTargets, isLeavingDock);
            if (result.status != RouteStatus::unreachable)
            {
                return result;
            }
        }
        return findDockRoute(start, waterLevel, currentDirection, blockedTiles, targets, isLeavingDock);
    }
}
