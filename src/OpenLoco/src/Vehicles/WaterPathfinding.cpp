#include "Vehicles/WaterPathfinding.h"

#include "Entities/EntityManager.h"
#include "Map/StationElement.h"
#include "Map/SurfaceElement.h"
#include "Map/Tile.h"
#include "Map/TileManager.h"
#include "Objects/DockObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/VehicleObject.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleHead.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <vector>

using namespace OpenLoco::World;

namespace OpenLoco::Vehicles::WaterPathfinding
{
    static constexpr uint32_t kUnreachable = std::numeric_limits<uint32_t>::max();
    static constexpr uint16_t kNoGoal = std::numeric_limits<uint16_t>::max();
    static constexpr size_t kMaxCachedRoutes = 8;
    static constexpr uint32_t kCardinalStepCost = 1000;
    static constexpr uint32_t kDiagonalStepCost = 1414;
    static constexpr Speed16 kUphillSpeedLimit{ 20 };
    static constexpr Speed16 kDownhillSpeedLimit{ 25 };
    // Integer slope interpolation may differ slightly across an otherwise continuous tile edge.
    static constexpr coord_t kSurfaceSampleTolerance = 2;

    // Ordered to match sprite yaw, starting at west and rotating towards south.
    static constexpr std::array<TilePos2, 8> kDirectionOffsets = {
        TilePos2{ -1, 0 },
        TilePos2{ -1, 1 },
        TilePos2{ 0, 1 },
        TilePos2{ 1, 1 },
        TilePos2{ 1, 0 },
        TilePos2{ 1, -1 },
        TilePos2{ 0, -1 },
        TilePos2{ -1, -1 },
    };

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

    bool isNavigable(const TilePos2 tilePos, const MicroZ waterLevel, const NavigationMode mode)
    {
        if (!validCoords(tilePos))
        {
            return false;
        }

        const auto tile = TileManager::get(tilePos);
        const auto* surfaceEntry = tile.surfaceEntry();
        const auto* surface = surfaceEntry == nullptr ? nullptr : surfaceEntry->as<SurfaceElement>();
        if (surface == nullptr)
        {
            return false;
        }

        SmallZ traversalZ;
        if (mode == NavigationMode::waterOnly)
        {
            if (waterLevel == 0 || surface->water() != waterLevel)
            {
                return false;
            }
            traversalZ = waterLevel * kMicroToSmallZStep;
        }
        else
        {
            const auto landTop = TileManager::getSurfaceCornerHeight(*surface);
            const auto waterTop = static_cast<SmallZ>(surface->water() * kMicroToSmallZStep);
            if (surface->isSlopeDoubleHeight() && landTop >= waterTop)
            {
                return false;
            }
            traversalZ = std::max(landTop, waterTop);
        }

        for (auto* element = surfaceEntry; !element->isLast();)
        {
            element = element->next();
            if (element->isGhost() || element->isAiAllocated())
            {
                continue;
            }

            return element->baseZ() >= traversalZ + kMicroToSmallZStep;
        }
        return true;
    }

    static bool isBlocked(const TilePos2 tilePos, std::span<const TilePos2> blockedTiles)
    {
        return std::ranges::find(blockedTiles, tilePos) != blockedTiles.end();
    }

    static constexpr bool isDiagonal(const uint8_t direction)
    {
        return (direction & 1) != 0;
    }

    static constexpr uint32_t getStepCost(const uint8_t direction)
    {
        return isDiagonal(direction) ? kDiagonalStepCost : kCardinalStepCost;
    }

    static coord_t getSurfaceHeight(const Pos2 pos)
    {
        const auto height = TileManager::getHeight(pos);
        return std::max(height.landHeight, height.waterHeight);
    }

    Speed16 getAmphibiousSpeedLimit(const Pos2 current, const Pos2 target, const Speed16 maximumSpeed)
    {
        const auto currentHeight = getSurfaceHeight(current);
        const auto targetHeight = getSurfaceHeight(target);
        if (targetHeight > currentHeight)
        {
            return std::min(maximumSpeed, kUphillSpeedLimit);
        }
        if (targetHeight < currentHeight)
        {
            return std::min(maximumSpeed, kDownhillSpeedLimit);
        }
        return maximumSpeed;
    }

    static bool hasContinuousSurface(const TilePos2 start, const TilePos2 destination)
    {
        const auto offset = destination - start;
        const auto startOrigin = toWorldSpace(start);
        const auto destinationOrigin = toWorldSpace(destination);
        const Pos2 startSample = startOrigin + Pos2{
            offset.x < 0 ? 0 : offset.x > 0 ? kTileSize - 1
                                            : kTileSize / 2,
            offset.y < 0 ? 0 : offset.y > 0 ? kTileSize - 1
                                            : kTileSize / 2,
        };
        const Pos2 destinationSample = destinationOrigin + Pos2{
            offset.x < 0 ? kTileSize - 1 : offset.x > 0 ? 0
                                                        : kTileSize / 2,
            offset.y < 0 ? kTileSize - 1 : offset.y > 0 ? 0
                                                        : kTileSize / 2,
        };
        const auto startHeight = getSurfaceHeight(startSample);
        const auto destinationHeight = getSurfaceHeight(destinationSample);
        return std::max(startHeight, destinationHeight) - std::min(startHeight, destinationHeight) <= kSurfaceSampleTolerance;
    }

    static bool hasContinuousDiagonalCorner(const TilePos2 start, const TilePos2 offset)
    {
        const auto corner = toWorldSpace(start) + Pos2{
            offset.x > 0 ? kTileSize : 0,
            offset.y > 0 ? kTileSize : 0,
        };
        const auto sourceX = offset.x > 0 ? -1 : 0;
        const auto destinationX = offset.x > 0 ? 0 : -1;
        const auto sourceY = offset.y > 0 ? -1 : 0;
        const auto destinationY = offset.y > 0 ? 0 : -1;
        const std::array samples = {
            corner + Pos2{ sourceX, sourceY },
            corner + Pos2{ destinationX, sourceY },
            corner + Pos2{ sourceX, destinationY },
            corner + Pos2{ destinationX, destinationY },
        };

        auto minHeight = std::numeric_limits<coord_t>::max();
        auto maxHeight = std::numeric_limits<coord_t>::min();
        for (const auto sample : samples)
        {
            const auto height = getSurfaceHeight(sample);
            minHeight = std::min(minHeight, height);
            maxHeight = std::max(maxHeight, height);
        }
        return maxHeight - minHeight <= kSurfaceSampleTolerance;
    }

    static bool isNavigableCached(const TilePos2 tilePos, const MicroZ waterLevel, const NavigationMode mode, std::span<int8_t> cache)
    {
        if (cache.empty())
        {
            return isNavigable(tilePos, waterLevel, mode);
        }

        auto& result = cache[getTileIndex(tilePos)];
        if (result < 0)
        {
            result = isNavigable(tilePos, waterLevel, mode);
        }
        return result != 0;
    }

    static bool isNavigableTransition(
        const TilePos2 start,
        const uint8_t direction,
        const MicroZ waterLevel,
        const NavigationMode mode,
        std::span<int8_t> navigabilityCache = {})
    {
        const auto offset = kDirectionOffsets[direction];
        const auto destination = start + offset;
        if (!isNavigableCached(destination, waterLevel, mode, navigabilityCache))
        {
            return false;
        }
        if (!isDiagonal(direction))
        {
            return mode == NavigationMode::waterOnly || hasContinuousSurface(start, destination);
        }
        const auto horizontal = start + TilePos2{ offset.x, 0 };
        const auto vertical = start + TilePos2{ 0, offset.y };
        if (!isNavigableCached(horizontal, waterLevel, mode, navigabilityCache)
            || !isNavigableCached(vertical, waterLevel, mode, navigabilityCache))
        {
            return false;
        }
        return mode == NavigationMode::waterOnly
            || (hasContinuousDiagonalCorner(start, offset)
                && hasContinuousSurface(start, horizontal)
                && hasContinuousSurface(start, vertical)
                && hasContinuousSurface(horizontal, destination)
                && hasContinuousSurface(vertical, destination));
    }

    static bool isBlockedTransition(const TilePos2 start, const uint8_t direction, std::span<const TilePos2> blockedTiles)
    {
        const auto offset = kDirectionOffsets[direction];
        if (isBlocked(start + offset, blockedTiles))
        {
            return true;
        }
        if (!isDiagonal(direction))
        {
            return false;
        }
        return isBlocked(start + TilePos2{ offset.x, 0 }, blockedTiles)
            || isBlocked(start + TilePos2{ 0, offset.y }, blockedTiles);
    }

    static constexpr std::array<uint8_t, 8> getDirectionOrder(const uint8_t currentYaw)
    {
        const auto currentDirection = static_cast<uint8_t>(((currentYaw + 3) >> 3) & 7);
        return {
            currentDirection,
            static_cast<uint8_t>((currentDirection + 1) & 7),
            static_cast<uint8_t>((currentDirection + 7) & 7),
            static_cast<uint8_t>((currentDirection + 2) & 7),
            static_cast<uint8_t>((currentDirection + 6) & 7),
            static_cast<uint8_t>((currentDirection + 3) & 7),
            static_cast<uint8_t>((currentDirection + 5) & 7),
            static_cast<uint8_t>((currentDirection + 4) & 7),
        };
    }

    using FrontierEntry = std::tuple<uint32_t, uint16_t, size_t>;
    using Frontier = std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, std::greater<>>;

    // Reverse shortest-path fields are expanded lazily and shared by ships with the same destination.
    struct RouteCache
    {
        MicroZ waterLevel;
        NavigationMode mode;
        std::vector<TilePos2> goals;
        std::vector<TilePos2> blockedTiles;
        std::vector<uint32_t> distances;
        std::vector<uint16_t> goalIndices;
        std::vector<bool> settled;
        Frontier frontier;
        uint64_t lastUse{};

        RouteCache(const MicroZ level, const NavigationMode navigationMode, std::span<const TilePos2> routeGoals, std::span<const TilePos2> routeBlockedTiles = {})
            : distances(kMapSize, kUnreachable)
            , goalIndices(kMapSize, kNoGoal)
            , settled(kMapSize, false)
        {
            initialise(level, navigationMode, routeGoals, routeBlockedTiles);
        }

        void initialise(const MicroZ level, const NavigationMode navigationMode, std::span<const TilePos2> routeGoals, std::span<const TilePos2> routeBlockedTiles = {})
        {
            waterLevel = level;
            mode = navigationMode;
            goals.assign(routeGoals.begin(), routeGoals.end());
            blockedTiles.assign(routeBlockedTiles.begin(), routeBlockedTiles.end());
            std::ranges::fill(distances, kUnreachable);
            std::ranges::fill(goalIndices, kNoGoal);
            std::fill(settled.begin(), settled.end(), false);
            frontier = {};

            for (size_t i = 0; i < goals.size(); ++i)
            {
                const auto goal = goals[i];
                if (!isNavigable(goal, waterLevel, mode) || isBlocked(goal, blockedTiles))
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
                frontier.emplace(0, static_cast<uint16_t>(i), index);
            }
        }

        void expandTo(const TilePos2 destination)
        {
            const auto destinationIndex = getTileIndex(destination);
            if (settled[destinationIndex])
            {
                return;
            }
            std::vector<int8_t> navigabilityCache(kMapSize, -1);
            while (!settled[destinationIndex] && !frontier.empty())
            {
                const auto [distance, goal, index] = frontier.top();
                frontier.pop();
                if (settled[index] || distances[index] != distance || goalIndices[index] != goal)
                {
                    continue;
                }
                settled[index] = true;

                const auto tilePos = getTilePos(index);
                for (auto direction = 0U; direction < kDirectionOffsets.size(); ++direction)
                {
                    const auto nextPos = tilePos + kDirectionOffsets[direction];
                    if (!validCoords(nextPos))
                    {
                        continue;
                    }

                    const auto nextIndex = getTileIndex(nextPos);
                    if (settled[nextIndex])
                    {
                        continue;
                    }

                    const auto nextDistance = distance + getStepCost(direction);
                    if (nextDistance > distances[nextIndex]
                        || (nextDistance == distances[nextIndex] && goal >= goalIndices[nextIndex]))
                    {
                        continue;
                    }
                    if (!isNavigableTransition(tilePos, direction, waterLevel, mode, navigabilityCache)
                        || isBlockedTransition(tilePos, direction, blockedTiles))
                    {
                        continue;
                    }

                    distances[nextIndex] = nextDistance;
                    goalIndices[nextIndex] = goal;
                    frontier.emplace(nextDistance, goal, nextIndex);
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

    static RouteCache& getRoute(const MicroZ waterLevel, const NavigationMode mode, std::span<const TilePos2> goals)
    {
        const auto revision = TileManager::getMapRevision();
        if (_mapRevision != revision)
        {
            reset();
        }

        const auto matches = [waterLevel, mode, goals](const RouteCache& route) {
            return route.waterLevel == waterLevel && route.mode == mode && std::ranges::equal(route.goals, goals);
        };
        auto route = std::ranges::find_if(_routeCache, matches);
        if (route == _routeCache.end())
        {
            if (_routeCache.size() == kMaxCachedRoutes)
            {
                route = std::ranges::min_element(_routeCache, {}, &RouteCache::lastUse);
                route->initialise(waterLevel, mode, goals);
            }
            else
            {
                _routeCache.emplace_back(waterLevel, mode, goals);
                route = std::prev(_routeCache.end());
            }
        }
        route->lastUse = ++_useCounter;
        return *route;
    }

    static RouteCache& getDetourRoute(const MicroZ waterLevel, const NavigationMode mode, std::span<const TilePos2> goals, std::span<const TilePos2> blockedTiles)
    {
        const auto matches = _detourCache.has_value()
            && _detourCache->waterLevel == waterLevel
            && _detourCache->mode == mode
            && std::ranges::equal(_detourCache->goals, goals)
            && std::ranges::equal(_detourCache->blockedTiles, blockedTiles);
        if (!matches)
        {
            if (_detourCache.has_value())
            {
                _detourCache->initialise(waterLevel, mode, goals, blockedTiles);
            }
            else
            {
                _detourCache.emplace(waterLevel, mode, goals, blockedTiles);
            }
        }
        return *_detourCache;
    }

    static SearchResult selectNextTile(
        const RouteCache& route,
        const TilePos2 start,
        std::span<const TilePos2> blockedTiles,
        const uint8_t currentYaw,
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
        for (const auto direction : getDirectionOrder(currentYaw))
        {
            const auto nextPos = start + kDirectionOffsets[direction];
            if (!validCoords(nextPos))
            {
                continue;
            }

            const auto nextIndex = getTileIndex(nextPos);
            const auto nextDistance = route.distances[nextIndex];
            if (nextDistance == kUnreachable
                || nextDistance + getStepCost(direction) != distance
                || route.goalIndices[nextIndex] != route.goalIndices[startIndex])
            {
                continue;
            }
            if (!isNavigableTransition(start, direction, route.waterLevel, route.mode))
            {
                hasStaleRoute = true;
                continue;
            }
            if (isBlockedTransition(start, direction, blockedTiles))
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
        const uint8_t currentYaw,
        const NavigationMode mode,
        const bool canRefresh)
    {
        SearchResult result{ RouteStatus::unreachable, start, kNoGoal, kUnreachable };
        if (!isNavigable(start, waterLevel, mode) || goals.empty() || goals.size() >= kNoGoal)
        {
            return result;
        }

        auto& route = getRoute(waterLevel, mode, goals);
        route.expandTo(start);

        bool hasStaleRoute = false;
        result = selectNextTile(route, start, blockedTiles, currentYaw, hasStaleRoute);
        if (hasStaleRoute && canRefresh)
        {
            _routeCache.clear();
            _detourCache.reset();
            return findNextTileImpl(start, waterLevel, goals, blockedTiles, currentYaw, mode, false);
        }
        if (result.status != RouteStatus::temporarilyBlocked)
        {
            return result;
        }

        // Static routes ignore traffic. Build a reusable traffic-aware field only when the
        // shortest-path gradient is blocked.
        auto& detour = getDetourRoute(waterLevel, mode, goals, blockedTiles);
        detour.expandTo(start);
        bool hasStaleDetour = false;
        const auto detourResult = selectNextTile(detour, start, {}, currentYaw, hasStaleDetour);
        if (hasStaleDetour && canRefresh)
        {
            _detourCache.reset();
            return findNextTileImpl(start, waterLevel, goals, blockedTiles, currentYaw, mode, false);
        }
        return detourResult.status == RouteStatus::found ? detourResult : result;
    }

    SearchResult findNextTile(
        const TilePos2 start,
        const MicroZ waterLevel,
        std::span<const TilePos2> goals,
        std::span<const TilePos2> blockedTiles,
        const uint8_t currentYaw,
        const NavigationMode mode)
    {
        const auto navigationLevel = mode == NavigationMode::waterOnly ? waterLevel : MicroZ{};
        return findNextTileImpl(start, navigationLevel, goals, blockedTiles, currentYaw, mode, true);
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
        for (const auto offset : kDirectionOffsets)
        {
            const auto tilePos = start + offset;
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
        const uint8_t currentYaw,
        const NavigationMode mode,
        std::span<const TilePos2> blockedTiles)
    {
        bool hasBlockedTarget = false;
        for (const auto direction : getDirectionOrder(currentYaw))
        {
            if (!isNavigableTransition(start, direction, waterLevel, mode))
            {
                continue;
            }
            if (isBlockedTransition(start, direction, blockedTiles))
            {
                hasBlockedTarget = true;
                continue;
            }
            const auto target = start + kDirectionOffsets[direction];
            return makeResult(RouteStatus::found, toWorldSpace(target) + Pos2{ 16, 16 });
        }
        return makeResult(hasBlockedTarget ? RouteStatus::temporarilyBlocked : RouteStatus::unreachable, toWorldSpace(start) + Pos2{ 16, 16 });
    }

    static PathingResult findDockRoute(
        const TilePos2 start,
        const MicroZ waterLevel,
        const uint8_t currentYaw,
        const NavigationMode mode,
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

        const auto route = findNextTile(start, waterLevel, goals, blockedTiles, currentYaw, mode);
        if (route.status != RouteStatus::found && route.status != RouteStatus::arrived)
        {
            return makeResult(route.status, toWorldSpace(start) + Pos2{ 16, 16 });
        }
        if (route.status == RouteStatus::arrived && isLeavingDock)
        {
            return findEgressTarget(start, waterLevel, currentYaw, mode, blockedTiles);
        }

        const auto& target = targets[route.goal];
        const auto isAtTarget = route.status == RouteStatus::arrived || route.nextTile == target.tile;
        if (isAtTarget && target.occupied)
        {
            return makeResult(RouteStatus::temporarilyBlocked, toWorldSpace(start) + Pos2{ 16, 16 });
        }
        if (isAtTarget)
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
        const auto mode = !train.cars.empty() && isSrn4HovercraftObject(train.cars.firstCar.body->objectId)
            ? NavigationMode::amphibious
            : NavigationMode::waterOnly;
        const auto waterLevel = static_cast<MicroZ>(vehicle2.position.z / kMicroZStep);
        const auto currentYaw = vehicle2.spriteYaw;
        const auto blockedTiles = findBlockedNeighbours(head, start);

        const auto orders = head.getCurrentOrders();
        const auto currentOrder = orders.begin();
        if (const auto* waypointOrder = currentOrder->as<OrderRouteWaypoint>(); waypointOrder != nullptr)
        {
            const std::array goals = { toTileSpace(waypointOrder->getWaypoint()) };
            const auto route = findNextTile(start, waterLevel, goals, blockedTiles, currentYaw, mode);
            if (route.status == RouteStatus::found)
            {
                return makeResult(RouteStatus::found, toWorldSpace(route.nextTile) + Pos2{ 16, 16 });
            }
            if (route.status == RouteStatus::arrived)
            {
                return findEgressTarget(start, waterLevel, currentYaw, mode, blockedTiles);
            }
            return makeResult(route.status, currentTarget);
        }

        const auto* stationOrder = currentOrder->as<OrderStation>();
        if (stationOrder == nullptr)
        {
            if (isLeavingDock)
            {
                return findEgressTarget(start, waterLevel, currentYaw, mode, blockedTiles);
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
            const auto result = findDockRoute(start, waterLevel, currentYaw, mode, blockedTiles, freeTargets, isLeavingDock);
            if (result.status != RouteStatus::unreachable)
            {
                return result;
            }
        }
        return findDockRoute(start, waterLevel, currentYaw, mode, blockedTiles, targets, isLeavingDock);
    }
}
