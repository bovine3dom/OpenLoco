#include "World/TownRoadPruning.h"
#include "Map/BuildingElement.h"
#include "Map/RoadElement.h"
#include "Map/SurfaceElement.h"
#include "Map/Tile.h"
#include "Map/TileManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Objects/BuildingObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/RoadObject.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "World/Industry.h"
#include "World/TownManager.h"
#include <algorithm>
#include <optional>
#include <tuple>

namespace OpenLoco::TownRoadPruning
{
    namespace
    {
        constexpr size_t kMaximumBlockingRoads = 2;
        constexpr size_t kMaximumDependentBuildings = 4;
        constexpr size_t kMaximumRoadVisits = 2048;

        struct RoadGeometry
        {
            World::Pos3 pos;
            uint8_t roadId;
            uint8_t rotation;

            constexpr bool operator==(const RoadGeometry&) const = default;
        };

        struct BuildingId
        {
            World::Pos3 pos;
            uint8_t objectId;
            uint8_t rotation;

            constexpr bool operator==(const BuildingId&) const = default;
        };

        struct AccessSummary
        {
            bool usesRemovedRoad{};
            bool hasSurvivingRoad{};
        };

        bool lessThan(const RoadPieceId& lhs, const RoadPieceId& rhs)
        {
            return std::tie(lhs.pos.x, lhs.pos.y, lhs.pos.z, lhs.roadId, lhs.rotation, lhs.objectId)
                < std::tie(rhs.pos.x, rhs.pos.y, rhs.pos.z, rhs.roadId, rhs.rotation, rhs.objectId);
        }

        RoadGeometry getGeometry(const RoadPieceId& road)
        {
            return { road.pos, road.roadId, road.rotation };
        }

        World::Pos3 getReversePosition(const World::Pos3& pos, const uint16_t tad)
        {
            const auto& roadSize = World::TrackData::getUnkRoad(tad);
            auto reversePos = pos + roadSize.pos;
            if (roadSize.rotationEnd < 12)
            {
                reversePos -= World::Pos3{ World::kRotationOffset[roadSize.rotationEnd], 0 };
            }
            return reversePos;
        }

        RoadGeometry getGeometry(const World::Pos3& pos, const uint16_t tad)
        {
            const auto start = tad & (1U << 2) ? getReversePosition(pos, tad) : pos;
            return { start, static_cast<uint8_t>((tad >> 3) & 0xF), static_cast<uint8_t>(tad & 0x3) };
        }

        bool isCompatibleRoadObject(const RoadObject* roadObj)
        {
            return roadObj != nullptr
                && roadObj->hasFlags(RoadObjectFlags::isRoad)
                && roadObj->hasFlags(RoadObjectFlags::anyRoadTypeCompatible);
        }

        bool isExcluded(const std::vector<RoadPieceId>& roads, const RoadGeometry& geometry)
        {
            return std::ranges::any_of(roads, [&](const auto& road) { return getGeometry(road) == geometry; });
        }

        const World::RoadElement* findRoadElement(const RoadPieceId& road, const World::TrackData::PreviewTrack& piece)
        {
            const auto pos = road.pos + World::Pos3{ Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, road.rotation), piece.z };
            const World::RoadElement* result = nullptr;
            for (const auto& element : World::TileManager::get(pos))
            {
                const auto* candidate = element.as<World::RoadElement>();
                if (candidate == nullptr
                    || candidate->baseHeight() != pos.z
                    || candidate->roadId() != road.roadId
                    || candidate->rotation() != road.rotation
                    || candidate->roadObjectId() != road.objectId
                    || candidate->sequenceIndex() != piece.index)
                {
                    continue;
                }
                if (result != nullptr)
                {
                    return nullptr;
                }
                result = candidate;
            }
            return result;
        }

        bool isCompleteRoad(const RoadPieceId& road)
        {
            std::optional<CompanyId> owner;
            for (const auto& piece : World::TrackData::getRoadPiece(road.roadId))
            {
                const auto* element = findRoadElement(road, piece);
                if (element == nullptr || element->isGhost() || element->isAiAllocated())
                {
                    return false;
                }
                if (owner.has_value() && *owner != element->owner())
                {
                    return false;
                }
                owner = element->owner();
            }
            return true;
        }

        bool hasTramTrack(const World::Pos3& pos, const World::RoadElement& road)
        {
            for (const auto& element : World::TileManager::get(pos))
            {
                const auto* other = element.as<World::RoadElement>();
                if (other == nullptr
                    || other == &road
                    || other->baseZ() >= road.clearZ()
                    || other->clearZ() <= road.baseZ()
                    || other->isGhost()
                    || other->isAiAllocated()
                    || (other->occupiedQuarter() & road.occupiedQuarter()) == 0)
                {
                    continue;
                }
                const auto* roadObj = ObjectManager::get<RoadObject>(other->roadObjectId());
                if (roadObj == nullptr || !roadObj->hasFlags(RoadObjectFlags::isRoad))
                {
                    return true;
                }
            }
            return false;
        }

        bool validateRoad(const TownId townId, const RoadPieceId& road)
        {
            // Keep automatic demolition to simple surface streets; special infrastructure fails closed.
            const auto* roadObj = ObjectManager::get<RoadObject>(road.objectId);
            if (!isCompatibleRoadObject(roadObj) || roadObj->hasFlags(RoadObjectFlags::isOneWay) || !isCompleteRoad(road))
            {
                return false;
            }

            for (const auto& piece : World::TrackData::getRoadPiece(road.roadId))
            {
                const auto pos = road.pos + World::Pos3{ Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, road.rotation), piece.z };
                const auto closestTown = TownManager::getClosestTown(pos);
                if (!closestTown.has_value() || *closestTown != townId)
                {
                    return false;
                }

                const auto* element = findRoadElement(road, piece);
                if (element->hasStationElement()
                    || element->hasBridge()
                    || element->hasLevelCrossing()
                    || element->hasUnk7_10()
                    || element->laneOccupation() != 0)
                {
                    return false;
                }

                const auto* surface = World::TileManager::get(pos).surface();
                if (surface == nullptr || surface->slope() != 0 || surface->baseHeight() != element->baseHeight() || hasTramTrack(pos, *element))
                {
                    return false;
                }
            }
            return true;
        }

        bool hasWaypoint(const RoadPieceId& road)
        {
            for (const auto* head : VehicleManager::VehicleList())
            {
                if (head->mode != TransportMode::road || head->sizeOfOrderTable == 0)
                {
                    continue;
                }
                for (const auto& order : Vehicles::OrderRingView(head->orderTableOffset))
                {
                    const auto* waypoint = order.as<Vehicles::OrderRouteWaypoint>();
                    if (waypoint != nullptr && waypoint->getTrackId() == road.roadId
                        && matchesWaypoint(road, waypoint->getWaypoint(), static_cast<uint16_t>((waypoint->getTrackId() << 3) | waypoint->getDirection())))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        bool isTraversable(const RoadGeometry& geometry)
        {
            const auto& firstPiece = World::TrackData::getRoadPiece(geometry.roadId).front();
            const auto pos = geometry.pos + World::Pos3{ Math::Vector::rotate(World::Pos2{ firstPiece.x, firstPiece.y }, geometry.rotation), firstPiece.z };
            bool found = false;
            for (const auto& element : World::TileManager::get(pos))
            {
                const auto* road = element.as<World::RoadElement>();
                if (road == nullptr
                    || road->baseHeight() != pos.z
                    || road->roadId() != geometry.roadId
                    || road->rotation() != geometry.rotation
                    || road->sequenceIndex() != firstPiece.index
                    || road->isGhost()
                    || road->isAiAllocated())
                {
                    continue;
                }
                const auto* roadObj = ObjectManager::get<RoadObject>(road->roadObjectId());
                if (found
                    || !isCompatibleRoadObject(roadObj)
                    || roadObj->hasFlags(RoadObjectFlags::isOneWay)
                    || !isCompleteRoad({ geometry.pos, geometry.roadId, geometry.rotation, road->roadObjectId() }))
                {
                    return false;
                }
                found = true;
            }
            return found;
        }

        std::optional<std::vector<RoadGeometry>> getConnections(const World::Pos3& pos, const uint8_t rotation, const std::vector<RoadPieceId>& excluded, const bool rejectUnsupported)
        {
            std::vector<RoadGeometry> result;
            const auto connections = World::Track::getRoadConnectionsAll(pos, rotation);
            for (const auto connection : connections.connections)
            {
                const auto tad = static_cast<uint16_t>(connection & World::Track::AdditionalTaDFlags::basicRaDMask);
                const auto geometry = getGeometry(pos, tad);
                if (isExcluded(excluded, geometry))
                {
                    continue;
                }
                if (!isTraversable(geometry))
                {
                    if (rejectUnsupported)
                    {
                        return std::nullopt;
                    }
                    continue;
                }
                if (std::ranges::find(result, geometry) == result.end())
                {
                    result.push_back(geometry);
                }
            }
            return result;
        }

        bool preservesConnectivity(const std::vector<RoadPieceId>& roads)
        {
            // Every surviving road attached to the removed set must remain in one connected component.
            std::vector<RoadGeometry> attachments;
            for (const auto& road : roads)
            {
                const auto tad = static_cast<uint16_t>((road.roadId << 3) | road.rotation);
                const auto& roadSize = World::TrackData::getUnkRoad(tad);
                auto append = [&](const World::Pos3& pos, const uint8_t rotation) {
                    const auto connections = getConnections(pos, rotation, roads, true);
                    if (!connections.has_value())
                    {
                        return false;
                    }
                    for (const auto& geometry : *connections)
                    {
                        if (std::ranges::find(attachments, geometry) == attachments.end())
                        {
                            attachments.push_back(geometry);
                        }
                    }
                    return true;
                };
                if (!append(road.pos, roadSize.rotationBegin))
                {
                    return false;
                }
                const auto end = World::Track::getRoadConnectionEnd(road.pos, tad);
                if (!append(end.nextPos, end.nextRotation))
                {
                    return false;
                }
            }

            if (attachments.size() <= 1)
            {
                return true;
            }

            std::vector<RoadGeometry> frontier{ attachments.front() };
            std::vector<RoadGeometry> visited;
            while (!frontier.empty() && visited.size() < kMaximumRoadVisits)
            {
                const auto geometry = frontier.back();
                frontier.pop_back();
                if (std::ranges::find(visited, geometry) != visited.end())
                {
                    continue;
                }
                visited.push_back(geometry);

                if (std::ranges::all_of(attachments, [&](const auto& attachment) { return std::ranges::find(visited, attachment) != visited.end(); }))
                {
                    return true;
                }

                const auto tad = static_cast<uint16_t>((geometry.roadId << 3) | geometry.rotation);
                const auto& roadSize = World::TrackData::getUnkRoad(tad);
                const auto startConnections = getConnections(geometry.pos, roadSize.rotationBegin, roads, false);
                frontier.insert(frontier.end(), startConnections->begin(), startConnections->end());
                const auto end = World::Track::getRoadConnectionEnd(geometry.pos, tad);
                const auto endConnections = getConnections(end.nextPos, end.nextRotation, roads, false);
                frontier.insert(frontier.end(), endConnections->begin(), endConnections->end());
            }
            return false;
        }

        BuildingId getBuildingId(const World::Pos2& pos, const World::BuildingElement& building)
        {
            return {
                World::Pos3{ pos - World::kOffsets[building.sequenceIndex()], building.baseHeight() },
                building.objectId(),
                building.rotation(),
            };
        }

        const World::BuildingElement* findBuildingElement(const BuildingId& building, const Unk4F9274& offset)
        {
            const auto pos = World::Pos2{ building.pos } + offset.pos;
            const World::BuildingElement* result = nullptr;
            for (const auto& element : World::TileManager::get(pos))
            {
                const auto* candidate = element.as<World::BuildingElement>();
                if (candidate == nullptr
                    || candidate->baseHeight() != building.pos.z
                    || candidate->objectId() != building.objectId
                    || candidate->rotation() != building.rotation
                    || candidate->sequenceIndex() != offset.index)
                {
                    continue;
                }
                if (result != nullptr)
                {
                    return nullptr;
                }
                result = candidate;
            }
            return result;
        }

        bool isRedevelopable(const TownId townId, const BuildingId& building)
        {
            const auto* buildingObj = ObjectManager::get<BuildingObject>(building.objectId);
            if (buildingObj == nullptr
                || buildingObj->hasFlags(BuildingObjectFlags::miscBuilding | BuildingObjectFlags::indestructible | BuildingObjectFlags::isHeadquarters))
            {
                return false;
            }
            for (const auto& offset : getBuildingTileOffsets(buildingObj->hasFlags(BuildingObjectFlags::largeTile)))
            {
                const auto closestTown = TownManager::getClosestTown(World::Pos2{ building.pos } + offset.pos);
                if (!closestTown.has_value() || *closestTown != townId)
                {
                    return false;
                }
                const auto* element = findBuildingElement(building, offset);
                if (element == nullptr || element->isGhost() || element->isAiAllocated() || element->isMiscBuilding() || !element->isConstructed() || element->age() <= 30)
                {
                    return false;
                }
            }
            return true;
        }

        bool buildingWillBeRemoved(const BuildingId& building, const World::Pos3& buildingPos, const int16_t clearHeight, const bool isLarge)
        {
            const auto* buildingObj = ObjectManager::get<BuildingObject>(building.objectId);
            if (buildingObj == nullptr)
            {
                return false;
            }

            const auto oldFootprint = getBuildingTileOffsets(buildingObj->hasFlags(BuildingObjectFlags::largeTile));
            for (const auto& newOffset : getBuildingTileOffsets(isLarge))
            {
                const auto pos = World::Pos2{ buildingPos } + newOffset.pos;
                const auto oldTile = std::ranges::find_if(oldFootprint, [&](const auto& oldOffset) {
                    return World::Pos2{ building.pos } + oldOffset.pos == pos;
                });
                if (oldTile == oldFootprint.end())
                {
                    continue;
                }

                const auto tile = World::TileManager::get(pos);
                const auto* surface = tile.surface();
                if (surface == nullptr)
                {
                    continue;
                }
                const auto baseZ = std::min<World::SmallZ>(surface->baseZ(), buildingPos.z / World::kSmallZStep);
                const auto clearZ = static_cast<World::SmallZ>((buildingPos.z + clearHeight) / World::kSmallZStep);
                const auto* existing = findBuildingElement(building, *oldTile);
                if (existing != nullptr && baseZ < existing->clearZ() && clearZ > existing->baseZ())
                {
                    return true;
                }
            }
            return false;
        }

        bool roadServesBuildingFrom(const BuildingId& building, const bool isLarge, const World::Pos3& roadPos, const uint16_t tad)
        {
            const auto rotation = static_cast<uint8_t>(tad & 0x3);
            for (const auto& nextTo : World::TrackData::getRoadUnkNextTo(tad))
            {
                auto candidate = roadPos + World::Pos3{ Math::Vector::rotate(World::Pos2{ nextTo.pos }, rotation), nextTo.pos.z };
                const auto candidateRotation = static_cast<uint8_t>((nextTo.rotation + rotation) & 0x3);
                if (isLarge)
                {
                    candidate -= World::Pos3{ World::kOffsets[candidateRotation], 0 };
                }
                if (candidate == building.pos)
                {
                    return true;
                }
            }
            return false;
        }

        bool roadServesBuilding(const BuildingId& building, const bool isLarge, const RoadPieceId& road)
        {
            const auto tad = static_cast<uint16_t>((road.roadId << 3) | road.rotation);
            if (roadServesBuildingFrom(building, isLarge, road.pos, tad))
            {
                return true;
            }
            return roadServesBuildingFrom(building, isLarge, getReversePosition(road.pos, tad), tad | (1U << 2));
        }

        std::optional<AccessSummary> getAccess(const BuildingId& building, const std::vector<RoadPieceId>& removedRoads)
        {
            const auto* buildingObj = ObjectManager::get<BuildingObject>(building.objectId);
            if (buildingObj == nullptr)
            {
                return std::nullopt;
            }
            const auto isLarge = buildingObj->hasFlags(BuildingObjectFlags::largeTile);
            const auto footprint = getBuildingTileOffsets(isLarge);
            AccessSummary result;
            for (const auto& offset : footprint)
            {
                const auto pos = World::Pos2{ building.pos } + offset.pos;
                for (uint8_t direction = 0; direction < 4; ++direction)
                {
                    const auto accessPos = pos + World::kRotationOffset[direction];
                    if (std::ranges::any_of(footprint, [&](const auto& other) { return World::Pos2{ building.pos } + other.pos == accessPos; })
                        || !World::validCoords(accessPos))
                    {
                        continue;
                    }
                    for (const auto& element : World::TileManager::get(accessPos))
                    {
                        const auto* road = element.as<World::RoadElement>();
                        if (road == nullptr || road->isGhost() || road->isAiAllocated())
                        {
                            continue;
                        }
                        const auto* roadObj = ObjectManager::get<RoadObject>(road->roadObjectId());
                        if (!isCompatibleRoadObject(roadObj))
                        {
                            continue;
                        }
                        const auto id = getRoadPieceId(accessPos, *road);
                        if (!id.has_value())
                        {
                            return std::nullopt;
                        }
                        if (isCompleteRoad(*id) && roadServesBuilding(building, isLarge, *id))
                        {
                            const auto isRemoved = isExcluded(removedRoads, getGeometry(*id));
                            result.usesRemovedRoad |= isRemoved;
                            result.hasSurvivingRoad |= !isRemoved;
                        }
                    }
                }
            }
            return result;
        }

        std::optional<std::vector<World::Pos3>> getDependentBuildings(const TownId townId, const std::vector<RoadPieceId>& roads, const World::Pos3& buildingPos, const int16_t clearHeight, const bool isLarge)
        {
            std::vector<BuildingId> buildings;
            for (const auto& road : roads)
            {
                for (const auto& piece : World::TrackData::getRoadPiece(road.roadId))
                {
                    const auto roadPos = World::Pos2{ road.pos } + Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, road.rotation);
                    for (uint8_t direction = 0; direction < 4; ++direction)
                    {
                        const auto pos = roadPos + World::kRotationOffset[direction];
                        if (!World::validCoords(pos))
                        {
                            continue;
                        }
                        for (const auto& element : World::TileManager::get(pos))
                        {
                            const auto* existing = element.as<World::BuildingElement>();
                            if (existing == nullptr || existing->isGhost() || existing->isAiAllocated())
                            {
                                continue;
                            }
                            const auto id = getBuildingId(pos, *existing);
                            if (std::ranges::find(buildings, id) == buildings.end())
                            {
                                buildings.push_back(id);
                            }
                        }
                    }
                }
            }

            std::vector<World::Pos3> result;
            for (const auto& building : buildings)
            {
                if (buildingWillBeRemoved(building, buildingPos, clearHeight, isLarge))
                {
                    continue;
                }
                const auto access = getAccess(building, roads);
                if (!access.has_value())
                {
                    return std::nullopt;
                }
                if (access->usesRemovedRoad && !access->hasSurvivingRoad)
                {
                    if (!isRedevelopable(townId, building) || result.size() == kMaximumDependentBuildings)
                    {
                        return std::nullopt;
                    }
                    result.push_back(building.pos);
                }
            }
            return result;
        }

    }

    std::optional<RoadPieceId> getRoadPieceId(const World::Pos2& pos, const World::RoadElement& road)
    {
        if (road.roadId() >= World::TrackData::kRoadPieceCount)
        {
            return std::nullopt;
        }
        const auto pieces = World::TrackData::getRoadPiece(road.roadId());
        if (road.sequenceIndex() >= pieces.size())
        {
            return std::nullopt;
        }
        const auto& piece = pieces[road.sequenceIndex()];
        return RoadPieceId{
            World::Pos3{ pos, road.baseHeight() } - World::Pos3{ Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, road.rotation()), piece.z },
            road.roadId(),
            road.rotation(),
            road.roadObjectId(),
        };
    }

    bool matchesWaypoint(const RoadPieceId& road, const World::Pos3& waypointPos, const uint16_t waypointTad)
    {
        const auto waypoint = getGeometry(waypointPos, waypointTad);
        return waypoint.roadId == road.roadId
            && waypoint.rotation == road.rotation
            && waypoint.pos.x == road.pos.x
            && waypoint.pos.y == road.pos.y
            && waypoint.pos.z / 8 == road.pos.z / 8;
    }

    bool isPotentialBlocker(const World::RoadElement& road)
    {
        if (road.roadId() >= World::TrackData::kRoadPieceCount || road.isGhost() || road.isAiAllocated())
        {
            return false;
        }
        if (road.sequenceIndex() >= World::TrackData::getRoadPiece(road.roadId()).size())
        {
            return false;
        }
        const auto* roadObj = ObjectManager::get<RoadObject>(road.roadObjectId());
        return isCompatibleRoadObject(roadObj)
            && !roadObj->hasFlags(RoadObjectFlags::isOneWay);
    }

    std::optional<Plan> plan(const TownId townId, const World::Pos3& buildingPos, const int16_t clearHeight, const bool isLarge)
    {
        Plan result;
        for (const auto& offset : getBuildingTileOffsets(isLarge))
        {
            const auto pos = World::Pos2{ buildingPos } + offset.pos;
            if (!World::validCoords(pos))
            {
                return std::nullopt;
            }
            const auto tile = World::TileManager::get(pos);
            const auto* surface = tile.surface();
            if (surface == nullptr)
            {
                return std::nullopt;
            }
            const auto baseZ = std::min<World::SmallZ>(surface->baseZ(), buildingPos.z / World::kSmallZStep);
            const auto clearZ = static_cast<World::SmallZ>((buildingPos.z + clearHeight) / World::kSmallZStep);
            for (const auto& element : tile)
            {
                const auto* road = element.as<World::RoadElement>();
                if (road == nullptr || element.isGhost() || baseZ >= element.clearZ() || clearZ <= element.baseZ() || element.occupiedQuarter() == 0)
                {
                    continue;
                }
                const auto id = getRoadPieceId(pos, *road);
                if (!id.has_value())
                {
                    return std::nullopt;
                }
                result.roads.push_back(*id);
            }
        }

        std::ranges::sort(result.roads, lessThan);
        result.roads.erase(std::unique(result.roads.begin(), result.roads.end()), result.roads.end());
        if (result.roads.empty() || result.roads.size() > kMaximumBlockingRoads)
        {
            return std::nullopt;
        }
        if (!std::ranges::all_of(result.roads, [&](const auto& road) { return validateRoad(townId, road) && !hasWaypoint(road); }))
        {
            return std::nullopt;
        }
        if (!preservesConnectivity(result.roads))
        {
            return std::nullopt;
        }
        const auto dependentBuildings = getDependentBuildings(townId, result.roads, buildingPos, clearHeight, isLarge);
        if (!dependentBuildings.has_value())
        {
            return std::nullopt;
        }
        result.buildings = *dependentBuildings;
        return result;
    }

    bool contains(const Plan& plan, const World::Pos2& pos, const World::RoadElement& road)
    {
        const auto id = getRoadPieceId(pos, road);
        return id.has_value() && std::ranges::find(plan.roads, *id) != plan.roads.end();
    }

    GameCommands::RoadRemovalArgs makeRemovalArgs(const RoadPieceId& road)
    {
        const auto& firstPiece = World::TrackData::getRoadPiece(road.roadId).front();
        GameCommands::RoadRemovalArgs args{};
        args.pos = road.pos + World::Pos3{ Math::Vector::rotate(World::Pos2{ firstPiece.x, firstPiece.y }, road.rotation), firstPiece.z };
        args.rotation = road.rotation;
        args.roadId = road.roadId;
        args.sequenceIndex = firstPiece.index;
        args.objectId = road.objectId;
        return args;
    }
}
