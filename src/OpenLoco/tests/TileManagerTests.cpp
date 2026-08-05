#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/Entities/EntityManager.h>
#include <OpenLoco/GameCommands/Track/CreateSignal.h>
#include <OpenLoco/GameCommands/Track/RemoveSignal.h>
#include <OpenLoco/GameCommands/Track/RemoveTrack.h>
#include <OpenLoco/GameState.h>
#include <OpenLoco/Map/RoadElement.h>
#include <OpenLoco/Map/SignalElement.h>
#include <OpenLoco/Map/StationElement.h>
#include <OpenLoco/Map/SurfaceElement.h>
#include <OpenLoco/Map/Tile.h>
#include <OpenLoco/Map/TileElement.h>
#include <OpenLoco/Map/TileManager.h>
#include <OpenLoco/Map/Track/Track.h>
#include <OpenLoco/Map/Track/TrackData.h>
#include <OpenLoco/Map/TrackElement.h>
#include <OpenLoco/Map/TreeElement.h>
#include <OpenLoco/S5/S5GameState.h>
#include <OpenLoco/S5/S5TileElement.h>
#include <OpenLoco/Vehicles/OrderManager.h>
#include <OpenLoco/Vehicles/Orders.h>
#include <OpenLoco/Vehicles/PathSignals.h>
#include <OpenLoco/Vehicles/RailPathfinding.h>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/RoutingManager.h>
#include <OpenLoco/Vehicles/Vehicle.h>
#include <OpenLoco/Vehicles/Vehicle1.h>
#include <OpenLoco/Vehicles/Vehicle2.h>
#include <OpenLoco/Vehicles/VehicleHead.h>
#include <OpenLoco/Vehicles/VehicleTail.h>
#include <OpenLoco/World/Station.h>
#include <OpenLoco/World/StationManager.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace OpenLoco::World;

namespace
{
    class TileManagerTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            TileManager::initialise();
        }

        static ElementType typeAt(Tile tile, size_t index)
        {
            size_t n = 0;
            for (auto& el : tile)
            {
                if (n++ == index)
                {
                    return el.type();
                }
            }
            ADD_FAILURE() << "index " << index << " out of range";
            return ElementType::surface;
        }

        using TileBytes = std::vector<std::array<uint8_t, kTileElementSize>>;

        static TileBytes snapshotBytes(Tile tile)
        {
            TileBytes out;
            for (size_t i = 0; i < tile.size(); ++i)
            {
                std::array<uint8_t, kTileElementSize> bytes{};
                std::memcpy(bytes.data(), tile[i]->rawData().data(), kTileElementSize);
                out.push_back(bytes);
            }
            return out;
        }
    };

    class PathSignalsTest : public ::testing::Test
    {
    protected:
        static constexpr Pos3 kFirstPos{ 320, 320, 32 };
        static constexpr uint16_t kStraightWest = 0;
        static constexpr uint16_t kTurnNorth = 2 << 3;
        OpenLoco::CompanyId _previousUpdatingCompany{};

        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            _previousUpdatingCompany = OpenLoco::GameCommands::getUpdatingCompanyId();
            OpenLoco::GameCommands::setUpdatingCompanyId(OpenLoco::CompanyId(0));
            TileManager::initialise();
            OpenLoco::EntityManager::reset();
            OpenLoco::Vehicles::OrderManager::reset();
            OpenLoco::Vehicles::RoutingManager::resetRoutingTable();
            OpenLoco::Vehicles::RailTraffic::reset();
        }

        void TearDown() override
        {
            OpenLoco::GameCommands::setUpdatingCompanyId(_previousUpdatingCompany);
            OpenLoco::EntityManager::reset();
            OpenLoco::Vehicles::OrderManager::reset();
            OpenLoco::Vehicles::RoutingManager::resetRoutingTable();
            OpenLoco::Vehicles::RailTraffic::reset();
        }

        template<typename T>
        static T* createVehicleComponent()
        {
            auto* base = OpenLoco::EntityManager::createEntityVehicle();
            if (base == nullptr)
            {
                return nullptr;
            }
            base->baseType = OpenLoco::EntityBaseType::vehicle;
            auto* vehicle = base->asBase<OpenLoco::Vehicles::VehicleBase>();
            vehicle->setSubType(T::kVehicleThingType);
            return static_cast<T*>(vehicle);
        }

        static OpenLoco::Vehicles::VehicleHead* createTrain(const Pos3& pos, const uint16_t routing)
        {
            using namespace OpenLoco::Vehicles;

            const auto routingHandle = RoutingManager::getAndAllocateFreeRoutingHandle();
            if (!routingHandle.has_value())
            {
                return nullptr;
            }

            auto* head = createVehicleComponent<VehicleHead>();
            auto* veh1 = createVehicleComponent<Vehicle1>();
            auto* veh2 = createVehicleComponent<Vehicle2>();
            auto* tail = createVehicleComponent<VehicleTail>();
            if (head == nullptr || veh1 == nullptr || veh2 == nullptr || tail == nullptr)
            {
                return nullptr;
            }

            OpenLoco::EntityManager::moveEntityToList(head, OpenLoco::EntityManager::EntityListType::vehicleHead);
            head->setNextCar(veh1->id);
            veh1->setNextCar(veh2->id);
            veh2->setNextCar(tail->id);
            tail->setNextCar(OpenLoco::EntityId::null);

            for (auto* component : std::array<OpenLoco::Vehicles::VehicleBase*, 4>{ head, veh1, veh2, tail })
            {
                component->owner = OpenLoco::CompanyId(0);
                component->head = head->id;
                component->mode = OpenLoco::TransportMode::rail;
                component->trackType = 0;
                component->tileX = pos.x;
                component->tileY = pos.y;
                component->tileBaseZ = pos.z / kSmallZStep;
                component->trackAndDirection = OpenLoco::Vehicles::TrackAndDirection(routing >> 3, routing & 0x7);
                component->routingHandle = *routingHandle;
            }
            veh2->maxSpeed = OpenLoco::Speed16{ 60 };
            veh2->rackRailMaxSpeed = OpenLoco::Speed16{ 60 };
            OpenLoco::Vehicles::RoutingManager::setRouting(*routingHandle, routing);
            OpenLoco::Vehicles::OrderManager::allocateOrders(*head);
            return head;
        }

        static void addTrack(const Pos3& start, const uint8_t trackId, const uint8_t rotation, const bool hasSignal = false, const SignalMode signalMode = SignalMode::block)
        {
            const auto pieces = TrackData::getTrackPiece(trackId);
            for (const auto& piece : pieces)
            {
                const auto offset = OpenLoco::Math::Vector::rotate(Pos2{ piece.x, piece.y }, rotation);
                const auto pos = start + Pos3{ offset, piece.z };
                const auto quarterTile = piece.subTileClearance.rotate(rotation);
                auto* trackEntry = TileManager::insertElement<TrackElement>(pos, pos.z / kSmallZStep, quarterTile.getBaseQuarterOccupied());
                ASSERT_NE(trackEntry, nullptr);
                auto& track = trackEntry->get<TrackElement>();
                track.setClearZ(track.baseZ() + 8);
                track.setRotation(rotation);
                track.setTrackObjectId(0);
                track.setSequenceIndex(piece.index);
                track.setTrackId(trackId);
                track.setOwner(OpenLoco::CompanyId(0));
                track.setFlag6(piece.index == pieces.size() - 1);
                if (!hasSignal)
                {
                    continue;
                }

                auto* signalEntry = TileManager::insertElementAfterNoReorg<SignalElement>(trackEntry, pos, track.baseZ(), quarterTile.getBaseQuarterOccupied());
                ASSERT_NE(signalEntry, nullptr);
                auto& signal = signalEntry->get<SignalElement>();
                signal.setClearZ(track.clearZ());
                signal.setRotation(rotation);
                signal.getLeft() = SignalElement::Side{};
                signal.getLeft().setHasSignal(true);
                track.setHasSignal(true);
                track.setLeftSignalMode(signalMode);
            }
        }

        static void addTrainStation(const Pos3 pos, const uint8_t trackId, const uint8_t rotation, const OpenLoco::StationId stationId)
        {
            auto tile = TileManager::get(pos);
            const auto trackIt = std::ranges::find_if(tile, [trackId, rotation](const auto& entry) {
                const auto* track = entry.template as<TrackElement>();
                return track != nullptr && track->trackId() == trackId && track->rotation() == rotation;
            });
            ASSERT_NE(trackIt, tile.end());
            auto* trackEntry = &*trackIt;
            auto& track = trackEntry->get<TrackElement>();
            track.setHasStationElement(true);
            auto* previousEntry = track.hasSignal() && trackEntry->next()->as<SignalElement>() != nullptr
                ? trackEntry->next()
                : trackEntry;
            auto* stationEntry = TileManager::insertElementAfterNoReorg<StationElement>(previousEntry, pos, track.baseZ(), 0xF);
            ASSERT_NE(stationEntry, nullptr);
            auto& station = stationEntry->get<StationElement>();
            station.setClearZ(track.clearZ());
            station.setOwner(OpenLoco::CompanyId(0));
            station.setStationId(stationId);
            station.setStationType(OpenLoco::StationType::trainStation);
        }

        static void addTwoRouteJunction(const Pos3& firstPos = kFirstPos)
        {
            addTrack(firstPos, 0, 0);
            addTrack(firstPos + Pos3{ -32, 0, 0 }, 0, 0);
            addTrack(firstPos + Pos3{ -64, 0, 0 }, 0, 0);
            addTrack(firstPos + Pos3{ -96, 0, 0 }, 0, 0, true);

            addTrack(firstPos + Pos3{ -32, 0, 0 }, 2, 0);
            addTrack(firstPos + Pos3{ -32, -32, 0 }, 0, 3);
            addTrack(firstPos + Pos3{ -32, -64, 0 }, 0, 3);
            addTrack(firstPos + Pos3{ -32, -96, 0 }, 0, 3, true);
        }

        static uint16_t getSecondReservedRouting(const OpenLoco::Vehicles::VehicleHead& head)
        {
            auto handle = head.routingHandle;
            handle.setIndex(handle.getIndex() + 2);
            return OpenLoco::Vehicles::RoutingManager::getRouting(handle) & OpenLoco::World::Track::AdditionalTaDFlags::basicTaDMask;
        }

        static void recordSlowTrack(const Pos3& pos, const uint16_t tad)
        {
            const OpenLoco::Vehicles::RailTraffic::Edge edge{ pos.x, pos.y, pos.z, tad, 0 };
            for (auto i = 0; i < 16; ++i)
            {
                OpenLoco::Vehicles::RailTraffic::recordTraversal(edge, 200 * OpenLoco::Vehicles::RailTraffic::kOneTick);
            }
        }
    };

    class RailPathfindingTest : public PathSignalsTest
    {
    };

    constexpr TilePos2 kTestTile{ 10, 5 };
    constexpr TilePos2 kOtherTile{ 11, 5 };
}

TEST_F(TileManagerTest, InitialiseLeavesOneSurfacePerTile)
{
    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 1u);
    ASSERT_NE(tile.surface(), nullptr);
    EXPECT_EQ(tile.surface()->baseZ(), 4);
    EXPECT_TRUE(tile.begin()->isLast());
}

TEST_F(TileManagerTest, InsertSingleElementSetsTypeBaseZAndIsLast)
{
    auto* inserted = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(inserted, nullptr);
    EXPECT_EQ(inserted->type(), ElementType::track);
    EXPECT_EQ(inserted->baseZ(), 8);
    EXPECT_TRUE(inserted->isLast());

    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 2u);
}

TEST_F(TileManagerTest, MultipleInsertsAreOrderedByBaseZ)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 24, 0);

    auto tile = TileManager::get(kTestTile);
    std::vector<uint8_t> baseZs;
    std::vector<ElementType> types;
    int lastCount = 0;
    for (auto& el : tile)
    {
        baseZs.push_back(el.baseZ());
        types.push_back(el.type());
        if (el.isLast())
        {
            ++lastCount;
        }
    }
    ASSERT_EQ(baseZs.size(), 4u);
    EXPECT_TRUE(std::ranges::is_sorted(baseZs));
    EXPECT_EQ(types.front(), ElementType::surface);
    EXPECT_EQ(lastCount, 1);
}

TEST_F(TileManagerTest, RemoveMiddleElementShiftsRemainder)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 24, 0);

    TileElementEntry* middle = nullptr;
    for (auto& el : TileManager::get(kTestTile))
    {
        if (el.type() == ElementType::tree)
        {
            middle = &el;
            break;
        }
    }
    ASSERT_NE(middle, nullptr);

    const auto freeBefore = TileManager::numFreeElements();
    TileManager::removeElement(*middle);
    EXPECT_EQ(TileManager::numFreeElements(), freeBefore + 1);

    auto tile = TileManager::get(kTestTile);
    std::vector<ElementType> types;
    int lastCount = 0;
    for (auto& el : tile)
    {
        types.push_back(el.type());
        if (el.isLast())
        {
            ++lastCount;
        }
    }
    ASSERT_EQ(types.size(), 3u);
    EXPECT_EQ(types[0], ElementType::surface);
    EXPECT_EQ(types[1], ElementType::track);
    EXPECT_EQ(types[2], ElementType::track);
    EXPECT_EQ(lastCount, 1);
}

TEST_F(TileManagerTest, RemoveLastElementTransfersIsLastFlag)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    auto* last = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 24, 0);
    ASSERT_NE(last, nullptr);
    ASSERT_TRUE(last->isLast());

    TileManager::setRemoveElementPointerChecker(*last);
    TileManager::removeElement(*last);
    EXPECT_TRUE(TileManager::wasRemoveOnLastElement());

    auto tile = TileManager::get(kTestTile);
    std::vector<uint8_t> baseZs;
    int lastCount = 0;
    uint8_t lastBaseZ = 0;
    for (auto& el : tile)
    {
        baseZs.push_back(el.baseZ());
        if (el.isLast())
        {
            ++lastCount;
            lastBaseZ = el.baseZ();
        }
    }
    ASSERT_EQ(baseZs.size(), 2u);
    EXPECT_EQ(lastCount, 1);
    EXPECT_EQ(lastBaseZ, 8);
}

TEST_F(TileManagerTest, TilesAreIsolatedFromEachOther)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kOtherTile), 16, 0);

    auto a = TileManager::get(kTestTile);
    auto b = TileManager::get(kOtherTile);

    std::vector<ElementType> aTypes;
    for (auto& el : a)
    {
        aTypes.push_back(el.type());
    }
    std::vector<ElementType> bTypes;
    for (auto& el : b)
    {
        bTypes.push_back(el.type());
    }

    ASSERT_EQ(aTypes.size(), 2u);
    ASSERT_EQ(bTypes.size(), 2u);
    EXPECT_EQ(aTypes[0], ElementType::surface);
    EXPECT_EQ(aTypes[1], ElementType::track);
    EXPECT_EQ(bTypes[0], ElementType::surface);
    EXPECT_EQ(bTypes[1], ElementType::tree);
}

TEST_F(TileManagerTest, NumFreeElementsDecreasesMonotonicallyOnInsert)
{
    const auto initial = TileManager::numFreeElements();
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    const auto afterFirst = TileManager::numFreeElements();
    EXPECT_LT(afterFirst, initial);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    const auto afterSecond = TileManager::numFreeElements();
    EXPECT_LT(afterSecond, afterFirst);
}

TEST_F(TileManagerTest, InsertElementPropagatesOccupiedQuads)
{
    auto* inserted = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0b1010);
    ASSERT_NE(inserted, nullptr);
    EXPECT_EQ(inserted->occupiedQuarter(), 0b1010);
}

TEST_F(TileManagerTest, GetOutOfBoundsReturnsNullTile)
{
    auto t = TileManager::get(TilePos2(kMapColumns + 100, kMapRows + 100));
    EXPECT_TRUE(t.isNull());
    EXPECT_EQ(t.begin(), t.end());
}

TEST_F(TileManagerTest, GetByPos2AndCoordsAgreeWithTilePos2)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    auto a = TileManager::get(kTestTile);
    auto b = TileManager::get(toWorldSpace(kTestTile));
    auto c = TileManager::get(toWorldSpace(kTestTile).x, toWorldSpace(kTestTile).y);
    EXPECT_EQ(a.size(), 2u);
    EXPECT_EQ(b.size(), 2u);
    EXPECT_EQ(c.size(), 2u);
    EXPECT_EQ(&*a.begin(), &*b.begin());
    EXPECT_EQ(&*a.begin(), &*c.begin());
}

TEST_F(TileManagerTest, TileSizeMatchesIteration)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    auto tile = TileManager::get(kTestTile);
    size_t walked = 0;
    for ([[maybe_unused]] auto& el : tile)
    {
        ++walked;
    }
    EXPECT_EQ(tile.size(), walked);
    EXPECT_EQ(tile.size(), 3u);
}

TEST_F(TileManagerTest, TileSubscriptMatchesIterationOrder)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    auto tile = TileManager::get(kTestTile);

    std::vector<std::pair<ElementType, uint8_t>> walked;
    for (auto& el : tile)
    {
        walked.emplace_back(el.type(), el.baseZ());
    }
    ASSERT_EQ(walked.size(), tile.size());
    for (size_t i = 0; i < tile.size(); ++i)
    {
        EXPECT_EQ(typeAt(tile, i), walked[i].first);
        EXPECT_EQ(tile[i]->baseZ(), walked[i].second);
    }
}

TEST_F(TileManagerTest, TileIndexOfFindsMemberAndMissesNonMember)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 3u);
    EXPECT_EQ(tile.indexOf(&TileManager::resolveEntry(tile[0])), 0u);
    EXPECT_EQ(tile.indexOf(&TileManager::resolveEntry(tile[1])), 1u);
    EXPECT_EQ(tile.indexOf(&TileManager::resolveEntry(tile[2])), 2u);

    auto otherTile = TileManager::get(kOtherTile);
    EXPECT_EQ(tile.indexOf(&TileManager::resolveEntry(otherTile[0])), Tile::npos);
}

TEST_F(TileManagerTest, BoundaryTilesAreUsable)
{
    constexpr TilePos2 origin{ 0, 0 };
    constexpr TilePos2 corner{ kMapColumns - 1, kMapRows - 1 };
    TileManager::insertElement(ElementType::track, toWorldSpace(origin), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(corner), 16, 0);

    auto a = TileManager::get(origin);
    auto b = TileManager::get(corner);
    EXPECT_EQ(a.size(), 2u);
    EXPECT_EQ(b.size(), 2u);
    EXPECT_EQ(typeAt(a, 1), ElementType::track);
    EXPECT_EQ(typeAt(b, 1), ElementType::tree);
}

TEST_F(TileManagerTest, InsertElementAfterNoReorgPlacesRightAfterTarget)
{
    auto* anchor = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(anchor, nullptr);

    auto* inserted = TileManager::insertElementAfterNoReorg(
        anchor, ElementType::tree, toWorldSpace(kTestTile), 4, 0);
    ASSERT_NE(inserted, nullptr);

    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 3u);

    bool sawAnchor = false;
    bool nextIsTree = false;
    for (auto& el : tile)
    {
        if (sawAnchor)
        {
            nextIsTree = (el.type() == ElementType::tree);
            break;
        }
        if (el.type() == ElementType::track && el.baseZ() == 8)
        {
            sawAnchor = true;
        }
    }
    EXPECT_TRUE(sawAnchor);
    EXPECT_TRUE(nextIsTree);
}

TEST_F(TileManagerTest, CheckFreeElementsAndReorganiseSucceedsWithDefaultSpace)
{
    EXPECT_TRUE(TileManager::checkFreeElementsAndReorganise());
}

TEST_F(TileManagerTest, ReorganisePreservesEveryTouchedTile)
{
    struct Entry
    {
        TilePos2 tile;
        ElementType type;
        uint8_t baseZ;
    };
    const std::vector<Entry> entries{
        { TilePos2{ 5, 5 }, ElementType::track, 8 },
        { TilePos2{ 5, 5 }, ElementType::tree, 16 },
        { TilePos2{ 5, 5 }, ElementType::track, 24 },
        { TilePos2{ 5, 6 }, ElementType::tree, 12 },
        { TilePos2{ 6, 5 }, ElementType::track, 4 },
        { TilePos2{ 6, 5 }, ElementType::tree, 32 },
        { TilePos2{ 50, 50 }, ElementType::track, 8 },
        { TilePos2{ kMapColumns - 1, kMapRows - 1 }, ElementType::tree, 8 },
    };
    for (const auto& e : entries)
    {
        TileManager::insertElement(e.type, toWorldSpace(e.tile), e.baseZ, 0);
    }

    auto snapshot = [&](const Entry& e) {
        auto tile = TileManager::get(e.tile);
        std::vector<std::pair<ElementType, uint8_t>> v;
        for (auto& el : tile)
        {
            v.emplace_back(el.type(), el.baseZ());
        }
        return v;
    };

    std::vector<std::vector<std::pair<ElementType, uint8_t>>> before;
    for (const auto& e : entries)
    {
        before.push_back(snapshot(e));
    }

    TileManager::reorganise();

    for (size_t i = 0; i < entries.size(); ++i)
    {
        EXPECT_EQ(snapshot(entries[i]), before[i]);
    }
}

TEST_F(TileManagerTest, DefragmentTilePeriodicCompactsTilePointerBackward)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);

    auto tileBefore = TileManager::get(kTestTile);
    ASSERT_EQ(tileBefore.size(), 2u);
    auto* entryAddrBefore = &*tileBefore.begin();

    constexpr size_t kNumTiles = static_cast<size_t>(kMapPitch) * static_cast<size_t>(kMapRows);
    for (size_t i = 0; i < kNumTiles + 1; ++i)
    {
        TileManager::defragmentTilePeriodic();
    }

    auto tileAfter = TileManager::get(kTestTile);
    ASSERT_EQ(tileAfter.size(), 2u);
    auto* entryAddrAfter = &*tileAfter.begin();
    EXPECT_LT(entryAddrAfter, entryAddrBefore);
    EXPECT_EQ(typeAt(tileAfter, 0), ElementType::surface);
    EXPECT_EQ(typeAt(tileAfter, 1), ElementType::track);
    EXPECT_EQ(tileAfter[1]->baseZ(), 8);
}

TEST_F(TileManagerTest, DefragmentTilePeriodicReducesElementsEnd)
{
    for (int i = 0; i < 100; ++i)
    {
        TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), static_cast<uint8_t>(8 + i), 0);
    }
    const auto liveBefore = TileManager::getEntries().size();

    constexpr size_t kNumTiles = static_cast<size_t>(kMapPitch) * static_cast<size_t>(kMapRows);
    for (size_t i = 0; i < kNumTiles + 1; ++i)
    {
        TileManager::defragmentTilePeriodic();
    }

    EXPECT_LT(TileManager::getEntries().size(), liveBefore);
}

TEST_F(TileManagerTest, DefragmentTilePeriodicPreservesByteContentAfterChurn)
{
    const std::vector<TilePos2> tiles{
        TilePos2{ 5, 5 }, TilePos2{ 5, 6 }, TilePos2{ 6, 5 }, TilePos2{ 100, 100 }, TilePos2{ 200, 200 }
    };
    for (auto t : tiles)
    {
        TileManager::insertElement(ElementType::track, toWorldSpace(t), 8, 0);
        TileManager::insertElement(ElementType::tree, toWorldSpace(t), 16, 0);
    }
    for (auto t : tiles)
    {
        TileElementEntry* victim = nullptr;
        for (auto& el : TileManager::get(t))
        {
            if (el.type() == ElementType::tree)
            {
                victim = &el;
                break;
            }
        }
        ASSERT_NE(victim, nullptr);
        TileManager::removeElement(*victim);
    }

    std::vector<TileBytes> before;
    for (auto t : tiles)
    {
        before.push_back(snapshotBytes(TileManager::get(t)));
    }

    constexpr size_t kNumTiles = static_cast<size_t>(kMapPitch) * static_cast<size_t>(kMapRows);
    for (size_t i = 0; i < 2 * kNumTiles; ++i)
    {
        TileManager::defragmentTilePeriodic();
    }

    for (size_t i = 0; i < tiles.size(); ++i)
    {
        EXPECT_EQ(before[i], snapshotBytes(TileManager::get(tiles[i])))
            << "tile (" << tiles[i].x << "," << tiles[i].y << ") diverged after periodic defrag";
    }
}

TEST_F(TileManagerTest, DisablePeriodicDefragSkipsNextCall)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);

    constexpr size_t kNumTiles = static_cast<size_t>(kMapPitch) * static_cast<size_t>(kMapRows);
    for (size_t i = 0; i < kNumTiles + 1; ++i)
    {
        TileManager::defragmentTilePeriodic();
    }

    TileManager::insertElement(ElementType::tree, toWorldSpace(kOtherTile), 8, 0);
    const auto liveBeforeDisabled = TileManager::getEntries().size();

    TileManager::disablePeriodicDefrag();
    TileManager::defragmentTilePeriodic();
    EXPECT_EQ(TileManager::getEntries().size(), liveBeforeDisabled);
}

TEST_F(TileManagerTest, DefragmentTilePeriodicIsSafeAfterChurn)
{
    const std::vector<TilePos2> tiles{
        { 5, 5 }, { 5, 6 }, { 6, 5 }, { 100, 100 }, { 200, 200 }
    };
    for (auto t : tiles)
    {
        TileManager::insertElement(ElementType::track, toWorldSpace(t), 8, 0);
        TileManager::insertElement(ElementType::tree, toWorldSpace(t), 16, 0);
    }
    for (auto t : tiles)
    {
        TileElementEntry* toRemove = nullptr;
        for (auto& el : TileManager::get(t))
        {
            if (el.type() == ElementType::tree)
            {
                toRemove = &el;
                break;
            }
        }
        ASSERT_NE(toRemove, nullptr);
        TileManager::removeElement(*toRemove);
    }

    for (int i = 0; i < 4096; ++i)
    {
        TileManager::defragmentTilePeriodic();
    }

    for (auto t : tiles)
    {
        auto tile = TileManager::get(t);
        size_t n = 0;
        int lastCount = 0;
        for (auto& el : tile)
        {
            ++n;
            if (el.isLast())
            {
                ++lastCount;
            }
        }
        EXPECT_EQ(n, 2u);
        EXPECT_EQ(lastCount, 1);
    }
}

TEST_F(TileManagerTest, GetEntriesReflectsLiveRegion)
{
    auto entries = TileManager::getEntries();
    EXPECT_FALSE(entries.empty());
    for (auto& entry : entries)
    {
        EXPECT_EQ(entry.type(), ElementType::surface);
        EXPECT_TRUE(entry.isLast());
    }
}

TEST_F(TileManagerTest, GetEntriesCountGrowsAfterInsert)
{
    const auto before = TileManager::getEntries().size();
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    EXPECT_GT(TileManager::getEntries().size(), before);
}

TEST_F(TileManagerTest, UpdateTilePointersRebuildsConsistently)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kOtherTile), 12, 0);
    TileManager::reorganise();
    TileManager::updateTilePointers();

    auto a = TileManager::get(kTestTile);
    auto b = TileManager::get(kOtherTile);
    ASSERT_EQ(a.size(), 2u);
    ASSERT_EQ(b.size(), 2u);
    EXPECT_EQ(typeAt(a, 0), ElementType::surface);
    EXPECT_EQ(typeAt(a, 1), ElementType::track);
    EXPECT_EQ(typeAt(b, 0), ElementType::surface);
    EXPECT_EQ(typeAt(b, 1), ElementType::tree);
}

TEST_F(TileManagerTest, BaseZTiesPreserveInsertionOrder)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 8, 0);

    auto tile = TileManager::get(kTestTile);
    std::vector<ElementType> types;
    for (auto& el : tile)
    {
        types.push_back(el.type());
    }
    ASSERT_EQ(types.size(), 3u);
    EXPECT_EQ(types[0], ElementType::surface);
    EXPECT_EQ(types[1], ElementType::track);
    EXPECT_EQ(types[2], ElementType::tree);
}

TEST_F(TileManagerTest, IteratorDefaultConstructedEqualsEnd)
{
    auto tile = TileManager::get(kTestTile);
    Tile::Iterator def{};
    EXPECT_EQ(def, tile.end());
}

TEST_F(TileManagerTest, IteratorPostIncrementReturnsOldPosition)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    auto tile = TileManager::get(kTestTile);
    auto it = tile.begin();
    auto pre = it;
    auto post = it++;
    EXPECT_EQ(pre, post);
    EXPECT_NE(it, post);
    EXPECT_EQ(post->type(), ElementType::surface);
    EXPECT_EQ(it->type(), ElementType::track);
}

TEST_F(TileManagerTest, ManyElementsOnOneTileAllWalk)
{
    constexpr int kCount = static_cast<int>(TileManager::kMaxElementsOnOneTile) - 1;
    for (int i = 0; i < kCount; ++i)
    {
        const auto baseZ = static_cast<uint8_t>(std::min(8 + i, 255));
        const auto type = (i % 2 == 0) ? ElementType::track : ElementType::tree;
        ASSERT_NE(TileManager::insertElement(type, toWorldSpace(kTestTile), baseZ, 0), nullptr);
    }

    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), static_cast<size_t>(kCount + 1));

    int lastCount = 0;
    uint8_t prevZ = 0;
    int idx = 0;
    for (auto& el : tile)
    {
        EXPECT_GE(el.baseZ(), prevZ);
        prevZ = el.baseZ();
        if (el.isLast())
        {
            ++lastCount;
        }
        if (idx == 0)
        {
            EXPECT_EQ(el.type(), ElementType::surface);
        }
        ++idx;
    }
    EXPECT_EQ(lastCount, 1);
}

TEST_F(TileManagerTest, ReorganiseIsIdempotent)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kOtherTile), 4, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(TilePos2{ 50, 50 }), 24, 0);

    TileManager::reorganise();
    const auto a1 = snapshotBytes(TileManager::get(kTestTile));
    const auto b1 = snapshotBytes(TileManager::get(kOtherTile));
    const auto c1 = snapshotBytes(TileManager::get(TilePos2{ 50, 50 }));
    const auto freeAfter1 = TileManager::numFreeElements();

    TileManager::reorganise();
    const auto a2 = snapshotBytes(TileManager::get(kTestTile));
    const auto b2 = snapshotBytes(TileManager::get(kOtherTile));
    const auto c2 = snapshotBytes(TileManager::get(TilePos2{ 50, 50 }));

    EXPECT_EQ(a1, a2);
    EXPECT_EQ(b1, b2);
    EXPECT_EQ(c1, c2);
    EXPECT_EQ(TileManager::numFreeElements(), freeAfter1);
}

TEST_F(TileManagerTest, ReorganisePreservesRawElementBytes)
{
    auto* el1 = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0b0101);
    ASSERT_NE(el1, nullptr);
    el1->setGhost(true);
    auto* el2 = TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0b1010);
    ASSERT_NE(el2, nullptr);
    el2->setAiAllocated(true);
    el2->setFlag6(true);

    const auto before = snapshotBytes(TileManager::get(kTestTile));
    TileManager::reorganise();
    const auto after = snapshotBytes(TileManager::get(kTestTile));

    ASSERT_EQ(before.size(), after.size());
    for (size_t i = 0; i < before.size(); ++i)
    {
        EXPECT_EQ(before[i], after[i]) << "byte mismatch at element " << i;
    }
}

TEST_F(TileManagerTest, RemoveDoesNotChangeOtherTileBytes)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kOtherTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kOtherTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(TilePos2{ 100, 100 }), 12, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(TilePos2{ 100, 100 }), 20, 0);

    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 24, 0);

    const auto otherBefore = snapshotBytes(TileManager::get(kOtherTile));
    const auto farBefore = snapshotBytes(TileManager::get(TilePos2{ 100, 100 }));

    TileElementEntry* victim = nullptr;
    for (auto& el : TileManager::get(kTestTile))
    {
        if (el.type() == ElementType::tree)
        {
            victim = &el;
            break;
        }
    }
    ASSERT_NE(victim, nullptr);
    TileManager::removeElement(*victim);

    EXPECT_EQ(otherBefore, snapshotBytes(TileManager::get(kOtherTile)));
    EXPECT_EQ(farBefore, snapshotBytes(TileManager::get(TilePos2{ 100, 100 })));
}

TEST_F(TileManagerTest, WasRemoveOnLastElementFalseForMiddleRemoval)
{
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    TileManager::insertElement(ElementType::tree, toWorldSpace(kTestTile), 16, 0);
    TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 24, 0);

    TileElementEntry* middle = nullptr;
    for (auto& el : TileManager::get(kTestTile))
    {
        if (el.type() == ElementType::tree)
        {
            middle = &el;
            break;
        }
    }
    ASSERT_NE(middle, nullptr);

    TileManager::setRemoveElementPointerChecker(*middle);
    TileManager::removeElement(*middle);
    EXPECT_FALSE(TileManager::wasRemoveOnLastElement());
}

TEST_F(TileManagerTest, InsertElementRoadBasicAddsRoadElement)
{
    auto* road = TileManager::insertElementRoad(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(road, nullptr);
    EXPECT_EQ(road->type(), ElementType::road);
    EXPECT_EQ(road->baseZ(), 8);

    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 2u);
    EXPECT_EQ(typeAt(tile, 0), ElementType::surface);
    EXPECT_EQ(typeAt(tile, 1), ElementType::road);
}

TEST_F(TileManagerTest, InsertElementRoadStopsBeforeRoadStationAtSameBaseZ)
{
    auto* stationEntry = TileManager::insertElement<StationElement>(toWorldSpace(kTestTile), 16, 0);
    ASSERT_NE(stationEntry, nullptr);
    stationEntry->get<StationElement>().setStationType(OpenLoco::StationType::roadStation);

    auto* road = TileManager::insertElementRoad(toWorldSpace(kTestTile), 16, 0);
    ASSERT_NE(road, nullptr);

    auto tile = TileManager::get(kTestTile);
    ASSERT_EQ(tile.size(), 3u);
    EXPECT_EQ(typeAt(tile, 0), ElementType::surface);
    EXPECT_EQ(typeAt(tile, 1), ElementType::road);
    EXPECT_EQ(typeAt(tile, 2), ElementType::station);
}

TEST_F(TileManagerTest, InsertElementTypedTemplateReturnsTypedPointer)
{
    auto* track = TileManager::insertElement<TrackElement>(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(track, nullptr);
    EXPECT_EQ(track->type(), ElementType::track);
    EXPECT_EQ(track->baseZ(), 8);

    auto* tree = TileManager::insertElement<TreeElement>(toWorldSpace(kTestTile), 12, 0);
    ASSERT_NE(tree, nullptr);
    EXPECT_EQ(tree->type(), ElementType::tree);
}

TEST_F(TileManagerTest, TrackSignalModesPreserveOtherTrackDataAndSanitiseInvalidValues)
{
    auto* trackEntry = TileManager::insertElement<TrackElement>(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(trackEntry, nullptr);
    auto& track = trackEntry->get<TrackElement>();
    track.setHasLevelCrossing(true);
    track.setBridgeObjectId(5);

    track.setLeftSignalMode(SignalMode::path);
    track.setRightSignalMode(SignalMode::oneWayPath);

    EXPECT_EQ(track.leftSignalMode(), SignalMode::path);
    EXPECT_EQ(track.rightSignalMode(), SignalMode::oneWayPath);
    EXPECT_TRUE(track.hasLevelCrossing());
    EXPECT_EQ(track.bridge(), 5);

    track.setSignalModes(0xF);

    EXPECT_EQ(track.leftSignalMode(), SignalMode::block);
    EXPECT_EQ(track.rightSignalMode(), SignalMode::block);
    EXPECT_TRUE(track.hasLevelCrossing());
    EXPECT_EQ(track.bridge(), 5);
}

TEST_F(TileManagerTest, TrackSignalModesAreExportedToS5)
{
    auto* trackEntry = TileManager::insertElement<TrackElement>(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(trackEntry, nullptr);
    auto& track = trackEntry->get<TrackElement>();
    track.setLeftSignalMode(SignalMode::oneWayPath);
    track.setRightSignalMode(SignalMode::path);

    const auto saved = OpenLoco::S5::toSaveElement(OpenLoco::getGameState(), *trackEntry);
    const auto* savedTrack = saved.as<OpenLoco::S5::TrackElement>();
    ASSERT_NE(savedTrack, nullptr);
    EXPECT_EQ(savedTrack->signalModes(), track.signalModes());
}

TEST(SignalModePersistenceTest, LastSelectedModeRoundTripsThroughS5)
{
    auto gameState = std::make_unique<OpenLoco::GameState>();
    gameState->scenarioConstruction.setLastSignalMode(static_cast<uint8_t>(SignalMode::path));

    const auto saved = OpenLoco::S5::exportGameState(*gameState);
    EXPECT_EQ(saved->general.scenarioConstruction.var_17A[0], static_cast<uint8_t>(SignalMode::path));

    const auto restored = OpenLoco::S5::importGameState(*saved);
    EXPECT_EQ(restored->scenarioConstruction.lastSignalMode(), static_cast<uint8_t>(SignalMode::path));
}

TEST_F(TileManagerTest, StandardPathSignalCanBePassedFromBehindButOneWayPathSignalCannot)
{
    auto* trackEntry = TileManager::insertElement<TrackElement>(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(trackEntry, nullptr);
    auto& track = trackEntry->get<TrackElement>();
    track.setTrackId(0);
    track.setRotation(0);
    track.setSequenceIndex(0);
    track.setTrackObjectId(0);
    track.setHasSignal(true);

    auto* signalEntry = TileManager::insertElementAfterNoReorg<SignalElement>(trackEntry, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(signalEntry, nullptr);
    auto& signal = signalEntry->get<SignalElement>();
    signal.setRotation(0);
    signal.getRight() = SignalElement::Side{};
    signal.getRight().setHasSignal(true);

    const OpenLoco::Vehicles::TrackAndDirection::_TrackAndDirection tad(0, 0);
    const Pos3 trackStart{ toWorldSpace(kTestTile), 8 * kSmallZStep };

    track.setRightSignalMode(SignalMode::path);
    EXPECT_EQ(OpenLoco::Vehicles::getSignalState(trackStart, tad, 0, 0), OpenLoco::Vehicles::SignalStateFlags::none);
    EXPECT_EQ(OpenLoco::Vehicles::getSignalMode(trackStart, tad, 0, 1U << 31), SignalMode::path);

    track.setRightSignalMode(SignalMode::oneWayPath);
    EXPECT_NE(OpenLoco::Vehicles::getSignalState(trackStart, tad, 0, 0) & OpenLoco::Vehicles::SignalStateFlags::blockedNoRoute, OpenLoco::Vehicles::SignalStateFlags::none);
}

TEST(SignalPlacementArgsTest, RoundTripsSignalModeThroughRegisters)
{
    OpenLoco::GameCommands::SignalPlacementArgs original{};
    original.pos = { 320, 640, 24 };
    original.rotation = 3;
    original.trackId = 17;
    original.index = 2;
    original.type = 7;
    original.mode = SignalMode::oneWayPath;
    original.trackObjType = 4;
    original.sides = 0xC000;

    const OpenLoco::GameCommands::SignalPlacementArgs decoded(static_cast<OpenLoco::GameCommands::registers>(original));

    EXPECT_EQ(decoded.pos, original.pos);
    EXPECT_EQ(decoded.rotation, original.rotation);
    EXPECT_EQ(decoded.trackId, original.trackId);
    EXPECT_EQ(decoded.index, original.index);
    EXPECT_EQ(decoded.type, original.type);
    EXPECT_EQ(decoded.mode, original.mode);
    EXPECT_EQ(decoded.trackObjType, original.trackObjType);
    EXPECT_EQ(decoded.sides, original.sides);
}

TEST(SignalPlacementArgsTest, SanitisesUnknownSignalModeFromRegisters)
{
    OpenLoco::GameCommands::SignalPlacementArgs original{};
    auto regs = static_cast<OpenLoco::GameCommands::registers>(original);
    regs.edi = (regs.edi & ~(0x3U << 24)) | (0x3U << 24);

    const OpenLoco::GameCommands::SignalPlacementArgs decoded(regs);

    EXPECT_EQ(decoded.mode, SignalMode::block);
}

TEST_F(PathSignalsTest, ReportsOnlyFutureRoutingEntriesAsReserved)
{
    constexpr Pos3 currentPos{ 320, 160, 32 };
    constexpr uint16_t straightWest = 0;
    auto* head = createTrain(currentPos, straightWest);
    ASSERT_NE(head, nullptr);

    auto nextHandle = head->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    OpenLoco::Vehicles::RoutingManager::setRouting(nextHandle, straightWest);

    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::isPathReserved(currentPos, straightWest));
    EXPECT_TRUE(OpenLoco::Vehicles::PathSignals::isPathReserved({ currentPos.x - kTileSize, currentPos.y, currentPos.z }, straightWest));

    OpenLoco::Vehicles::RoutingManager::freeRouting(nextHandle);
    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::isPathReserved({ currentPos.x - kTileSize, currentPos.y, currentPos.z }, straightWest));
}

TEST_F(PathSignalsTest, GhostInfrastructureRemovalIgnoresFutureRouting)
{
    constexpr Pos3 currentPos{ 320, 160, 32 };
    constexpr Pos3 ghostPos{ currentPos.x - kTileSize, currentPos.y, currentPos.z };
    addTrack(ghostPos, 0, 0);

    auto* head = createTrain(currentPos, kStraightWest);
    ASSERT_NE(head, nullptr);
    auto nextHandle = head->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    OpenLoco::Vehicles::RoutingManager::setRouting(nextHandle, kStraightWest);
    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::isPathReserved(ghostPos, kStraightWest));

    auto tile = TileManager::get(ghostPos);
    const auto trackIt = std::ranges::find_if(tile, [](const auto& entry) { return entry.template as<TrackElement>() != nullptr; });
    ASSERT_NE(trackIt, tile.end());

    OpenLoco::GameCommands::TrackRemovalArgs args{};
    args.pos = ghostPos;
    args.rotation = 0;
    args.trackId = 0;
    args.index = 0;
    args.trackObjectId = 0;
    const auto commandFlags = OpenLoco::GameCommands::Flags::apply | OpenLoco::GameCommands::Flags::noErrorWindow | OpenLoco::GameCommands::Flags::noPayment;

    EXPECT_EQ(OpenLoco::GameCommands::doCommand(args, commandFlags), OpenLoco::GameCommands::kFailure);
    trackIt->get<TrackElement>().setGhost(true);
    EXPECT_EQ(OpenLoco::GameCommands::doCommand(args, commandFlags | OpenLoco::GameCommands::Flags::ghost), 0U);
    EXPECT_EQ(TileManager::get(ghostPos).size(), 1U);

    addTrack(ghostPos, 0, 0, true);
    auto signalTile = TileManager::get(ghostPos);
    const auto signalTrackIt = std::ranges::find_if(signalTile, [](const auto& entry) { return entry.template as<TrackElement>() != nullptr; });
    ASSERT_NE(signalTrackIt, signalTile.end());
    auto* trackEntry = &*signalTrackIt;
    auto& track = trackEntry->get<TrackElement>();
    auto& signal = trackEntry->next()->get<SignalElement>();
    signal.setGhost(true);
    signal.setLeftGhost(true);

    OpenLoco::GameCommands::SignalRemovalArgs signalArgs{};
    signalArgs.pos = ghostPos;
    signalArgs.rotation = 0;
    signalArgs.trackId = 0;
    signalArgs.index = 0;
    signalArgs.trackObjType = 0;
    signalArgs.flags = 1U << 15;

    EXPECT_EQ(OpenLoco::GameCommands::doCommand(signalArgs, commandFlags), OpenLoco::GameCommands::kFailure);
    EXPECT_EQ(OpenLoco::GameCommands::doCommand(signalArgs, commandFlags | OpenLoco::GameCommands::Flags::ghost), 0U);
    EXPECT_FALSE(track.hasSignal());
    EXPECT_EQ(TileManager::get(ghostPos).size(), 2U);
}

TEST_F(PathSignalsTest, MarksExactPathReservationEntries)
{
    addTwoRouteJunction();
    auto* reservingTrain = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));

    auto handle = reservingTrain->routingHandle;
    for (auto i = 0; i < 3; ++i)
    {
        handle.setIndex(handle.getIndex() + 1);
        EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(handle));
    }
    handle.setIndex(handle.getIndex() + 1);
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::isPathReserved(handle));
}

TEST_F(PathSignalsTest, StreamsWaypointReservationBeyondRoutingCapacity)
{
    constexpr Pos3 firstPos{ 3200, 3200, 32 };
    constexpr Pos3 junction{ firstPos.x - kTileSize, firstPos.y, firstPos.z };
    constexpr Pos3 waypointPos{ firstPos.x - 5 * kTileSize, firstPos.y, firstPos.z };
    constexpr auto longBlockLength = 80;

    addTrack(firstPos, 0, 0);
    for (auto i = 1; i <= longBlockLength; ++i)
    {
        addTrack(firstPos + Pos3{ -i * kTileSize, 0, 0 }, 0, 0, i == longBlockLength);
    }
    addTrack(junction, 2, 0);
    addTrack(junction + Pos3{ 0, -kTileSize, 0 }, 0, 3);
    addTrack(junction + Pos3{ 0, -2 * kTileSize, 0 }, 0, 3, true);

    auto* train = createTrain(firstPos + Pos3{ kTileSize, 0, 0 }, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderRouteWaypoint waypoint{ toTileSpace(waypointPos), waypointPos.z / 8, 0, 0 };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &waypoint);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*train, firstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*train), kStraightWest);
    const auto suffixPos = firstPos + Pos3{ -60 * kTileSize, 0, 0 };
    ASSERT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).size(), 20);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).front(), kStraightWest);
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::validateState(OpenLoco::Vehicles::RoutingManager::captureState()));
    EXPECT_TRUE(OpenLoco::Vehicles::PathSignals::isPathReserved(suffixPos, kStraightWest));
    EXPECT_TRUE(OpenLoco::Vehicles::PathSignals::hasPathReservationConflict(OpenLoco::EntityId::null, suffixPos, kStraightWest));

    auto handle = train->routingHandle;
    for (auto i = 0; i < 60; ++i)
    {
        handle.setIndex(handle.getIndex() + 1);
        EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(handle));
    }
    handle.setIndex(handle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(handle), OpenLoco::Vehicles::RoutingManager::kAllocatedButFreeRouting);

    auto headAfterMove = train->routingHandle;
    headAfterMove.setIndex(headAfterMove.getIndex() + 1);
    const auto oldTailHandle = train->routingHandle;
    train->routingHandle = headAfterMove;
    train->tileX = firstPos.x;
    train->tileY = firstPos.y;
    train->tileBaseZ = firstPos.z / OpenLoco::World::kSmallZStep;
    OpenLoco::Vehicles::RoutingManager::freeTailRoutingAndRefill(oldTailHandle, *train);

    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).size(), 19);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(handle) & OpenLoco::World::Track::AdditionalTaDFlags::basicTaDMask, kStraightWest);
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(handle));

    const auto countFreeSlots = [&]() {
        size_t count = 0;
        for (uint8_t i = 0; i < OpenLoco::Limits::kMaxRoutingsPerVehicle; ++i)
        {
            count += OpenLoco::Vehicles::RoutingManager::getRouting({ train->routingHandle.getVehicleRef(), i }) == OpenLoco::Vehicles::RoutingManager::kAllocatedButFreeRouting;
        }
        return count;
    };
    EXPECT_EQ(countFreeSlots(), OpenLoco::Vehicles::RoutingManager::kRequiredFreeRoutingSlots);
    for (auto i = 1; i < 20; ++i)
    {
        const auto oldHeadHandle = train->routingHandle;
        train->routingHandle.setIndex(train->routingHandle.getIndex() + 1);
        train->tileX = firstPos.x - i * kTileSize;
        OpenLoco::Vehicles::RoutingManager::freeTailRoutingAndRefill(oldHeadHandle, *train);

        EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).size(), 19 - i);
        EXPECT_EQ(countFreeSlots(), OpenLoco::Vehicles::RoutingManager::kRequiredFreeRoutingSlots);
        if (!OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).empty())
        {
            const auto nextPos = firstPos + Pos3{ -(61 + i) * kTileSize, 0, 0 };
            EXPECT_TRUE(OpenLoco::Vehicles::PathSignals::isPathReserved(nextPos, OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).front()));
        }
    }
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).empty());
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::validateState(OpenLoco::Vehicles::RoutingManager::captureState()));
}

TEST_F(PathSignalsTest, LongReservationIsAtomicWhenSuffixConflicts)
{
    constexpr Pos3 firstPos{ 3200, 3200, 32 };
    constexpr Pos3 waypointPos{ firstPos.x - 5 * kTileSize, firstPos.y, firstPos.z };
    constexpr auto longBlockLength = 80;
    addTrack(firstPos, 0, 0);
    for (auto i = 1; i <= longBlockLength; ++i)
    {
        addTrack(firstPos + Pos3{ -i * kTileSize, 0, 0 }, 0, 0, i == longBlockLength);
    }

    auto* train = createTrain(firstPos + Pos3{ kTileSize, 0, 0 }, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderRouteWaypoint waypoint{ toTileSpace(waypointPos), waypointPos.z / 8, 0, 0 };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &waypoint);
    ASSERT_NE(createTrain(firstPos + Pos3{ -70 * kTileSize, 0, 0 }, kStraightWest), nullptr);

    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::tryReservePath(*train, firstPos, kStraightWest));
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::hasPathReservations(train->routingHandle));
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).empty());
}

TEST_F(PathSignalsTest, OrdinaryMovementDoesNotEnterOccupiedPathReservation)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(kFirstPos, 0, 0);
    auto* waitingTrain = createTrain(currentPos, kStraightWest);
    ASSERT_NE(waitingTrain, nullptr);
    auto* reservingTrain = createTrain(kFirstPos, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::hasPathReservationConflict(waitingTrain->id, kFirstPos, kStraightWest));
    OpenLoco::Vehicles::RoutingManager::markPathReserved(reservingTrain->routingHandle);

    const auto result = waitingTrain->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 1);
    auto nextHandle = waitingTrain->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(nextHandle), OpenLoco::Vehicles::RoutingManager::kAllocatedButFreeRouting);
}

TEST_F(PathSignalsTest, AdvancesWaypointConsumedFromReservedRouting)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 waypointPos{ 320, 320, 32 };
    constexpr Pos3 nextPos{ 288, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(waypointPos, 0, 0);
    addTrack(nextPos, 0, 0);

    auto* train = createTrain(currentPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderStopAt precedingOrder{ OpenLoco::StationId(0) };
    const OpenLoco::Vehicles::OrderRouteWaypoint waypoint{ toTileSpace(waypointPos), waypointPos.z / 8, 0, 0 };
    const OpenLoco::Vehicles::OrderRouteWaypoint followingWaypoint{ toTileSpace(currentPos), currentPos.z / 8, 0, 0 };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &precedingOrder);
    OpenLoco::Vehicles::OrderManager::insertOrder(train, sizeof(precedingOrder), &waypoint);
    OpenLoco::Vehicles::OrderManager::insertOrder(train, sizeof(precedingOrder) + sizeof(waypoint), &followingWaypoint);
    train->currentOrder = sizeof(precedingOrder);

    auto reservedHandle = train->routingHandle;
    reservedHandle.setIndex(reservedHandle.getIndex() + 1);
    OpenLoco::Vehicles::RoutingManager::setRouting(reservedHandle, kStraightWest);

    train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(train->currentOrder, sizeof(precedingOrder) + sizeof(waypoint));
}

TEST_F(PathSignalsTest, FreeingRoutingClearsPathReservation)
{
    auto* train = createTrain(kFirstPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    OpenLoco::Vehicles::RoutingManager::markPathReserved(train->routingHandle);
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(train->routingHandle));

    const auto state = OpenLoco::Vehicles::RoutingManager::captureState();
    OpenLoco::Vehicles::RoutingManager::clearPathReservations(train->routingHandle);
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::restoreState(state));
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(train->routingHandle));

    OpenLoco::Vehicles::RoutingManager::freeRouting(train->routingHandle);

    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::isPathReserved(train->routingHandle));
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::validateState(state));
}

TEST_F(PathSignalsTest, RoutingContinuationStateRoundTripsAndClears)
{
    auto* train = createTrain(kFirstPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    auto reservedHandle = train->routingHandle;
    reservedHandle.setIndex(reservedHandle.getIndex() + 1);
    OpenLoco::Vehicles::RoutingManager::setRouting(reservedHandle, kStraightWest);
    OpenLoco::Vehicles::RoutingManager::markPathReserved(reservedHandle);
    OpenLoco::Vehicles::RoutingManager::setReservedContinuation(train->routingHandle, { kStraightWest });

    const auto state = OpenLoco::Vehicles::RoutingManager::captureState();
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::validateState(state));
    auto malformedState = state;
    malformedState.continuations[train->routingHandle.getVehicleRef()].front() = 63 << 3;
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::validateState(malformedState));
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::restoreState(malformedState));
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::captureState(), state);
    malformedState = state;
    malformedState.continuations[train->routingHandle.getVehicleRef()].clear();
    OpenLoco::Vehicles::RoutingManager::setRouting(reservedHandle, 63 << 3);
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::validateState(malformedState));
    OpenLoco::Vehicles::RoutingManager::setRouting(reservedHandle, kStraightWest);
    train->status = OpenLoco::Vehicles::Status::crashed;
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::validateState(state));
    train->status = OpenLoco::Vehicles::Status::stopped;
    OpenLoco::Vehicles::RoutingManager::clearPathReservations(train->routingHandle);
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::restoreState(state));

    ASSERT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).size(), 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).front(), state.continuations[train->routingHandle.getVehicleRef()].front());

    OpenLoco::Vehicles::RoutingManager::resetPathReservationState();
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::hasPathReservations(train->routingHandle));
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(reservedHandle), kStraightWest);
    ASSERT_TRUE(OpenLoco::Vehicles::RoutingManager::restoreState(state));

    OpenLoco::Vehicles::RoutingManager::resetRoutings(train->routingHandle);
    EXPECT_FALSE(OpenLoco::Vehicles::RoutingManager::hasPathReservations(train->routingHandle));
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).empty());
}

TEST_F(PathSignalsTest, CrashReleasesUnmaterializedContinuation)
{
    auto* train = createTrain(kFirstPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    train->owner = OpenLoco::CompanyId(1);
    OpenLoco::Vehicles::RoutingManager::setReservedContinuation(train->routingHandle, { kStraightWest });

    train->destroyTrain();

    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::getReservedContinuation(train->routingHandle).empty());
}

TEST_F(PathSignalsTest, OneWayPathFailureReportsOneWayWaitState)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 blockedPos{ 288, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(kFirstPos, 0, 0, true, SignalMode::oneWayPath);
    addTrack(blockedPos, 0, 0);
    auto* waitingTrain = createTrain(currentPos, kStraightWest);
    ASSERT_NE(waitingTrain, nullptr);
    ASSERT_NE(createTrain(blockedPos, kStraightWest), nullptr);

    const auto result = waitingTrain->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 3);
    EXPECT_NE(result.flags & OpenLoco::enumValue(OpenLoco::Vehicles::SignalStateFlags::blockedNoRoute), 0);
    EXPECT_EQ(result.flags & OpenLoco::enumValue(OpenLoco::Vehicles::SignalStateFlags::occupiedOneWay), 0);
}

TEST_F(PathSignalsTest, ChoosesLongerRouteWhenShortRouteBeyondSignalIsOccupied)
{
    addTwoRouteJunction();
    addTrack({ 192, 320, 32 }, 0, 0);

    auto* reservingTrain = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    ASSERT_NE(createTrain({ 192, 320, 32 }, kStraightWest), nullptr);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*reservingTrain), kTurnNorth);
}

TEST_F(PathSignalsTest, DetectsCongestionBeyondPrevious64PieceLookahead)
{
    constexpr Pos3 firstPos{ 6400, 6400, 32 };
    constexpr auto continuationLength = 80;
    addTwoRouteJunction(firstPos);
    for (auto i = 0; i < continuationLength; ++i)
    {
        addTrack(firstPos + Pos3{ -128 - i * kTileSize, 0, 0 }, 0, 0);
    }

    auto* reservingTrain = createTrain(firstPos + Pos3{ kTileSize, 0, 0 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    const auto blockedPos = firstPos + Pos3{ -128 - (continuationLength - 1) * kTileSize, 0, 0 };
    ASSERT_NE(createTrain(blockedPos, kStraightWest), nullptr);
    recordSlowTrack(firstPos + Pos3{ -128 - 70 * kTileSize, 0, 0 }, kStraightWest);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, firstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*reservingTrain), kTurnNorth);
}

TEST_F(PathSignalsTest, RejectsDisconnectedStationRouteAtLookaheadLimit)
{
    constexpr Pos3 firstPos{ 200 * kTileSize, 350 * kTileSize, 32 };
    constexpr auto stationDistance = 264;
    addTwoRouteJunction(firstPos);
    addTrack(firstPos + Pos3{ -4 * kTileSize, 0, 0 }, 0, 0);
    for (auto i = 4; i <= stationDistance; ++i)
    {
        addTrack(firstPos + Pos3{ -kTileSize, -i * kTileSize, 0 }, 0, 3);
    }
    const auto stationPos = firstPos + Pos3{ -kTileSize, -stationDistance * kTileSize, 0 };
    addTrainStation(stationPos, 0, 3, OpenLoco::StationId(0));

    auto* train = createTrain(firstPos + Pos3{ kTileSize, 0, 0 }, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderStopAt order{ OpenLoco::StationId(0) };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &order);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*train, firstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*train), kTurnNorth);
}

TEST_F(PathSignalsTest, ChoosesLongerRouteWhenShortRouteBeyondSignalIsReserved)
{
    constexpr uint16_t straightNorth = 3;

    addTwoRouteJunction();
    auto* reservingTrain = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    auto* blockingTrain = createTrain({ 224, 352, 32 }, straightNorth);
    ASSERT_NE(blockingTrain, nullptr);

    auto blockingReservation = blockingTrain->routingHandle;
    blockingReservation.setIndex(blockingReservation.getIndex() + 1);
    OpenLoco::Vehicles::RoutingManager::setRouting(blockingReservation, straightNorth);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*reservingTrain), kTurnNorth);
}

TEST_F(PathSignalsTest, ExcludesRouteWithConflictInsideImmediateReservation)
{
    addTwoRouteJunction();
    auto* reservingTrain = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    ASSERT_NE(createTrain({ 256, 320, 32 }, kStraightWest), nullptr);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*reservingTrain), kTurnNorth);
}

TEST_F(PathSignalsTest, WaitsInsteadOfReservingExcessiveDetour)
{
    constexpr Pos3 firstPos{ 1280, 1280, 32 };
    constexpr auto detourLength = 12;
    const auto junction = firstPos + Pos3{ -32, 0, 0 };
    const auto directPlatform = firstPos + Pos3{ -128, 0, 0 };

    addTrack(firstPos, 0, 0);
    addTrack(junction, 0, 0);
    addTrack(firstPos + Pos3{ -64, 0, 0 }, 0, 0);
    addTrack(firstPos + Pos3{ -96, 0, 0 }, 0, 0, true);
    addTrack(directPlatform, 0, 0);
    addTrack(junction, 2, 0);
    addTrack(junction + Pos3{ 0, -32, 0 }, 0, 3);
    addTrack(junction + Pos3{ 0, -64, 0 }, 0, 3);
    addTrack(junction + Pos3{ 0, -96, 0 }, 0, 3, true);
    for (auto i = 4; i < detourLength + 4; ++i)
    {
        addTrack(junction + Pos3{ 0, -i * kTileSize, 0 }, 0, 3);
    }
    const auto detourPlatform = junction + Pos3{ 0, -(detourLength + 3) * kTileSize, 0 };
    addTrainStation(directPlatform, 0, 0, OpenLoco::StationId(0));
    addTrainStation(detourPlatform, 0, 3, OpenLoco::StationId(0));
    auto* station = OpenLoco::StationManager::get(OpenLoco::StationId(0));
    station->x = directPlatform.x;
    station->y = directPlatform.y;
    station->z = directPlatform.z;

    auto* reservingTrain = createTrain(firstPos + Pos3{ 32, 0, 0 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    const OpenLoco::Vehicles::OrderStopAt order{ OpenLoco::StationId(0) };
    OpenLoco::Vehicles::OrderManager::insertOrder(reservingTrain, 0, &order);
    ASSERT_NE(createTrain(firstPos + Pos3{ -64, 0, 0 }, kStraightWest), nullptr);

    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, firstPos, kStraightWest));
}

TEST_F(PathSignalsTest, DepartureFromCurrentPathSignalWaitsForOccupiedRoute)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    addTrack(currentPos, 0, 0, true, SignalMode::path);
    addTrack(kFirstPos, 0, 0);
    auto* waitingTrain = createTrain(currentPos, kStraightWest | OpenLoco::World::Track::AdditionalTaDFlags::hasSignal);
    ASSERT_NE(waitingTrain, nullptr);
    ASSERT_NE(createTrain(kFirstPos, kStraightWest), nullptr);

    const auto result = waitingTrain->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 3);
}

TEST_F(PathSignalsTest, AdjacentPathSignalsReserveOnlyOnce)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 endSignalPos{ 256, 320, 32 };
    addTrack(currentPos, 0, 0, true, SignalMode::path);
    addTrack(kFirstPos, 0, 0, true, SignalMode::path);
    addTrack({ 288, 320, 32 }, 0, 0);
    addTrack(endSignalPos, 0, 0, true);
    auto* train = createTrain(currentPos, kStraightWest | OpenLoco::World::Track::AdditionalTaDFlags::hasSignal);
    ASSERT_NE(train, nullptr);

    const auto result = train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 0);
    auto nextHandle = train->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_TRUE(OpenLoco::Vehicles::RoutingManager::isPathReserved(nextHandle));
}

TEST_F(PathSignalsTest, CurrentPathSignalTriesAlternateJunctionBranch)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    addTrack(currentPos, 0, 0, true, SignalMode::path);
    addTrack(kFirstPos, 0, 0);
    addTrack(kFirstPos, 2, 0);
    addTrack({ 288, 320, 32 }, 0, 0);
    addTrack({ 256, 320, 32 }, 0, 0);
    auto* train = createTrain(currentPos, kStraightWest | OpenLoco::World::Track::AdditionalTaDFlags::hasSignal);
    ASSERT_NE(train, nullptr);
    ASSERT_NE(createTrain({ 256, 320, 32 }, kStraightWest), nullptr);

    const auto result = train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 0);
    auto nextHandle = train->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(nextHandle) & OpenLoco::World::Track::AdditionalTaDFlags::basicTaDMask, kTurnNorth);
}

TEST_F(PathSignalsTest, CurrentPathSignalWaitsInsteadOfIgnoringWaypoint)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 waypointPos{ 288, 320, 32 };
    addTrack(currentPos, 0, 0, true, SignalMode::path);
    addTrack(kFirstPos, 0, 0);
    addTrack(kFirstPos, 2, 0);
    addTrack(waypointPos, 0, 0);
    addTrack({ 256, 320, 32 }, 0, 0);
    auto* train = createTrain(currentPos, kStraightWest | OpenLoco::World::Track::AdditionalTaDFlags::hasSignal);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderRouteWaypoint waypoint{ toTileSpace(waypointPos), waypointPos.z / 8, 0, 0 };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &waypoint);
    ASSERT_NE(createTrain({ 256, 320, 32 }, kStraightWest), nullptr);

    const auto result = train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 3);
    auto nextHandle = train->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(nextHandle), OpenLoco::Vehicles::RoutingManager::kAllocatedButFreeRouting);
}

TEST_F(PathSignalsTest, RejectsRouteWhenNextSignalTrackIsOccupied)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 nextSignalPos{ 288, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(kFirstPos, 0, 0);
    addTrack(nextSignalPos, 0, 0, true, SignalMode::path);
    auto* reservingTrain = createTrain(currentPos, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);
    ASSERT_NE(createTrain(nextSignalPos, kStraightWest), nullptr);

    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));
}

TEST_F(PathSignalsTest, AdjacentDiagonalTrackDoesNotBlockReservation)
{
    constexpr uint16_t diagonalSouthWest = 1 << 3;
    constexpr uint16_t diagonalNorthEast = diagonalSouthWest | (1 << 2);
    constexpr Pos3 currentPos{ 352, 288, 32 };
    constexpr Pos3 firstPos{ 320, 320, 32 };
    constexpr Pos3 secondPos{ 288, 352, 32 };
    constexpr Pos3 adjacentTrackStart{ 320, 352, 32 };
    constexpr Pos3 adjacentTrainPos{ 288, 384, 32 };

    addTrack(currentPos, 1, 0);
    addTrack(firstPos, 1, 0);
    addTrack(secondPos, 1, 0);
    addTrack(adjacentTrackStart, 1, 0);
    auto* reservingTrain = createTrain(currentPos, diagonalSouthWest);
    ASSERT_NE(reservingTrain, nullptr);
    ASSERT_NE(createTrain(adjacentTrainPos, diagonalNorthEast), nullptr);

    EXPECT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, firstPos, diagonalSouthWest));
}

TEST_F(PathSignalsTest, SameDiagonalTrackBlocksReservation)
{
    constexpr uint16_t diagonalSouthWest = 1 << 3;
    constexpr Pos3 currentPos{ 352, 288, 32 };
    constexpr Pos3 firstPos{ 320, 320, 32 };
    constexpr Pos3 blockedPos{ 288, 352, 32 };

    addTrack(currentPos, 1, 0);
    addTrack(firstPos, 1, 0);
    addTrack(blockedPos, 1, 0);
    auto* reservingTrain = createTrain(currentPos, diagonalSouthWest);
    ASSERT_NE(reservingTrain, nullptr);
    ASSERT_NE(createTrain(blockedPos, diagonalSouthWest), nullptr);

    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, firstPos, diagonalSouthWest));
}

TEST_F(PathSignalsTest, ChoosesShorterRouteWhenBothRoutesAreClear)
{
    addTwoRouteJunction();
    auto* reservingTrain = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(reservingTrain, nullptr);

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*reservingTrain, kFirstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*reservingTrain), kStraightWest);
}

TEST_F(PathSignalsTest, LooksBeyondStationWhenReservingPlatform)
{
    constexpr Pos3 junction{ 288, 320, 32 };
    constexpr Pos3 deadEndPlatform{ 256, 320, 32 };
    constexpr Pos3 throughPlatform{ 288, 288, 32 };
    constexpr Pos3 nextStation{ 288, 256, 32 };

    addTrack(kFirstPos, 0, 0);
    addTrack(junction, 0, 0);
    addTrack(deadEndPlatform, 0, 0);
    addTrack(junction, 2, 0);
    addTrack(throughPlatform, 0, 3);
    addTrack(nextStation, 0, 3);
    addTrack({ 288, 224, 32 }, 0, 3, true);
    addTrainStation(deadEndPlatform, 0, 0, OpenLoco::StationId(0));
    addTrainStation(throughPlatform, 0, 3, OpenLoco::StationId(0));
    addTrainStation(nextStation, 0, 3, OpenLoco::StationId(1));

    auto* currentStation = OpenLoco::StationManager::get(OpenLoco::StationId(0));
    currentStation->x = deadEndPlatform.x;
    currentStation->y = deadEndPlatform.y;
    currentStation->z = deadEndPlatform.z;
    auto* onwardStation = OpenLoco::StationManager::get(OpenLoco::StationId(1));
    onwardStation->x = nextStation.x;
    onwardStation->y = nextStation.y;
    onwardStation->z = nextStation.z;

    auto* train = createTrain({ 352, 320, 32 }, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderStopAt currentOrder{ OpenLoco::StationId(0) };
    const OpenLoco::Vehicles::OrderStopAt onwardOrder{ OpenLoco::StationId(1) };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &currentOrder);
    OpenLoco::Vehicles::OrderManager::insertOrder(train, sizeof(currentOrder), &onwardOrder);

    auto* blockingTrain = createTrain(throughPlatform, 3);
    ASSERT_NE(blockingTrain, nullptr);
    EXPECT_FALSE(OpenLoco::Vehicles::PathSignals::tryReservePath(*train, kFirstPos, kStraightWest));
    blockingTrain->tileX = -1;

    ASSERT_TRUE(OpenLoco::Vehicles::PathSignals::tryReservePath(*train, kFirstPos, kStraightWest));

    EXPECT_EQ(getSecondReservedRouting(*train), kTurnNorth);
}

TEST(RailPathfindingResultTest, UsesProjectedTimeInsteadOfSignalClass)
{
    using namespace OpenLoco::Vehicles::RailPathfinding;

    const RouteResult disconnectedPlatform{ 0, 64, SignalState::signalClear, 1 };
    const RouteResult blockedRoute{ 0, 320, SignalState::signalBlockedTwoWay, 2 };
    const RouteResult localClearDetour{ 0, 640, SignalState::signalClear, 2 };
    const RouteResult excessiveClearDetour{ 0, 672, SignalState::signalClear, 2 };

    EXPECT_TRUE(isBetterRoute(disconnectedPlatform, blockedRoute));
    EXPECT_FALSE(isBetterRoute(blockedRoute, disconnectedPlatform));
    EXPECT_FALSE(isBetterRoute(blockedRoute, localClearDetour));
    EXPECT_TRUE(isBetterRoute(localClearDetour, blockedRoute));
    EXPECT_FALSE(isBetterRoute(blockedRoute, excessiveClearDetour));
    EXPECT_TRUE(isBetterRoute(excessiveClearDetour, blockedRoute));
}

TEST_F(RailPathfindingTest, DoesNotChooseDisconnectedTargetFallback)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    constexpr Pos3 junction{ 320, 320, 32 };
    constexpr auto wrongBranchLength = 5;
    constexpr auto northLength = 2;
    constexpr auto destinationDistance = 8;
    addTrack(currentPos, 0, 0);
    addTrack(junction, 0, 0);
    addTrack(junction, 2, 0);
    for (auto i = 1; i < wrongBranchLength; ++i)
    {
        addTrack(junction + Pos3{ -i * kTileSize, 0, 0 }, 0, 0);
    }
    for (auto i = 1; i <= northLength; ++i)
    {
        addTrack(junction + Pos3{ 0, -i * kTileSize, 0 }, 0, 3);
    }
    const auto westTurn = junction + Pos3{ 0, -(northLength + 1) * kTileSize, 0 };
    addTrack(westTurn, 3, 3);
    for (auto i = 1; i <= destinationDistance; ++i)
    {
        addTrack(westTurn + Pos3{ -i * kTileSize, 0, 0 }, 0, 0);
    }
    const auto targetPos = westTurn + Pos3{ -destinationDistance * kTileSize, 0, 0 };
    addTrainStation(targetPos, 0, 0, OpenLoco::StationId(0));
    auto* station = OpenLoco::StationManager::get(OpenLoco::StationId(0));
    station->x = targetPos.x;
    station->y = targetPos.y;
    station->z = targetPos.z;

    auto* train = createTrain(currentPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderStopAt order{ OpenLoco::StationId(0) };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &order);

    const auto result = train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 0);
    auto nextHandle = train->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(nextHandle) & OpenLoco::World::Track::AdditionalTaDFlags::basicTaDMask, kTurnNorth);
}

TEST_F(RailPathfindingTest, ReportsNoRouteWhenNoBranchReachesTarget)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(kFirstPos, 0, 0);
    addTrack(kFirstPos, 2, 0);
    auto* train = createTrain(currentPos, kStraightWest);
    ASSERT_NE(train, nullptr);
    const OpenLoco::Vehicles::OrderRouteWaypoint waypoint{ TilePos2{ 20, 20 }, 4, 0, 0 };
    OpenLoco::Vehicles::OrderManager::insertOrder(train, 0, &waypoint);

    const auto result = train->sub_4ACEE7(0xD4CB00, 0xD4CB00, false);

    EXPECT_EQ(result.status, 2);
    auto nextHandle = train->routingHandle;
    nextHandle.setIndex(nextHandle.getIndex() + 1);
    EXPECT_EQ(OpenLoco::Vehicles::RoutingManager::getRouting(nextHandle), OpenLoco::Vehicles::RoutingManager::kAllocatedButFreeRouting);
}

TEST_F(RailPathfindingTest, LooksBeyondStationToSelectConnectedPlatform)
{
    constexpr Pos3 junction{ 200 * kTileSize, 200 * kTileSize, 32 };
    constexpr Pos3 deadEndPlatform{ junction.x - kTileSize, junction.y, junction.z };
    constexpr Pos3 throughPlatform{ junction.x, junction.y - kTileSize, junction.z };
    constexpr Pos3 nextWaypoint{ throughPlatform.x, throughPlatform.y - kTileSize, throughPlatform.z };

    addTrack(junction, 0, 0);
    addTrack(deadEndPlatform, 0, 0);
    addTrack(junction, 2, 0);
    addTrack(throughPlatform, 0, 3);
    addTrack(nextWaypoint, 0, 3);

    addTrainStation(deadEndPlatform, 0, 0, OpenLoco::StationId(0));
    addTrainStation(throughPlatform, 0, 3, OpenLoco::StationId(0));

    OpenLoco::Vehicles::RailPathfinding::Target stationTarget{};
    stationTarget.stationId = OpenLoco::StationId(0);
    stationTarget.pos = deadEndPlatform;
    OpenLoco::Vehicles::RailPathfinding::Target waypointTarget{};
    waypointTarget.pos = nextWaypoint;
    waypointTarget.tad = 3;

    const auto deadEnd = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kStraightWest, OpenLoco::CompanyId(0), 0, 0, 0, stationTarget, &waypointTarget);
    const auto through = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kTurnNorth, OpenLoco::CompanyId(0), 0, 0, 0, stationTarget, &waypointTarget);

    EXPECT_EQ(deadEnd.numTargetsReached, 1);
    EXPECT_EQ(through.numTargetsReached, 2);
    EXPECT_TRUE(OpenLoco::Vehicles::RailPathfinding::isBetterRoute(deadEnd, through));
    EXPECT_FALSE(OpenLoco::Vehicles::RailPathfinding::isBetterRoute(through, deadEnd));
}

TEST_F(RailPathfindingTest, AssociatesStationWithSelectedJunctionBranch)
{
    constexpr Pos3 junction{ 200 * kTileSize, 200 * kTileSize, 32 };
    addTrack(junction, 0, 0);
    addTrack(junction, 2, 0);
    addTrainStation(junction, 0, 0, OpenLoco::StationId(0));

    OpenLoco::Vehicles::RailPathfinding::Target target{};
    target.stationId = OpenLoco::StationId(0);
    target.pos = junction;

    const auto platform = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kStraightWest, OpenLoco::CompanyId(0), 0, 0, 0, target);
    const auto bypass = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kTurnNorth, OpenLoco::CompanyId(0), 0, 0, 0, target);

    EXPECT_EQ(platform.numTargetsReached, 1);
    EXPECT_EQ(bypass.numTargetsReached, 0);
}

TEST_F(RailPathfindingTest, DoesNotReachWaypointBehindWrongWaySignal)
{
    addTrack(kFirstPos, 0, 0, true, SignalMode::oneWayPath);
    auto tile = TileManager::get(kFirstPos);
    const auto trackIt = std::ranges::find_if(tile, [](const auto& entry) { return entry.template as<TrackElement>() != nullptr; });
    ASSERT_NE(trackIt, tile.end());
    auto* track = trackIt->as<TrackElement>();
    auto* signal = trackIt->next()->as<SignalElement>();
    ASSERT_NE(signal, nullptr);
    signal->getLeft().setHasSignal(false);
    signal->getRight().setHasSignal(true);
    track->setRightSignalMode(SignalMode::oneWayPath);
    OpenLoco::Vehicles::RailPathfinding::Target target{};
    target.pos = kFirstPos;
    target.tad = kStraightWest;

    const auto result = OpenLoco::Vehicles::RailPathfinding::findRoute(kFirstPos, kStraightWest | OpenLoco::World::Track::AdditionalTaDFlags::hasSignal, OpenLoco::CompanyId(0), 0, 0, 0, target);

    EXPECT_EQ(result.numTargetsReached, 0);
}

TEST_F(RailPathfindingTest, AppliesSignalPenaltyBeforeSelectingGoal)
{
    constexpr Pos3 currentPos{ 352, 320, 32 };
    addTrack(currentPos, 0, 0);
    addTrack(kFirstPos, 0, 0, true);
    addTrack(kFirstPos, 2, 0, true);
    addTrainStation(kFirstPos, 0, 0, OpenLoco::StationId(0));
    addTrainStation(kFirstPos, 2, 0, OpenLoco::StationId(0));
    auto tile = TileManager::get(kFirstPos);
    const auto directTrack = std::ranges::find_if(tile, [](const auto& entry) {
        const auto* track = entry.template as<TrackElement>();
        return track != nullptr && track->trackId() == 0;
    });
    ASSERT_NE(directTrack, tile.end());
    auto* directSignal = directTrack->next()->as<SignalElement>();
    ASSERT_NE(directSignal, nullptr);
    directSignal->getLeft().setIsOccupied(true);

    OpenLoco::Vehicles::RailPathfinding::Target target{};
    target.stationId = OpenLoco::StationId(0);
    target.pos = kFirstPos;
    const OpenLoco::Vehicles::RailTraffic::SpeedProfile speedProfile{ OpenLoco::Speed16{ 60 }, OpenLoco::Speed16{ 60 } };

    const auto result = OpenLoco::Vehicles::RailPathfinding::findRoute(currentPos, kStraightWest, OpenLoco::CompanyId(0), 0, 0, 0, speedProfile, target);

    EXPECT_EQ(result.numTargetsReached, 1);
    EXPECT_EQ(result.signalState, OpenLoco::Vehicles::RailPathfinding::SignalState::signalClear);
    EXPECT_EQ(result.bestTrackWeighting, OpenLoco::Vehicles::RailTraffic::getFreeFlowTime(speedProfile, kStraightWest, 0));
}

TEST_F(RailPathfindingTest, AdvancesConsecutiveTargetsOnSameStationTrack)
{
    addTrack(kFirstPos, 0, 0);
    addTrainStation(kFirstPos, 0, 0, OpenLoco::StationId(0));
    OpenLoco::Vehicles::RailPathfinding::Target waypoint{};
    waypoint.pos = kFirstPos;
    waypoint.tad = kStraightWest;
    OpenLoco::Vehicles::RailPathfinding::Target station{};
    station.stationId = OpenLoco::StationId(0);
    station.pos = kFirstPos;

    const auto result = OpenLoco::Vehicles::RailPathfinding::findRoute(kFirstPos, kStraightWest, OpenLoco::CompanyId(0), 0, 0, 0, waypoint, &station);

    EXPECT_EQ(result.numTargetsReached, 2);
}

TEST_F(RailPathfindingTest, FindsLongStationRouteThatInitiallyMovesAwayFromTarget)
{
    constexpr Pos3 junction{ 320 * kTileSize, 200 * kTileSize, 32 };
    constexpr auto wrongBranchLength = 60;
    constexpr auto northLength = 5;
    constexpr auto destinationDistance = 250;
    addTrack(junction, 0, 0);
    addTrack(junction, 2, 0);
    for (auto i = 1; i < wrongBranchLength; ++i)
    {
        addTrack(junction + Pos3{ -i * kTileSize, 0, 0 }, 0, 0);
    }

    for (auto i = 1; i <= northLength; ++i)
    {
        addTrack(junction + Pos3{ 0, -i * kTileSize, 0 }, 0, 3);
    }
    const auto westTurn = junction + Pos3{ 0, -(northLength + 1) * kTileSize, 0 };
    addTrack(westTurn, 3, 3);
    for (auto i = 1; i <= destinationDistance; ++i)
    {
        addTrack(westTurn + Pos3{ -i * kTileSize, 0, 0 }, 0, 0);
    }

    const auto targetPos = westTurn + Pos3{ -destinationDistance * kTileSize, 0, 0 };
    addTrainStation(targetPos, 0, 0, OpenLoco::StationId(0));

    OpenLoco::Vehicles::RailPathfinding::Target target{};
    target.stationId = OpenLoco::StationId(0);
    target.pos = targetPos;

    const auto direct = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kStraightWest, OpenLoco::CompanyId(0), 0, 0, 0, target);
    const auto detour = OpenLoco::Vehicles::RailPathfinding::findRoute(junction, kTurnNorth, OpenLoco::CompanyId(0), 0, 0, 0, target);

    EXPECT_EQ(direct.numTargetsReached, 0);
    ASSERT_EQ(detour.numTargetsReached, 1);
    EXPECT_TRUE(OpenLoco::Vehicles::RailPathfinding::isBetterRoute(direct, detour));
}

TEST_F(TileManagerTest, InsertElementAfterNoReorgTypedTemplateReturnsTypedPointer)
{
    auto* anchor = TileManager::insertElement(ElementType::track, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(anchor, nullptr);
    auto* station = TileManager::insertElementAfterNoReorg<StationElement>(anchor, toWorldSpace(kTestTile), 4, 0);
    ASSERT_NE(station, nullptr);
    EXPECT_EQ(station->type(), ElementType::station);
}

TEST_F(TileManagerTest, TileSurfaceReturnsTheSurfaceElement)
{
    auto tile = TileManager::get(kTestTile);
    auto* surface = tile.surface();
    ASSERT_NE(surface, nullptr);
    EXPECT_EQ(typeAt(tile, 0), ElementType::surface);
    EXPECT_EQ(surface->baseZ(), 4);
    EXPECT_EQ(static_cast<TileElement*>(surface), &TileManager::resolveEntry(tile[0]));
}

TEST_F(TileManagerTest, TileTrainStationFindsStationFollowingMatchingTrack)
{
    auto* trackEntry = TileManager::insertElement<TrackElement>(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(trackEntry, nullptr);
    auto& track = trackEntry->get<TrackElement>();
    track.setRotation(2);
    track.setTrackId(5);
    track.setHasStationElement(true);

    TileElementEntry* anchor = nullptr;
    for (auto& el : TileManager::get(kTestTile))
    {
        if (el.type() == ElementType::track)
        {
            anchor = &el;
            break;
        }
    }
    ASSERT_NE(anchor, nullptr);
    auto* stationEntry = TileManager::insertElementAfterNoReorg<StationElement>(anchor, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(stationEntry, nullptr);
    auto* station = stationEntry->as<StationElement>();

    auto tile = TileManager::get(kTestTile);
    auto* found = tile.trainStation(5, 2, 8);
    EXPECT_EQ(found, station);

    EXPECT_EQ(tile.trainStation(6, 2, 8), nullptr);
    EXPECT_EQ(tile.trainStation(5, 3, 8), nullptr);
    EXPECT_EQ(tile.trainStation(5, 2, 12), nullptr);
}

TEST_F(TileManagerTest, TileRoadStationFindsStationFollowingMatchingRoad)
{
    auto* roadEntry = TileManager::insertElementRoad(toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(roadEntry, nullptr);
    auto& road = roadEntry->get<RoadElement>();
    road.setRotation(1);
    road.setRoadId(3);
    road.setHasStationElement(true);

    TileElementEntry* anchor = nullptr;
    for (auto& el : TileManager::get(kTestTile))
    {
        if (el.type() == ElementType::road)
        {
            anchor = &el;
            break;
        }
    }
    ASSERT_NE(anchor, nullptr);
    auto* stationEntry = TileManager::insertElementAfterNoReorg<StationElement>(anchor, toWorldSpace(kTestTile), 8, 0);
    ASSERT_NE(stationEntry, nullptr);
    auto* station = stationEntry->as<StationElement>();

    auto tile = TileManager::get(kTestTile);
    auto* found = tile.roadStation(3, 1, 8);
    EXPECT_EQ(found, station);

    EXPECT_EQ(tile.roadStation(4, 1, 8), nullptr);
}

TEST_F(TileManagerTest, ManyInsertsTriggerReorganiseInternallyWithoutBreakingTile)
{
    constexpr int kInserts = 20000;
    for (int i = 0; i < kInserts; ++i)
    {
        const auto baseZ = static_cast<uint8_t>(8 + (i % 200));
        const TilePos2 t{ static_cast<int16_t>(i % 64), static_cast<int16_t>((i / 64) % 64) };
        ASSERT_NE(TileManager::insertElement(ElementType::track, toWorldSpace(t), baseZ, 0), nullptr);
    }

    for (int row = 0; row < 64; ++row)
    {
        for (int col = 0; col < 64; ++col)
        {
            auto tile = TileManager::get(TilePos2{ static_cast<int16_t>(col), static_cast<int16_t>(row) });
            ASSERT_GE(tile.size(), 1u);
            int lastCount = 0;
            uint8_t prev = 0;
            for (auto& el : tile)
            {
                EXPECT_GE(el.baseZ(), prev);
                prev = el.baseZ();
                if (el.isLast())
                {
                    ++lastCount;
                }
            }
            EXPECT_EQ(lastCount, 1);
        }
    }
}

TEST_F(TileManagerTest, ReorganiseReclaimsFragmentationGarbage)
{
    const auto freeAfterInit = TileManager::numFreeElements();
    for (int i = 0; i < 2000; ++i)
    {
        const TilePos2 t{ static_cast<int16_t>(i % 64), static_cast<int16_t>((i / 64) % 64) };
        ASSERT_NE(TileManager::insertElement(ElementType::track, toWorldSpace(t), static_cast<uint8_t>(8 + (i % 200)), 0), nullptr);
    }
    const auto freeAfterChurn = TileManager::numFreeElements();
    EXPECT_LT(freeAfterChurn, freeAfterInit);

    TileManager::reorganise();
    const auto freeAfterReorg = TileManager::numFreeElements();
    EXPECT_GT(freeAfterReorg, freeAfterChurn);
}

TEST_F(TileManagerTest, ReorganiseAfterChurnMatchesByteShadow)
{
    struct Step
    {
        enum class Kind
        {
            Insert,
            Remove,
        };
        Kind kind;
        TilePos2 tile;
        ElementType type{ ElementType::surface };
        uint8_t baseZ{ 0 };
    };

    const std::vector<Step> steps{
        { Step::Kind::Insert, TilePos2{ 5, 5 }, ElementType::track, 8 },
        { Step::Kind::Insert, TilePos2{ 5, 5 }, ElementType::tree, 16 },
        { Step::Kind::Insert, TilePos2{ 5, 6 }, ElementType::track, 12 },
        { Step::Kind::Insert, TilePos2{ 6, 5 }, ElementType::tree, 24 },
        { Step::Kind::Insert, TilePos2{ 5, 5 }, ElementType::track, 32 },
        { Step::Kind::Remove, TilePos2{ 5, 5 }, ElementType::tree, 16 },
        { Step::Kind::Insert, TilePos2{ 6, 5 }, ElementType::track, 8 },
        { Step::Kind::Remove, TilePos2{ 6, 5 }, ElementType::tree, 24 },
        { Step::Kind::Insert, TilePos2{ 50, 50 }, ElementType::track, 8 },
    };

    for (const auto& s : steps)
    {
        if (s.kind == Step::Kind::Insert)
        {
            ASSERT_NE(TileManager::insertElement(s.type, toWorldSpace(s.tile), s.baseZ, 0), nullptr);
        }
        else
        {
            TileElementEntry* victim = nullptr;
            for (auto& el : TileManager::get(s.tile))
            {
                if (el.type() == s.type && el.baseZ() == s.baseZ)
                {
                    victim = &el;
                    break;
                }
            }
            ASSERT_NE(victim, nullptr);
            TileManager::removeElement(*victim);
        }
    }

    const std::vector<TilePos2> tilesOfInterest{
        TilePos2{ 5, 5 }, TilePos2{ 5, 6 }, TilePos2{ 6, 5 }, TilePos2{ 50, 50 }
    };
    std::vector<TileBytes> before;
    for (auto t : tilesOfInterest)
    {
        before.push_back(snapshotBytes(TileManager::get(t)));
    }

    TileManager::reorganise();

    for (size_t i = 0; i < tilesOfInterest.size(); ++i)
    {
        EXPECT_EQ(before[i], snapshotBytes(TileManager::get(tilesOfInterest[i])))
            << "tile (" << tilesOfInterest[i].x << "," << tilesOfInterest[i].y << ") diverged after reorganise";
    }
}
