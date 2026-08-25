#include <OpenLoco/Date.h>
#include <OpenLoco/Entities/EntityManager.h>
#include <OpenLoco/GameState.h>
#include <OpenLoco/Map/Track/TrackData.h>
#include <OpenLoco/Map/TrackElement.h>
#include <OpenLoco/Math/Vector.hpp>
#include <OpenLoco/Scenario/ScenarioManager.h>
#include <OpenLoco/Ui/Windows/RailSpeedOverlay.h>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/Vehicle.h>
#include <OpenLoco/Vehicles/Vehicle1.h>
#include <OpenLoco/Vehicles/VehicleHead.h>
#include <array>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::Literals;
using namespace OpenLoco::Vehicles;

namespace
{
    class RailTrafficTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            EntityManager::reset();
            RailTraffic::reset();
            ScenarioManager::setScenarioTicks(0);
            setCurrentDay(0);
        }

        void TearDown() override
        {
            RailTraffic::reset();
            EntityManager::reset();
        }

        template<typename T>
        static T* createVehicleComponent()
        {
            auto* entity = EntityManager::createEntityVehicle();
            EXPECT_NE(entity, nullptr);
            if (entity == nullptr)
            {
                return nullptr;
            }
            entity->baseType = EntityBaseType::vehicle;
            auto* vehicle = entity->asBase<VehicleBase>();
            vehicle->setSubType(T::kVehicleThingType);
            return static_cast<T*>(vehicle);
        }

        static std::pair<VehicleHead*, Vehicle1*> createRailVehicle(const RailTraffic::Edge& edge)
        {
            auto* head = createVehicleComponent<VehicleHead>();
            auto* vehicle = createVehicleComponent<Vehicle1>();
            if (head == nullptr || vehicle == nullptr)
            {
                return { head, vehicle };
            }
            for (auto* component : std::array<VehicleBase*, 2>{ head, vehicle })
            {
                component->tileX = edge.x;
                component->tileY = edge.y;
                component->tileBaseZ = edge.z / World::kSmallZStep;
                component->trackType = edge.trackType;
                component->mode = TransportMode::rail;
                component->head = head->id;
                component->trackAndDirection = TrackAndDirection(edge.tad >> 3, edge.tad & 0x7);
            }
            return { head, vehicle };
        }

        static void moveToEdge(Vehicle1& vehicle, const RailTraffic::Edge& edge)
        {
            vehicle.tileX = edge.x;
            vehicle.tileY = edge.y;
            vehicle.tileBaseZ = edge.z / World::kSmallZStep;
            vehicle.trackType = edge.trackType;
            vehicle.trackAndDirection = TrackAndDirection(edge.tad >> 3, edge.tad & 0x7);
        }
    };

    constexpr RailTraffic::Edge kEdge{ 320, 320, 32, 0, 0 };
    constexpr RailTraffic::Edge kSecondEdge{ 288, 320, 32, 0, 0 };
    constexpr RailTraffic::Edge kThirdEdge{ 256, 320, 32, 0, 0 };
}

TEST_F(RailTrafficTest, RawTraversalSpeedCapsOnlyFasterTrains)
{
    const RailTraffic::SpeedProfile fast{ 100_mph, 100_mph };
    const RailTraffic::SpeedProfile slow{ 20_mph, 20_mph };
    const auto fastFreeFlow = RailTraffic::getFreeFlowTime(fast, 0, 0);
    const auto slowFreeFlow = RailTraffic::getFreeFlowTime(slow, 0, 0);
    const auto thirtyMphTraversal = (uint64_t{ 32 } * 21 * RailTraffic::kOneTick + 29) / 30;

    for (auto i = 0; i < 16; ++i)
    {
        RailTraffic::recordTraversal(kEdge, thirtyMphTraversal);
    }

    EXPECT_GT(RailTraffic::getTravelTime(fast, { 320, 320, 32 }, 0, 0), fastFreeFlow);
    EXPECT_EQ(RailTraffic::getTravelTime(slow, { 320, 320, 32 }, 0, 0), slowFreeFlow);
}

TEST_F(RailTrafficTest, ReportsMeasuredAverageSpeed)
{
    EXPECT_FALSE(RailTraffic::getAverageSpeed(kEdge).has_value());

    const auto thirtyMphTraversal = (uint64_t{ 32 } * 21 * RailTraffic::kOneTick + 29) / 30;
    RailTraffic::recordTraversal(kEdge, thirtyMphTraversal);

    const auto speed = RailTraffic::getAverageSpeed(kEdge);
    ASSERT_TRUE(speed.has_value());
    EXPECT_EQ(*speed, 30_mph);
}

TEST_F(RailTrafficTest, RetainsOverlayHistoryAfterRoutingConfidenceDecays)
{
    const RailTraffic::SpeedProfile profile{ 60_mph, 60_mph };
    const auto freeFlow = RailTraffic::getFreeFlowTime(profile, kEdge.tad, kEdge.trackType);
    RailTraffic::recordTraversal(kEdge, 30 * RailTraffic::kOneTick);

    setCurrentDay(1);
    RailTraffic::updateDaily();
    ASSERT_TRUE(RailTraffic::getAverageSpeed(kEdge).has_value());
    EXPECT_EQ(RailTraffic::getTravelTime(profile, { kEdge.x, kEdge.y, kEdge.z }, kEdge.tad, kEdge.trackType), freeFlow);

    for (uint32_t day = 2; day <= 30; ++day)
    {
        setCurrentDay(day);
        RailTraffic::updateDaily();
    }
    EXPECT_TRUE(RailTraffic::getAverageSpeed(kEdge).has_value());

    setCurrentDay(31);
    RailTraffic::updateDaily();
    EXPECT_FALSE(RailTraffic::getAverageSpeed(kEdge).has_value());
}

TEST_F(RailTrafficTest, DropsHistoryAfterDateMovesBackwards)
{
    setCurrentDay(1);
    RailTraffic::recordTraversal(kEdge, 12 * RailTraffic::kOneTick);

    setCurrentDay(0);
    RailTraffic::updateDaily();

    EXPECT_FALSE(RailTraffic::getAverageSpeed(kEdge).has_value());
}

TEST_F(RailTrafficTest, OverlayUsesSlowestObservedDirection)
{
    constexpr RailTraffic::Edge forwardEdge{ 320, 320, 32, 4 << 3, 0 };
    constexpr RailTraffic::Edge reverseEdge{ 288, 288, 32, (4 << 3) | 4, 0 };
    const World::TrackElement track(8, 8, 0, 0, 0, 0, 4, std::nullopt, CompanyId{ 0 }, 0);
    RailTraffic::recordTraversal(reverseEdge, 15 * RailTraffic::kOneTick);

    EXPECT_EQ(Ui::Windows::RailSpeedOverlay::getTrackSpeed({ 320, 320 }, track), RailTraffic::getAverageSpeed(reverseEdge));

    RailTraffic::recordTraversal(forwardEdge, 5 * RailTraffic::kOneTick);

    EXPECT_EQ(Ui::Windows::RailSpeedOverlay::getTrackSpeed({ 320, 320 }, track), RailTraffic::getAverageSpeed(reverseEdge));
}

TEST_F(RailTrafficTest, OverlayMapsEveryTrackTileToTheWholePiece)
{
    constexpr World::Pos3 start{ 1024, 1024, 128 };
    for (uint8_t trackId = 0; trackId < World::TrackData::kTrackPieceCount; ++trackId)
    {
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const RailTraffic::Edge edge{ start.x, start.y, start.z, static_cast<uint16_t>((trackId << 3) | rotation), 0 };
            RailTraffic::recordTraversal(edge, 12 * RailTraffic::kOneTick);
            const auto expected = RailTraffic::getAverageSpeed(edge);
            for (const auto& piece : World::TrackData::getTrackPiece(trackId))
            {
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, rotation);
                const auto baseZ = static_cast<World::SmallZ>((start.z + piece.z) / World::kSmallZStep);
                const World::TrackElement track(baseZ, baseZ, rotation, 0, piece.index, 0, trackId, std::nullopt, CompanyId{ 0 }, 0);
                EXPECT_EQ(Ui::Windows::RailSpeedOverlay::getTrackSpeed(World::Pos2{ start.x, start.y } + offset, track), expected)
                    << "track=" << static_cast<int>(trackId) << " rotation=" << static_cast<int>(rotation) << " sequence=" << static_cast<int>(piece.index);
            }
        }
    }
}

TEST_F(RailTrafficTest, OverlayFindsReverseEdgeWithDiagonalEndpoint)
{
    constexpr RailTraffic::Edge reverseEdge{ 256, 288, 32, (8 << 3) | 4, 0 };
    const World::TrackElement track(8, 8, 0, 0, 0, 0, 8, std::nullopt, CompanyId{ 0 }, 0);
    RailTraffic::recordTraversal(reverseEdge, 12 * RailTraffic::kOneTick);

    EXPECT_EQ(Ui::Windows::RailSpeedOverlay::getTrackSpeed({ 320, 320 }, track), RailTraffic::getAverageSpeed(reverseEdge));
}

TEST(RailSpeedOverlay, UsesAbsoluteSpeedBands)
{
    using Ui::Windows::RailSpeedOverlay::getSpeedBucket;
    EXPECT_EQ(getSpeedBucket(0_mph), 1);
    EXPECT_EQ(getSpeedBucket(9_mph), 1);
    EXPECT_EQ(getSpeedBucket(10_mph), 2);
    EXPECT_EQ(getSpeedBucket(29_mph), 3);
    EXPECT_EQ(getSpeedBucket(30_mph), 4);
    EXPECT_EQ(getSpeedBucket(45_mph), 5);
    EXPECT_EQ(getSpeedBucket(60_mph), 6);
    EXPECT_EQ(getSpeedBucket(80_mph), 7);
    EXPECT_EQ(getSpeedBucket(120_mph), 8);
}

TEST_F(RailTrafficTest, TraversalIncludesAllElapsedOccupancyTime)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);

    ScenarioManager::setScenarioTicks(10);
    RailTraffic::reconcile(*vehicle);
    RailTraffic::onPieceTransition(*vehicle, kEdge, kSecondEdge, RailTraffic::kOneTick / 4);

    head->status = Status::loading;
    ScenarioManager::setScenarioTicks(15);
    RailTraffic::onPieceTransition(*vehicle, kSecondEdge, kThirdEdge, 3 * RailTraffic::kOneTick / 4);

    const auto state = RailTraffic::captureState();
    ASSERT_EQ(state.history.size(), 1U);
    EXPECT_EQ(state.history.front().edge, kSecondEdge);
    EXPECT_EQ(state.history.front().meanTraversalTime, 5 * RailTraffic::kOneTick + RailTraffic::kOneTick / 2);
}

TEST_F(RailTrafficTest, StateRoundTripsAndRejectsDuplicateEdges)
{
    RailTraffic::recordTraversal(kEdge, 12 * RailTraffic::kOneTick);
    const auto state = RailTraffic::captureState();
    ASSERT_TRUE(RailTraffic::validateState(state));

    RailTraffic::reset();
    ASSERT_TRUE(RailTraffic::restoreState(state));
    EXPECT_EQ(RailTraffic::captureState(), state);

    auto duplicate = state;
    duplicate.history.push_back(duplicate.history.front());
    EXPECT_FALSE(RailTraffic::validateState(duplicate));
}

TEST_F(RailTrafficTest, RestartDiscardsPartialTraversalAtSameEdge)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);
    ScenarioManager::setScenarioTicks(10);
    RailTraffic::reconcile(*vehicle);
    RailTraffic::onPieceTransition(*vehicle, kEdge, kSecondEdge, 0);
    moveToEdge(*vehicle, kSecondEdge);
    ScenarioManager::setScenarioTicks(15);
    RailTraffic::restart(*vehicle);
    ScenarioManager::setScenarioTicks(20);
    RailTraffic::onPieceTransition(*vehicle, kSecondEdge, kThirdEdge, 0);
    moveToEdge(*vehicle, kThirdEdge);

    EXPECT_TRUE(RailTraffic::captureState().history.empty());
}

TEST_F(RailTrafficTest, ActiveStateValidatesAndRestoresPublishedOccupancy)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);
    ScenarioManager::setScenarioTicks(9);
    RailTraffic::reconcile(*vehicle);
    ScenarioManager::setScenarioTicks(10);
    RailTraffic::beginTick();
    RailTraffic::onPieceTransition(*vehicle, kEdge, kSecondEdge, RailTraffic::kOneTick / 2);
    moveToEdge(*vehicle, kSecondEdge);
    RailTraffic::endTick();
    const auto state = RailTraffic::captureState();
    ASSERT_TRUE(RailTraffic::validateState(state, getGameState()));

    ScenarioManager::setScenarioTicks(30);
    ASSERT_TRUE(RailTraffic::restoreState(state));
    const RailTraffic::SpeedProfile profile{ 60_mph, 60_mph };
    EXPECT_GT(RailTraffic::getTravelTime(profile, { kSecondEdge.x, kSecondEdge.y, kSecondEdge.z }, kSecondEdge.tad, kSecondEdge.trackType), RailTraffic::getFreeFlowTime(profile, kSecondEdge.tad, kSecondEdge.trackType));

    auto invalid = state;
    invalid.active.front().edge.x -= World::kTileSize;
    EXPECT_FALSE(RailTraffic::validateState(invalid, getGameState()));
    invalid = state;
    invalid.active.front().enteredAt = 31 * RailTraffic::kOneTick;
    EXPECT_FALSE(RailTraffic::validateState(invalid, getGameState()));
    invalid = state;
    invalid.active.front().enteredAt |= uint64_t{ 1 } << 63;
    EXPECT_FALSE(RailTraffic::validateState(invalid));
}

TEST_F(RailTrafficTest, TraversalTimingSurvivesScenarioTickWrap)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);
    ScenarioManager::setScenarioTicks(std::numeric_limits<uint32_t>::max() - 1);
    RailTraffic::reconcile(*vehicle);
    ScenarioManager::setScenarioTicks(std::numeric_limits<uint32_t>::max());
    RailTraffic::beginTick();
    RailTraffic::onPieceTransition(*vehicle, kEdge, kSecondEdge, RailTraffic::kOneTick);
    moveToEdge(*vehicle, kSecondEdge);
    RailTraffic::endTick();
    ScenarioManager::setScenarioTicks(0);
    RailTraffic::beginTick();
    RailTraffic::onPieceTransition(*vehicle, kSecondEdge, kThirdEdge, RailTraffic::kOneTick / 2);
    moveToEdge(*vehicle, kThirdEdge);
    RailTraffic::endTick();

    const auto state = RailTraffic::captureState();
    ASSERT_EQ(state.history.size(), 1U);
    EXPECT_EQ(state.history.front().edge, kSecondEdge);
    EXPECT_EQ(state.history.front().meanTraversalTime, RailTraffic::kOneTick / 2);
}

TEST_F(RailTrafficTest, TickStartHandlesTraversalRestartedByCommand)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);
    const RailTraffic::SpeedProfile profile{ 60_mph, 60_mph };

    ScenarioManager::setScenarioTicks(10);
    RailTraffic::restart(*vehicle);
    RailTraffic::beginTick();

    EXPECT_EQ(RailTraffic::getTravelTime(profile, { kEdge.x, kEdge.y, kEdge.z }, kEdge.tad, kEdge.trackType), RailTraffic::getFreeFlowTime(profile, kEdge.tad, kEdge.trackType));
    RailTraffic::endTick();
}

TEST_F(RailTrafficTest, TickStartSeedsPlacedVehiclesThatSkippedUpdates)
{
    auto [head, vehicle] = createRailVehicle(kEdge);
    ASSERT_NE(head, nullptr);
    ASSERT_NE(vehicle, nullptr);

    ScenarioManager::setScenarioTicks(10);
    RailTraffic::beginTick();

    const auto state = RailTraffic::captureState();
    ASSERT_EQ(state.active.size(), 1U);
    EXPECT_EQ(state.active.front().vehicle, vehicle->id);
    RailTraffic::endTick();
}
