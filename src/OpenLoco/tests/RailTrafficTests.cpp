#include <OpenLoco/Entities/EntityManager.h>
#include <OpenLoco/GameState.h>
#include <OpenLoco/Scenario/ScenarioManager.h>
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
