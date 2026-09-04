#include "Entities/EntityManager.h"
#include "Vehicles/RailPathfinding.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleTail.h"
#include <array>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::Literals;
using namespace OpenLoco::Vehicles;

namespace
{
    class VehicleMovementTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            EntityManager::reset();
        }

        void TearDown() override
        {
            EntityManager::reset();
        }

        template<typename T>
        static T* createComponent()
        {
            auto* entity = EntityManager::createEntityVehicle();
            if (entity == nullptr)
            {
                return nullptr;
            }
            entity->baseType = EntityBaseType::vehicle;
            auto* component = reinterpret_cast<VehicleBase*>(entity);
            component->setSubType(T::kVehicleThingType);
            return static_cast<T*>(component);
        }
    };
}

TEST_F(VehicleMovementTest, StationaryLandBodySkipsGeometryButAdvancesAnimation)
{
    auto* head = createComponent<VehicleHead>();
    auto* veh1 = createComponent<Vehicle1>();
    auto* veh2 = createComponent<Vehicle2>();
    auto* front = createComponent<VehicleBogie>();
    auto* back = createComponent<VehicleBogie>();
    auto* body = createComponent<VehicleBody>();
    auto* tail = createComponent<VehicleTail>();
    ASSERT_NE(head, nullptr);
    ASSERT_NE(veh1, nullptr);
    ASSERT_NE(veh2, nullptr);
    ASSERT_NE(front, nullptr);
    ASSERT_NE(back, nullptr);
    ASSERT_NE(body, nullptr);
    ASSERT_NE(tail, nullptr);

    body->setSubType(VehicleEntityType::body_start);
    const std::array<VehicleBase*, 7> components = { head, veh1, veh2, front, back, body, tail };
    for (size_t i = 0; i < std::size(components); ++i)
    {
        components[i]->head = head->id;
        components[i]->position = { 64, 64, 16 };
        components[i]->mode = TransportMode::air;
        components[i]->setNextCar(i + 1 < std::size(components) ? components[i + 1]->id : EntityId::null);
    }

    head->mode = TransportMode::road;
    head->status = Status::stopped;
    head->vehicleFlags = VehicleFlags::commandStop;
    veh2->sound.objectId = 0xFFFF;
    tail->sound.objectId = 0xFFFF;
    veh2->currentSpeed = 8.0_mph;

    front->position = { 64, 64, 16 };
    back->position = { 72, 64, 16 };
    body->position = { 68, 64, 16 };
    body->mode = TransportMode::road;
    body->var_38 = Flags38::isGhost;
    body->objectSpriteType = 0xFF;
    // moveTo refreshes this bound even when the world position is unchanged.
    body->spriteLeft = 12345;
    const auto initialAnimationProgress = body->var_44;

    head->updateVehicle();

    EXPECT_EQ(body->position, (World::Pos3{ 68, 64, 16 }));
    EXPECT_EQ(body->spriteLeft, 12345);
    EXPECT_NE(body->var_44, initialAnimationProgress);
}

TEST(VehicleMovement, OccupiedContinuationCreatesAnotherRoadStationStoppingPosition)
{
    constexpr auto currentStation = StationId(1);

    EXPECT_FALSE(isRoadStationStoppingPosition(currentStation, currentStation, RoadOccupationFlags::none));
    EXPECT_FALSE(isRoadStationStoppingPosition(currentStation, currentStation, RoadOccupationFlags::isLevelCrossingClosed));
    EXPECT_TRUE(isRoadStationStoppingPosition(currentStation, currentStation, RoadOccupationFlags::isLaneOccupied));
    EXPECT_TRUE(isRoadStationStoppingPosition(currentStation, StationId(2), RoadOccupationFlags::none));
    EXPECT_FALSE(isRoadStationStoppingPosition(StationId::null, StationId::null, RoadOccupationFlags::isLaneOccupied));
}

TEST(VehicleMovement, RoadVehicleCanReverseInCurrentLane)
{
    TrackAndDirection::_RoadAndDirection tad{ 0, 0 };
    EXPECT_TRUE(canReverseRoadVehicleInCurrentLane(tad));

    tad._data = (1U << 7);
    EXPECT_TRUE(canReverseRoadVehicleInCurrentLane(tad));

    tad._data = (1U << 8);
    EXPECT_FALSE(canReverseRoadVehicleInCurrentLane(tad));

    tad._data = (1U << 7) | (1U << 8);
    EXPECT_TRUE(canReverseRoadVehicleInCurrentLane(tad));
}

TEST(VehicleMovement, OrdinaryRoadVehicleCanYieldFromDifferentRoadPiece)
{
    TrackAndDirection::_RoadAndDirection headTad{ 0, 0 };
    TrackAndDirection::_RoadAndDirection vehicleTad{ 1, 0 };

    EXPECT_TRUE(canYieldByReversingInCurrentLane(headTad, vehicleTad, true));
    EXPECT_FALSE(canYieldByReversingInCurrentLane(headTad, vehicleTad, false));

    vehicleTad = headTad;
    EXPECT_TRUE(canYieldByReversingInCurrentLane(headTad, vehicleTad, false));

    vehicleTad._data = 1U << 8;
    EXPECT_FALSE(canYieldByReversingInCurrentLane(headTad, vehicleTad, true));

    vehicleTad = headTad;
    headTad._data = 1U << 8;
    EXPECT_FALSE(canYieldByReversingInCurrentLane(headTad, vehicleTad, true));
}

TEST(VehicleMovement, RoadVehicleResetsTurnaroundTimeoutForRetry)
{
    constexpr uint16_t kFirstTimeout = 160;
    constexpr uint16_t kTurnaroundTimeout = 960;

    auto elapsed = static_cast<uint16_t>(kFirstTimeout - 1);
    EXPECT_EQ(advanceRoadSignalTimeout(elapsed, kFirstTimeout, kTurnaroundTimeout), SignalTimeoutStatus::firstTimeout);
    EXPECT_EQ(elapsed, kFirstTimeout);

    elapsed = kTurnaroundTimeout - 1;
    EXPECT_EQ(advanceRoadSignalTimeout(elapsed, kFirstTimeout, kTurnaroundTimeout), SignalTimeoutStatus::turnaroundAtSignalTimeout);
    EXPECT_EQ(elapsed, 0);
    EXPECT_EQ(advanceRoadSignalTimeout(elapsed, kFirstTimeout, kTurnaroundTimeout), SignalTimeoutStatus::ok);
    EXPECT_EQ(elapsed, 1);

    elapsed = 10792;
    EXPECT_EQ(advanceRoadSignalTimeout(elapsed, kFirstTimeout, kTurnaroundTimeout), SignalTimeoutStatus::turnaroundAtSignalTimeout);
    EXPECT_EQ(elapsed, 0);

    elapsed = std::numeric_limits<uint16_t>::max();
    EXPECT_EQ(advanceRoadSignalTimeout(elapsed, kFirstTimeout, kTurnaroundTimeout), SignalTimeoutStatus::turnaroundAtSignalTimeout);
    EXPECT_EQ(elapsed, 0);
}

TEST(VehicleMovement, RoadPathingRetainsBestRecursiveResult)
{
    using RailPathfinding::RouteResult;
    using RailPathfinding::SignalState;

    RouteResult result{ 64, 64, SignalState::null };
    const RouteResult routeToTarget{ 0, 128, SignalState::noSignals };
    const RouteResult laterDeadEnd{ 32, 32, SignalState::null };

    mergeRoadRoutingResult(result, routeToTarget);
    mergeRoadRoutingResult(result, laterDeadEnd);

    EXPECT_EQ(result.bestDistToTarget, 0);
    EXPECT_EQ(result.bestTrackWeighting, 128);
    EXPECT_EQ(result.signalState, SignalState::noSignals);
}
