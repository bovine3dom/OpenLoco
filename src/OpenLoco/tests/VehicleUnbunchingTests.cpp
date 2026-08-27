#include "Entities/EntityManager.h"
#include "GameCommands/Vehicles/VehicleOrderInsert.h"
#include "GameCommands/Vehicles/VehicleOrderToggleUnbunching.h"
#include "S5/S5Entity.h"
#include "Scenario/ScenarioManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/VehicleHead.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::GameCommands;
using namespace OpenLoco::Vehicles;

namespace
{
    class VehicleUnbunchingTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            EntityManager::reset();
            OrderManager::reset();
            ScenarioManager::setScenarioTicks(1);
            setUpdatingCompanyId(CompanyId(0));
        }

        static VehicleHead& createVehicle(const CompanyId owner = CompanyId(0), const VehicleType type = VehicleType::train, const bool express = false)
        {
            auto* headEntity = EntityManager::createEntityVehicle();
            headEntity->baseType = EntityBaseType::vehicle;
            auto* headBase = reinterpret_cast<VehicleBase*>(headEntity);
            headBase->setSubType(VehicleEntityType::head);
            auto* head = headBase->asVehicleHead();
            head->head = head->id;
            head->owner = owner;
            head->vehicleType = type;
            head->status = Status::travelling;
            head->stationId = StationId::null;
            head->trainAcceptedCargoTypes = std::numeric_limits<uint32_t>::max();
            EntityManager::moveEntityToList(head, EntityManager::EntityListType::vehicleHead);

            auto* vehicle1Entity = EntityManager::createEntityVehicle();
            vehicle1Entity->baseType = EntityBaseType::vehicle;
            auto* vehicle1Base = reinterpret_cast<VehicleBase*>(vehicle1Entity);
            vehicle1Base->setSubType(VehicleEntityType::vehicle_1);
            auto* vehicle1 = vehicle1Base->asVehicle1();
            vehicle1->head = head->id;
            vehicle1->var_48 = express ? Flags48::expressMode : Flags48::none;
            head->nextCarId = vehicle1->id;

            OrderManager::allocateOrders(*head);
            return *head;
        }

        static uint16_t appendOrder(VehicleHead& head, const Order& order)
        {
            const auto offset = head.sizeOfOrderTable - sizeof(OrderEnd);
            OrderManager::insertOrder(&head, offset, &order);
            return offset;
        }

        static uint16_t appendStop(VehicleHead& head, const uint16_t station)
        {
            const OrderStopAt order{ StationId(station) };
            return appendOrder(head, order);
        }

        static void addRoute(VehicleHead& head, const uint16_t firstStation = 1, const uint16_t secondStation = 2)
        {
            appendStop(head, firstStation);
            appendStop(head, secondStation);
        }

        static uint32_t toggleUnbunching(VehicleHead& head, const uint32_t orderOffset, const uint8_t flags)
        {
            VehicleOrderToggleUnbunchingArgs args{};
            args.head = head.id;
            args.orderOffset = orderOffset;
            auto regs = static_cast<registers>(args);
            vehicleOrderToggleUnbunching(regs, flags);
            return regs.ebx;
        }
    };
}

TEST(OrderStopAtTest, UnbunchingFlagPreservesPackedOrderData)
{
    OrderStopAt order{ StationId(0x2AB) };

    EXPECT_FALSE(order.isUnbunching());
    order.setUnbunching(true);
    EXPECT_TRUE(order.isUnbunching());
    EXPECT_EQ(order.getType(), OrderType::StopAt);
    EXPECT_EQ(order.getStation(), StationId(0x2AB));
    order.setUnbunching(false);
    EXPECT_FALSE(order.isUnbunching());
    EXPECT_EQ(order.getStation(), StationId(0x2AB));
}

TEST_F(VehicleUnbunchingTest, RouteIdentityIncludesOwnerTypeModeAndOrders)
{
    auto& reference = createVehicle();
    auto& identical = createVehicle();
    auto& otherOwner = createVehicle(CompanyId(1));
    auto& otherType = createVehicle(CompanyId(0), VehicleType::bus);
    auto& express = createVehicle(CompanyId(0), VehicleType::train, true);
    auto& otherOrders = createVehicle();
    addRoute(reference);
    addRoute(identical);
    addRoute(otherOwner);
    addRoute(otherType);
    addRoute(express);
    addRoute(otherOrders, 1, 3);

    EXPECT_TRUE(OrderManager::areVehiclesOnSameRoute(reference, identical));
    EXPECT_FALSE(OrderManager::areVehiclesOnSameRoute(reference, otherOwner));
    EXPECT_FALSE(OrderManager::areVehiclesOnSameRoute(reference, otherType));
    EXPECT_FALSE(OrderManager::areVehiclesOnSameRoute(reference, express));
    EXPECT_FALSE(OrderManager::areVehiclesOnSameRoute(reference, otherOrders));
}

TEST_F(VehicleUnbunchingTest, CommandTogglesEveryVehicleOnTheSameRoute)
{
    auto& first = createVehicle();
    auto& second = createVehicle();
    auto& different = createVehicle();
    const auto selectedOffset = appendStop(first, 1);
    appendStop(first, 2);
    addRoute(second);
    addRoute(different, 1, 3);
    first.unbunchingState = 123;
    second.unbunchingState = 456;

    EXPECT_EQ(toggleUnbunching(first, selectedOffset, 0), 0U);
    EXPECT_FALSE(first.hasUnbunchingOrder());
    EXPECT_FALSE(second.hasUnbunchingOrder());
    EXPECT_EQ(first.unbunchingState, 123);
    EXPECT_EQ(second.unbunchingState, 456);

    EXPECT_EQ(toggleUnbunching(first, selectedOffset, Flags::apply), 0U);
    EXPECT_TRUE(first.hasUnbunchingOrder());
    EXPECT_TRUE(second.hasUnbunchingOrder());
    EXPECT_FALSE(different.hasUnbunchingOrder());
    EXPECT_EQ(first.unbunchingState, 0);
    EXPECT_EQ(second.unbunchingState, 0);

    EXPECT_EQ(toggleUnbunching(first, selectedOffset, Flags::apply), 0U);
    EXPECT_FALSE(first.hasUnbunchingOrder());
    EXPECT_FALSE(second.hasUnbunchingOrder());
}

TEST_F(VehicleUnbunchingTest, CommandKeepsSharedLocalAndExpressRoutesSynchronized)
{
    auto& sharedLocal = createVehicle();
    auto& sharedExpress = createVehicle(CompanyId(0), VehicleType::train, true);
    auto& matchingLocal = createVehicle();
    auto& matchingExpress = createVehicle(CompanyId(0), VehicleType::train, true);
    const auto selectedOffset = appendStop(sharedLocal, 1);
    appendStop(sharedLocal, 2);
    addRoute(sharedExpress);
    addRoute(matchingLocal);
    addRoute(matchingExpress);
    ASSERT_TRUE(SharedOrderManager::join(sharedExpress.id, sharedLocal.id));
    for (auto* vehicle : { &sharedLocal, &sharedExpress, &matchingLocal, &matchingExpress })
    {
        vehicle->unbunchingState = 1;
    }

    EXPECT_EQ(toggleUnbunching(sharedLocal, selectedOffset, 0), 0U);
    EXPECT_FALSE(sharedLocal.hasUnbunchingOrder());
    EXPECT_EQ(toggleUnbunching(sharedLocal, selectedOffset, Flags::apply), 0U);
    for (const auto* vehicle : { &sharedLocal, &sharedExpress, &matchingLocal, &matchingExpress })
    {
        EXPECT_TRUE(vehicle->hasUnbunchingOrder());
        EXPECT_EQ(vehicle->unbunchingState, 0U);
    }
    EXPECT_TRUE(SharedOrderManager::areOrdersEqual(sharedLocal, sharedExpress));
}

TEST_F(VehicleUnbunchingTest, CommandRejectsUnrepresentableOrderOffset)
{
    auto& head = createVehicle();
    appendStop(head, 1);
    appendStop(head, 2);
    OrderManager::orders()[head.orderTableOffset + 0x10000].setType(OrderType::StopAt);

    EXPECT_EQ(toggleUnbunching(head, 0x10000, Flags::apply), kFailure);
    EXPECT_FALSE(head.hasUnbunchingOrder());
}

TEST_F(VehicleUnbunchingTest, CommandRejectsFullLoadAndMultipleUnbunchingStops)
{
    auto& fullLoadRoute = createVehicle();
    const auto fullLoadOffset = appendStop(fullLoadRoute, 1);
    const OrderWaitFor waitFor{ 0 };
    appendOrder(fullLoadRoute, waitFor);
    appendStop(fullLoadRoute, 2);

    EXPECT_EQ(toggleUnbunching(fullLoadRoute, fullLoadOffset, Flags::apply), kFailure);
    EXPECT_FALSE(fullLoadRoute.hasUnbunchingOrder());

    auto& multipleRoute = createVehicle();
    const auto firstOffset = appendStop(multipleRoute, 1);
    const auto secondOffset = appendStop(multipleRoute, 2);
    auto* secondStop = OrderRingView(multipleRoute.orderTableOffset, secondOffset).begin()->as<OrderStopAt>();
    ASSERT_NE(secondStop, nullptr);
    secondStop->setUnbunching(true);

    EXPECT_EQ(toggleUnbunching(multipleRoute, firstOffset, Flags::apply), kFailure);
    EXPECT_FALSE(OrderRingView(multipleRoute.orderTableOffset, firstOffset).begin()->as<OrderStopAt>()->isUnbunching());
}

TEST_F(VehicleUnbunchingTest, CommandRejectsAnyMatchingRouteWithATimetable)
{
    auto& selected = createVehicle();
    auto& timetabled = createVehicle();
    const auto selectedOffset = appendStop(selected, 1);
    appendStop(selected, 2);
    addRoute(timetabled);
    ASSERT_TRUE(TimetableManager::enableForVehicle(timetabled.id));

    EXPECT_EQ(toggleUnbunching(selected, selectedOffset, Flags::apply), kFailure);
    EXPECT_FALSE(selected.hasUnbunchingOrder());
    EXPECT_FALSE(timetabled.hasUnbunchingOrder());
}

TEST_F(VehicleUnbunchingTest, FullLoadOrderCannotBeInsertedOnUnbunchingRoute)
{
    auto& head = createVehicle();
    const auto selectedOffset = appendStop(head, 1);
    appendStop(head, 2);
    ASSERT_EQ(toggleUnbunching(head, selectedOffset, Flags::apply), 0U);

    const OrderWaitFor waitFor{ 0 };
    VehicleOrderInsertArgs args{};
    args.head = head.id;
    args.orderOffset = head.sizeOfOrderTable - sizeof(OrderEnd);
    args.rawOrder = waitFor.getRaw();
    auto regs = static_cast<registers>(args);
    vehicleOrderInsert(regs, Flags::apply);

    EXPECT_EQ(static_cast<uint32_t>(regs.ebx), kFailure);
    EXPECT_TRUE(head.hasUnbunchingOrder());
}

TEST_F(VehicleUnbunchingTest, SpacesDeparturesUsingMeasuredAverageRoundTrip)
{
    auto& first = createVehicle();
    auto& second = createVehicle();
    const auto selectedOffset = appendStop(first, 1);
    appendStop(first, 2);
    addRoute(second);
    ASSERT_EQ(toggleUnbunching(first, selectedOffset, Flags::apply), 0U);

    ScenarioManager::setScenarioTicks(100);
    first.arriveAtUnbunchingStop();
    EXPECT_FALSE(first.isWaitingForUnbunching());
    first.leaveUnbunchingStop();

    ScenarioManager::setScenarioTicks(110);
    second.arriveAtUnbunchingStop();
    EXPECT_FALSE(second.isWaitingForUnbunching());
    second.leaveUnbunchingStop();

    ScenarioManager::setScenarioTicks(1000);
    first.arriveAtUnbunchingStop();
    EXPECT_FALSE(first.isWaitingForUnbunching());
    first.leaveUnbunchingStop();

    ScenarioManager::setScenarioTicks(1010);
    second.arriveAtUnbunchingStop();
    EXPECT_TRUE(second.isWaitingForUnbunching());
    EXPECT_TRUE(second.isWaitingToUnbunch());

    ScenarioManager::setScenarioTicks(1463);
    EXPECT_TRUE(second.isWaitingForUnbunching());
    ScenarioManager::setScenarioTicks(1464);
    EXPECT_FALSE(second.isWaitingForUnbunching());
    EXPECT_FALSE(second.isWaitingToUnbunch());
    second.leaveUnbunchingStop();
    EXPECT_EQ(second.unbunchingLastDepartureTick, 1464U);
}

TEST_F(VehicleUnbunchingTest, TimingStateRoundTripsThroughS5Entity)
{
    auto& head = createVehicle();
    head.unbunchingLastDepartureTick = 123456;
    head.unbunchingState = 0xA123;

    const auto exported = S5::exportEntity(reinterpret_cast<const Entity&>(head));
    const auto& exportedHead = reinterpret_cast<const S5::VehicleHead&>(exported);
    EXPECT_EQ(exportedHead.unbunchingLastDepartureTick, head.unbunchingLastDepartureTick);
    EXPECT_EQ(exportedHead.unbunchingState, head.unbunchingState);

    const auto imported = S5::importEntity(exported);
    const auto& importedHead = reinterpret_cast<const VehicleHead&>(imported);
    EXPECT_EQ(importedHead.unbunchingLastDepartureTick, head.unbunchingLastDepartureTick);
    EXPECT_EQ(importedHead.unbunchingState, head.unbunchingState);
}
