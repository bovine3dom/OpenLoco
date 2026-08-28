#include "Economy/Expenditures.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Undo.h"
#include "GameCommands/Vehicles/VehicleOrderDelete.h"
#include "GameCommands/Vehicles/VehicleOrderDown.h"
#include "GameCommands/Vehicles/VehicleOrderInsert.h"
#include "GameCommands/Vehicles/VehicleOrderReverse.h"
#include "GameCommands/Vehicles/VehicleOrderShare.h"
#include "GameCommands/Vehicles/VehicleOrderSkip.h"
#include "GameCommands/Vehicles/VehicleOrderUp.h"
#include "GameCommands/Vehicles/VehicleTimetable.h"
#include "GameState.h"
#include "S5/Limits.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleHead.h"
#include "World/StationManager.h"
#include <array>
#include <cstring>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <vector>

using namespace OpenLoco;
using namespace OpenLoco::GameCommands;
using namespace OpenLoco::Vehicles;

namespace
{
    class SharedOrderTest : public ::testing::Test
    {
    protected:
        static constexpr CompanyId kOwner{ 0 };

        void SetUp() override
        {
            EntityManager::reset();
            OrderManager::reset();
            setUpdatingCompanyId(kOwner);
            getGameState().playerCompanies[0] = kOwner;
            Undo::clear();
            resetCommandNestLevel();
        }

        void TearDown() override
        {
            OrderManager::reset();
            EntityManager::reset();
            Undo::clear();
            setUpdatingCompanyId(CompanyId::neutral);
            resetCommandNestLevel();
        }

        static std::vector<uint8_t> makeOrderTable(const std::initializer_list<uint64_t> rawOrders)
        {
            std::vector<uint8_t> table;
            for (const auto rawOrder : rawOrders)
            {
                const auto type = static_cast<OrderType>(rawOrder & 0x7);
                const auto size = OrderManager::getOrderSize(type);
                const auto oldSize = table.size();
                table.resize(oldSize + size);
                std::memcpy(table.data() + oldSize, &rawOrder, size);
            }
            table.push_back(static_cast<uint8_t>(OrderEnd{}.getRaw()));
            return table;
        }

        static SharedOrderManager::State makeState(const std::initializer_list<std::initializer_list<EntityId>> groups)
        {
            SharedOrderManager::State state;
            for (const auto members : groups)
            {
                state.groups.push_back({ std::vector<EntityId>(members) });
            }
            return state;
        }

        static uint64_t unload(const uint8_t cargo)
        {
            return OrderUnloadAll(cargo).getRaw();
        }

        static uint64_t waitFor(const uint8_t cargo)
        {
            return OrderWaitFor(cargo).getRaw();
        }

        static uint64_t unbunch(const uint16_t station)
        {
            OrderStopAt order{ StationId(station) };
            order.setUnbunching(true);
            return order.getRaw();
        }

        static uint64_t waypoint()
        {
            return OrderRouteWaypoint({ 10, 20 }, 4, 1, 2).getRaw();
        }

        VehicleHead* createHead()
        {
            auto* base = EntityManager::createEntityVehicle();
            if (base == nullptr)
            {
                return nullptr;
            }

            base->baseType = EntityBaseType::vehicle;
            auto* vehicle = base->asBase<VehicleBase>();
            vehicle->setSubType(VehicleHead::kVehicleThingType);
            EntityManager::moveEntityToList(vehicle, EntityManager::EntityListType::vehicleHead);

            auto* head = static_cast<VehicleHead*>(vehicle);
            head->owner = kOwner;
            head->head = head->id;
            head->mode = TransportMode::rail;
            head->trackType = 0;
            head->vehicleType = VehicleType::train;
            head->trainAcceptedCargoTypes = std::numeric_limits<uint32_t>::max();
            OrderManager::allocateOrders(*head);
            return head;
        }

        static void setOrders(VehicleHead& head, const std::initializer_list<uint64_t> rawOrders)
        {
            const auto table = makeOrderTable(rawOrders);
            OrderManager::replaceOrderTable(head, table);
        }

        static bool shareWith(VehicleHead& source, const std::initializer_list<VehicleHead*> targets)
        {
            for (const auto* target : targets)
            {
                if (!SharedOrderManager::join(target->id, source.id))
                {
                    return false;
                }
            }
            return true;
        }

        std::array<VehicleHead*, 3> createSharedHeads(const std::initializer_list<uint64_t> rawOrders)
        {
            std::array heads{ createHead(), createHead(), createHead() };
            if (heads[0] == nullptr || heads[1] == nullptr || heads[2] == nullptr)
            {
                return heads;
            }
            for (auto* head : heads)
            {
                setOrders(*head, rawOrders);
            }
            if (!shareWith(*heads[0], { heads[1], heads[2] }))
            {
                return {};
            }
            return heads;
        }

        template<typename T>
        static uint32_t runCommand(const T& args, void (*handler)(registers&, uint8_t), const uint8_t flags = Flags::apply)
        {
            auto regs = static_cast<registers>(args);
            handler(regs, flags);
            return static_cast<uint32_t>(regs.ebx);
        }

        static void expectOrders(const VehicleHead& head, const std::vector<uint8_t>& expected)
        {
            EXPECT_EQ(OrderManager::copyOrderTable(head), expected);
        }

        static void expectPacked(const std::initializer_list<const VehicleHead*> heads)
        {
            uint32_t offset = 0;
            for (const auto* head : heads)
            {
                EXPECT_EQ(head->orderTableOffset, offset);
                offset += head->sizeOfOrderTable;
            }
            EXPECT_EQ(OrderManager::orderTableLength(), offset);
        }
    };
}

TEST_F(SharedOrderTest, ManagerJoinsExtendsLeavesAndRecanonicalisesGroups)
{
    auto* first = createHead();
    auto* second = createHead();
    auto* third = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);

    EXPECT_FALSE(SharedOrderManager::join(first->id, first->id));
    ASSERT_TRUE(SharedOrderManager::join(second->id, first->id));
    EXPECT_EQ(SharedOrderManager::getGroupId(first->id), first->id);
    EXPECT_EQ(SharedOrderManager::getMembers(second->id), (std::vector{ first->id, second->id }));

    ASSERT_TRUE(SharedOrderManager::join(third->id, second->id));
    EXPECT_TRUE(SharedOrderManager::join(third->id, first->id));
    EXPECT_EQ(SharedOrderManager::getMembers(first->id), (std::vector{ first->id, second->id, third->id }));

    ASSERT_TRUE(SharedOrderManager::leave(first->id));
    EXPECT_FALSE(SharedOrderManager::isShared(first->id));
    EXPECT_EQ(SharedOrderManager::getGroupId(second->id), second->id);
    EXPECT_EQ(SharedOrderManager::getMembers(third->id), (std::vector{ second->id, third->id }));

    ASSERT_TRUE(SharedOrderManager::leave(second->id));
    EXPECT_FALSE(SharedOrderManager::isShared(second->id));
    EXPECT_FALSE(SharedOrderManager::isShared(third->id));
    EXPECT_EQ(SharedOrderManager::getMembers(third->id), (std::vector{ third->id }));
    EXPECT_FALSE(SharedOrderManager::leave(first->id));
}

TEST_F(SharedOrderTest, SharedOrderLifecycleMergesAndSplitsTimetableServices)
{
    auto* source = createHead();
    auto* target = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(target, nullptr);
    setOrders(*source, { OrderStopAt(StationId(1)).getRaw() });
    setOrders(*target, { OrderStopAt(StationId(1)).getRaw() });

    ASSERT_TRUE(TimetableManager::enableForVehicle(source->id));
    ASSERT_TRUE(TimetableManager::setTravelMinutes(source->id, 0, 12));
    ASSERT_TRUE(TimetableManager::setDwellMinutes(source->id, 0, 3));
    ASSERT_TRUE(TimetableManager::enableForVehicle(target->id));
    const auto sourceService = TimetableManager::getServiceId(source->id);
    const auto discardedService = TimetableManager::getServiceId(target->id);
    ASSERT_NE(sourceService, discardedService);

    ASSERT_TRUE(SharedOrderManager::join(target->id, source->id));
    EXPECT_EQ(TimetableManager::getServiceId(target->id), sourceService);
    EXPECT_EQ(TimetableManager::getService(discardedService), nullptr);

    ASSERT_TRUE(SharedOrderManager::leave(target->id));
    const auto splitService = TimetableManager::getServiceId(target->id);
    EXPECT_NE(splitService, sourceService);
    ASSERT_NE(TimetableManager::getEntry(target->id, 0), nullptr);
    EXPECT_EQ(TimetableManager::getEntry(target->id, 0)->travelMinutes, 12);
    EXPECT_EQ(TimetableManager::getEntry(target->id, 0)->dwellMinutes, 3);
}

TEST_F(SharedOrderTest, TimetableStateValidationMatchesRoutesAndSharedGroups)
{
    auto* source = createHead();
    auto* target = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(target, nullptr);
    setOrders(*source, { OrderStopAt(StationId(1)).getRaw(), unload(2) });
    setOrders(*target, { OrderStopAt(StationId(1)).getRaw(), unload(2) });
    ASSERT_TRUE(TimetableManager::enableForVehicle(source->id));
    ASSERT_TRUE(SharedOrderManager::join(target->id, source->id));

    const auto shared = SharedOrderManager::captureState();
    const auto canonical = TimetableManager::captureState();
    EXPECT_TRUE(TimetableManager::validateState(canonical, getGameState(), shared));

    auto wrongStation = canonical;
    wrongStation.services.front().entries.front().station = StationId(2);
    EXPECT_FALSE(TimetableManager::validateState(wrongStation, getGameState(), shared));

    auto missingMember = canonical;
    std::erase_if(missingMember.assignments, [target](const auto& assignment) { return assignment.vehicle == target->id; });
    std::erase_if(missingMember.vehicles, [target](const auto& runtime) { return runtime.vehicle == target->id; });
    EXPECT_TRUE(TimetableManager::validateState(missingMember));
    EXPECT_FALSE(TimetableManager::validateState(missingMember, getGameState(), shared));

    auto* independent = createHead();
    ASSERT_NE(independent, nullptr);
    setOrders(*independent, { OrderStopAt(StationId(3)).getRaw() });
    ASSERT_TRUE(TimetableManager::enableForVehicle(independent->id));
    const auto withIndependent = TimetableManager::captureState();
    independent->currentOrder = 1;
    EXPECT_FALSE(TimetableManager::validateState(withIndependent, getGameState(), shared));
}

TEST_F(SharedOrderTest, TimetableCommandQueriesAreAtomic)
{
    EXPECT_EQ(enumValue(VehicleTimetableArgs::Action::removeDispatchSlot), 7);
    EXPECT_EQ(enumValue(VehicleTimetableArgs::Action::clearDispatch), 8);
    EXPECT_EQ(enumValue(VehicleTimetableArgs::Action::setClockRate), 9);
    EXPECT_EQ(enumValue(VehicleTimetableArgs::Action::resetDispatch), 10);
    EXPECT_EQ(enumValue(VehicleTimetableArgs::Action::setEvenlySpacedSlots), 11);

    auto* head = createHead();
    ASSERT_NE(head, nullptr);
    setOrders(*head, { OrderStopAt(StationId(1)).getRaw() });

    VehicleTimetableArgs args{};
    args.head = head->id;
    args.action = VehicleTimetableArgs::Action::setEnabled;
    args.value = 1;
    const auto initial = TimetableManager::captureState();
    EXPECT_EQ(runCommand(args, vehicleTimetable, 0), 0U);
    EXPECT_EQ(TimetableManager::captureState(), initial);

    ASSERT_EQ(runCommand(args, vehicleTimetable), 0U);
    const auto enabled = TimetableManager::captureState();
    args.action = VehicleTimetableArgs::Action::setTravelMinutes;
    args.orderIndex = 0;
    args.value = 18;
    EXPECT_EQ(runCommand(args, vehicleTimetable, 0), 0U);
    EXPECT_EQ(TimetableManager::captureState(), enabled);
    ASSERT_EQ(runCommand(args, vehicleTimetable), 0U);
    ASSERT_NE(TimetableManager::getEntry(head->id, 0), nullptr);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->travelMinutes, 18);

    args.action = VehicleTimetableArgs::Action::setEvenlySpacedSlots;
    args.value = 4;
    const auto beforeSlots = TimetableManager::captureState();
    EXPECT_EQ(runCommand(args, vehicleTimetable, 0), 0U);
    EXPECT_EQ(TimetableManager::captureState(), beforeSlots);
    ASSERT_EQ(runCommand(args, vehicleTimetable), 0U);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->dispatch->slots, (std::vector<uint32_t>{ 0, 15, 30, 45 }));
}

TEST_F(SharedOrderTest, TimetableCommandQueryPreservesFleetMeasurement)
{
    TimetableManager::reset();
    auto* head = createHead();
    ASSERT_NE(head, nullptr);
    setOrders(*head, { OrderStopAt(StationId(1)).getRaw() });
    ASSERT_TRUE(TimetableManager::enableForVehicle(head->id));
    ASSERT_TRUE(TimetableManager::setTicksPerMinute(1));
    ASSERT_TRUE(TimetableManager::setEvenlySpacedSlots(head->id, 0, 4));
    ASSERT_TRUE(TimetableManager::arriveAtOrder(head->id, 0));
    EXPECT_FALSE(TimetableManager::isWaitingForDeparture(head->id));
    TimetableManager::departFromOrder(head->id);
    for (uint32_t i = 0; i < 20; ++i)
    {
        TimetableManager::tick();
    }
    ASSERT_TRUE(TimetableManager::arriveAtOrder(head->id, 0));
    TimetableManager::isWaitingForDeparture(head->id);
    ASSERT_EQ(TimetableManager::getFleetEstimate(head->id, 0)->sampleCount, 1U);

    VehicleTimetableArgs args{};
    args.head = head->id;
    args.action = VehicleTimetableArgs::Action::setDispatchPhase;
    args.orderIndex = 0;
    args.value = 5;
    EXPECT_EQ(runCommand(args, vehicleTimetable, 0), 0U);
    EXPECT_EQ(TimetableManager::getFleetEstimate(head->id, 0)->sampleCount, 1U);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->dispatch->phaseMinutes, 0U);
}

TEST_F(SharedOrderTest, UndoRestoresTimetableWithoutRewindingItsClock)
{
    auto* head = createHead();
    ASSERT_NE(head, nullptr);
    setOrders(*head, { OrderStopAt(StationId(1)).getRaw() });

    VehicleTimetableArgs args{};
    args.head = head->id;
    args.action = VehicleTimetableArgs::Action::setEnabled;
    args.value = 1;
    auto regs = static_cast<registers>(args);
    Undo::prepare(GameCommand::vehicleTimetable, kOwner, regs, Flags::apply);
    vehicleTimetable(regs, Flags::apply);
    ASSERT_EQ(static_cast<uint32_t>(regs.ebx), 0U);
    Undo::commit(0, ExpenditureType::VehiclePurchases, {});
    ASSERT_NE(TimetableManager::getServiceId(head->id), TimetableManager::kInvalidServiceId);

    TimetableManager::tick();
    ASSERT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_EQ(TimetableManager::getServiceId(head->id), TimetableManager::kInvalidServiceId);
    EXPECT_EQ(TimetableManager::getClockTicks(), 1U);
}

TEST_F(SharedOrderTest, UndoPreservesUnrelatedDispatchProgress)
{
    auto* edited = createHead();
    auto* waiting = createHead();
    ASSERT_NE(edited, nullptr);
    ASSERT_NE(waiting, nullptr);
    setOrders(*edited, { OrderStopAt(StationId(1)).getRaw() });
    setOrders(*waiting, { OrderStopAt(StationId(2)).getRaw() });
    ASSERT_TRUE(TimetableManager::enableForVehicle(edited->id));
    ASSERT_TRUE(TimetableManager::enableForVehicle(waiting->id));
    ASSERT_TRUE(TimetableManager::addDispatchSlot(waiting->id, 0, 15));
    ASSERT_TRUE(TimetableManager::arriveAtOrder(waiting->id, 0));
    ASSERT_TRUE(TimetableManager::isWaitingForDeparture(waiting->id));

    VehicleTimetableArgs args{};
    args.head = edited->id;
    args.action = VehicleTimetableArgs::Action::setTravelMinutes;
    args.orderIndex = 0;
    args.value = 12;
    auto regs = static_cast<registers>(args);
    Undo::prepare(GameCommand::vehicleTimetable, kOwner, regs, Flags::apply);
    vehicleTimetable(regs, Flags::apply);
    ASSERT_EQ(static_cast<uint32_t>(regs.ebx), 0U);
    Undo::commit(0, ExpenditureType::VehiclePurchases, {});

    TimetableManager::tick();
    const auto runtime = *TimetableManager::getVehicleRuntime(waiting->id);
    const auto claimedMinute = TimetableManager::getEntry(waiting->id, 0)->dispatch->lastClaimedMinute;
    ASSERT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_FALSE(TimetableManager::getEntry(edited->id, 0)->travelMinutes.has_value());
    EXPECT_EQ(*TimetableManager::getVehicleRuntime(waiting->id), runtime);
    EXPECT_EQ(TimetableManager::getEntry(waiting->id, 0)->dispatch->lastClaimedMinute, claimedMinute);
}

TEST_F(SharedOrderTest, FleetEstimateCountsActiveSharedVehicles)
{
    TimetableManager::reset();
    auto heads = createSharedHeads({ OrderStopAt(StationId(1)).getRaw() });
    ASSERT_NE(heads[0], nullptr);
    for (auto* head : heads)
    {
        head->tileX = 0;
        head->status = Status::travelling;
    }
    ASSERT_TRUE(TimetableManager::enableForVehicle(heads[0]->id));
    ASSERT_TRUE(TimetableManager::setTicksPerMinute(1));
    ASSERT_TRUE(TimetableManager::setEvenlySpacedSlots(heads[0]->id, 0, 6));

    ASSERT_TRUE(TimetableManager::arriveAtOrder(heads[0]->id, 0));
    EXPECT_FALSE(TimetableManager::isWaitingForDeparture(heads[0]->id));
    TimetableManager::departFromOrder(heads[0]->id);
    for (uint32_t i = 0; i < 20; ++i)
    {
        TimetableManager::tick();
    }
    ASSERT_TRUE(TimetableManager::arriveAtOrder(heads[0]->id, 0));
    TimetableManager::isWaitingForDeparture(heads[0]->id);

    auto estimate = TimetableManager::getFleetEstimate(heads[0]->id, 0);
    ASSERT_TRUE(estimate.has_value());
    EXPECT_EQ(estimate->activeVehicles, 3U);
    EXPECT_EQ(estimate->requiredVehicles, 2U);

    heads[1]->vehicleFlags |= VehicleFlags::commandStop;
    heads[2]->status = Status::crashed;
    estimate = TimetableManager::getFleetEstimate(heads[0]->id, 0);
    ASSERT_TRUE(estimate.has_value());
    EXPECT_EQ(estimate->activeVehicles, 1U);
}

TEST_F(SharedOrderTest, TimetableEntriesFollowOrderEdits)
{
    auto* head = createHead();
    ASSERT_NE(head, nullptr);
    setOrders(*head, { OrderStopAt(StationId(1)).getRaw(), unload(2), OrderStopAt(StationId(2)).getRaw() });
    ASSERT_TRUE(TimetableManager::enableForVehicle(head->id));
    ASSERT_TRUE(TimetableManager::setTravelMinutes(head->id, 0, 10));
    ASSERT_TRUE(TimetableManager::setTravelMinutes(head->id, 2, 20));
    const auto firstEntry = TimetableManager::getEntry(head->id, 0)->id;
    const auto secondEntry = TimetableManager::getEntry(head->id, 2)->id;

    VehicleOrderInsertArgs insertArgs{};
    insertArgs.head = head->id;
    insertArgs.orderOffset = sizeof(OrderStopAt) + sizeof(OrderUnloadAll);
    insertArgs.rawOrder = waypoint();
    ASSERT_EQ(runCommand(insertArgs, vehicleOrderInsert), 0U);
    ASSERT_NE(TimetableManager::getEntry(head->id, 2), nullptr);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->id, firstEntry);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 3)->id, secondEntry);

    VehicleOrderUpArgs upArgs{};
    upArgs.head = head->id;
    upArgs.orderOffset = insertArgs.orderOffset;
    ASSERT_EQ(runCommand(upArgs, vehicleOrderUp), 0U);
    const auto waypointEntry = TimetableManager::getEntry(head->id, 1)->id;

    VehicleOrderReverseArgs reverseArgs{};
    reverseArgs.head = head->id;
    ASSERT_EQ(runCommand(reverseArgs, vehicleOrderReverse), 0U);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->id, secondEntry);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 2)->id, waypointEntry);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 3)->id, firstEntry);

    VehicleOrderDeleteArgs deleteArgs{};
    deleteArgs.head = head->id;
    deleteArgs.orderOffset = sizeof(OrderStopAt) + sizeof(OrderUnloadAll);
    ASSERT_EQ(runCommand(deleteArgs, vehicleOrderDelete), 0U);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 0)->id, secondEntry);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 2)->id, firstEntry);
    EXPECT_EQ(TimetableManager::getEntry(head->id, 2)->travelMinutes, 10);
}

TEST_F(SharedOrderTest, RemovingStationDeletesEveryAdjacentTimetableEntry)
{
    auto heads = createSharedHeads({
        OrderStopAt(StationId(1)).getRaw(),
        OrderRouteThrough(StationId(1)).getRaw(),
        OrderStopAt(StationId(1)).getRaw(),
        OrderStopAt(StationId(2)).getRaw(),
    });
    ASSERT_NE(heads[0], nullptr);
    ASSERT_TRUE(TimetableManager::enableForVehicle(heads[0]->id));
    const auto remainingEntry = TimetableManager::getEntry(heads[0]->id, 3)->id;

    OrderManager::removeOrdersForStation(StationId(1));

    const auto expected = makeOrderTable({ OrderStopAt(StationId(2)).getRaw() });
    for (const auto* head : heads)
    {
        expectOrders(*head, expected);
    }
    const auto* service = TimetableManager::getServiceForVehicle(heads[0]->id);
    ASSERT_NE(service, nullptr);
    ASSERT_EQ(service->entries.size(), 1U);
    EXPECT_EQ(service->entries.front().id, remainingEntry);
    EXPECT_EQ(service->entries.front().orderIndex, 0);
    EXPECT_TRUE(TimetableManager::validateState(TimetableManager::captureState(), getGameState(), SharedOrderManager::captureState()));
}

TEST_F(SharedOrderTest, FreeingOrdersAndEntitiesCleansUpGroupsAndPackedTables)
{
    auto* first = createHead();
    auto* second = createHead();
    auto* third = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    ASSERT_TRUE(shareWith(*first, { second, third }));

    const auto secondId = second->id;
    OrderManager::freeOrders(second);
    EntityManager::freeEntity(second);

    EXPECT_EQ(EntityManager::get<VehicleHead>(secondId), nullptr);
    EXPECT_EQ(SharedOrderManager::getMembers(first->id), (std::vector{ first->id, third->id }));
    EXPECT_EQ(SharedOrderManager::getGroupId(third->id), first->id);
    expectPacked({ first, third });

    OrderManager::freeOrders(first);
    EntityManager::freeEntity(first);

    EXPECT_FALSE(SharedOrderManager::isShared(third->id));
    EXPECT_EQ(SharedOrderManager::getMembers(third->id), (std::vector{ third->id }));
    expectPacked({ third });
}

TEST_F(SharedOrderTest, CaptureRestoreValidatesEntitiesGroupsCompatibilityAndOrders)
{
    auto* first = createHead();
    auto* second = createHead();
    auto* third = createHead();
    auto* fourth = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    ASSERT_NE(fourth, nullptr);
    for (auto* head : { first, second, third, fourth })
    {
        setOrders(*head, { unload(2), waitFor(3) });
    }
    ASSERT_TRUE(SharedOrderManager::join(second->id, first->id));
    ASSERT_TRUE(SharedOrderManager::join(fourth->id, third->id));

    const auto expected = makeState({ { first->id, second->id }, { third->id, fourth->id } });
    EXPECT_EQ(SharedOrderManager::captureState(), expected);
    EXPECT_TRUE(SharedOrderManager::validateState(expected));

    SharedOrderManager::reset();
    ASSERT_TRUE(SharedOrderManager::restoreState(expected));
    EXPECT_EQ(SharedOrderManager::captureState(), expected);

    const auto singleton = makeState({ { first->id } });
    const auto unsorted = makeState({ { second->id, first->id } });
    const auto duplicate = makeState({ { first->id, second->id }, { second->id, third->id } });
    const auto missing = makeState({ { first->id, EntityId(9999) } });
    EXPECT_FALSE(SharedOrderManager::validateState(singleton));
    EXPECT_FALSE(SharedOrderManager::validateState(unsorted));
    EXPECT_FALSE(SharedOrderManager::validateState(duplicate));
    EXPECT_FALSE(SharedOrderManager::validateState(missing));
    EXPECT_FALSE(SharedOrderManager::restoreState(singleton));
    EXPECT_EQ(SharedOrderManager::captureState(), expected);

    setOrders(*fourth, { unload(9), waitFor(3) });
    EXPECT_FALSE(SharedOrderManager::validateState(expected));
    setOrders(*fourth, { unload(2), waitFor(3) });
    fourth->owner = CompanyId(1);
    EXPECT_FALSE(SharedOrderManager::validateState(expected));
    fourth->owner = kOwner;

    const auto orderTableOffset = fourth->orderTableOffset;
    fourth->orderTableOffset = S5::Limits::kMaxOrders;
    EXPECT_FALSE(SharedOrderManager::validateState(expected));
    fourth->orderTableOffset = orderTableOffset;

    fourth->currentOrder = fourth->sizeOfOrderTable;
    EXPECT_FALSE(SharedOrderManager::validateState(expected));
    fourth->currentOrder = 0;

    fourth->setSubType(VehicleEntityType::body_start);
    EXPECT_FALSE(SharedOrderManager::validateState(expected));
    fourth->setSubType(VehicleEntityType::head);
}

TEST_F(SharedOrderTest, OrderEqualityIsByteExactAndIndependentOfCursorAndLocation)
{
    auto* first = createHead();
    auto* second = createHead();
    auto* third = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    setOrders(*first, { unload(1), waitFor(2) });
    setOrders(*second, { unload(1), waitFor(2) });
    setOrders(*third, { unload(1), unload(2) });
    second->currentOrder = 1;

    EXPECT_NE(first->orderTableOffset, second->orderTableOffset);
    EXPECT_TRUE(SharedOrderManager::areOrdersEqual(*first, *second));
    EXPECT_FALSE(SharedOrderManager::areOrdersEqual(*first, *third));

    setOrders(*third, { unload(1) });
    EXPECT_FALSE(SharedOrderManager::areOrdersEqual(*first, *third));
}

TEST_F(SharedOrderTest, VehicleCompatibilityChecksIdentityTrackAndRequiredCargo)
{
    auto* target = createHead();
    auto* source = createHead();
    ASSERT_NE(target, nullptr);
    ASSERT_NE(source, nullptr);
    setOrders(*source, { unload(5), waitFor(7) });

    EXPECT_TRUE(SharedOrderManager::areVehiclesCompatible(*target, *source));
    target->trainAcceptedCargoTypes = 1U << 5;
    EXPECT_FALSE(SharedOrderManager::areVehiclesCompatible(*target, *source));
    target->trainAcceptedCargoTypes = std::numeric_limits<uint32_t>::max();

    target->owner = CompanyId(1);
    EXPECT_FALSE(SharedOrderManager::areVehiclesCompatible(*target, *source));
    target->owner = source->owner;
    target->vehicleType = VehicleType::bus;
    EXPECT_FALSE(SharedOrderManager::areVehiclesCompatible(*target, *source));
    target->vehicleType = source->vehicleType;
    target->mode = TransportMode::road;
    EXPECT_FALSE(SharedOrderManager::areVehiclesCompatible(*target, *source));
    target->mode = source->mode;
    target->trackType = 1;
    EXPECT_FALSE(SharedOrderManager::areVehiclesCompatible(*target, *source));

    target->mode = TransportMode::water;
    source->mode = TransportMode::water;
    target->vehicleType = VehicleType::ship;
    source->vehicleType = VehicleType::ship;
    EXPECT_TRUE(SharedOrderManager::areVehiclesCompatible(*target, *source));
}

TEST_F(SharedOrderTest, IncompatibleCargoChangeDetachesVehicleFromGroup)
{
    auto* source = createHead();
    auto* target = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(target, nullptr);
    setOrders(*source, { unload(5), waitFor(7) });
    setOrders(*target, { unload(5), waitFor(7) });
    ASSERT_TRUE(SharedOrderManager::join(target->id, source->id));

    EXPECT_FALSE(SharedOrderManager::detachIfIncompatible(target->id));
    target->trainAcceptedCargoTypes = (1U << 5) | (1U << 7);
    EXPECT_FALSE(SharedOrderManager::detachIfIncompatible(target->id));
    target->trainAcceptedCargoTypes = 1U << 4;
    EXPECT_TRUE(SharedOrderManager::detachIfIncompatible(target->id));
    EXPECT_FALSE(SharedOrderManager::isShared(source->id));
    EXPECT_FALSE(SharedOrderManager::isShared(target->id));
}

TEST_F(SharedOrderTest, ReplaceOrderTableGrowsAndShrinksPackedStorage)
{
    auto* first = createHead();
    auto* middle = createHead();
    auto* last = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(middle, nullptr);
    ASSERT_NE(last, nullptr);
    expectPacked({ first, middle, last });

    const auto large = makeOrderTable({ unload(1), waypoint(), waitFor(3) });
    OrderManager::replaceOrderTable(*middle, large);
    expectOrders(*middle, large);
    expectOrders(*first, makeOrderTable({}));
    expectOrders(*last, makeOrderTable({}));
    expectPacked({ first, middle, last });
    EXPECT_EQ(last->orderTableOffset, 10U);

    middle->currentOrder = 7;
    const auto small = makeOrderTable({ waitFor(4) });
    OrderManager::replaceOrderTable(*middle, small);
    expectOrders(*middle, small);
    expectOrders(*last, makeOrderTable({}));
    expectPacked({ first, middle, last });
    EXPECT_EQ(last->orderTableOffset, 3U);
    EXPECT_EQ(middle->currentOrder, 0);
}

TEST_F(SharedOrderTest, ShareCommandAddsToGroupPreservesEqualCursorAndLeavesCleanly)
{
    auto* source = createHead();
    auto* peer = createHead();
    auto* target = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(peer, nullptr);
    ASSERT_NE(target, nullptr);
    for (auto* head : { source, peer, target })
    {
        setOrders(*head, { unload(1), waitFor(2), unload(3) });
    }
    source->currentOrder = 0;
    peer->currentOrder = 1;
    target->currentOrder = 2;
    ASSERT_TRUE(SharedOrderManager::join(peer->id, source->id));

    VehicleOrderShareArgs joinArgs{};
    joinArgs.target = target->id;
    joinArgs.source = source->id;
    joinArgs.mode = VehicleOrderShareArgs::Mode::joinSource;
    const auto beforeQuery = SharedOrderManager::captureState();
    EXPECT_EQ(runCommand(joinArgs, vehicleOrderShare, 0), 0U);
    EXPECT_EQ(SharedOrderManager::captureState(), beforeQuery);
    EXPECT_EQ(target->currentOrder, 2);

    ASSERT_EQ(runCommand(joinArgs, vehicleOrderShare), 0U);
    EXPECT_EQ(SharedOrderManager::getMembers(source->id), (std::vector{ source->id, peer->id, target->id }));
    EXPECT_EQ(source->currentOrder, 0);
    EXPECT_EQ(peer->currentOrder, 1);
    EXPECT_EQ(target->currentOrder, 2);

    VehicleOrderShareArgs leaveArgs{};
    leaveArgs.target = target->id;
    leaveArgs.mode = VehicleOrderShareArgs::Mode::leave;
    EXPECT_EQ(runCommand(leaveArgs, vehicleOrderShare, 0), 0U);
    EXPECT_TRUE(SharedOrderManager::isShared(target->id));
    ASSERT_EQ(runCommand(leaveArgs, vehicleOrderShare), 0U);
    EXPECT_FALSE(SharedOrderManager::isShared(target->id));
    EXPECT_EQ(SharedOrderManager::getMembers(source->id), (std::vector{ source->id, peer->id }));
    EXPECT_EQ(target->currentOrder, 2);
}

TEST_F(SharedOrderTest, ShareCommandReplacesDifferentRouteAndResetsOnlyTargetCursor)
{
    auto* target = createHead();
    auto* source = createHead();
    ASSERT_NE(target, nullptr);
    ASSERT_NE(source, nullptr);
    setOrders(*target, { unload(1), waitFor(2), unload(3) });
    setOrders(*source, { waitFor(4), unload(5) });
    target->currentOrder = 2;
    source->currentOrder = 1;

    VehicleOrderShareArgs args{};
    args.target = target->id;
    args.source = source->id;
    const auto targetBefore = OrderManager::copyOrderTable(*target);
    const auto lengthBefore = OrderManager::orderTableLength();
    const auto sourceOffsetBefore = source->orderTableOffset;

    ASSERT_EQ(runCommand(args, vehicleOrderShare, 0), 0U);
    expectOrders(*target, targetBefore);
    EXPECT_EQ(target->currentOrder, 2);
    EXPECT_EQ(source->orderTableOffset, sourceOffsetBefore);
    EXPECT_EQ(OrderManager::orderTableLength(), lengthBefore);
    EXPECT_FALSE(SharedOrderManager::isShared(target->id));

    ASSERT_EQ(runCommand(args, vehicleOrderShare), 0U);
    EXPECT_TRUE(SharedOrderManager::areOrdersEqual(*target, *source));
    EXPECT_EQ(target->currentOrder, 0);
    EXPECT_EQ(source->currentOrder, 1);
    EXPECT_EQ(SharedOrderManager::getMembers(target->id), (std::vector{ target->id, source->id }));
    expectPacked({ target, source });
}

TEST_F(SharedOrderTest, UndoRestoresSharedOrderMembership)
{
    auto* source = createHead();
    auto* target = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(target, nullptr);
    setOrders(*source, { unload(1), waitFor(2) });
    setOrders(*target, { unload(1), waitFor(2) });

    VehicleOrderShareArgs args{};
    args.target = target->id;
    args.source = source->id;
    auto regs = static_cast<registers>(args);
    Undo::prepare(GameCommand::vehicleOrderShare, kOwner, regs, Flags::apply);
    vehicleOrderShare(regs, Flags::apply);
    ASSERT_EQ(static_cast<uint32_t>(regs.ebx), 0U);
    Undo::commit(0, ExpenditureType::VehiclePurchases, {});
    ASSERT_TRUE(SharedOrderManager::isShared(source->id));

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_FALSE(SharedOrderManager::isShared(source->id));
    EXPECT_FALSE(SharedOrderManager::isShared(target->id));
}

TEST_F(SharedOrderTest, ShareQueryCapacityFailureDoesNotMutateRuntimeState)
{
    auto* target = createHead();
    auto* source = createHead();
    ASSERT_NE(target, nullptr);
    ASSERT_NE(source, nullptr);
    setOrders(*source, { unload(1), waitFor(2), unload(3) });
    const auto targetOrders = OrderManager::copyOrderTable(*target);
    const auto sourceOrders = OrderManager::copyOrderTable(*source);
    const auto targetOffset = target->orderTableOffset;
    const auto sourceOffset = source->orderTableOffset;
    OrderManager::orderTableLength() = S5::Limits::kMaxOrders - 2;

    VehicleOrderShareArgs args{};
    args.target = target->id;
    args.source = source->id;
    EXPECT_EQ(runCommand(args, vehicleOrderShare, 0), kFailure);
    EXPECT_EQ(runCommand(args, vehicleOrderShare), kFailure);

    expectOrders(*target, targetOrders);
    expectOrders(*source, sourceOrders);
    EXPECT_EQ(target->orderTableOffset, targetOffset);
    EXPECT_EQ(source->orderTableOffset, sourceOffset);
    EXPECT_EQ(OrderManager::orderTableLength(), S5::Limits::kMaxOrders - 2);
    EXPECT_EQ(SharedOrderManager::captureState(), SharedOrderManager::State{});
}

TEST_F(SharedOrderTest, JoinAllMatchingSharesOnlyNonEmptyCompatibleExactRoutes)
{
    auto* source = createHead();
    auto* firstMatch = createHead();
    auto* secondMatch = createHead();
    auto* empty = createHead();
    auto* different = createHead();
    auto* incompatible = createHead();
    ASSERT_NE(source, nullptr);
    ASSERT_NE(firstMatch, nullptr);
    ASSERT_NE(secondMatch, nullptr);
    ASSERT_NE(empty, nullptr);
    ASSERT_NE(different, nullptr);
    ASSERT_NE(incompatible, nullptr);
    for (auto* head : { source, firstMatch, secondMatch, incompatible })
    {
        setOrders(*head, { unload(1), waitFor(2) });
    }
    setOrders(*different, { unload(1), waitFor(3) });
    incompatible->owner = CompanyId(1);
    firstMatch->currentOrder = 1;

    VehicleOrderShareArgs args{};
    args.target = source->id;
    args.mode = VehicleOrderShareArgs::Mode::joinAllMatching;
    EXPECT_EQ(runCommand(args, vehicleOrderShare, 0), 0U);
    EXPECT_EQ(SharedOrderManager::captureState(), SharedOrderManager::State{});
    ASSERT_EQ(runCommand(args, vehicleOrderShare), 0U);

    EXPECT_EQ(SharedOrderManager::getMembers(source->id), (std::vector{ source->id, firstMatch->id, secondMatch->id }));
    EXPECT_FALSE(SharedOrderManager::isShared(empty->id));
    EXPECT_FALSE(SharedOrderManager::isShared(different->id));
    EXPECT_FALSE(SharedOrderManager::isShared(incompatible->id));
    EXPECT_EQ(firstMatch->currentOrder, 1);

    const auto shared = SharedOrderManager::captureState();
    EXPECT_EQ(runCommand(args, vehicleOrderShare), 0U);
    EXPECT_EQ(SharedOrderManager::captureState(), shared);
    args.target = empty->id;
    EXPECT_EQ(runCommand(args, vehicleOrderShare), kFailure);
    EXPECT_EQ(SharedOrderManager::captureState(), shared);
}

TEST_F(SharedOrderTest, InsertCommandPropagatesToThreeTablesAndAdjustsEachCursor)
{
    auto [first, second, third] = createSharedHeads({ unload(0), waypoint(), waitFor(2) });
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    first->currentOrder = 0;
    second->currentOrder = 1;
    third->currentOrder = 7;

    VehicleOrderInsertArgs args{};
    args.head = second->id;
    args.orderOffset = 1;
    args.rawOrder = waitFor(3);
    const std::array before{
        OrderManager::copyOrderTable(*first),
        OrderManager::copyOrderTable(*second),
        OrderManager::copyOrderTable(*third),
    };
    const auto lengthBefore = OrderManager::orderTableLength();
    ASSERT_EQ(runCommand(args, vehicleOrderInsert, 0), 0U);
    EXPECT_EQ(OrderManager::copyOrderTable(*first), before[0]);
    EXPECT_EQ(OrderManager::copyOrderTable(*second), before[1]);
    EXPECT_EQ(OrderManager::copyOrderTable(*third), before[2]);
    EXPECT_EQ(OrderManager::orderTableLength(), lengthBefore);

    ASSERT_EQ(runCommand(args, vehicleOrderInsert), 0U);
    const auto expected = makeOrderTable({ unload(0), waitFor(3), waypoint(), waitFor(2) });
    expectOrders(*first, expected);
    expectOrders(*second, expected);
    expectOrders(*third, expected);
    EXPECT_EQ(first->currentOrder, 0);
    EXPECT_EQ(second->currentOrder, 2);
    EXPECT_EQ(third->currentOrder, 8);
    expectPacked({ first, second, third });
}

TEST_F(SharedOrderTest, DeleteCommandPropagatesToThreeTablesAndAdjustsEachCursor)
{
    auto [first, second, third] = createSharedHeads({ unload(0), waypoint(), waitFor(2) });
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    first->currentOrder = 0;
    second->currentOrder = 1;
    third->currentOrder = 7;

    VehicleOrderDeleteArgs args{};
    args.head = third->id;
    args.orderOffset = 1;
    ASSERT_EQ(runCommand(args, vehicleOrderDelete), 0U);

    const auto expected = makeOrderTable({ unload(0), waitFor(2) });
    expectOrders(*first, expected);
    expectOrders(*second, expected);
    expectOrders(*third, expected);
    EXPECT_EQ(first->currentOrder, 0);
    EXPECT_EQ(second->currentOrder, 1);
    EXPECT_EQ(third->currentOrder, 1);
    expectPacked({ first, second, third });
}

TEST_F(SharedOrderTest, MoveCommandsPropagateAndTrackDifferentActiveOrders)
{
    auto [first, second, third] = createSharedHeads({ unload(0), waypoint(), waitFor(2) });
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    first->currentOrder = 0;
    second->currentOrder = 1;
    third->currentOrder = 7;

    VehicleOrderUpArgs upArgs{};
    upArgs.head = second->id;
    upArgs.orderOffset = 7;
    ASSERT_EQ(runCommand(upArgs, vehicleOrderUp), 0U);
    const auto moved = makeOrderTable({ unload(0), waitFor(2), waypoint() });
    for (const auto* head : { first, second, third })
    {
        expectOrders(*head, moved);
    }
    EXPECT_EQ(first->currentOrder, 0);
    EXPECT_EQ(second->currentOrder, 2);
    EXPECT_EQ(third->currentOrder, 1);

    VehicleOrderDownArgs downArgs{};
    downArgs.head = first->id;
    downArgs.orderOffset = 1;
    ASSERT_EQ(runCommand(downArgs, vehicleOrderDown), 0U);
    const auto original = makeOrderTable({ unload(0), waypoint(), waitFor(2) });
    for (const auto* head : { first, second, third })
    {
        expectOrders(*head, original);
    }
    EXPECT_EQ(first->currentOrder, 0);
    EXPECT_EQ(second->currentOrder, 1);
    EXPECT_EQ(third->currentOrder, 7);
}

TEST_F(SharedOrderTest, StructuralEditsPreserveUnbunchingAndResetEveryMember)
{
    auto heads = createSharedHeads({ unbunch(1), waypoint() });
    ASSERT_NE(heads[0], nullptr);

    for (auto* head : heads)
    {
        head->unbunchingLastDepartureTick = 100;
        head->unbunchingState = 200;
    }
    VehicleOrderDownArgs down{};
    down.head = heads[0]->id;
    down.orderOffset = 0;
    EXPECT_EQ(runCommand(down, vehicleOrderDown, 0), 0U);
    for (const auto* head : heads)
    {
        EXPECT_EQ(head->unbunchingLastDepartureTick, 100U);
        EXPECT_EQ(head->unbunchingState, 200U);
    }

    ASSERT_EQ(runCommand(down, vehicleOrderDown), 0U);
    for (const auto* head : heads)
    {
        const auto* stop = OrderRingView(head->orderTableOffset, sizeof(OrderRouteWaypoint)).begin()->as<OrderStopAt>();
        ASSERT_NE(stop, nullptr);
        EXPECT_TRUE(stop->isUnbunching());
        EXPECT_EQ(head->unbunchingLastDepartureTick, 0U);
        EXPECT_EQ(head->unbunchingState, 0U);
    }

    for (auto* head : heads)
    {
        head->unbunchingState = 1;
    }
    VehicleOrderUpArgs up{};
    up.head = heads[1]->id;
    up.orderOffset = sizeof(OrderRouteWaypoint);
    ASSERT_EQ(runCommand(up, vehicleOrderUp), 0U);
    for (const auto* head : heads)
    {
        const auto* stop = OrderRingView(head->orderTableOffset).begin()->as<OrderStopAt>();
        ASSERT_NE(stop, nullptr);
        EXPECT_TRUE(stop->isUnbunching());
        EXPECT_EQ(head->unbunchingState, 0U);
    }

    for (auto* head : heads)
    {
        head->unbunchingState = 1;
    }
    VehicleOrderReverseArgs reverse{};
    reverse.head = heads[2]->id;
    ASSERT_EQ(runCommand(reverse, vehicleOrderReverse), 0U);
    for (const auto* head : heads)
    {
        const auto* stop = OrderRingView(head->orderTableOffset, sizeof(OrderRouteWaypoint)).begin()->as<OrderStopAt>();
        ASSERT_NE(stop, nullptr);
        EXPECT_TRUE(stop->isUnbunching());
        EXPECT_EQ(head->unbunchingState, 0U);
    }
}

TEST_F(SharedOrderTest, FullLoadInsertionRejectsSharedUnbunchingRouteAtomically)
{
    auto heads = createSharedHeads({ unbunch(1), unload(2) });
    ASSERT_NE(heads[0], nullptr);
    const auto original = OrderManager::copyOrderTable(*heads[0]);

    VehicleOrderInsertArgs args{};
    args.head = heads[0]->id;
    args.orderOffset = heads[0]->sizeOfOrderTable - sizeof(OrderEnd);
    args.rawOrder = waitFor(2);
    EXPECT_EQ(runCommand(args, vehicleOrderInsert, 0), kFailure);
    EXPECT_EQ(runCommand(args, vehicleOrderInsert), kFailure);
    for (const auto* head : heads)
    {
        expectOrders(*head, original);
    }
}

TEST_F(SharedOrderTest, DuplicateStopConversionClearsUnbunchingForEveryMember)
{
    constexpr StationId stationId{ 1 };
    auto heads = createSharedHeads({ unbunch(enumValue(stationId)), unload(2) });
    ASSERT_NE(heads[0], nullptr);
    auto* station = StationManager::get(stationId);
    ASSERT_NE(station, nullptr);
    const auto previousOwner = station->owner;
    station->owner = kOwner;

    for (auto* head : heads)
    {
        head->unbunchingState = 1;
    }

    VehicleOrderInsertArgs args{};
    args.head = heads[0]->id;
    args.orderOffset = sizeof(OrderStopAt);
    args.rawOrder = OrderStopAt(stationId).getRaw();
    EXPECT_EQ(runCommand(args, vehicleOrderInsert, 0), 0U);
    EXPECT_TRUE(heads[0]->hasUnbunchingOrder());
    EXPECT_EQ(runCommand(args, vehicleOrderInsert), 0U);
    for (const auto* head : heads)
    {
        const auto* routeThrough = OrderRingView(head->orderTableOffset).begin()->as<OrderRouteThrough>();
        EXPECT_NE(routeThrough, nullptr);
        if (routeThrough != nullptr)
        {
            EXPECT_EQ(routeThrough->getStation(), stationId);
        }
        EXPECT_FALSE(head->hasUnbunchingOrder());
        EXPECT_EQ(head->unbunchingState, 0U);
    }

    station->owner = previousOwner;
}

TEST_F(SharedOrderTest, ReverseCommandSupportsLargeGlobalOrderTableOffsets)
{
    OrderManager::orderTableLength() = std::numeric_limits<uint16_t>::max() + 1U;
    auto heads = createSharedHeads({ unload(0), waypoint(), waitFor(2) });
    ASSERT_NE(heads[0], nullptr);
    ASSERT_GT(heads[0]->orderTableOffset, std::numeric_limits<uint16_t>::max());

    VehicleOrderReverseArgs args{};
    args.head = heads[0]->id;
    ASSERT_EQ(runCommand(args, vehicleOrderReverse), 0U);
    const auto expected = makeOrderTable({ waitFor(2), waypoint(), unload(0) });
    for (const auto* head : heads)
    {
        expectOrders(*head, expected);
    }
}

TEST_F(SharedOrderTest, ReverseCommandPropagatesAndMapsEachCursorIndependently)
{
    auto [first, second, third] = createSharedHeads({ unload(0), waypoint(), waitFor(2) });
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    first->currentOrder = 0;
    second->currentOrder = 1;
    third->currentOrder = 7;

    VehicleOrderReverseArgs args{};
    args.head = second->id;
    ASSERT_EQ(runCommand(args, vehicleOrderReverse), 0U);

    const auto expected = makeOrderTable({ waitFor(2), waypoint(), unload(0) });
    expectOrders(*first, expected);
    expectOrders(*second, expected);
    expectOrders(*third, expected);
    EXPECT_EQ(first->currentOrder, 7);
    EXPECT_EQ(second->currentOrder, 1);
    EXPECT_EQ(third->currentOrder, 0);
}

TEST_F(SharedOrderTest, SkipCommandRemainsLocalToSelectedVehicle)
{
    auto* selected = createHead();
    auto* peer = createHead();
    ASSERT_NE(selected, nullptr);
    ASSERT_NE(peer, nullptr);
    for (auto* head : { selected, peer })
    {
        setOrders(*head, { unload(0), waitFor(1), unload(2) });
    }
    selected->currentOrder = 0;
    peer->currentOrder = 2;
    ASSERT_TRUE(SharedOrderManager::join(peer->id, selected->id));

    VehicleOrderSkipArgs args{};
    args.head = selected->id;
    EXPECT_EQ(runCommand(args, vehicleOrderSkip, 0), 0U);
    EXPECT_EQ(selected->currentOrder, 0);
    EXPECT_EQ(peer->currentOrder, 2);
    ASSERT_EQ(runCommand(args, vehicleOrderSkip), 0U);
    EXPECT_EQ(selected->currentOrder, 1);
    EXPECT_EQ(peer->currentOrder, 2);
}

TEST_F(SharedOrderTest, OrderCommandsRejectDivergedSharedTablesWithoutPartialMutation)
{
    auto* first = createHead();
    auto* second = createHead();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    setOrders(*first, { unload(0), waitFor(1) });
    setOrders(*second, { unload(0), waitFor(1) });
    ASSERT_TRUE(SharedOrderManager::join(second->id, first->id));
    setOrders(*second, { unload(0), waitFor(2) });
    const auto firstBefore = OrderManager::copyOrderTable(*first);
    const auto secondBefore = OrderManager::copyOrderTable(*second);
    const auto lengthBefore = OrderManager::orderTableLength();

    VehicleOrderInsertArgs args{};
    args.head = first->id;
    args.orderOffset = 1;
    args.rawOrder = unload(3);
    EXPECT_EQ(runCommand(args, vehicleOrderInsert, 0), kFailure);
    EXPECT_EQ(runCommand(args, vehicleOrderInsert), kFailure);

    expectOrders(*first, firstBefore);
    expectOrders(*second, secondBefore);
    EXPECT_EQ(OrderManager::orderTableLength(), lengthBefore);
    EXPECT_EQ(SharedOrderManager::getMembers(first->id), (std::vector{ first->id, second->id }));
}
