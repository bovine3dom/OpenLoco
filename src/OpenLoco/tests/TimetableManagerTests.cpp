#include <OpenLoco/Vehicles/TimetableManager.h>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles;

TEST(TimetableManager, ClaimsRepeatingSlotsWithMaximumDelay)
{
    TimetableManager::DispatchPattern pattern;
    pattern.periodMinutes = 60;
    pattern.maxDelayMinutes = 5;
    pattern.slots = { 0, 15, 30, 45 };

    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 11), (TimetableManager::SlotClaim{ 15, 1 }));
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 18), (TimetableManager::SlotClaim{ 30, 2 }));

    pattern.lastClaimedMinute.reset();
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 18), (TimetableManager::SlotClaim{ 15, 1 }));
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 23), (TimetableManager::SlotClaim{ 30, 2 }));
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 61), (TimetableManager::SlotClaim{ 60, 0 }));
}

TEST(TimetableManager, NeverReusesAClaimedSlot)
{
    TimetableManager::DispatchPattern pattern;
    pattern.periodMinutes = 60;
    pattern.maxDelayMinutes = 10;
    pattern.slots = { 0, 15 };

    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 15)->scheduledMinute, 15);
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 15)->scheduledMinute, 60);
    EXPECT_EQ(TimetableManager::claimNextSlot(pattern, 60)->scheduledMinute, 75);
}

TEST(TimetableManager, PhaseOffsetsSlotsAgainstTheGlobalClock)
{
    TimetableManager::DispatchPattern pattern;
    pattern.periodMinutes = 60;
    pattern.phaseMinutes = 5;
    pattern.slots = { 0, 20, 40 };

    EXPECT_EQ(TimetableManager::findNextSlot(pattern, 0)->scheduledMinute, 5);
    EXPECT_EQ(TimetableManager::findNextSlot(pattern, 26)->scheduledMinute, 45);
    EXPECT_EQ(TimetableManager::findNextSlot(pattern, 66)->scheduledMinute, 85);
}

TEST(TimetableManager, ClockRateChangesPreserveClockPhaseAndResetRuntime)
{
    TimetableManager::reset(95);
    ASSERT_TRUE(TimetableManager::setTicksPerMinute(32));
    EXPECT_EQ(TimetableManager::getClockMinute(), 2);

    const auto service = TimetableManager::createService();
    ASSERT_NE(service, TimetableManager::kInvalidServiceId);
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(3), service));
    ASSERT_NE(TimetableManager::getVehicleRuntime(EntityId(3)), nullptr);

    ASSERT_TRUE(TimetableManager::setTicksPerMinute(16));
    EXPECT_EQ(TimetableManager::getClockMinute(), 2);
    EXPECT_EQ(TimetableManager::getVehicleRuntime(EntityId(3)), nullptr);
    EXPECT_FALSE(TimetableManager::setTicksPerMinute(0));
}

TEST(TimetableManager, StateRoundTripsCanonically)
{
    TimetableManager::reset(1234);
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);
    service->revision = 4;
    service->entries.push_back({
        .id = TimetableManager::allocateEntryId(),
        .orderIndex = 1,
        .orderType = OrderType::StopAt,
        .station = StationId(7),
        .travelMinutes = 12,
        .dwellMinutes = 3,
        .dispatch = TimetableManager::DispatchPattern{ .periodMinutes = 60, .phaseMinutes = 0, .maxDelayMinutes = 0, .slots = { 0, 15, 30, 45 }, .lastClaimedMinute = std::nullopt },
    });
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));
    auto* runtime = TimetableManager::resetVehicleRuntime(EntityId(9));
    ASSERT_NE(runtime, nullptr);
    runtime->serviceRevision = service->revision;
    runtime->currentEntry = service->entries.front().id;
    runtime->timetableStarted = true;

    const auto state = TimetableManager::captureState();
    ASSERT_TRUE(TimetableManager::validateState(state));
    TimetableManager::reset();
    ASSERT_TRUE(TimetableManager::restoreState(state));
    EXPECT_EQ(TimetableManager::captureState(), state);
}

TEST(TimetableManager, RejectsMalformedSchedules)
{
    TimetableManager::State state;
    TimetableManager::Service service;
    service.id = 1;
    service.entries.push_back({
        .id = 1,
        .orderIndex = 0,
        .orderType = OrderType::StopAt,
        .station = StationId(1),
        .travelMinutes = std::nullopt,
        .dwellMinutes = std::nullopt,
        .dispatch = TimetableManager::DispatchPattern{ .periodMinutes = 60, .phaseMinutes = 0, .maxDelayMinutes = 0, .slots = { 15, 15 }, .lastClaimedMinute = std::nullopt },
    });
    state.services.push_back(service);
    state.nextServiceId = 2;
    state.nextEntryId = 2;
    EXPECT_FALSE(TimetableManager::validateState(state));

    state.services.front().entries.front().dispatch->slots = { 15 };
    EXPECT_FALSE(TimetableManager::validateState(state));
}

TEST(TimetableManager, DwellAndTravelTimesKeepAnAbsoluteSchedule)
{
    TimetableManager::reset(100);
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);

    TimetableManager::TimetableEntry first;
    first.id = TimetableManager::allocateEntryId();
    first.orderIndex = 0;
    first.orderType = OrderType::StopAt;
    first.dwellMinutes = 10;
    TimetableManager::TimetableEntry second;
    second.id = TimetableManager::allocateEntryId();
    second.orderIndex = 1;
    second.orderType = OrderType::StopAt;
    second.travelMinutes = 20;
    second.dwellMinutes = 5;
    service->entries = { first, second };
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));

    ASSERT_TRUE(TimetableManager::arriveAtOrder(EntityId(9), 0));
    auto* runtime = TimetableManager::getVehicleRuntime(EntityId(9));
    ASSERT_NE(runtime, nullptr);
    EXPECT_EQ(runtime->scheduledArrivalTick, 100U);
    EXPECT_EQ(runtime->scheduledDepartureTick, 420U);
    EXPECT_TRUE(TimetableManager::isWaitingForDeparture(EntityId(9)));
    EXPECT_TRUE(TimetableManager::isWaitingAtTimedStop(EntityId(9)));

    for (size_t i = 0; i < 320; ++i)
    {
        TimetableManager::tick();
    }
    EXPECT_FALSE(TimetableManager::isWaitingForDeparture(EntityId(9)));
    EXPECT_FALSE(TimetableManager::isWaitingAtTimedStop(EntityId(9)));
    TimetableManager::departFromOrder(EntityId(9));
    EXPECT_EQ(runtime->currentEntry, second.id);
    EXPECT_EQ(runtime->scheduledArrivalTick, 1060U);

    for (size_t i = 0; i < 645; ++i)
    {
        TimetableManager::tick();
    }
    ASSERT_TRUE(TimetableManager::arriveAtOrder(EntityId(9), 1));
    EXPECT_EQ(runtime->latenessTicks, 5);
    EXPECT_EQ(runtime->scheduledDepartureTick, 1220U);
}

TEST(TimetableManager, DispatchClaimsLateSlotsOnceAcrossSharedVehicles)
{
    TimetableManager::reset(18 * TimetableManager::kDefaultTicksPerMinute);
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);

    TimetableManager::TimetableEntry entry;
    entry.id = TimetableManager::allocateEntryId();
    entry.orderIndex = 0;
    entry.orderType = OrderType::StopAt;
    entry.dispatch.emplace();
    entry.dispatch->maxDelayMinutes = 5;
    entry.dispatch->slots = { 0, 15, 30, 45 };
    service->entries.push_back(entry);
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(10), serviceId));

    ASSERT_TRUE(TimetableManager::arriveAtOrder(EntityId(9), 0));
    EXPECT_FALSE(TimetableManager::isWaitingForDeparture(EntityId(9)));
    auto* first = TimetableManager::getVehicleRuntime(EntityId(9));
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first->assignedSlotMinute, 15);
    EXPECT_EQ(first->latenessTicks, 3 * TimetableManager::kDefaultTicksPerMinute);

    ASSERT_TRUE(TimetableManager::arriveAtOrder(EntityId(10), 0));
    EXPECT_TRUE(TimetableManager::isWaitingForDeparture(EntityId(10)));
    auto* second = TimetableManager::getVehicleRuntime(EntityId(10));
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(second->assignedSlotMinute, 30);
    EXPECT_EQ(second->scheduledDepartureTick, 30U * TimetableManager::kDefaultTicksPerMinute);
}

TEST(TimetableManager, ResetDispatchKeepsWaitingVehiclesAtTheirStop)
{
    TimetableManager::reset();
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);

    TimetableManager::TimetableEntry entry;
    entry.id = TimetableManager::allocateEntryId();
    entry.orderIndex = 0;
    entry.orderType = OrderType::StopAt;
    entry.dwellMinutes = 10;
    entry.dispatch.emplace();
    entry.dispatch->slots = { 15 };
    service->entries.push_back(entry);
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));
    ASSERT_TRUE(TimetableManager::arriveAtOrder(EntityId(9), 0));
    ASSERT_TRUE(TimetableManager::isWaitingForDeparture(EntityId(9)));
    ASSERT_EQ(TimetableManager::getVehicleRuntime(EntityId(9))->scheduledDepartureTick, 15U * TimetableManager::kDefaultTicksPerMinute);

    TimetableManager::resetDispatchState(serviceId);
    auto* runtime = TimetableManager::getVehicleRuntime(EntityId(9));
    ASSERT_NE(runtime, nullptr);
    EXPECT_TRUE(runtime->atTimedStop);
    EXPECT_FALSE(runtime->assignedSlotMinute.has_value());
    EXPECT_EQ(runtime->scheduledDepartureTick, 10U * TimetableManager::kDefaultTicksPerMinute);
    EXPECT_TRUE(TimetableManager::isWaitingForDeparture(EntityId(9)));
    EXPECT_EQ(runtime->assignedSlotMinute, 15);
}

TEST(TimetableManager, DispatchScheduleEditsClearIncompatibleClaims)
{
    TimetableManager::reset();
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);

    TimetableManager::TimetableEntry entry;
    entry.id = TimetableManager::allocateEntryId();
    entry.orderIndex = 0;
    entry.orderType = OrderType::StopAt;
    entry.dispatch.emplace();
    entry.dispatch->slots = { 15 };
    entry.dispatch->lastClaimedMinute = 15;
    service->entries.push_back(entry);
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));

    ASSERT_TRUE(TimetableManager::setDispatchPhase(EntityId(9), 0, 5));
    EXPECT_FALSE(service->entries.front().dispatch->lastClaimedMinute.has_value());
    service->entries.front().dispatch->lastClaimedMinute = 20;
    ASSERT_TRUE(TimetableManager::removeDispatchSlot(EntityId(9), 0, 15));
    EXPECT_FALSE(service->entries.front().dispatch->lastClaimedMinute.has_value());
    EXPECT_TRUE(TimetableManager::validateState(TimetableManager::captureState()));
}

TEST(TimetableManager, RejectsDuplicateRuntimeSlotClaims)
{
    TimetableManager::reset();
    const auto serviceId = TimetableManager::createService();
    auto* service = TimetableManager::getService(serviceId);
    ASSERT_NE(service, nullptr);

    TimetableManager::TimetableEntry entry;
    entry.id = TimetableManager::allocateEntryId();
    entry.orderIndex = 0;
    entry.orderType = OrderType::StopAt;
    entry.dispatch.emplace();
    entry.dispatch->slots = { 15 };
    entry.dispatch->lastClaimedMinute = 15;
    service->entries.push_back(entry);
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(9), serviceId));
    ASSERT_TRUE(TimetableManager::assignVehicle(EntityId(10), serviceId));
    for (const auto vehicle : { EntityId(9), EntityId(10) })
    {
        auto* runtime = TimetableManager::getVehicleRuntime(vehicle);
        ASSERT_NE(runtime, nullptr);
        runtime->currentEntry = entry.id;
        runtime->assignedSlotMinute = 15;
        runtime->timetableStarted = true;
        runtime->atTimedStop = true;
        runtime->waiting = true;
    }
    EXPECT_FALSE(TimetableManager::validateState(TimetableManager::captureState()));
}
