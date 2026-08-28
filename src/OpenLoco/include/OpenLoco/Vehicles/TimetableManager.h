#pragma once

#include "Types.hpp"
#include "Vehicles/Orders.h"
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles::SharedOrderManager
{
    struct State;
}

namespace OpenLoco::Vehicles::TimetableManager
{
    using ServiceId = uint32_t;
    using EntryId = uint32_t;

    constexpr ServiceId kInvalidServiceId = 0;
    constexpr EntryId kInvalidEntryId = 0;
    constexpr uint16_t kDefaultTicksPerMinute = 128;
    constexpr uint16_t kMinTicksPerMinute = 1;
    constexpr uint16_t kMaxTicksPerMinute = 1024;
    constexpr uint32_t kDefaultPeriodMinutes = 60;
    constexpr uint32_t kMaxPeriodMinutes = 7 * 24 * 60;
    constexpr size_t kMaxSlots = 512;

    struct DispatchPattern
    {
        uint32_t periodMinutes = kDefaultPeriodMinutes;
        uint32_t phaseMinutes{};
        uint32_t maxDelayMinutes{};
        std::vector<uint32_t> slots;
        std::optional<int64_t> lastClaimedMinute;

        bool operator==(const DispatchPattern&) const = default;
    };

    struct TimetableEntry
    {
        EntryId id = kInvalidEntryId;
        uint8_t orderIndex{};
        OrderType orderType = OrderType::End;
        StationId station = StationId::null;
        std::optional<uint32_t> travelMinutes;
        std::optional<uint32_t> dwellMinutes;
        std::optional<DispatchPattern> dispatch;

        bool operator==(const TimetableEntry&) const = default;
    };

    struct Service
    {
        ServiceId id = kInvalidServiceId;
        uint32_t revision{};
        std::vector<TimetableEntry> entries;

        bool operator==(const Service&) const = default;
    };

    struct VehicleAssignment
    {
        EntityId vehicle = EntityId::null;
        ServiceId service = kInvalidServiceId;

        bool operator==(const VehicleAssignment&) const = default;
    };

    struct VehicleRuntime
    {
        EntityId vehicle = EntityId::null;
        ServiceId service = kInvalidServiceId;
        uint32_t serviceRevision{};
        EntryId currentEntry = kInvalidEntryId;
        uint64_t scheduledArrivalTick{};
        uint64_t scheduledDepartureTick{};
        std::optional<int64_t> assignedSlotMinute;
        int64_t latenessTicks{};
        bool timetableStarted{};
        bool atTimedStop{};
        bool released{};
        bool waiting{};

        bool operator==(const VehicleRuntime&) const = default;
    };

    struct State
    {
        uint16_t ticksPerMinute = kDefaultTicksPerMinute;
        uint64_t clockTicks{};
        ServiceId nextServiceId = 1;
        EntryId nextEntryId = 1;
        std::vector<Service> services;
        std::vector<VehicleAssignment> assignments;
        std::vector<VehicleRuntime> vehicles;

        bool operator==(const State&) const = default;
    };

    struct SlotClaim
    {
        int64_t scheduledMinute{};
        size_t slotIndex{};

        bool operator==(const SlotClaim&) const = default;
    };

    struct FleetEstimate
    {
        uint32_t measuredCycleMinutes{};
        uint32_t requiredVehicles{};
        uint32_t activeVehicles{};
        uint32_t sampleCount{};
    };

    struct MeasurementState
    {
        struct Anchor
        {
            EntityId vehicle = EntityId::null;
            ServiceId service = kInvalidServiceId;
            uint32_t serviceRevision{};
            EntryId entry = kInvalidEntryId;
            uint16_t ticksPerMinute{};
            uint64_t departureTick{};
            std::optional<uint64_t> readyTick;
        };

        struct Sample
        {
            EntityId vehicle = EntityId::null;
            ServiceId service = kInvalidServiceId;
            uint32_t serviceRevision{};
            EntryId entry = kInvalidEntryId;
            uint16_t ticksPerMinute{};
            uint64_t durationTicks{};
        };

        std::vector<Anchor> anchors;
        std::vector<Sample> samples;
    };

    void reset(uint64_t clockTicks = 0);
    void tick();
    uint64_t getClockTicks();
    uint64_t getClockMinute();
    uint16_t getTicksPerMinute();
    bool setTicksPerMinute(uint16_t value);

    std::optional<SlotClaim> findNextSlot(const DispatchPattern& pattern, int64_t earliestMinute);
    std::optional<SlotClaim> claimNextSlot(DispatchPattern& pattern, int64_t earliestMinute);

    ServiceId createService();
    ServiceId cloneService(ServiceId source);
    bool removeService(ServiceId id);
    Service* getService(ServiceId id);
    Service* getServiceForVehicle(EntityId vehicle);
    ServiceId getServiceId(EntityId vehicle);
    bool assignVehicle(EntityId vehicle, ServiceId service);
    void unassignVehicle(EntityId vehicle);
    std::vector<EntityId> getAssignedVehicles(ServiceId service);
    bool enableForVehicle(EntityId vehicle);
    bool disableForVehicle(EntityId vehicle);
    bool adoptService(EntityId target, EntityId source);
    bool canSplitService(EntityId vehicle);
    bool splitService(EntityId vehicle);
    void removeVehicle(EntityId vehicle);

    TimetableEntry* getEntry(EntityId vehicle, uint8_t orderIndex);
    bool setTravelMinutes(EntityId vehicle, uint8_t orderIndex, std::optional<uint32_t> minutes);
    bool setDwellMinutes(EntityId vehicle, uint8_t orderIndex, std::optional<uint32_t> minutes);
    bool setDispatchPeriod(EntityId vehicle, uint8_t orderIndex, uint32_t minutes);
    bool setDispatchPhase(EntityId vehicle, uint8_t orderIndex, uint32_t minutes);
    bool setDispatchMaxDelay(EntityId vehicle, uint8_t orderIndex, uint32_t minutes);
    bool addDispatchSlot(EntityId vehicle, uint8_t orderIndex, uint32_t minute);
    bool setEvenlySpacedSlots(EntityId vehicle, uint8_t orderIndex, uint32_t count);
    bool removeDispatchSlot(EntityId vehicle, uint8_t orderIndex, uint32_t minute);
    bool clearDispatch(EntityId vehicle, uint8_t orderIndex);
    bool onOrderInserted(EntityId vehicle, uint8_t orderIndex, OrderType type, StationId station = StationId::null);
    bool onOrderDeleted(EntityId vehicle, uint8_t orderIndex);
    bool onOrderReplaced(EntityId vehicle, uint8_t orderIndex, OrderType type, StationId station = StationId::null);
    bool onOrdersSwapped(EntityId vehicle, uint8_t firstIndex, uint8_t secondIndex);
    bool onOrdersReversed(EntityId vehicle, uint8_t orderCount);

    bool arriveAtOrder(EntityId vehicle, uint8_t orderIndex);
    bool isWaitingForDeparture(EntityId vehicle);
    bool isWaitingAtTimedStop(EntityId vehicle);
    void departFromOrder(EntityId vehicle);
    std::optional<FleetEstimate> getFleetEstimate(EntityId vehicle, uint8_t orderIndex);
    MeasurementState captureMeasurementState();
    void restoreMeasurementState(const MeasurementState& state);

    VehicleRuntime* getVehicleRuntime(EntityId vehicle);
    VehicleRuntime* resetVehicleRuntime(EntityId vehicle);
    void clearVehicleRuntime(EntityId vehicle);

    EntryId allocateEntryId();
    void resetServiceRuntime(ServiceId service);
    void resetDispatchState(ServiceId service);

    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState, const SharedOrderManager::State& sharedOrders);
    bool restoreState(const State& state);
}
