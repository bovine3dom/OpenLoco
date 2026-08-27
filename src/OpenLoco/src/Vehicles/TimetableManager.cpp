#include "Vehicles/TimetableManager.h"
#include "Entities/EntityManager.h"
#include "GameState.h"
#include "S5/Limits.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleHead.h"
#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <numeric>
#include <ranges>
#include <tuple>

namespace OpenLoco::Vehicles::TimetableManager
{
    namespace
    {
        constexpr auto kInvalidMinute = std::numeric_limits<int64_t>::min();

        uint16_t _ticksPerMinute = kDefaultTicksPerMinute;
        uint64_t _clockTicks{};
        ServiceId _nextServiceId = 1;
        EntryId _nextEntryId = 1;
        std::vector<Service> _services;
        std::array<ServiceId, S5::Limits::kMaxEntities> _assignments{};
        std::array<std::optional<VehicleRuntime>, S5::Limits::kMaxEntities> _vehicleRuntime{};

        bool isValidVehicleId(const EntityId id)
        {
            return id != EntityId::null && enumValue(id) < S5::Limits::kMaxEntities;
        }

        bool isValidOrderType(const OrderType type)
        {
            return type == OrderType::StopAt || type == OrderType::RouteThrough || type == OrderType::RouteWaypoint;
        }

        bool isValidDispatchPattern(const DispatchPattern& pattern)
        {
            if (pattern.periodMinutes == 0 || pattern.periodMinutes > kMaxPeriodMinutes
                || pattern.phaseMinutes >= pattern.periodMinutes || pattern.maxDelayMinutes > kMaxPeriodMinutes
                || pattern.slots.size() > kMaxSlots
                || !std::ranges::is_sorted(pattern.slots))
            {
                return false;
            }
            for (size_t i = 0; i < pattern.slots.size(); ++i)
            {
                if (pattern.slots[i] >= pattern.periodMinutes || (i != 0 && pattern.slots[i - 1] == pattern.slots[i]))
                {
                    return false;
                }
            }
            if (pattern.lastClaimedMinute.has_value())
            {
                if (*pattern.lastClaimedMinute < 0 || *pattern.lastClaimedMinute == std::numeric_limits<int64_t>::max())
                {
                    return false;
                }
                const auto residue = static_cast<uint32_t>(*pattern.lastClaimedMinute % pattern.periodMinutes);
                if (std::ranges::none_of(pattern.slots, [&](const uint32_t slot) {
                        return (pattern.phaseMinutes + slot) % pattern.periodMinutes == residue;
                    }))
                {
                    return false;
                }
            }
            return true;
        }

        int64_t firstOccurrenceAtOrAfter(const uint32_t residue, const uint32_t period, const int64_t minimum)
        {
            if (minimum <= static_cast<int64_t>(residue))
            {
                return residue;
            }
            const auto delta = static_cast<uint64_t>(minimum - residue);
            const auto cycles = (delta + period - 1) / period;
            if (cycles > static_cast<uint64_t>((std::numeric_limits<int64_t>::max() - residue) / period))
            {
                return kInvalidMinute;
            }
            return residue + static_cast<int64_t>(cycles * period);
        }

        auto findService(const ServiceId id)
        {
            return std::ranges::lower_bound(_services, id, {}, &Service::id);
        }

        bool isCanonicalService(const Service& service, std::vector<bool>& seenEntryIds)
        {
            if (service.id == kInvalidServiceId || service.entries.size() > S5::Limits::kMaxOrdersPerVehicle)
            {
                return false;
            }
            uint8_t previousOrderIndex{};
            bool first = true;
            for (const auto& entry : service.entries)
            {
                if (entry.id == kInvalidEntryId || entry.id >= seenEntryIds.size() || seenEntryIds[entry.id]
                    || !isValidOrderType(entry.orderType) || (!first && entry.orderIndex <= previousOrderIndex)
                    || (entry.orderType != OrderType::StopAt && (entry.dwellMinutes.has_value() || entry.dispatch.has_value()))
                    || (entry.travelMinutes.has_value() && *entry.travelMinutes > kMaxPeriodMinutes)
                    || (entry.dwellMinutes.has_value() && *entry.dwellMinutes > kMaxPeriodMinutes)
                    || (entry.dispatch.has_value() && !isValidDispatchPattern(*entry.dispatch)))
                {
                    return false;
                }
                seenEntryIds[entry.id] = true;
                previousOrderIndex = entry.orderIndex;
                first = false;
            }
            return true;
        }

        VehicleHead* getHead(const EntityId id)
        {
            auto* vehicle = isValidVehicleId(id) ? EntityManager::get<VehicleBase>(id) : nullptr;
            return vehicle != nullptr && vehicle->isVehicleHead() ? vehicle->asVehicleHead() : nullptr;
        }

        bool buildEntries(Service& service, const VehicleHead& head)
        {
            service.entries.clear();
            const auto tableSize = static_cast<uint32_t>(head.sizeOfOrderTable);
            const auto tableOffset = head.orderTableOffset;
            const auto globalTableSize = OrderManager::orderTableLength();
            if (tableSize < sizeof(OrderEnd) || tableOffset > globalTableSize || tableSize > globalTableSize - tableOffset)
            {
                return false;
            }

            const auto nextEntryId = _nextEntryId;
            uint32_t orderIndex = 0;
            for (uint32_t offset = 0; offset < tableSize; ++orderIndex)
            {
                auto* order = OrderManager::orders() + tableOffset + offset;
                const auto type = order->getType();
                const auto orderSize = OrderManager::getOrderSize(type);
                if (orderSize == 0 || orderSize > tableSize - offset)
                {
                    break;
                }
                if (type == OrderType::End)
                {
                    if (offset + orderSize == tableSize)
                    {
                        return true;
                    }
                    break;
                }
                if (orderIndex >= S5::Limits::kMaxOrdersPerVehicle)
                {
                    break;
                }
                if (isValidOrderType(type))
                {
                    const auto id = allocateEntryId();
                    if (id == kInvalidEntryId)
                    {
                        service.entries.clear();
                        _nextEntryId = nextEntryId;
                        return false;
                    }
                    TimetableEntry entry;
                    entry.id = id;
                    entry.orderIndex = static_cast<uint8_t>(orderIndex);
                    entry.orderType = type;
                    if (const auto* station = order->as<OrderStation>(); station != nullptr)
                    {
                        entry.station = station->getStation();
                    }
                    service.entries.push_back(std::move(entry));
                }
                offset += orderSize;
            }
            service.entries.clear();
            _nextEntryId = nextEntryId;
            return false;
        }

        void removeServiceIfUnused(const ServiceId service)
        {
            if (service != kInvalidServiceId && getAssignedVehicles(service).empty())
            {
                removeService(service);
            }
        }

        bool updateService(const EntityId vehicle, auto&& update)
        {
            auto* service = getServiceForVehicle(vehicle);
            if (service == nullptr || !update(*service))
            {
                return false;
            }
            ++service->revision;
            resetServiceRuntime(service->id);
            return true;
        }

        uint64_t addMinutes(const uint64_t tick, const uint32_t minutes)
        {
            const auto duration = static_cast<uint64_t>(minutes) * _ticksPerMinute;
            return duration > std::numeric_limits<uint64_t>::max() - tick
                ? std::numeric_limits<uint64_t>::max()
                : tick + duration;
        }

        int64_t tickDifference(const uint64_t actual, const uint64_t scheduled)
        {
            if (actual >= scheduled)
            {
                return static_cast<int64_t>(std::min<uint64_t>(actual - scheduled, std::numeric_limits<int64_t>::max()));
            }
            return -static_cast<int64_t>(std::min<uint64_t>(scheduled - actual, std::numeric_limits<int64_t>::max()));
        }

        uint64_t minuteToTick(const int64_t minute)
        {
            const auto value = static_cast<uint64_t>(std::max<int64_t>(minute, 0));
            return value > std::numeric_limits<uint64_t>::max() / _ticksPerMinute
                ? std::numeric_limits<uint64_t>::max()
                : value * _ticksPerMinute;
        }

        void scheduleNextEntry(Service& service, VehicleRuntime& runtime)
        {
            const auto current = std::ranges::find(service.entries, runtime.currentEntry, &TimetableEntry::id);
            if (current == service.entries.end() || service.entries.empty())
            {
                const auto vehicle = runtime.vehicle;
                const auto serviceId = runtime.service;
                const auto revision = runtime.serviceRevision;
                runtime = {};
                runtime.vehicle = vehicle;
                runtime.service = serviceId;
                runtime.serviceRevision = revision;
                return;
            }

            auto next = std::next(current);
            if (next == service.entries.end())
            {
                next = service.entries.begin();
            }
            runtime.currentEntry = next->id;
            runtime.atTimedStop = false;
            runtime.released = false;
            runtime.waiting = false;
            runtime.assignedSlotMinute.reset();
            if (next->travelMinutes.has_value())
            {
                runtime.scheduledArrivalTick = addMinutes(runtime.scheduledDepartureTick, *next->travelMinutes);
                runtime.timetableStarted = true;
            }
            else
            {
                runtime.scheduledArrivalTick = _clockTicks;
                runtime.timetableStarted = false;
            }
        }

        const VehicleHead* getHead(const GameState& gameState, const EntityId id)
        {
            if (!isValidVehicleId(id))
            {
                return nullptr;
            }
            const auto* vehicle = gameState.entities[enumValue(id)].asBase<VehicleBase>();
            return vehicle != nullptr && vehicle->isVehicleHead() && vehicle->id == id
                ? reinterpret_cast<const VehicleHead*>(vehicle)
                : nullptr;
        }

        bool entriesMatchOrders(const Service& service, const VehicleHead& head, const GameState& gameState)
        {
            const auto tableSize = static_cast<uint32_t>(head.sizeOfOrderTable);
            const auto tableOffset = head.orderTableOffset;
            if (tableSize < sizeof(OrderEnd) || gameState.orderTableLength > S5::Limits::kMaxOrders
                || tableOffset > gameState.orderTableLength || tableSize > gameState.orderTableLength - tableOffset)
            {
                return false;
            }

            auto entry = service.entries.begin();
            uint8_t orderIndex = 0;
            bool currentOrderIsValid = false;
            for (uint32_t offset = 0; offset < tableSize; ++orderIndex)
            {
                const auto* order = reinterpret_cast<const Order*>(&gameState.orders[tableOffset + offset]);
                const auto type = order->getType();
                const auto orderSize = OrderManager::getOrderSize(type);
                if (orderSize == 0 || orderSize > tableSize - offset)
                {
                    return false;
                }
                if (type == OrderType::End)
                {
                    return offset + orderSize == tableSize && entry == service.entries.end()
                        && (currentOrderIsValid || (offset == 0 && head.currentOrder == 0));
                }
                if (orderIndex >= S5::Limits::kMaxOrdersPerVehicle)
                {
                    return false;
                }
                if (isValidOrderType(type))
                {
                    StationId station = StationId::null;
                    if (const auto* stationOrder = order->as<OrderStation>(); stationOrder != nullptr)
                    {
                        station = stationOrder->getStation();
                    }
                    if (entry == service.entries.end() || entry->orderIndex != orderIndex || entry->orderType != type || entry->station != station
                        || (type == OrderType::StopAt && order->as<OrderStopAt>()->isUnbunching()))
                    {
                        return false;
                    }
                    ++entry;
                }
                currentOrderIsValid |= head.currentOrder == offset;
                offset += orderSize;
            }
            return false;
        }
    }

    void reset(const uint64_t clockTicks)
    {
        _ticksPerMinute = kDefaultTicksPerMinute;
        _clockTicks = clockTicks;
        _nextServiceId = 1;
        _nextEntryId = 1;
        _services.clear();
        _assignments.fill(kInvalidServiceId);
        _vehicleRuntime = {};
    }

    void tick()
    {
        if (_clockTicks != std::numeric_limits<uint64_t>::max())
        {
            ++_clockTicks;
        }
    }

    uint64_t getClockTicks()
    {
        return _clockTicks;
    }

    uint64_t getClockMinute()
    {
        return _clockTicks / _ticksPerMinute;
    }

    uint16_t getTicksPerMinute()
    {
        return _ticksPerMinute;
    }

    bool setTicksPerMinute(const uint16_t value)
    {
        if (value < kMinTicksPerMinute || value > kMaxTicksPerMinute)
        {
            return false;
        }
        if (value == _ticksPerMinute)
        {
            return true;
        }

        const auto minute = _clockTicks / _ticksPerMinute;
        const auto remainder = _clockTicks % _ticksPerMinute;
        if (minute > std::numeric_limits<uint64_t>::max() / value)
        {
            _clockTicks = std::numeric_limits<uint64_t>::max();
        }
        else
        {
            const auto wholeMinutes = minute * value;
            const auto partialMinute = remainder * value / _ticksPerMinute;
            _clockTicks = partialMinute > std::numeric_limits<uint64_t>::max() - wholeMinutes
                ? std::numeric_limits<uint64_t>::max()
                : wholeMinutes + partialMinute;
        }
        _ticksPerMinute = value;
        for (auto& service : _services)
        {
            resetDispatchState(service.id);
        }
        for (auto& runtime : _vehicleRuntime)
        {
            runtime.reset();
        }
        return true;
    }

    std::optional<SlotClaim> findNextSlot(const DispatchPattern& pattern, const int64_t earliestMinute)
    {
        if (!isValidDispatchPattern(pattern) || pattern.slots.empty() || earliestMinute < 0)
        {
            return std::nullopt;
        }

        int64_t minimum = std::max<int64_t>(0, earliestMinute - pattern.maxDelayMinutes);
        if (pattern.lastClaimedMinute.has_value())
        {
            if (*pattern.lastClaimedMinute == std::numeric_limits<int64_t>::max())
            {
                return std::nullopt;
            }
            minimum = std::max(minimum, *pattern.lastClaimedMinute + 1);
        }

        SlotClaim result{ kInvalidMinute, 0 };
        for (size_t i = 0; i < pattern.slots.size(); ++i)
        {
            const auto residue = (pattern.phaseMinutes + pattern.slots[i]) % pattern.periodMinutes;
            const auto candidate = firstOccurrenceAtOrAfter(residue, pattern.periodMinutes, minimum);
            if (candidate != kInvalidMinute && (result.scheduledMinute == kInvalidMinute || candidate < result.scheduledMinute))
            {
                result = { candidate, i };
            }
        }
        return result.scheduledMinute == kInvalidMinute ? std::nullopt : std::optional<SlotClaim>{ result };
    }

    std::optional<SlotClaim> claimNextSlot(DispatchPattern& pattern, const int64_t earliestMinute)
    {
        auto result = findNextSlot(pattern, earliestMinute);
        if (result.has_value())
        {
            pattern.lastClaimedMinute = result->scheduledMinute;
        }
        return result;
    }

    ServiceId createService()
    {
        if (_services.size() >= S5::Limits::kMaxVehicles || _nextServiceId == kInvalidServiceId
            || _nextServiceId == std::numeric_limits<ServiceId>::max())
        {
            return kInvalidServiceId;
        }
        const auto id = _nextServiceId++;
        Service service;
        service.id = id;
        _services.push_back(std::move(service));
        return id;
    }

    ServiceId cloneService(const ServiceId sourceId)
    {
        const auto* sourceService = getService(sourceId);
        if (sourceService == nullptr)
        {
            return kInvalidServiceId;
        }
        const auto source = *sourceService;
        const auto entryCount = std::transform_reduce(_services.begin(), _services.end(), size_t{}, std::plus{}, [](const Service& service) { return service.entries.size(); });
        constexpr auto kMaxEntries = S5::Limits::kMaxVehicles * S5::Limits::kMaxOrdersPerVehicle;
        if (_services.size() >= S5::Limits::kMaxVehicles || _nextServiceId == std::numeric_limits<ServiceId>::max()
            || source.entries.size() > kMaxEntries - entryCount)
        {
            return kInvalidServiceId;
        }
        const auto id = createService();
        auto* clone = getService(id);
        if (clone == nullptr)
        {
            return kInvalidServiceId;
        }
        clone->revision = source.revision;
        clone->entries = source.entries;
        for (auto& entry : clone->entries)
        {
            entry.id = allocateEntryId();
            if (entry.id == kInvalidEntryId)
            {
                removeService(id);
                return kInvalidServiceId;
            }
            if (entry.dispatch.has_value())
            {
                entry.dispatch->lastClaimedMinute.reset();
            }
        }
        return id;
    }

    bool removeService(const ServiceId id)
    {
        const auto it = findService(id);
        if (it == _services.end() || it->id != id)
        {
            return false;
        }
        _services.erase(it);
        for (size_t i = 0; i < _assignments.size(); ++i)
        {
            if (_assignments[i] == id)
            {
                _assignments[i] = kInvalidServiceId;
                _vehicleRuntime[i].reset();
            }
        }
        return true;
    }

    Service* getService(const ServiceId id)
    {
        const auto it = findService(id);
        return it != _services.end() && it->id == id ? &*it : nullptr;
    }

    ServiceId getServiceId(const EntityId vehicle)
    {
        return isValidVehicleId(vehicle) ? _assignments[enumValue(vehicle)] : kInvalidServiceId;
    }

    Service* getServiceForVehicle(const EntityId vehicle)
    {
        return getService(getServiceId(vehicle));
    }

    bool assignVehicle(const EntityId vehicle, const ServiceId service)
    {
        if (!isValidVehicleId(vehicle) || getService(service) == nullptr)
        {
            return false;
        }
        _assignments[enumValue(vehicle)] = service;
        return resetVehicleRuntime(vehicle) != nullptr;
    }

    void unassignVehicle(const EntityId vehicle)
    {
        if (!isValidVehicleId(vehicle))
        {
            return;
        }
        _assignments[enumValue(vehicle)] = kInvalidServiceId;
        _vehicleRuntime[enumValue(vehicle)].reset();
    }

    std::vector<EntityId> getAssignedVehicles(const ServiceId service)
    {
        std::vector<EntityId> result;
        for (size_t i = 0; i < _assignments.size(); ++i)
        {
            if (_assignments[i] == service)
            {
                result.push_back(EntityId(i));
            }
        }
        return result;
    }

    bool enableForVehicle(const EntityId vehicle)
    {
        const auto* head = getHead(vehicle);
        if (head == nullptr)
        {
            return false;
        }
        const auto members = SharedOrderManager::getMembers(vehicle);
        ServiceId existing = kInvalidServiceId;
        for (const auto member : members)
        {
            const auto* memberHead = getHead(member);
            if (memberHead == nullptr || memberHead->hasUnbunchingOrder()
                || !SharedOrderManager::areOrdersEqual(*head, *memberHead)
                || (memberHead->sizeOfOrderTable == sizeof(OrderEnd)
                    ? memberHead->currentOrder != 0
                    : !OrderManager::isOrderOffsetValid(*memberHead, memberHead->currentOrder, false)))
            {
                return false;
            }
            const auto service = getServiceId(member);
            if (service != kInvalidServiceId && existing != kInvalidServiceId && service != existing)
            {
                return false;
            }
            existing = service != kInvalidServiceId ? service : existing;
        }
        if (existing != kInvalidServiceId)
        {
            for (const auto member : members)
            {
                assignVehicle(member, existing);
            }
            return true;
        }

        const auto serviceId = createService();
        auto* service = getService(serviceId);
        if (service == nullptr || !buildEntries(*service, *head))
        {
            removeService(serviceId);
            return false;
        }
        for (const auto member : members)
        {
            assignVehicle(member, serviceId);
        }
        return true;
    }

    bool disableForVehicle(const EntityId vehicle)
    {
        const auto service = getServiceId(vehicle);
        return service != kInvalidServiceId && removeService(service);
    }

    bool adoptService(const EntityId target, const EntityId source)
    {
        const auto oldService = getServiceId(target);
        const auto sourceService = getServiceId(source);
        if (sourceService == kInvalidServiceId)
        {
            unassignVehicle(target);
        }
        else if (!assignVehicle(target, sourceService))
        {
            return false;
        }
        removeServiceIfUnused(oldService);
        return true;
    }

    bool splitService(const EntityId vehicle)
    {
        const auto oldService = getServiceId(vehicle);
        if (oldService == kInvalidServiceId)
        {
            return true;
        }
        const auto newService = cloneService(oldService);
        if (newService == kInvalidServiceId || !assignVehicle(vehicle, newService))
        {
            removeService(newService);
            return false;
        }
        removeServiceIfUnused(oldService);
        return true;
    }

    bool canSplitService(const EntityId vehicle)
    {
        const auto oldService = getServiceId(vehicle);
        if (oldService == kInvalidServiceId)
        {
            return true;
        }
        const auto* service = getService(oldService);
        if (service == nullptr || _services.size() >= S5::Limits::kMaxVehicles
            || _nextServiceId == std::numeric_limits<ServiceId>::max())
        {
            return false;
        }
        const auto entryCount = std::transform_reduce(_services.begin(), _services.end(), size_t{}, std::plus{}, [](const Service& item) { return item.entries.size(); });
        constexpr auto kMaxEntries = S5::Limits::kMaxVehicles * S5::Limits::kMaxOrdersPerVehicle;
        return service->entries.size() <= kMaxEntries - entryCount;
    }

    void removeVehicle(const EntityId vehicle)
    {
        const auto service = getServiceId(vehicle);
        unassignVehicle(vehicle);
        removeServiceIfUnused(service);
    }

    TimetableEntry* getEntry(const EntityId vehicle, const uint8_t orderIndex)
    {
        auto* service = getServiceForVehicle(vehicle);
        if (service == nullptr)
        {
            return nullptr;
        }
        const auto it = std::ranges::find(service->entries, orderIndex, &TimetableEntry::orderIndex);
        return it != service->entries.end() ? &*it : nullptr;
    }

    bool setTravelMinutes(const EntityId vehicle, const uint8_t orderIndex, const std::optional<uint32_t> minutes)
    {
        if (minutes.has_value() && *minutes > kMaxPeriodMinutes)
        {
            return false;
        }
        const auto* entry = getEntry(vehicle, orderIndex);
        if (entry == nullptr)
        {
            return false;
        }
        if (entry->travelMinutes == minutes)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex, minutes](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end())
            {
                return false;
            }
            it->travelMinutes = minutes;
            return true;
        });
    }

    bool setDwellMinutes(const EntityId vehicle, const uint8_t orderIndex, const std::optional<uint32_t> minutes)
    {
        if (minutes.has_value() && *minutes > kMaxPeriodMinutes)
        {
            return false;
        }
        const auto* entry = getEntry(vehicle, orderIndex);
        if (entry == nullptr || entry->orderType != OrderType::StopAt)
        {
            return false;
        }
        if (entry->dwellMinutes == minutes)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex, minutes](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || it->orderType != OrderType::StopAt)
            {
                return false;
            }
            it->dwellMinutes = minutes;
            return true;
        });
    }

    bool setDispatchPeriod(const EntityId vehicle, const uint8_t orderIndex, const uint32_t minutes)
    {
        const auto* entry = getEntry(vehicle, orderIndex);
        if (entry != nullptr && entry->dispatch.has_value() && entry->dispatch->periodMinutes == minutes)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex, minutes](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || it->orderType != OrderType::StopAt || minutes == 0 || minutes > kMaxPeriodMinutes)
            {
                return false;
            }
            auto dispatch = it->dispatch.value_or(DispatchPattern{});
            if (dispatch.phaseMinutes >= minutes || std::ranges::any_of(dispatch.slots, [minutes](const auto slot) { return slot >= minutes; }))
            {
                return false;
            }
            dispatch.periodMinutes = minutes;
            dispatch.lastClaimedMinute.reset();
            it->dispatch = std::move(dispatch);
            return true;
        });
    }

    bool setDispatchPhase(const EntityId vehicle, const uint8_t orderIndex, const uint32_t minutes)
    {
        const auto* entry = getEntry(vehicle, orderIndex);
        if (entry != nullptr && entry->dispatch.has_value() && entry->dispatch->phaseMinutes == minutes)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex, minutes](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || it->orderType != OrderType::StopAt)
            {
                return false;
            }
            auto dispatch = it->dispatch.value_or(DispatchPattern{});
            if (minutes >= dispatch.periodMinutes)
            {
                return false;
            }
            dispatch.phaseMinutes = minutes;
            dispatch.lastClaimedMinute.reset();
            it->dispatch = std::move(dispatch);
            return true;
        });
    }

    bool setDispatchMaxDelay(const EntityId vehicle, const uint8_t orderIndex, const uint32_t minutes)
    {
        const auto* entry = getEntry(vehicle, orderIndex);
        if (entry != nullptr && entry->dispatch.has_value() && entry->dispatch->maxDelayMinutes == minutes)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex, minutes](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || it->orderType != OrderType::StopAt || minutes > kMaxPeriodMinutes)
            {
                return false;
            }
            auto dispatch = it->dispatch.value_or(DispatchPattern{});
            dispatch.maxDelayMinutes = minutes;
            it->dispatch = std::move(dispatch);
            return true;
        });
    }

    bool addDispatchSlot(const EntityId vehicle, const uint8_t orderIndex, const uint32_t minute)
    {
        return updateService(vehicle, [orderIndex, minute](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || it->orderType != OrderType::StopAt)
            {
                return false;
            }
            auto dispatch = it->dispatch.value_or(DispatchPattern{});
            if (minute >= dispatch.periodMinutes || dispatch.slots.size() >= kMaxSlots)
            {
                return false;
            }
            const auto slot = std::ranges::lower_bound(dispatch.slots, minute);
            if (slot != dispatch.slots.end() && *slot == minute)
            {
                return false;
            }
            dispatch.slots.insert(slot, minute);
            it->dispatch = std::move(dispatch);
            return true;
        });
    }

    bool removeDispatchSlot(const EntityId vehicle, const uint8_t orderIndex, const uint32_t minute)
    {
        return updateService(vehicle, [orderIndex, minute](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || !it->dispatch.has_value())
            {
                return false;
            }
            if (std::erase(it->dispatch->slots, minute) == 0)
            {
                return false;
            }
            it->dispatch->lastClaimedMinute.reset();
            return true;
        });
    }

    bool clearDispatch(const EntityId vehicle, const uint8_t orderIndex)
    {
        return updateService(vehicle, [orderIndex](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (it == service.entries.end() || !it->dispatch.has_value())
            {
                return false;
            }
            it->dispatch.reset();
            return true;
        });
    }

    bool onOrderInserted(const EntityId vehicle, const uint8_t orderIndex, const OrderType type, const StationId station)
    {
        if (getServiceForVehicle(vehicle) == nullptr)
        {
            return true;
        }
        EntryId entryId = kInvalidEntryId;
        if (isValidOrderType(type))
        {
            entryId = allocateEntryId();
            if (entryId == kInvalidEntryId)
            {
                return false;
            }
        }
        return updateService(vehicle, [=](Service& service) {
            for (auto& entry : service.entries)
            {
                if (entry.orderIndex >= orderIndex)
                {
                    ++entry.orderIndex;
                }
            }
            if (entryId != kInvalidEntryId)
            {
                TimetableEntry entry;
                entry.id = entryId;
                entry.orderIndex = orderIndex;
                entry.orderType = type;
                entry.station = station;
                service.entries.insert(std::ranges::lower_bound(service.entries, orderIndex, {}, &TimetableEntry::orderIndex), std::move(entry));
            }
            return true;
        });
    }

    bool onOrderDeleted(const EntityId vehicle, const uint8_t orderIndex)
    {
        if (getServiceForVehicle(vehicle) == nullptr)
        {
            return true;
        }
        return updateService(vehicle, [orderIndex](Service& service) {
            std::erase_if(service.entries, [orderIndex](const TimetableEntry& entry) { return entry.orderIndex == orderIndex; });
            for (auto& entry : service.entries)
            {
                if (entry.orderIndex > orderIndex)
                {
                    --entry.orderIndex;
                }
            }
            return true;
        });
    }

    bool onOrderReplaced(const EntityId vehicle, const uint8_t orderIndex, const OrderType type, const StationId station)
    {
        if (getServiceForVehicle(vehicle) == nullptr)
        {
            return true;
        }
        auto* existing = getEntry(vehicle, orderIndex);
        EntryId entryId = kInvalidEntryId;
        if (existing == nullptr && isValidOrderType(type))
        {
            entryId = allocateEntryId();
            if (entryId == kInvalidEntryId)
            {
                return false;
            }
        }
        return updateService(vehicle, [=](Service& service) {
            const auto it = std::ranges::find(service.entries, orderIndex, &TimetableEntry::orderIndex);
            if (!isValidOrderType(type))
            {
                if (it != service.entries.end())
                {
                    service.entries.erase(it);
                }
                return true;
            }
            if (it == service.entries.end())
            {
                TimetableEntry entry;
                entry.id = entryId;
                entry.orderIndex = orderIndex;
                entry.orderType = type;
                entry.station = station;
                service.entries.insert(std::ranges::lower_bound(service.entries, orderIndex, {}, &TimetableEntry::orderIndex), std::move(entry));
                return true;
            }
            it->orderType = type;
            it->station = station;
            if (type != OrderType::StopAt)
            {
                it->dwellMinutes.reset();
                it->dispatch.reset();
            }
            return true;
        });
    }

    bool onOrdersSwapped(const EntityId vehicle, const uint8_t firstIndex, const uint8_t secondIndex)
    {
        if (getServiceForVehicle(vehicle) == nullptr)
        {
            return true;
        }
        return updateService(vehicle, [=](Service& service) {
            for (auto& entry : service.entries)
            {
                if (entry.orderIndex == firstIndex)
                {
                    entry.orderIndex = secondIndex;
                }
                else if (entry.orderIndex == secondIndex)
                {
                    entry.orderIndex = firstIndex;
                }
            }
            std::ranges::sort(service.entries, {}, &TimetableEntry::orderIndex);
            return true;
        });
    }

    bool onOrdersReversed(const EntityId vehicle, const uint8_t orderCount)
    {
        const auto* service = getServiceForVehicle(vehicle);
        if (service == nullptr)
        {
            return true;
        }
        if (std::ranges::any_of(service->entries, [orderCount](const TimetableEntry& entry) { return entry.orderIndex >= orderCount; }))
        {
            return false;
        }
        return updateService(vehicle, [orderCount](Service& service) {
            for (auto& entry : service.entries)
            {
                entry.orderIndex = orderCount - entry.orderIndex - 1;
            }
            std::ranges::sort(service.entries, {}, &TimetableEntry::orderIndex);
            return true;
        });
    }

    bool arriveAtOrder(const EntityId vehicle, const uint8_t orderIndex)
    {
        auto* service = getServiceForVehicle(vehicle);
        auto* entry = getEntry(vehicle, orderIndex);
        if (service == nullptr || entry == nullptr)
        {
            return false;
        }

        auto* runtime = getVehicleRuntime(vehicle);
        if (runtime == nullptr || runtime->service != service->id || runtime->serviceRevision != service->revision)
        {
            runtime = resetVehicleRuntime(vehicle);
            if (runtime == nullptr)
            {
                return false;
            }
        }
        if (!runtime->timetableStarted || runtime->currentEntry != entry->id)
        {
            runtime->scheduledArrivalTick = _clockTicks;
        }
        runtime->currentEntry = entry->id;
        runtime->latenessTicks = tickDifference(_clockTicks, runtime->scheduledArrivalTick);
        runtime->timetableStarted = true;
        runtime->scheduledDepartureTick = addMinutes(runtime->scheduledArrivalTick, entry->dwellMinutes.value_or(0));
        runtime->assignedSlotMinute.reset();
        runtime->released = false;
        runtime->waiting = false;
        runtime->atTimedStop = entry->orderType == OrderType::StopAt;
        if (!runtime->atTimedStop)
        {
            scheduleNextEntry(*service, *runtime);
        }
        return true;
    }

    bool isWaitingForDeparture(const EntityId vehicle)
    {
        auto* service = getServiceForVehicle(vehicle);
        auto* runtime = getVehicleRuntime(vehicle);
        if (service == nullptr || runtime == nullptr || runtime->service != service->id
            || runtime->serviceRevision != service->revision || !runtime->atTimedStop || runtime->released)
        {
            return false;
        }

        const auto entry = std::ranges::find(service->entries, runtime->currentEntry, &TimetableEntry::id);
        if (entry == service->entries.end() || entry->orderType != OrderType::StopAt)
        {
            clearVehicleRuntime(vehicle);
            return false;
        }
        if (entry->dispatch.has_value() && !entry->dispatch->slots.empty() && !runtime->assignedSlotMinute.has_value())
        {
            const auto dwellPending = _clockTicks < runtime->scheduledDepartureTick;
            const auto earliestTick = std::max(_clockTicks, runtime->scheduledDepartureTick);
            const auto earliestMinute = static_cast<int64_t>(std::min<uint64_t>(
                earliestTick / _ticksPerMinute + (earliestTick % _ticksPerMinute != 0),
                std::numeric_limits<int64_t>::max()));
            auto claimPattern = *entry->dispatch;
            if (dwellPending)
            {
                claimPattern.maxDelayMinutes = 0;
            }
            const auto claim = findNextSlot(claimPattern, earliestMinute);
            if (!claim.has_value())
            {
                runtime->waiting = true;
                return true;
            }
            entry->dispatch->lastClaimedMinute = claim->scheduledMinute;
            runtime->assignedSlotMinute = claim->scheduledMinute;
            const auto slotTick = minuteToTick(claim->scheduledMinute);
            runtime->scheduledDepartureTick = entry->dwellMinutes.value_or(0) == 0
                ? slotTick
                : std::max(runtime->scheduledDepartureTick, slotTick);
        }

        if (_clockTicks < runtime->scheduledDepartureTick)
        {
            runtime->waiting = true;
            return true;
        }
        runtime->latenessTicks = tickDifference(_clockTicks, runtime->scheduledDepartureTick);
        runtime->released = true;
        runtime->waiting = false;
        return false;
    }

    bool isWaitingAtTimedStop(const EntityId vehicle)
    {
        const auto* runtime = getVehicleRuntime(vehicle);
        return runtime != nullptr && runtime->atTimedStop && runtime->waiting && !runtime->released;
    }

    void departFromOrder(const EntityId vehicle)
    {
        auto* service = getServiceForVehicle(vehicle);
        auto* runtime = getVehicleRuntime(vehicle);
        if (service == nullptr || runtime == nullptr || runtime->service != service->id
            || runtime->serviceRevision != service->revision || !runtime->atTimedStop)
        {
            return;
        }
        runtime->latenessTicks = tickDifference(_clockTicks, runtime->scheduledDepartureTick);
        runtime->waiting = false;
        scheduleNextEntry(*service, *runtime);
    }

    VehicleRuntime* getVehicleRuntime(const EntityId vehicle)
    {
        return isValidVehicleId(vehicle) && _vehicleRuntime[enumValue(vehicle)].has_value() ? &*_vehicleRuntime[enumValue(vehicle)] : nullptr;
    }

    VehicleRuntime* resetVehicleRuntime(const EntityId vehicle)
    {
        if (!isValidVehicleId(vehicle))
        {
            return nullptr;
        }
        auto& result = _vehicleRuntime[enumValue(vehicle)].emplace();
        result.vehicle = vehicle;
        result.service = getServiceId(vehicle);
        if (const auto* service = getService(result.service); service != nullptr)
        {
            result.serviceRevision = service->revision;
        }
        return &result;
    }

    void clearVehicleRuntime(const EntityId vehicle)
    {
        if (isValidVehicleId(vehicle))
        {
            _vehicleRuntime[enumValue(vehicle)].reset();
        }
    }

    EntryId allocateEntryId()
    {
        constexpr auto kMaxEntries = S5::Limits::kMaxVehicles * S5::Limits::kMaxOrdersPerVehicle;
        if (_nextEntryId != kInvalidEntryId && _nextEntryId <= kMaxEntries)
        {
            return _nextEntryId++;
        }

        std::vector<bool> used(kMaxEntries + 1);
        for (const auto& service : _services)
        {
            for (const auto& entry : service.entries)
            {
                used[entry.id] = true;
            }
        }
        const auto available = std::ranges::find(used.begin() + 1, used.end(), false);
        return available == used.end() ? kInvalidEntryId : static_cast<EntryId>(std::distance(used.begin(), available));
    }

    void resetServiceRuntime(const ServiceId serviceId)
    {
        for (const auto vehicle : getAssignedVehicles(serviceId))
        {
            resetVehicleRuntime(vehicle);
        }
    }

    void resetDispatchState(const ServiceId serviceId)
    {
        auto* service = getService(serviceId);
        if (service == nullptr)
        {
            return;
        }
        for (auto& entry : service->entries)
        {
            if (entry.dispatch.has_value())
            {
                entry.dispatch->lastClaimedMinute.reset();
            }
        }
        for (const auto vehicle : getAssignedVehicles(serviceId))
        {
            auto* runtime = getVehicleRuntime(vehicle);
            if (runtime != nullptr && runtime->service == serviceId && runtime->serviceRevision == service->revision && runtime->atTimedStop)
            {
                const auto entry = std::ranges::find(service->entries, runtime->currentEntry, &TimetableEntry::id);
                if (entry == service->entries.end())
                {
                    resetVehicleRuntime(vehicle);
                    continue;
                }
                runtime->scheduledDepartureTick = addMinutes(runtime->scheduledArrivalTick, entry->dwellMinutes.value_or(0));
                runtime->assignedSlotMinute.reset();
                runtime->released = false;
                runtime->waiting = false;
            }
            else
            {
                resetVehicleRuntime(vehicle);
            }
        }
    }

    State captureState()
    {
        State state;
        state.ticksPerMinute = _ticksPerMinute;
        state.clockTicks = _clockTicks;
        state.nextServiceId = _nextServiceId;
        state.nextEntryId = _nextEntryId;
        state.services = _services;
        for (size_t i = 0; i < _assignments.size(); ++i)
        {
            if (_assignments[i] != kInvalidServiceId)
            {
                state.assignments.push_back({ EntityId(i), _assignments[i] });
            }
            if (_vehicleRuntime[i].has_value())
            {
                state.vehicles.push_back(*_vehicleRuntime[i]);
            }
        }
        return state;
    }

    bool validateState(const State& state)
    {
        if (state.ticksPerMinute < kMinTicksPerMinute || state.ticksPerMinute > kMaxTicksPerMinute
            || state.clockTicks / state.ticksPerMinute > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())
            || state.services.size() > S5::Limits::kMaxVehicles || !std::ranges::is_sorted(state.services, {}, &Service::id))
        {
            return false;
        }

        if (state.nextEntryId == kInvalidEntryId || state.nextEntryId > S5::Limits::kMaxVehicles * S5::Limits::kMaxOrdersPerVehicle + 1)
        {
            return false;
        }
        std::vector<bool> seenEntryIds(state.nextEntryId);
        ServiceId maxServiceId{};
        EntryId maxEntryId{};
        for (size_t i = 0; i < state.services.size(); ++i)
        {
            const auto& service = state.services[i];
            if ((i != 0 && state.services[i - 1].id == service.id) || !isCanonicalService(service, seenEntryIds))
            {
                return false;
            }
            maxServiceId = std::max(maxServiceId, service.id);
            for (const auto& entry : service.entries)
            {
                maxEntryId = std::max(maxEntryId, entry.id);
            }
        }
        if (state.nextServiceId <= maxServiceId || state.nextEntryId <= maxEntryId)
        {
            return false;
        }

        std::array<ServiceId, S5::Limits::kMaxEntities> assignments{};
        EntityId previousAssignment = EntityId::null;
        for (const auto& assignment : state.assignments)
        {
            if (!isValidVehicleId(assignment.vehicle) || (previousAssignment != EntityId::null && enumValue(assignment.vehicle) <= enumValue(previousAssignment))
                || std::ranges::none_of(state.services, [service = assignment.service](const Service& item) { return item.id == service; }))
            {
                return false;
            }
            assignments[enumValue(assignment.vehicle)] = assignment.service;
            previousAssignment = assignment.vehicle;
        }
        if (std::ranges::any_of(state.services, [&](const Service& service) {
                return std::ranges::none_of(state.assignments, [id = service.id](const VehicleAssignment& assignment) { return assignment.service == id; });
            }))
        {
            return false;
        }

        EntityId previousRuntime = EntityId::null;
        std::vector<std::tuple<ServiceId, EntryId, int64_t>> claimedSlots;
        for (const auto& runtime : state.vehicles)
        {
            if (!isValidVehicleId(runtime.vehicle) || (previousRuntime != EntityId::null && enumValue(runtime.vehicle) <= enumValue(previousRuntime))
                || assignments[enumValue(runtime.vehicle)] != runtime.service
                || (runtime.waiting && (!runtime.atTimedStop || runtime.released)))
            {
                return false;
            }
            const auto service = std::ranges::find(state.services, runtime.service, &Service::id);
            if (service == state.services.end() || runtime.serviceRevision != service->revision)
            {
                return false;
            }
            const auto entry = std::ranges::find(service->entries, runtime.currentEntry, &TimetableEntry::id);
            if (runtime.currentEntry == kInvalidEntryId)
            {
                if (runtime.timetableStarted || runtime.atTimedStop || runtime.released || runtime.waiting || runtime.assignedSlotMinute.has_value())
                {
                    return false;
                }
            }
            else if (entry == service->entries.end()
                || (runtime.atTimedStop && (entry->orderType != OrderType::StopAt || !runtime.timetableStarted))
                || (!runtime.atTimedStop && (runtime.released || runtime.waiting || runtime.assignedSlotMinute.has_value()))
                || (runtime.released && runtime.waiting))
            {
                return false;
            }
            if (runtime.assignedSlotMinute.has_value())
            {
                const auto& dispatch = entry->dispatch;
                if (!dispatch.has_value() || !dispatch->lastClaimedMinute.has_value()
                    || *runtime.assignedSlotMinute > *dispatch->lastClaimedMinute || *runtime.assignedSlotMinute < 0)
                {
                    return false;
                }
                const auto residue = static_cast<uint32_t>(*runtime.assignedSlotMinute % dispatch->periodMinutes);
                if (std::ranges::none_of(dispatch->slots, [&](const uint32_t slot) {
                        return (dispatch->phaseMinutes + slot) % dispatch->periodMinutes == residue;
                    }))
                {
                    return false;
                }
                const auto claim = std::tuple{ runtime.service, runtime.currentEntry, *runtime.assignedSlotMinute };
                if (std::ranges::find(claimedSlots, claim) != claimedSlots.end())
                {
                    return false;
                }
                claimedSlots.push_back(claim);
            }
            previousRuntime = runtime.vehicle;
        }
        return true;
    }

    bool validateState(const State& state, const GameState& gameState, const SharedOrderManager::State& sharedOrders)
    {
        if (!validateState(state) || !SharedOrderManager::validateState(sharedOrders, gameState))
        {
            return false;
        }

        std::array<ServiceId, S5::Limits::kMaxEntities> assignments{};
        std::array<EntityId, S5::Limits::kMaxEntities> groups;
        groups.fill(EntityId::null);
        for (const auto& group : sharedOrders.groups)
        {
            for (const auto member : group.members)
            {
                groups[enumValue(member)] = group.members.front();
            }
        }

        std::vector<std::pair<ServiceId, EntityId>> serviceOwners;
        for (const auto& assignment : state.assignments)
        {
            const auto* head = getHead(gameState, assignment.vehicle);
            const auto service = std::ranges::find(state.services, assignment.service, &Service::id);
            if (head == nullptr || service == state.services.end() || !entriesMatchOrders(*service, *head, gameState))
            {
                return false;
            }
            assignments[enumValue(assignment.vehicle)] = assignment.service;

            const auto owner = std::ranges::find(serviceOwners, assignment.service, &std::pair<ServiceId, EntityId>::first);
            if (owner == serviceOwners.end())
            {
                serviceOwners.emplace_back(assignment.service, assignment.vehicle);
            }
            else if (groups[enumValue(owner->second)] == EntityId::null || groups[enumValue(owner->second)] != groups[enumValue(assignment.vehicle)])
            {
                return false;
            }
        }

        for (const auto& group : sharedOrders.groups)
        {
            const auto service = assignments[enumValue(group.members.front())];
            if (std::ranges::any_of(group.members, [&](const EntityId member) { return assignments[enumValue(member)] != service; }))
            {
                return false;
            }
        }
        return true;
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state))
        {
            return false;
        }
        reset(state.clockTicks);
        _ticksPerMinute = state.ticksPerMinute;
        _nextServiceId = state.nextServiceId;
        _nextEntryId = state.nextEntryId;
        _services = state.services;
        for (const auto& assignment : state.assignments)
        {
            _assignments[enumValue(assignment.vehicle)] = assignment.service;
        }
        for (const auto& runtime : state.vehicles)
        {
            _vehicleRuntime[enumValue(runtime.vehicle)] = runtime;
        }
        return true;
    }
}
