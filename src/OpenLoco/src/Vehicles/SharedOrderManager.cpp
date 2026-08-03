#include "Vehicles/SharedOrderManager.h"
#include "Entities/EntityManager.h"
#include "GameState.h"
#include "S5/Limits.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <ranges>

namespace OpenLoco::Vehicles::SharedOrderManager
{
    static constexpr auto getIdValue = [](const EntityId id) { return enumValue(id); };

    static std::array<EntityId, Limits::kMaxEntities> _groups = [] {
        std::array<EntityId, Limits::kMaxEntities> result;
        result.fill(EntityId::null);
        return result;
    }();

    static bool isValidId(const EntityId id)
    {
        return id != EntityId::null && enumValue(id) < _groups.size();
    }

    static VehicleHead* getHead(const EntityId id)
    {
        auto* vehicle = isValidId(id) ? EntityManager::get<VehicleBase>(id) : nullptr;
        return vehicle != nullptr && vehicle->isVehicleHead() ? vehicle->asVehicleHead() : nullptr;
    }

    static const VehicleHead* getHead(const GameState& gameState, const EntityId id)
    {
        if (!isValidId(id))
        {
            return nullptr;
        }
        const auto& entity = gameState.entities[enumValue(id)];
        const auto* vehicle = entity.asBase<VehicleBase>();
        if (vehicle == nullptr || !vehicle->isVehicleHead() || vehicle->id != id)
        {
            return nullptr;
        }
        return reinterpret_cast<const VehicleHead*>(vehicle);
    }

    static bool hasValidOrderTable(const VehicleHead& head)
    {
        if (head.sizeOfOrderTable < sizeof(OrderEnd)
            || !OrderManager::isOrderOffsetValid(head, head.sizeOfOrderTable - sizeof(OrderEnd), true))
        {
            return false;
        }
        return head.sizeOfOrderTable == sizeof(OrderEnd)
            ? head.currentOrder == 0
            : OrderManager::isOrderOffsetValid(head, head.currentOrder);
    }

    static bool hasValidOrderTable(const GameState& gameState, const VehicleHead& head)
    {
        const auto tableOffset = head.orderTableOffset;
        const auto tableSize = static_cast<uint32_t>(head.sizeOfOrderTable);
        if (tableSize < sizeof(OrderEnd) || gameState.orderTableLength > Limits::kMaxOrders
            || tableOffset > gameState.orderTableLength || tableSize > gameState.orderTableLength - tableOffset)
        {
            return false;
        }

        bool currentOrderIsValid = tableSize == sizeof(OrderEnd) && head.currentOrder == 0;
        size_t orderCount = 0;
        for (uint32_t offset = 0; offset < tableSize;)
        {
            const auto type = static_cast<OrderType>(gameState.orders[tableOffset + offset] & 0x7);
            const auto orderSize = OrderManager::getOrderSize(type);
            if (orderSize == 0 || orderSize > tableSize - offset)
            {
                return false;
            }
            if (type == OrderType::End)
            {
                return offset + orderSize == tableSize && currentOrderIsValid;
            }
            if (++orderCount > Limits::kMaxOrdersPerVehicle)
            {
                return false;
            }
            currentOrderIsValid |= head.currentOrder == offset;
            offset += orderSize;
        }
        return false;
    }

    static bool areOrdersEqual(const GameState& gameState, const VehicleHead& lhs, const VehicleHead& rhs)
    {
        return lhs.sizeOfOrderTable == rhs.sizeOfOrderTable
            && std::memcmp(&gameState.orders[lhs.orderTableOffset], &gameState.orders[rhs.orderTableOffset], lhs.sizeOfOrderTable) == 0;
    }

    template<typename GetOrderByte>
    static bool areVehiclesCompatible(const VehicleHead& target, const VehicleHead& source, GetOrderByte&& getOrderByte)
    {
        if (target.owner != source.owner || target.vehicleType != source.vehicleType || target.mode != source.mode)
        {
            return false;
        }
        if ((target.mode == TransportMode::rail || target.mode == TransportMode::road) && target.trackType != source.trackType)
        {
            return false;
        }

        for (uint32_t offset = 0; offset < source.sizeOfOrderTable;)
        {
            const auto type = static_cast<OrderType>(getOrderByte(offset) & 0x7);
            if (type == OrderType::End)
            {
                break;
            }
            if (type == OrderType::UnloadAll || type == OrderType::WaitFor)
            {
                const auto cargo = getOrderByte(offset) >> 3;
                if ((target.trainAcceptedCargoTypes & (1U << cargo)) == 0)
                {
                    return false;
                }
            }
            offset += OrderManager::getOrderSize(type);
        }
        return true;
    }

    static void setGroup(const std::vector<EntityId>& members)
    {
        if (members.size() < 2)
        {
            for (const auto id : members)
            {
                _groups[enumValue(id)] = EntityId::null;
            }
            return;
        }

        const auto leader = *std::ranges::min_element(members, {}, getIdValue);
        for (const auto id : members)
        {
            _groups[enumValue(id)] = leader;
        }
    }

    void reset()
    {
        _groups.fill(EntityId::null);
    }

    EntityId getGroupId(const EntityId vehicle)
    {
        return isValidId(vehicle) ? _groups[enumValue(vehicle)] : EntityId::null;
    }

    bool isShared(const EntityId vehicle)
    {
        return getGroupId(vehicle) != EntityId::null;
    }

    std::vector<EntityId> getMembers(const EntityId vehicle)
    {
        std::vector<EntityId> result;
        const auto group = getGroupId(vehicle);
        if (group == EntityId::null)
        {
            if (getHead(vehicle) != nullptr)
            {
                result.push_back(vehicle);
            }
            return result;
        }

        for (auto* head : VehicleManager::VehicleList())
        {
            if (getGroupId(head->id) == group)
            {
                result.push_back(head->id);
            }
        }
        std::ranges::sort(result, {}, getIdValue);
        return result;
    }

    size_t getMemberCount(const EntityId vehicle)
    {
        const auto group = getGroupId(vehicle);
        if (group == EntityId::null)
        {
            return getHead(vehicle) != nullptr ? 1 : 0;
        }
        size_t count = 0;
        for (const auto* head : VehicleManager::VehicleList())
        {
            count += getGroupId(head->id) == group;
        }
        return count;
    }

    bool areOrdersEqual(const VehicleHead& lhs, const VehicleHead& rhs)
    {
        if (!hasValidOrderTable(lhs) || !hasValidOrderTable(rhs) || lhs.sizeOfOrderTable != rhs.sizeOfOrderTable)
        {
            return false;
        }
        const auto* lhsOrders = reinterpret_cast<const uint8_t*>(OrderManager::orders() + lhs.orderTableOffset);
        const auto* rhsOrders = reinterpret_cast<const uint8_t*>(OrderManager::orders() + rhs.orderTableOffset);
        return std::memcmp(lhsOrders, rhsOrders, lhs.sizeOfOrderTable) == 0;
    }

    bool areVehiclesCompatible(const VehicleHead& target, const VehicleHead& source)
    {
        return hasValidOrderTable(source)
            && areVehiclesCompatible(target, source, [&source](const uint32_t offset) {
                   return reinterpret_cast<const uint8_t*>(OrderManager::orders() + source.orderTableOffset)[offset];
               });
    }

    bool join(const EntityId target, const EntityId source)
    {
        if (target == source || getHead(target) == nullptr || getHead(source) == nullptr)
        {
            return false;
        }
        if (isShared(target))
        {
            return getGroupId(target) == getGroupId(source);
        }

        auto members = getMembers(source);
        members.push_back(target);
        std::ranges::sort(members, {}, getIdValue);
        members.erase(std::ranges::unique(members).begin(), members.end());
        setGroup(members);
        return true;
    }

    bool leave(const EntityId vehicle)
    {
        if (!isShared(vehicle))
        {
            return false;
        }

        auto members = getMembers(vehicle);
        _groups[enumValue(vehicle)] = EntityId::null;
        std::erase(members, vehicle);
        setGroup(members);
        return true;
    }

    void remove(const EntityId vehicle)
    {
        const auto members = getMembers(vehicle);
        if (leave(vehicle))
        {
            for (const auto member : members)
            {
                Ui::WindowManager::invalidateOrderPageByVehicleNumber(enumValue(member));
            }
            Ui::WindowManager::invalidate(Ui::WindowType::vehicleList);
        }
        if (isValidId(vehicle))
        {
            _groups[enumValue(vehicle)] = EntityId::null;
        }
    }

    bool detachIfIncompatible(const EntityId vehicle)
    {
        if (!isShared(vehicle))
        {
            return false;
        }

        const auto members = getMembers(vehicle);
        auto* head = getHead(vehicle);
        const auto sourceId = std::ranges::find_if(members, [vehicle](const EntityId member) { return member != vehicle; });
        auto* source = sourceId != members.end() ? getHead(*sourceId) : nullptr;
        if (head != nullptr && source != nullptr && areVehiclesCompatible(*head, *source))
        {
            return false;
        }

        leave(vehicle);
        for (const auto member : members)
        {
            Ui::WindowManager::invalidateOrderPageByVehicleNumber(enumValue(member));
        }
        Ui::WindowManager::invalidate(Ui::WindowType::vehicleList);
        return true;
    }

    bool joinAllMatching(const EntityId source)
    {
        auto* sourceHead = getHead(source);
        if (sourceHead == nullptr || sourceHead->sizeOfOrderTable <= sizeof(OrderEnd)
            || !areVehiclesCompatible(*sourceHead, *sourceHead))
        {
            return false;
        }

        std::vector<EntityId> matching;
        for (auto* head : VehicleManager::VehicleList())
        {
            if (areVehiclesCompatible(*head, *sourceHead) && areOrdersEqual(*head, *sourceHead))
            {
                auto members = getMembers(head->id);
                if (!std::ranges::all_of(members, [sourceHead](const EntityId member) {
                        const auto* memberHead = getHead(member);
                        return memberHead != nullptr && areVehiclesCompatible(*memberHead, *sourceHead)
                            && areOrdersEqual(*memberHead, *sourceHead);
                    }))
                {
                    return false;
                }
                matching.insert(matching.end(), members.begin(), members.end());
            }
        }

        std::ranges::sort(matching, {}, getIdValue);
        matching.erase(std::ranges::unique(matching).begin(), matching.end());
        if (matching.size() < 2)
        {
            return false;
        }
        const auto currentGroup = getGroupId(matching.front());
        if (currentGroup != EntityId::null && std::ranges::all_of(matching, [currentGroup](const EntityId id) {
                return getGroupId(id) == currentGroup;
            }))
        {
            return true;
        }
        setGroup(matching);
        return true;
    }

    State captureState()
    {
        State state;
        std::vector<EntityId> seenGroups;
        for (auto* head : VehicleManager::VehicleList())
        {
            const auto group = getGroupId(head->id);
            if (group == EntityId::null || std::ranges::find(seenGroups, group) != seenGroups.end())
            {
                continue;
            }
            seenGroups.push_back(group);
            auto members = getMembers(head->id);
            if (members.size() >= 2)
            {
                state.groups.push_back({ std::move(members) });
            }
        }
        std::ranges::sort(state.groups, {}, [](const Group& group) { return enumValue(group.members.front()); });
        return state;
    }

    bool validateState(const State& state)
    {
        return validateState(state, getGameState());
    }

    bool validateState(const State& state, const GameState& gameState)
    {
        std::array<bool, Limits::kMaxEntities> seen{};
        for (const auto& group : state.groups)
        {
            if (group.members.size() < 2 || !std::ranges::is_sorted(group.members, {}, getIdValue))
            {
                return false;
            }
            const auto* source = getHead(gameState, group.members.front());
            if (source == nullptr || !hasValidOrderTable(gameState, *source))
            {
                return false;
            }
            for (const auto member : group.members)
            {
                if (!isValidId(member) || seen[enumValue(member)])
                {
                    return false;
                }
                const auto* head = getHead(gameState, member);
                if (head == nullptr || !hasValidOrderTable(gameState, *head)
                    || !areVehiclesCompatible(*head, *source, [&gameState, source](const uint32_t offset) {
                           return gameState.orders[source->orderTableOffset + offset];
                       })
                    || !areOrdersEqual(gameState, *head, *source))
                {
                    return false;
                }
                seen[enumValue(member)] = true;
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
        reset();
        for (const auto& group : state.groups)
        {
            setGroup(group.members);
        }
        return true;
    }
}
