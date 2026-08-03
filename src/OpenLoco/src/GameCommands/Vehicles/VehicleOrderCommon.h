#pragma once

#include "Entities/EntityManager.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/VehicleHead.h"
#include <algorithm>
#include <ranges>
#include <vector>

namespace OpenLoco::GameCommands::VehicleOrderCommon
{
    inline Vehicles::VehicleHead* getHead(const EntityId id)
    {
        auto* vehicle = EntityManager::get<Vehicles::VehicleBase>(id);
        return vehicle != nullptr && vehicle->isVehicleHead() ? vehicle->asVehicleHead() : nullptr;
    }

    inline bool hasValidOrderTable(const Vehicles::VehicleHead& head)
    {
        if (head.sizeOfOrderTable < sizeof(Vehicles::OrderEnd))
        {
            return false;
        }
        return Vehicles::OrderManager::isOrderOffsetValid(head, head.sizeOfOrderTable - sizeof(Vehicles::OrderEnd), true);
    }

    inline bool hasValidCurrentOrder(const Vehicles::VehicleHead& head)
    {
        return head.sizeOfOrderTable == sizeof(Vehicles::OrderEnd)
            ? head.currentOrder == 0
            : Vehicles::OrderManager::isOrderOffsetValid(head, head.currentOrder, false);
    }

    inline bool hasConsistentOrderTables(const Vehicles::VehicleHead& reference, const std::vector<EntityId>& members)
    {
        if (members.empty() || std::ranges::find(members, reference.id) == members.end() || !hasValidOrderTable(reference))
        {
            return false;
        }

        for (const auto id : members)
        {
            const auto* member = getHead(id);
            if (member == nullptr || !hasValidOrderTable(*member) || !hasValidCurrentOrder(*member)
                || !Vehicles::SharedOrderManager::areOrdersEqual(reference, *member))
            {
                return false;
            }
        }
        return true;
    }

    inline bool isOrderOffsetValidForMembers(const std::vector<EntityId>& members, const uint32_t orderOffset, const bool allowEnd)
    {
        for (const auto id : members)
        {
            const auto* member = getHead(id);
            if (member == nullptr || !Vehicles::OrderManager::isOrderOffsetValid(*member, orderOffset, allowEnd))
            {
                return false;
            }
        }
        return true;
    }

    inline void sortByDescendingOrderTableOffset(std::vector<EntityId>& members)
    {
        std::ranges::sort(members, [](const EntityId lhs, const EntityId rhs) {
            const auto* lhsHead = getHead(lhs);
            const auto* rhsHead = getHead(rhs);
            if (lhsHead == nullptr || rhsHead == nullptr)
            {
                return enumValue(lhs) < enumValue(rhs);
            }
            if (lhsHead->orderTableOffset != rhsHead->orderTableOffset)
            {
                return lhsHead->orderTableOffset > rhsHead->orderTableOffset;
            }
            return enumValue(lhs) < enumValue(rhs);
        });
    }

    inline void invalidateOrderWindows(const std::vector<EntityId>& members)
    {
        Vehicles::OrderManager::clearNumDisplayFrames();
        for (const auto id : members)
        {
            Ui::WindowManager::invalidateOrderPageByVehicleNumber(enumValue(id));
        }
        Ui::WindowManager::invalidate(Ui::WindowType::vehicleList);
    }
}
