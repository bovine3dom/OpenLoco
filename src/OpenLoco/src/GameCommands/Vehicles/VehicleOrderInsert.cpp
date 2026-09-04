#include "GameCommands/Vehicles/VehicleOrderInsert.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleHead.h"
#include "World/StationManager.h"
#include <OpenLoco/CargoDist/CargoDist.h>
#include <cstring>
#include <span>

using namespace OpenLoco::Vehicles;

namespace OpenLoco::GameCommands
{
    static bool isOrderLegalForVehicle(const Order& order, const VehicleHead& head)
    {
        if (!checkCompanyCompatibility(head.owner))
        {
            return false;
        }

        if (const auto* stationOrder = order.as<OrderStation>(); stationOrder != nullptr)
        {
            const auto* station = StationManager::get(stationOrder->getStation());
            if (station == nullptr)
            {
                setErrorText(StringIds::empty);
                return false;
            }
            if (station->owner != head.owner)
            {
                setErrorText(StringIds::stationOwnedByAnotherCompany);
                return false;
            }
        }

        if (order.is<OrderRouteThrough>() || order.is<OrderRouteWaypoint>())
        {
            if (head.mode == TransportMode::water && order.is<OrderRouteThrough>())
            {
                setErrorText(StringIds::orderTypeNotValidForShips);
                return false;
            }
            if (head.mode == TransportMode::air)
            {
                setErrorText(StringIds::orderTypeNotValidForAircraft);
                return false;
            }
        }

        const OrderCargo* cargoOrder = order.as<OrderUnloadAll>();
        if (cargoOrder == nullptr)
        {
            cargoOrder = order.as<OrderWaitFor>();
        }
        if (cargoOrder != nullptr && (head.trainAcceptedCargoTypes & (1U << cargoOrder->getCargo())) == 0)
        {
            setErrorText(StringIds::empty);
            return false;
        }

        if (const auto* stopOrder = order.as<OrderStopAt>(); stopOrder != nullptr && stopOrder->isUnbunching())
        {
            if (TimetableManager::getServiceId(head.id) != TimetableManager::kInvalidServiceId)
            {
                setErrorText(StringIds::timetable_unbunching_incompatible);
                return false;
            }
            if (head.hasUnbunchingOrder())
            {
                setErrorText(StringIds::only_one_unbunching_stop_allowed);
                return false;
            }
            for (const auto& existingOrder : head.getCurrentOrders())
            {
                if (existingOrder.is<OrderWaitFor>())
                {
                    setErrorText(StringIds::unbunching_incompatible_with_full_load);
                    return false;
                }
            }
        }
        else if (order.is<OrderWaitFor>() && head.hasUnbunchingOrder())
        {
            setErrorText(StringIds::unbunching_incompatible_with_full_load);
            return false;
        }
        return true;
    }

    static uint16_t getPreviousOrderOffset(const std::span<const uint8_t> orders, const uint16_t targetOffset)
    {
        uint16_t offset = 0;
        uint16_t previousOffset = 0;
        while (offset < targetOffset)
        {
            previousOffset = offset;
            offset += OrderManager::getOrderSize(static_cast<OrderType>(orders[offset] & 0x7));
        }
        return previousOffset;
    }

    // 0x0047036E
    static uint32_t vehicleOrderInsert(const VehicleOrderInsertArgs& args, uint8_t flags)
    {
        auto* head = VehicleOrderCommon::getHead(args.head);
        if (head == nullptr)
        {
            return kFailure;
        }

        setPosition(head->position);
        const auto rawOrder = args.rawOrder;
        const auto* order = reinterpret_cast<const Order*>(&rawOrder);
        const auto orderSize = OrderManager::getOrderSize(order->getType());
        if (order->getType() == OrderType::End || orderSize == 0)
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }

        auto members = SharedOrderManager::getMembers(args.head);
        if (!VehicleOrderCommon::hasConsistentOrderTables(*head, members)
            || !VehicleOrderCommon::isOrderOffsetValidForMembers(members, args.orderOffset, true))
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        for (const auto id : members)
        {
            const auto* member = VehicleOrderCommon::getHead(id);
            if (!isOrderLegalForVehicle(*order, *member))
            {
                return kFailure;
            }
        }

        const auto table = OrderManager::copyOrderTable(*head);
        const auto previousOffset = getPreviousOrderOffset(table, static_cast<uint16_t>(args.orderOffset));
        uint64_t previousRawOrder = 0;
        const auto previousOrderSize = OrderManager::getOrderSize(static_cast<OrderType>(table[previousOffset] & 0x7));
        std::memcpy(&previousRawOrder, table.data() + previousOffset, previousOrderSize);
        const auto* previousOrder = reinterpret_cast<const Order*>(&previousRawOrder);
        const auto* previousStop = previousOrder->as<OrderStopAt>();
        const auto* newStop = order->as<OrderStopAt>();
        const bool convertDuplicateStop = previousStop != nullptr && newStop != nullptr
            && previousStop->getStation() == newStop->getStation()
            && head->mode != TransportMode::water && head->mode != TransportMode::air;

        if (!convertDuplicateStop && !OrderManager::spaceLeftInGlobalOrderTable(static_cast<size_t>(orderSize) * members.size()))
        {
            setErrorText(StringIds::no_space_for_more_vehicle_orders);
            return kFailure;
        }
        if (!convertDuplicateStop)
        {
            for (const auto id : members)
            {
                if (!OrderManager::spaceLeftInVehicleOrderTable(VehicleOrderCommon::getHead(id)))
                {
                    setErrorText(StringIds::tooManyOrdersForThisVehicle);
                    return kFailure;
                }
            }
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        const auto orderIndex = VehicleOrderCommon::getOrderIndex(*head, args.orderOffset);
        const auto* stationOrder = order->as<OrderStation>();
        const auto station = stationOrder != nullptr ? stationOrder->getStation() : StationId::null;
        const auto timetableUpdated = convertDuplicateStop
            ? TimetableManager::onOrderReplaced(args.head, VehicleOrderCommon::getOrderIndex(*head, previousOffset), OrderType::RouteThrough, station)
            : TimetableManager::onOrderInserted(args.head, orderIndex, order->getType(), station);
        if (!timetableUpdated)
        {
            return kFailure;
        }

        VehicleOrderCommon::invalidateOrderWindows(members);
        if (convertDuplicateStop)
        {
            for (const auto id : members)
            {
                auto* member = VehicleOrderCommon::getHead(id);
                auto& existingOrder = OrderManager::orders()[member->orderTableOffset + previousOffset];
                existingOrder.as<OrderStopAt>()->setUnbunching(false);
                existingOrder.as<OrderStopAt>()->setCrushLoading(false);
                existingOrder.setType(OrderType::RouteThrough);
                member->resetUnbunching();
            }
            CargoDist::markServicesDirty();
            return 0;
        }

        VehicleOrderCommon::sortByDescendingOrderTableOffset(members);
        for (const auto id : members)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            OrderManager::insertOrder(member, static_cast<uint16_t>(args.orderOffset), order);
        }

        return 0;
    }

    void vehicleOrderInsert(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderInsert(VehicleOrderInsertArgs(regs), flags);
    }
}
