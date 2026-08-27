#include "GameCommands/Vehicles/VehicleOrderUp.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleHead.h"

namespace OpenLoco::GameCommands
{
    // 0x00470CD2
    static uint32_t vehicleOrderUp(const VehicleOrderUpArgs& args, uint8_t flags)
    {
        auto* head = VehicleOrderCommon::getHead(args.head);
        if (head == nullptr)
        {
            return kFailure;
        }

        setPosition(head->position);
        auto members = Vehicles::SharedOrderManager::getMembers(args.head);
        if (!VehicleOrderCommon::hasConsistentOrderTables(*head, members)
            || !VehicleOrderCommon::isOrderOffsetValidForMembers(members, args.orderOffset, false))
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        for (const auto id : members)
        {
            if (!checkCompanyCompatibility(VehicleOrderCommon::getHead(id)->owner))
            {
                return kFailure;
            }
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        VehicleOrderCommon::invalidateOrderWindows(members);
        if (args.orderOffset == 0)
        {
            return 0;
        }

        const auto table = Vehicles::OrderManager::copyOrderTable(*head);
        uint16_t offset = 0;
        uint16_t previousOffset = 0;
        while (offset < args.orderOffset)
        {
            previousOffset = offset;
            offset += Vehicles::OrderManager::getOrderSize(static_cast<Vehicles::OrderType>(table[offset] & 0x7));
        }
        const auto orderIndex = VehicleOrderCommon::getOrderIndex(*head, args.orderOffset);
        if (!Vehicles::TimetableManager::onOrdersSwapped(args.head, orderIndex - 1, orderIndex))
        {
            return kFailure;
        }

        for (const auto id : members)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            auto* memberOrders = Vehicles::OrderManager::orders() + member->orderTableOffset;
            auto& previousOrder = memberOrders[previousOffset];
            auto& currentOrder = memberOrders[args.orderOffset];
            const bool previousIsActive = member->currentOrder == previousOffset;
            const bool currentIsActive = member->currentOrder == args.orderOffset;
            const auto oldOffsetDiff = static_cast<uint16_t>(args.orderOffset - previousOffset);
            const auto newOffsetDiff = Vehicles::OrderManager::swapAdjacentOrders(previousOrder, currentOrder);
            member->resetUnbunching();

            if (previousIsActive)
            {
                member->currentOrder += newOffsetDiff;
            }
            else if (currentIsActive)
            {
                member->currentOrder -= oldOffsetDiff;
            }
        }

        return 0;
    }

    void vehicleOrderUp(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderUp(VehicleOrderUpArgs(regs), flags);
    }
}
