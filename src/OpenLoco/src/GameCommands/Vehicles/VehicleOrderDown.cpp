#include "GameCommands/Vehicles/VehicleOrderDown.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/VehicleHead.h"

namespace OpenLoco::GameCommands
{
    // 0x00470E06
    static uint32_t vehicleOrderDown(const VehicleOrderDownArgs& args, uint8_t flags)
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
        const auto table = Vehicles::OrderManager::copyOrderTable(*head);
        const auto currentOrderSize = Vehicles::OrderManager::getOrderSize(static_cast<Vehicles::OrderType>(table[args.orderOffset] & 0x7));
        const auto nextOffset = static_cast<uint16_t>(args.orderOffset + currentOrderSize);
        if (nextOffset == table.size() - sizeof(Vehicles::OrderEnd))
        {
            return 0;
        }

        for (const auto id : members)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            auto* memberOrders = Vehicles::OrderManager::orders() + member->orderTableOffset;
            auto& currentOrder = memberOrders[args.orderOffset];
            auto& nextOrder = memberOrders[nextOffset];
            const bool currentIsActive = member->currentOrder == args.orderOffset;
            const bool nextIsActive = member->currentOrder == nextOffset;
            const auto newOffsetDiff = Vehicles::OrderManager::swapAdjacentOrders(currentOrder, nextOrder);
            member->resetUnbunching();

            if (currentIsActive)
            {
                member->currentOrder += newOffsetDiff;
            }
            else if (nextIsActive)
            {
                member->currentOrder -= currentOrderSize;
            }
        }

        return 0;
    }

    void vehicleOrderDown(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderDown(VehicleOrderDownArgs(regs), flags);
    }
}
