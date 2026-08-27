#include "GameCommands/Vehicles/VehicleOrderReverse.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleHead.h"

namespace OpenLoco::GameCommands
{
    static uint32_t vehicleOrderReverse(const VehicleOrderReverseArgs& args, uint8_t flags)
    {
        auto* head = VehicleOrderCommon::getHead(args.head);
        if (head == nullptr)
        {
            return kFailure;
        }

        setPosition(head->position);
        const auto members = Vehicles::SharedOrderManager::getMembers(args.head);
        if (!VehicleOrderCommon::hasConsistentOrderTables(*head, members))
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        for (const auto id : members)
        {
            const auto* member = VehicleOrderCommon::getHead(id);
            if (!checkCompanyCompatibility(member->owner))
            {
                return kFailure;
            }
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        if (!Vehicles::TimetableManager::onOrdersReversed(args.head, VehicleOrderCommon::getOrderCount(*head)))
        {
            return kFailure;
        }
        VehicleOrderCommon::invalidateOrderWindows(members);
        for (const auto id : members)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            member->currentOrder = Vehicles::OrderManager::reverseVehicleOrderTable(member->orderTableOffset, member->currentOrder);
            member->resetUnbunching();
        }

        return 0;
    }

    void vehicleOrderReverse(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderReverse(VehicleOrderReverseArgs(regs), flags);
    }
}
