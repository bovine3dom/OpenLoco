#include "GameCommands/Vehicles/VehicleOrderDelete.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleHead.h"

namespace OpenLoco::GameCommands
{
    // 0x0047057A
    static uint32_t vehicleOrderDelete(const VehicleOrderDeleteArgs& args, uint8_t flags)
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

        if (!Vehicles::TimetableManager::onOrderDeleted(args.head, VehicleOrderCommon::getOrderIndex(*head, args.orderOffset)))
        {
            return kFailure;
        }
        VehicleOrderCommon::invalidateOrderWindows(members);
        VehicleOrderCommon::sortByDescendingOrderTableOffset(members);
        for (const auto id : members)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            Vehicles::OrderManager::deleteOrder(member, static_cast<uint16_t>(args.orderOffset));
        }

        return 0;
    }

    void vehicleOrderDelete(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderDelete(VehicleOrderDeleteArgs(regs), flags);
    }
}
