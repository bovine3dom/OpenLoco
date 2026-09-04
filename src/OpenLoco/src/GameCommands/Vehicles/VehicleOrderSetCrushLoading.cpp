#include "GameCommands/Vehicles/VehicleOrderSetCrushLoading.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include <OpenLoco/CargoDist/CargoDist.h>
#include <algorithm>
#include <ranges>
#include <vector>

namespace OpenLoco::GameCommands
{
    static uint32_t vehicleOrderSetCrushLoading(const VehicleOrderSetCrushLoadingArgs& args, const uint8_t flags)
    {
        auto* head = VehicleOrderCommon::getHead(args.head);
        if (head == nullptr || args.enabled > 1 || !Vehicles::OrderManager::isOrderOffsetValid(*head, args.orderOffset))
        {
            return kFailure;
        }

        std::vector<EntityId> affected{ head->id };
        for (size_t i = 0; i < affected.size(); ++i)
        {
            auto* current = VehicleOrderCommon::getHead(affected[i]);
            if (current == nullptr)
            {
                return kFailure;
            }
            for (auto* candidate : VehicleManager::VehicleList())
            {
                if (Vehicles::OrderManager::areVehiclesOnSameRoute(*current, *candidate)
                    && std::ranges::find(affected, candidate->id) == affected.end())
                {
                    affected.push_back(candidate->id);
                }
            }
            for (const auto member : Vehicles::SharedOrderManager::getMembers(current->id))
            {
                if (std::ranges::find(affected, member) == affected.end())
                {
                    affected.push_back(member);
                }
            }
        }
        std::ranges::sort(affected, {}, [](const EntityId id) { return enumValue(id); });

        setPosition(head->position);
        for (const auto id : affected)
        {
            const auto* member = VehicleOrderCommon::getHead(id);
            if (member == nullptr || !checkCompanyCompatibility(member->owner)
                || !Vehicles::SharedOrderManager::areOrdersEqual(*member, *head)
                || !Vehicles::OrderManager::isOrderOffsetValid(*member, args.orderOffset))
            {
                return kFailure;
            }
        }

        auto* selectedOrder = Vehicles::OrderRingView(head->orderTableOffset, args.orderOffset).begin()->as<Vehicles::OrderStopAt>();
        if (selectedOrder == nullptr)
        {
            return kFailure;
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        VehicleOrderCommon::invalidateOrderWindows(affected);
        for (const auto id : affected)
        {
            auto* member = VehicleOrderCommon::getHead(id);
            auto* memberOrder = Vehicles::OrderRingView(member->orderTableOffset, args.orderOffset).begin()->as<Vehicles::OrderStopAt>();
            memberOrder->setCrushLoading(args.enabled != 0);
        }
        CargoDist::markServicesDirty();
        return 0;
    }

    void vehicleOrderSetCrushLoading(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderSetCrushLoading(VehicleOrderSetCrushLoadingArgs(regs), flags);
    }
}
