#include "GameCommands/Vehicles/VehicleOrderShare.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleManager.h"
#include <algorithm>
#include <ranges>
#include <vector>

namespace OpenLoco::GameCommands
{
    static uint32_t failShareCommand()
    {
        setErrorText(StringIds::empty);
        return kFailure;
    }

    static void sortAndUnique(std::vector<EntityId>& ids)
    {
        std::ranges::sort(ids, {}, [](const EntityId id) { return enumValue(id); });
        ids.erase(std::ranges::unique(ids).begin(), ids.end());
    }

    static bool validateSourceGroup(const Vehicles::VehicleHead& source, const std::vector<EntityId>& members)
    {
        if (!VehicleOrderCommon::hasConsistentOrderTables(source, members))
        {
            return false;
        }

        for (const auto id : members)
        {
            const auto* member = VehicleOrderCommon::getHead(id);
            if (!Vehicles::SharedOrderManager::areVehiclesCompatible(*member, source))
            {
                return false;
            }
        }
        return true;
    }

    static uint32_t joinSource(const VehicleOrderShareArgs& args, const uint8_t flags)
    {
        auto* target = VehicleOrderCommon::getHead(args.target);
        auto* source = VehicleOrderCommon::getHead(args.source);
        if (target == nullptr || source == nullptr || args.target == args.source || Vehicles::SharedOrderManager::isShared(args.target))
        {
            return failShareCommand();
        }

        setPosition(target->position);
        if (!checkCompanyCompatibility(target->owner) || !checkCompanyCompatibility(source->owner))
        {
            return kFailure;
        }
        auto affected = Vehicles::SharedOrderManager::getMembers(args.source);
        if (!VehicleOrderCommon::hasValidOrderTable(*target) || !VehicleOrderCommon::hasValidCurrentOrder(*target)
            || !validateSourceGroup(*source, affected)
            || !Vehicles::SharedOrderManager::areVehiclesCompatible(*target, *source))
        {
            return failShareCommand();
        }

        const auto sourceOrders = Vehicles::OrderManager::copyOrderTable(*source);
        const bool ordersEqual = Vehicles::SharedOrderManager::areOrdersEqual(*target, *source);
        if (sourceOrders.size() > target->sizeOfOrderTable
            && !Vehicles::OrderManager::spaceLeftInGlobalOrderTable(sourceOrders.size() - target->sizeOfOrderTable))
        {
            setErrorText(StringIds::no_space_for_more_vehicle_orders);
            return kFailure;
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        if (!ordersEqual)
        {
            Vehicles::OrderManager::replaceOrderTable(*target, sourceOrders);
        }
        if (!Vehicles::SharedOrderManager::join(args.target, args.source))
        {
            return failShareCommand();
        }

        affected.push_back(args.target);
        sortAndUnique(affected);
        VehicleOrderCommon::invalidateOrderWindows(affected);
        return 0;
    }

    static uint32_t leave(const VehicleOrderShareArgs& args, const uint8_t flags)
    {
        auto* target = VehicleOrderCommon::getHead(args.target);
        if (target == nullptr || !Vehicles::SharedOrderManager::isShared(args.target))
        {
            return failShareCommand();
        }

        setPosition(target->position);
        if (!checkCompanyCompatibility(target->owner))
        {
            return kFailure;
        }

        auto affected = Vehicles::SharedOrderManager::getMembers(args.target);
        if (!Vehicles::TimetableManager::canSplitService(args.target))
        {
            return failShareCommand();
        }
        if (!(flags & Flags::apply))
        {
            return 0;
        }
        if (!Vehicles::SharedOrderManager::leave(args.target))
        {
            return failShareCommand();
        }

        VehicleOrderCommon::invalidateOrderWindows(affected);
        return 0;
    }

    static uint32_t joinAllMatching(const VehicleOrderShareArgs& args, const uint8_t flags)
    {
        auto* source = VehicleOrderCommon::getHead(args.target);
        if (source == nullptr || source->sizeOfOrderTable <= sizeof(Vehicles::OrderEnd))
        {
            return failShareCommand();
        }

        setPosition(source->position);
        if (!checkCompanyCompatibility(source->owner))
        {
            return kFailure;
        }

        const auto sourceMembers = Vehicles::SharedOrderManager::getMembers(args.target);
        if (!validateSourceGroup(*source, sourceMembers))
        {
            return failShareCommand();
        }

        std::vector<EntityId> affected;
        for (auto* candidate : VehicleManager::VehicleList())
        {
            if (!VehicleOrderCommon::hasValidOrderTable(*candidate)
                || !Vehicles::SharedOrderManager::areVehiclesCompatible(*candidate, *source)
                || !Vehicles::SharedOrderManager::areOrdersEqual(*candidate, *source))
            {
                continue;
            }

            const auto members = Vehicles::SharedOrderManager::getMembers(candidate->id);
            affected.insert(affected.end(), members.begin(), members.end());
        }
        sortAndUnique(affected);

        if (affected.size() < 2 || std::ranges::find(affected, args.target) == affected.end())
        {
            return failShareCommand();
        }
        for (const auto id : affected)
        {
            const auto* member = VehicleOrderCommon::getHead(id);
            if (member == nullptr || !VehicleOrderCommon::hasValidOrderTable(*member)
                || !Vehicles::SharedOrderManager::areVehiclesCompatible(*member, *source)
                || !Vehicles::SharedOrderManager::areOrdersEqual(*member, *source))
            {
                return failShareCommand();
            }
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }
        if (!Vehicles::SharedOrderManager::joinAllMatching(args.target))
        {
            return failShareCommand();
        }

        VehicleOrderCommon::invalidateOrderWindows(affected);
        return 0;
    }

    static uint32_t vehicleOrderShare(const VehicleOrderShareArgs& args, const uint8_t flags)
    {
        switch (args.mode)
        {
            case VehicleOrderShareArgs::Mode::joinSource:
                return joinSource(args, flags);
            case VehicleOrderShareArgs::Mode::leave:
                return leave(args, flags);
            case VehicleOrderShareArgs::Mode::joinAllMatching:
                return joinAllMatching(args, flags);
        }
        return failShareCommand();
    }

    void vehicleOrderShare(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderShare(VehicleOrderShareArgs(regs), flags);
    }
}
