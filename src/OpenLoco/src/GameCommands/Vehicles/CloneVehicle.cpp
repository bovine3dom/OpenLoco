#include "GameCommands/Vehicles/CloneVehicle.h"
#include "Economy/Expenditures.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/CreateVehicle.h"
#include "GameCommands/Vehicles/VehicleChangeRunningMode.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "GameCommands/Vehicles/VehicleRefit.h"
#include "Localisation/StringIds.h"
#include "Objects/VehicleObject.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"

#include <vector>

namespace OpenLoco::GameCommands
{
    static void copyVehicleColours(Vehicles::Vehicle& source, Vehicles::Vehicle& target)
    {
        auto srcIter = source.cars.begin();
        auto tgtIter = target.cars.begin();
        auto srcEnd = source.cars.end();
        for (; srcIter != srcEnd; srcIter++, tgtIter++)
        {
            auto srcCarIter = (*srcIter).begin();
            auto tgtCarIter = (*tgtIter).begin();
            auto srcCarEnd = (*srcIter).end();
            for (; srcCarIter != srcCarEnd; srcCarIter++, tgtCarIter++)
            {
                const auto objectId = (*tgtCarIter).body->objectId;
                (*tgtCarIter).body->colourScheme = getEffectiveVehicleColourScheme(objectId, (*srcCarIter).body->colourScheme);
                (*tgtCarIter).front->colourScheme = getEffectiveVehicleColourScheme(objectId, (*srcCarIter).front->colourScheme);
                (*tgtCarIter).back->colourScheme = getEffectiveVehicleColourScheme(objectId, (*srcCarIter).back->colourScheme);
            }
        }
    }

    static void copyVehicleReversals(Vehicles::Vehicle& source, Vehicles::Vehicle& target)
    {
        std::vector<bool> sourceReversals;
        std::vector<bool> targetReversals;
        std::vector<Vehicles::VehicleBogie*> targetFronts;
        for (auto& car : source.cars)
        {
            sourceReversals.push_back(car.body->has38Flags(Vehicles::Flags38::isReversed));
        }
        for (auto& car : target.cars)
        {
            targetReversals.push_back(car.body->has38Flags(Vehicles::Flags38::isReversed));
            targetFronts.push_back(car.front);
        }
        for (size_t i = 0; i < sourceReversals.size(); ++i)
        {
            if (targetReversals[i] != sourceReversals[i])
            {
                Vehicles::flipCar(*targetFronts[i]);
            }
        }
    }

    static uint32_t cloneVehicle(const VehicleCloneArgs& args, uint8_t flags)
    {
        auto* sourceHead = VehicleOrderCommon::getHead(args.vehicleHeadId);
        if (sourceHead == nullptr)
        {
            return kFailure;
        }
        Vehicles::Vehicle existingTrain(*sourceHead);
        if (existingTrain.cars.empty())
        {
            return kFailure;
        }
        Vehicles::VehicleHead* newHead = nullptr;

        size_t requiredEntities = 0;
        existingTrain.applyToComponents([&requiredEntities](const auto&) { ++requiredEntities; });
        if (!EntityManager::checkNumFreeEntities(requiredEntities))
        {
            return kFailure;
        }

        const auto sourceOrderTableSize = existingTrain.head->sizeOfOrderTable;
        if (sourceOrderTableSize < sizeof(Vehicles::OrderEnd)
            || !Vehicles::OrderManager::isOrderOffsetValid(*existingTrain.head, sourceOrderTableSize - sizeof(Vehicles::OrderEnd), true))
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        if (!checkCompanyCompatibility(existingTrain.head->owner))
        {
            return kFailure;
        }
        if (args.shareOrders && existingTrain.head->owner != getUpdatingCompanyId())
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        const auto sourceMembers = Vehicles::SharedOrderManager::getMembers(existingTrain.head->id);
        if (args.shareOrders
            && (!VehicleOrderCommon::hasConsistentOrderTables(*existingTrain.head, sourceMembers)
                || !Vehicles::SharedOrderManager::areVehiclesCompatible(*existingTrain.head, *existingTrain.head)))
        {
            setErrorText(StringIds::empty);
            return kFailure;
        }
        const auto sourceOrders = Vehicles::OrderManager::copyOrderTable(*existingTrain.head);
        if (!Vehicles::OrderManager::spaceLeftInGlobalOrderTable(sourceOrders.size()))
        {
            setErrorText(StringIds::no_space_for_more_vehicle_orders);
            return kFailure;
        }

        // Get total cost for a new vehicle
        if (!(flags & Flags::apply))
        {
            uint32_t totalCost = 0;
            for (auto& car : existingTrain.cars)
            {
                VehicleCreateArgs args{};
                args.vehicleId = EntityId::null;
                args.vehicleType = car.front->objectId;

                const auto cost = doCommand(args, 0);
                if (cost == kFailure)
                {
                    totalCost = kFailure;
                    break;
                }
                else
                {
                    totalCost += cost;
                }
            }

            if (totalCost == kFailure)
            {
                return kFailure;
            }
            return totalCost;
        }

        uint16_t cargoType = 0;
        uint32_t totalCost = 0;
        for (auto& car : existingTrain.cars)
        {
            uint32_t cost = 0;
            if (newHead == nullptr)
            {
                VehicleCreateArgs args{};
                args.vehicleId = EntityId::null;
                args.vehicleType = car.front->objectId;

                cost = doCommand(args, Flags::apply);
                cargoType = car.body->primaryCargo.type;

                auto* newVeh = EntityManager::get<Vehicles::VehicleBase>(getLegacyReturnState().lastCreatedVehicleId);
                if (newVeh == nullptr)
                {
                    return kFailure;
                }
                newHead = EntityManager::get<Vehicles::VehicleHead>(newVeh->getHead());
                if (newHead == nullptr)
                {
                    return kFailure;
                }
                newHead->vehicleFlags |= Vehicles::VehicleFlags::shuntCheat;
            }
            else
            {
                VehicleCreateArgs args{};
                args.vehicleId = newHead->head;
                args.vehicleType = car.front->objectId;

                cost = doCommand(args, Flags::apply);
            }
            if (cost == kFailure)
            {
                if (newHead != nullptr)
                {
                    VehicleManager::deleteTrain(*newHead);
                }
                return kFailure;
            }
            else
            {
                totalCost += cost;
            }
        }
        if (newHead == nullptr)
        {
            return kFailure;
        }

        auto newTrain = Vehicles::Vehicle(*newHead);
        newHead->vehicleFlags &= ~Vehicles::VehicleFlags::shuntCheat;
        copyVehicleReversals(existingTrain, newTrain);
        newTrain = Vehicles::Vehicle(*newHead);
        copyVehicleColours(existingTrain, newTrain);

        // The new head is unshared, so replacing its table cannot fan out to a group.
        Vehicles::OrderManager::replaceOrderTable(*newHead, sourceOrders);

        // Copy express/local
        if ((existingTrain.veh1->var_48 & Vehicles::Flags48::expressMode) != Vehicles::Flags48::none)
        {
            GameCommands::VehicleChangeRunningModeArgs args{};
            args.head = newHead->id;
            args.mode = GameCommands::VehicleChangeRunningModeArgs::Mode::toggleLocalExpress;
            if (GameCommands::doCommand(args, GameCommands::Flags::apply) == kFailure)
            {
                VehicleManager::deleteTrain(*newHead);
                return kFailure;
            }
        }

        // Copy cargo refit status (only applies to boats and airplanes)
        if (newHead->vehicleType == VehicleType::ship || newHead->vehicleType == VehicleType::aircraft)
        {
            VehicleRefitArgs args{};
            args.head = newHead->head;
            args.cargoType = cargoType;
            if (doCommand(args, Flags::apply) == kFailure)
            {
                VehicleManager::deleteTrain(*newHead);
                return kFailure;
            }
        }

        if (args.shareOrders)
        {
            const auto joined = Vehicles::SharedOrderManager::areVehiclesCompatible(*newHead, *existingTrain.head)
                && Vehicles::SharedOrderManager::areOrdersEqual(*newHead, *existingTrain.head)
                && Vehicles::SharedOrderManager::join(newHead->id, existingTrain.head->id);
            if (!joined)
            {
                VehicleManager::deleteTrain(*newHead);
                return kFailure;
            }
            for (const auto member : Vehicles::SharedOrderManager::getMembers(newHead->id))
            {
                Ui::WindowManager::invalidateOrderPageByVehicleNumber(enumValue(member));
            }
            Ui::WindowManager::invalidate(Ui::WindowType::vehicleList);
        }

        // Finally, set the expenditure type
        // Note we explicitly set this *after* running all the sub commands!
        setExpenditureType(ExpenditureType::VehiclePurchases);

        return totalCost;
    }

    void cloneVehicle(registers& regs, const uint8_t flags)
    {
        regs.ebx = cloneVehicle(VehicleCloneArgs(regs), flags);
    }
}
