#include "GameCommands/Vehicles/VehicleOrderToggleUnbunching.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"

namespace OpenLoco::GameCommands
{
    static uint32_t vehicleOrderToggleUnbunching(const VehicleOrderToggleUnbunchingArgs& args, uint8_t flags)
    {
        auto* head = EntityManager::get<Vehicles::VehicleHead>(args.head);
        if (head == nullptr || args.orderOffset >= head->sizeOfOrderTable)
        {
            return kFailure;
        }
        if (!checkCompanyCompatibility(head->owner))
        {
            return kFailure;
        }

        setPosition(head->position);
        Vehicles::OrderStopAt* selectedOrder = nullptr;
        for (auto& order : head->getCurrentOrders())
        {
            if (order.getOffset() - head->orderTableOffset == args.orderOffset)
            {
                selectedOrder = order.as<Vehicles::OrderStopAt>();
                break;
            }
        }
        if (selectedOrder == nullptr)
        {
            return kFailure;
        }

        const bool enable = !selectedOrder->isUnbunching();
        if (enable)
        {
            for (const auto& order : head->getCurrentOrders())
            {
                if (order.is<Vehicles::OrderWaitFor>())
                {
                    setErrorText(StringIds::unbunching_incompatible_with_full_load);
                    return kFailure;
                }

                const auto* stopOrder = order.as<Vehicles::OrderStopAt>();
                if (stopOrder != nullptr && stopOrder != selectedOrder && stopOrder->isUnbunching())
                {
                    setErrorText(StringIds::only_one_unbunching_stop_allowed);
                    return kFailure;
                }
            }
        }

        if (!(flags & Flags::apply))
        {
            return 0;
        }

        for (auto* other : VehicleManager::VehicleList())
        {
            if (other == head || !Vehicles::OrderManager::areVehiclesOnSameRoute(*head, *other))
            {
                continue;
            }

            auto* otherOrder = Vehicles::OrderRingView(other->orderTableOffset, args.orderOffset).begin()->as<Vehicles::OrderStopAt>();
            otherOrder->setUnbunching(enable);
            other->resetUnbunching();
            Ui::WindowManager::invalidate(Ui::WindowType::vehicle, enumValue(other->id));
        }

        selectedOrder->setUnbunching(enable);
        head->resetUnbunching();
        Ui::WindowManager::invalidate(Ui::WindowType::vehicle, enumValue(head->id));
        return 0;
    }

    void vehicleOrderToggleUnbunching(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleOrderToggleUnbunching(VehicleOrderToggleUnbunchingArgs(regs), flags);
    }
}
