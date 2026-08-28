#include "GameCommands/Vehicles/VehicleTimetable.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/VehicleOrderCommon.h"
#include "Localisation/StringIds.h"
#include "Ui/WindowManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/VehicleHead.h"
#include <OpenLoco/CargoDist/CargoDist.h>

namespace OpenLoco::GameCommands
{
    static bool applyAction(const VehicleTimetableArgs& args)
    {
        using Action = VehicleTimetableArgs::Action;
        using namespace Vehicles::TimetableManager;

        if (args.orderIndex > std::numeric_limits<uint8_t>::max())
        {
            return false;
        }
        const auto orderIndex = static_cast<uint8_t>(args.orderIndex);
        switch (args.action)
        {
            case Action::setEnabled:
                return args.value != 0 ? enableForVehicle(args.head) : disableForVehicle(args.head);

            case Action::setTravelMinutes:
                return setTravelMinutes(args.head, orderIndex, args.value == VehicleTimetableArgs::kClearValue ? std::nullopt : std::optional<uint32_t>{ args.value });

            case Action::setDwellMinutes:
                return setDwellMinutes(args.head, orderIndex, args.value == VehicleTimetableArgs::kClearValue ? std::nullopt : std::optional<uint32_t>{ args.value });

            case Action::setDispatchPeriod:
                return setDispatchPeriod(args.head, orderIndex, args.value);

            case Action::setDispatchPhase:
                return setDispatchPhase(args.head, orderIndex, args.value);

            case Action::setDispatchMaxDelay:
                return setDispatchMaxDelay(args.head, orderIndex, args.value);

            case Action::addDispatchSlot:
                return addDispatchSlot(args.head, orderIndex, args.value);

            case Action::setEvenlySpacedSlots:
                return setEvenlySpacedSlots(args.head, orderIndex, args.value);

            case Action::removeDispatchSlot:
                return removeDispatchSlot(args.head, orderIndex, args.value);

            case Action::clearDispatch:
                return clearDispatch(args.head, orderIndex);

            case Action::setClockRate:
                return args.value <= std::numeric_limits<uint16_t>::max() && setTicksPerMinute(static_cast<uint16_t>(args.value));

            case Action::resetDispatch: {
                const auto service = getServiceId(args.head);
                if (service == kInvalidServiceId)
                {
                    return false;
                }
                resetDispatchState(service);
                return true;
            }
        }
        return false;
    }

    static uint32_t vehicleTimetable(const VehicleTimetableArgs& args, const uint8_t flags)
    {
        auto* head = VehicleOrderCommon::getHead(args.head);
        if (head == nullptr || !checkCompanyCompatibility(head->owner))
        {
            return kFailure;
        }
        setPosition(head->position);

        const auto members = Vehicles::SharedOrderManager::getMembers(args.head);
        if (!VehicleOrderCommon::hasConsistentOrderTables(*head, members))
        {
            setErrorText(StringIds::timetable_invalid_value);
            return kFailure;
        }

        const bool enabling = args.action == VehicleTimetableArgs::Action::setEnabled && args.value != 0;
        const bool editingDispatch = (args.action >= VehicleTimetableArgs::Action::setDispatchPeriod
                                         && args.action <= VehicleTimetableArgs::Action::clearDispatch)
            || args.action == VehicleTimetableArgs::Action::setEvenlySpacedSlots;
        if ((enabling || editingDispatch) && head->hasUnbunchingOrder())
        {
            setErrorText(StringIds::timetable_unbunching_incompatible);
            return kFailure;
        }
        const auto before = Vehicles::TimetableManager::captureState();
        const auto beforeMeasurements = Vehicles::TimetableManager::captureMeasurementState();
        if (!applyAction(args))
        {
            Vehicles::TimetableManager::restoreState(before);
            Vehicles::TimetableManager::restoreMeasurementState(beforeMeasurements);
            setErrorText(StringIds::timetable_invalid_value);
            return kFailure;
        }
        if (!(flags & Flags::apply))
        {
            Vehicles::TimetableManager::restoreState(before);
            Vehicles::TimetableManager::restoreMeasurementState(beforeMeasurements);
            return 0;
        }

        VehicleOrderCommon::invalidateOrderWindows(members);
        for (const auto member : members)
        {
            Ui::WindowManager::invalidate(Ui::WindowType::vehicle, enumValue(member));
        }
        CargoDist::markServicesDirty();
        return 0;
    }

    void vehicleTimetable(registers& regs, const uint8_t flags)
    {
        regs.ebx = vehicleTimetable(VehicleTimetableArgs(regs), flags);
    }
}
