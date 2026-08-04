#include "GameCommands/Company/SetVehicleAutoRenewal.h"

#include "Ui/WindowManager.h"
#include "Vehicles/VehicleAutoRenewal.h"
#include "World/Company.h"
#include "World/CompanyManager.h"

namespace OpenLoco::GameCommands
{
    static uint32_t setVehicleAutoRenewal(const SetVehicleAutoRenewalArgs& args, const uint8_t flags)
    {
        const auto companyId = getUpdatingCompanyId();
        const auto* company = CompanyManager::get(companyId);
        if (company == nullptr || company->empty() || args.enabled > 1
            || args.reliabilityThreshold > Vehicles::VehicleAutoRenewal::kMaxReliabilityThreshold)
        {
            return kFailure;
        }

        if ((flags & Flags::apply) != 0)
        {
            Vehicles::VehicleAutoRenewal::setSettings(companyId, {
                                                                     args.enabled != 0,
                                                                     args.reliabilityThreshold,
                                                                 });
            Ui::WindowManager::invalidate(Ui::WindowType::company, enumValue(companyId));
        }
        return 0;
    }

    void setVehicleAutoRenewal(registers& regs, const uint8_t flags)
    {
        regs.ebx = setVehicleAutoRenewal(SetVehicleAutoRenewalArgs(regs), flags);
    }
}
