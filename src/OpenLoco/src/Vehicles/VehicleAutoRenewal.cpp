#include "Vehicles/VehicleAutoRenewal.h"

#include "Date.h"
#include "Economy/Economy.h"
#include "GameState.h"
#include "Objects/ObjectManager.h"
#include "Objects/VehicleObject.h"
#include "Ui/WindowManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "World/Company.h"
#include "World/CompanyManager.h"
#include <algorithm>
#include <limits>
#include <optional>

namespace OpenLoco::Vehicles::VehicleAutoRenewal
{
    namespace
    {
        State _state;

        bool isValidCompany(const CompanyId company)
        {
            return enumValue(company) < Limits::kMaxCompanies;
        }

        struct RenewalQuote
        {
            currency32_t purchaseCost;
            currency32_t resaleValue;
            uint8_t reliability;
        };

        std::optional<RenewalQuote> quoteRenewal(const Vehicle& train)
        {
            if (train.cars.empty())
            {
                return std::nullopt;
            }

            int64_t purchaseCost = 0;
            int64_t resaleValue = 0;
            uint16_t minReliability = std::numeric_limits<uint16_t>::max();
            for (const auto& car : train.cars)
            {
                const auto* vehicleObject = ObjectManager::get<VehicleObject>(car.front->objectId);
                if (vehicleObject == nullptr)
                {
                    return std::nullopt;
                }

                const auto carCost = Economy::getInflationAdjustedCost(vehicleObject->costFactor, vehicleObject->costIndex, 6);
                if (carCost < 0)
                {
                    return std::nullopt;
                }
                purchaseCost += carCost;
                resaleValue += car.front->refundCost;
                if (purchaseCost > std::numeric_limits<currency32_t>::max()
                    || resaleValue > std::numeric_limits<currency32_t>::max())
                {
                    return std::nullopt;
                }

                const auto reliability = calculateInitialReliability(*vehicleObject);
                if (reliability != 0)
                {
                    minReliability = std::min(minReliability, reliability);
                }
            }

            return RenewalQuote{
                static_cast<currency32_t>(purchaseCost),
                static_cast<currency32_t>(resaleValue),
                static_cast<uint8_t>(minReliability == std::numeric_limits<uint16_t>::max() ? 0 : minReliability / 256),
            };
        }
    }

    void reset()
    {
        _state = {};
    }

    void reset(const CompanyId company)
    {
        if (isValidCompany(company))
        {
            _state.companies[enumValue(company)] = {};
        }
    }

    const Settings& getSettings(const CompanyId company)
    {
        static constexpr Settings kDefaultSettings;
        return isValidCompany(company) ? _state.companies[enumValue(company)] : kDefaultSettings;
    }

    bool setSettings(const CompanyId company, const Settings settings)
    {
        if (!isValidCompany(company) || settings.reliabilityThreshold > kMaxReliabilityThreshold)
        {
            return false;
        }
        _state.companies[enumValue(company)] = settings;
        return true;
    }

    State captureState()
    {
        return _state;
    }

    bool validateState(const State& state)
    {
        return std::ranges::all_of(state.companies, [](const auto& settings) {
            return settings.reliabilityThreshold <= kMaxReliabilityThreshold;
        });
    }

    bool validateState(const State& state, const GameState& gameState)
    {
        if (!validateState(state))
        {
            return false;
        }
        for (size_t i = 0; i < state.companies.size(); ++i)
        {
            if (gameState.companies[i].empty() && state.companies[i] != Settings{})
            {
                return false;
            }
        }
        return true;
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state))
        {
            return false;
        }
        _state = state;
        return true;
    }

    bool isDefault(const State& state)
    {
        return state == State{};
    }

    bool tryRenew(VehicleHead& head)
    {
        const auto settings = getSettings(head.owner);
        if (!settings.enabled)
        {
            return false;
        }

        Vehicle train(head);
        if (train.veh2->reliability >= settings.reliabilityThreshold)
        {
            return false;
        }

        const auto quote = quoteRenewal(train);
        if (!quote.has_value() || quote->reliability < settings.reliabilityThreshold)
        {
            return false;
        }

        auto* company = CompanyManager::get(head.owner);
        if (company == nullptr || company->empty()
            || (company->challengeFlags & CompanyFlags::bankrupt) != CompanyFlags::none)
        {
            return false;
        }

        const auto netCost = static_cast<int64_t>(quote->purchaseCost) - quote->resaleValue;
        if (netCost > 0 && company->cash.asInt64() < netCost)
        {
            return false;
        }

        CompanyManager::applyPaymentToCompany(head.owner, quote->purchaseCost, ExpenditureType::VehiclePurchases);
        CompanyManager::applyPaymentToCompany(head.owner, -quote->resaleValue, ExpenditureType::VehicleDisposals);

        constexpr auto kBreakdownMask = BreakdownFlags::breakdownPending | BreakdownFlags::brokenDown;
        const auto currentDay = getCurrentDay();
        for (const auto& car : train.cars)
        {
            const auto* vehicleObject = ObjectManager::get<VehicleObject>(car.front->objectId);
            const auto reliability = calculateInitialReliability(*vehicleObject);
            const auto cost = Economy::getInflationAdjustedCost(vehicleObject->costFactor, vehicleObject->costIndex, 6);
            for (const auto& component : car)
            {
                component.front->creationDay = currentDay;
                component.back->creationDay = currentDay;
                component.body->creationDay = currentDay;
                component.front->wheelSlipping = 0;
                component.back->wheelSlipping = 0;
                component.body->wheelSlipping = 0;
                component.front->breakdownFlags &= ~kBreakdownMask;
                component.back->breakdownFlags &= ~kBreakdownMask;
                component.body->breakdownFlags &= ~kBreakdownMask;
                component.front->breakdownTimeout = 0;
                component.back->breakdownTimeout = 0;
                component.body->breakdownTimeout = 0;
                component.front->reliability = reliability;
                component.front->refundCost = cost - cost / 8;
                sub_4BA873(*component.front);
            }
        }
        head.breakdownFlags &= ~kBreakdownMask;
        head.updateTrainProperties();
        head.applyBreakdownToTrain();

        Ui::WindowManager::invalidate(Ui::WindowType::vehicle, enumValue(head.id));
        Ui::WindowManager::invalidate(Ui::WindowType::vehicleList, enumValue(head.owner));
        Ui::WindowManager::invalidate(Ui::WindowType::company, enumValue(head.owner));
        return true;
    }
}
