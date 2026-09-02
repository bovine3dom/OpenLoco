#include "Date.h"
#include "GameCommands/Company/SetVehicleAutoRenewal.h"
#include "GameCommands/GameCommands.h"
#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Objects/VehicleObject.h"
#include "Vehicles/VehicleAutoRenewal.h"
#include "Vehicles/VehicleBogie.h"
#include "World/CompanyManager.h"
#include <gtest/gtest.h>
#include <memory>

using namespace OpenLoco;
using namespace OpenLoco::GameCommands;
using namespace OpenLoco::Vehicles;

namespace
{
    class VehicleAutoRenewalTest : public ::testing::Test
    {
    protected:
        static constexpr CompanyId kCompany{ 0 };

        StringId _oldCompanyName{};
        StringId _oldOtherCompanyName{};
        CompanyId _oldControllingCompany{};
        CompanyId _oldUpdatingCompany{};

        void SetUp() override
        {
            auto* company = CompanyManager::get(kCompany);
            _oldCompanyName = company->name;
            _oldOtherCompanyName = CompanyManager::get(CompanyId(1))->name;
            _oldControllingCompany = CompanyManager::getControllingId();
            _oldUpdatingCompany = getUpdatingCompanyId();
            company->name = StringIds::new_company;
            CompanyManager::setControllingId(kCompany);
            setUpdatingCompanyId(kCompany);
            VehicleAutoRenewal::reset();
        }

        void TearDown() override
        {
            VehicleAutoRenewal::reset();
            CompanyManager::get(kCompany)->name = _oldCompanyName;
            CompanyManager::get(CompanyId(1))->name = _oldOtherCompanyName;
            CompanyManager::setControllingId(_oldControllingCompany);
            setUpdatingCompanyId(_oldUpdatingCompany);
        }

        static uint32_t runCommand(const SetVehicleAutoRenewalArgs& args, const uint8_t flags)
        {
            auto regs = static_cast<registers>(args);
            setVehicleAutoRenewal(regs, flags);
            return static_cast<uint32_t>(regs.ebx);
        }
    };
}

TEST_F(VehicleAutoRenewalTest, StateDefaultsValidatesAndRestoresPerCompanySettings)
{
    const auto defaults = VehicleAutoRenewal::captureState();
    EXPECT_TRUE(VehicleAutoRenewal::isDefault(defaults));
    for (const auto& settings : defaults.companies)
    {
        EXPECT_FALSE(settings.enabled);
        EXPECT_EQ(settings.reliabilityThreshold, 25);
    }

    EXPECT_TRUE(VehicleAutoRenewal::setSettings(CompanyId(3), { true, 40 }));
    EXPECT_EQ(VehicleAutoRenewal::getSettings(CompanyId(3)), (VehicleAutoRenewal::Settings{ true, 40 }));
    EXPECT_EQ(VehicleAutoRenewal::getSettings(CompanyId(2)), VehicleAutoRenewal::Settings{});

    const auto captured = VehicleAutoRenewal::captureState();
    VehicleAutoRenewal::reset(CompanyId(3));
    EXPECT_EQ(VehicleAutoRenewal::getSettings(CompanyId(3)), VehicleAutoRenewal::Settings{});
    ASSERT_TRUE(VehicleAutoRenewal::restoreState(captured));
    EXPECT_EQ(VehicleAutoRenewal::getSettings(CompanyId(3)), (VehicleAutoRenewal::Settings{ true, 40 }));

    auto invalid = captured;
    invalid.companies[0].reliabilityThreshold = 101;
    EXPECT_FALSE(VehicleAutoRenewal::validateState(invalid));
    EXPECT_FALSE(VehicleAutoRenewal::restoreState(invalid));
    EXPECT_EQ(VehicleAutoRenewal::captureState(), captured);
    EXPECT_FALSE(VehicleAutoRenewal::setSettings(CompanyId::null, { true, 25 }));

    auto gameState = std::make_unique<GameState>();
    EXPECT_FALSE(VehicleAutoRenewal::validateState(captured, *gameState));
    gameState->companies[3].name = StringIds::new_company;
    EXPECT_TRUE(VehicleAutoRenewal::validateState(captured, *gameState));
}

TEST_F(VehicleAutoRenewalTest, SettingsCommandSeparatesQueryFromApplyAndValidatesArguments)
{
    const SetVehicleAutoRenewalArgs args{ 1, 45 };
    EXPECT_EQ(runCommand(args, 0), 0U);
    EXPECT_EQ(VehicleAutoRenewal::getSettings(kCompany), VehicleAutoRenewal::Settings{});

    EXPECT_EQ(runCommand(args, Flags::apply), 0U);
    EXPECT_EQ(VehicleAutoRenewal::getSettings(kCompany), (VehicleAutoRenewal::Settings{ true, 45 }));

    auto invalidEnabled = args;
    invalidEnabled.enabled = 2;
    EXPECT_EQ(runCommand(invalidEnabled, Flags::apply), kFailure);
    auto invalidThreshold = args;
    invalidThreshold.reliabilityThreshold = 101;
    EXPECT_EQ(runCommand(invalidThreshold, Flags::apply), kFailure);

    setUpdatingCompanyId(CompanyId::neutral);
    EXPECT_EQ(runCommand(args, Flags::apply), kFailure);
    EXPECT_EQ(VehicleAutoRenewal::getSettings(kCompany), (VehicleAutoRenewal::Settings{ true, 45 }));

    const auto otherCompany = CompanyId(1);
    CompanyManager::get(otherCompany)->name = StringIds::new_company;
    setUpdatingCompanyId(otherCompany);
    EXPECT_EQ(runCommand({ 0, 30 }, Flags::apply), 0U);
    EXPECT_EQ(VehicleAutoRenewal::getSettings(otherCompany), (VehicleAutoRenewal::Settings{ false, 30 }));
    EXPECT_EQ(VehicleAutoRenewal::getSettings(kCompany), (VehicleAutoRenewal::Settings{ true, 45 }));
}

TEST_F(VehicleAutoRenewalTest, InitialReliabilityMatchesVehicleCreationRules)
{
    const auto oldYear = getCurrentYear();
    setCurrentYear(2000);
    VehicleObject vehicleObject{};
    vehicleObject.reliability = 80;

    vehicleObject.designed = 1990;
    EXPECT_EQ(calculateInitialReliability(vehicleObject), 80 * 256 + 255);

    vehicleObject.designed = 1999;
    EXPECT_EQ(calculateInitialReliability(vehicleObject), 15935);

    vehicleObject.reliability = 0;
    EXPECT_EQ(calculateInitialReliability(vehicleObject), 0);
    setCurrentYear(oldYear);
}

TEST_F(VehicleAutoRenewalTest, ReliabilityLossPerDayUsesNormalRateBeforeObsolescence)
{
    VehicleObject vehicleObject{};
    vehicleObject.reliability = 80;
    vehicleObject.obsolete = 2001;

    EXPECT_EQ(calculateReliabilityLossPerDay(vehicleObject, 2000), 4);
}

TEST_F(VehicleAutoRenewalTest, ReliabilityLossPerDayUsesObsoleteRateAtBoundary)
{
    VehicleObject vehicleObject{};
    vehicleObject.reliability = 80;
    vehicleObject.obsolete = 2000;

    EXPECT_EQ(calculateReliabilityLossPerDay(vehicleObject, 2000), 10);
}

TEST_F(VehicleAutoRenewalTest, ReliabilityLossPerDayIsZeroWhenBreakdownsAreDisabled)
{
    VehicleObject vehicleObject{};
    vehicleObject.reliability = 0;
    vehicleObject.obsolete = 2000;

    EXPECT_EQ(calculateReliabilityLossPerDay(vehicleObject, 2000), 0);
}
