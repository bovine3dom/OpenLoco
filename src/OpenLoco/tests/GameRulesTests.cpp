// SPDX-License-Identifier: MIT
#include "GameRules.h"

#include "GameCommands/General/SetVehiclesNeverExpire.h"
#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Objects/VehicleObject.h"
#include "SceneManager.h"
#include "Vehicles/VehicleManager.h"
#include <gtest/gtest.h>

using namespace OpenLoco;

namespace
{
    class GameRulesTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            GameRules::reset();
        }

        void TearDown() override
        {
            GameRules::reset();
        }
    };
}

TEST_F(GameRulesTest, DefaultsResetCaptureAndRestore)
{
    EXPECT_EQ(GameRules::captureState(), GameRules::kDefaultState);
    EXPECT_FALSE(GameRules::vehiclesNeverExpire());
    EXPECT_FALSE(GameRules::extendedVehicleObjects());

    GameRules::setVehiclesNeverExpire(true);
    const GameRules::State enabled{ .vehiclesNeverExpire = true, .extendedVehicleObjects = true };
    GameRules::restoreState(enabled);
    EXPECT_TRUE(GameRules::vehiclesNeverExpire());
    EXPECT_TRUE(GameRules::extendedVehicleObjects());

    GameRules::reset();
    EXPECT_EQ(GameRules::captureState(), GameRules::kDefaultState);

    GameRules::restoreState(enabled);
    EXPECT_EQ(GameRules::captureState(), enabled);
}

TEST_F(GameRulesTest, ExtendedVehicleRuleIsEditorOnly)
{
    const auto previousFlags = SceneManager::getSceneFlags();
    SceneManager::setSceneFlags(SceneManager::Flags::none);
    EXPECT_FALSE(GameRules::setExtendedVehicleObjects(true));
    EXPECT_FALSE(GameRules::extendedVehicleObjects());

    SceneManager::setSceneFlags(SceneManager::Flags::editor);
    EXPECT_TRUE(GameRules::setExtendedVehicleObjects(true));
    EXPECT_TRUE(GameRules::extendedVehicleObjects());
    SceneManager::setSceneFlags(previousFlags);
}

TEST_F(GameRulesTest, RestoreAndResetDoNotRecalculateCompanyUnlocks)
{
    auto& company = getGameState().companies[0];
    const auto previousName = company.name;
    const auto previousUnlocks = company.unlockedVehicles;
    company.name = StringIds::new_company;
    company.unlockedVehicles.set(999, true);

    GameRules::restoreState({ .vehiclesNeverExpire = true, .extendedVehicleObjects = true });
    EXPECT_TRUE(company.unlockedVehicles[999]);
    GameRules::reset();
    EXPECT_TRUE(company.unlockedVehicles[999]);

    company.unlockedVehicles = previousUnlocks;
    company.name = previousName;
}

TEST_F(GameRulesTest, LiveNeverExpireSetterRecalculatesCompanyUnlocks)
{
    auto& company = getGameState().companies[0];
    const auto previousName = company.name;
    const auto previousUnlocks = company.unlockedVehicles;
    const auto previousAvailableVehicles = company.availableVehicles;
    company.name = StringIds::new_company;
    company.unlockedVehicles.set(999, true);
    GameRules::restoreState({ .vehiclesNeverExpire = true });

    GameRules::setVehiclesNeverExpire(true);
    EXPECT_FALSE(company.unlockedVehicles[999]);

    company.unlockedVehicles = previousUnlocks;
    company.availableVehicles = previousAvailableVehicles;
    company.name = previousName;
}

TEST_F(GameRulesTest, VehicleAvailabilityAlwaysHonoursDesignedYear)
{
    VehicleObject vehicle{};
    vehicle.designed = 1950;
    vehicle.obsolete = 2000;

    EXPECT_FALSE(VehicleManager::isVehicleObjectAvailable(vehicle, 1949));
    EXPECT_TRUE(VehicleManager::isVehicleObjectAvailable(vehicle, 1950));
    EXPECT_TRUE(VehicleManager::isVehicleObjectAvailable(vehicle, 1999));
    EXPECT_FALSE(VehicleManager::isVehicleObjectAvailable(vehicle, 2000));

    GameRules::setVehiclesNeverExpire(true);
    EXPECT_FALSE(VehicleManager::isVehicleObjectAvailable(vehicle, 1949));
    EXPECT_TRUE(VehicleManager::isVehicleObjectAvailable(vehicle, 2000));
}

TEST_F(GameRulesTest, CommandArgumentsRoundTripThroughRegisters)
{
    const GameCommands::SetVehiclesNeverExpireArgs original(1);

    const GameCommands::SetVehiclesNeverExpireArgs decoded(static_cast<GameCommands::registers>(original));

    EXPECT_EQ(decoded.enabled, 1);
}

TEST_F(GameRulesTest, CommandRejectsInvalidValues)
{
    auto regs = static_cast<GameCommands::registers>(GameCommands::SetVehiclesNeverExpireArgs(2));

    GameCommands::setVehiclesNeverExpire(regs, GameCommands::Flags::apply);

    EXPECT_EQ(static_cast<uint32_t>(regs.ebx), GameCommands::kFailure);
    EXPECT_FALSE(GameRules::vehiclesNeverExpire());
}

TEST_F(GameRulesTest, CommandQueriesAndAppliesState)
{
    GameRules::restoreState({ .extendedVehicleObjects = true });
    auto regs = static_cast<GameCommands::registers>(GameCommands::SetVehiclesNeverExpireArgs(1));

    GameCommands::setVehiclesNeverExpire(regs, 0);
    EXPECT_EQ(regs.ebx, 0);
    EXPECT_FALSE(GameRules::vehiclesNeverExpire());

    GameCommands::setVehiclesNeverExpire(regs, GameCommands::Flags::apply);
    EXPECT_EQ(regs.ebx, 0);
    EXPECT_TRUE(GameRules::vehiclesNeverExpire());
    EXPECT_TRUE(GameRules::extendedVehicleObjects());

    regs = static_cast<GameCommands::registers>(GameCommands::SetVehiclesNeverExpireArgs(0));
    GameCommands::setVehiclesNeverExpire(regs, GameCommands::Flags::apply);
    EXPECT_EQ(regs.ebx, 0);
    EXPECT_FALSE(GameRules::vehiclesNeverExpire());
    EXPECT_TRUE(GameRules::extendedVehicleObjects());
}
