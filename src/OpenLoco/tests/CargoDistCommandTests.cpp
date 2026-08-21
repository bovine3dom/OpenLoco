// SPDX-License-Identifier: MIT
#include "Entities/EntityManager.h"
#include "GameCommands/General/SetCargoDistMode.h"

#include <algorithm>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;
using namespace OpenLoco::GameCommands;

TEST(CargoDistCommand, ArgumentsRoundTripThroughRegisters)
{
    const SetCargoDistModeArgs original(7, DistributionMode::asymmetric);

    const SetCargoDistModeArgs decoded(static_cast<registers>(original));

    EXPECT_EQ(decoded.cargo, 7);
    EXPECT_EQ(decoded.mode, DistributionMode::asymmetric);

    const SetCargoDistModeArgs allDecoded(static_cast<registers>(SetCargoDistModeArgs(kAllCargo, DistributionMode::manual)));
    EXPECT_EQ(allDecoded.cargo, kAllCargo);
}

TEST(CargoDistCommand, QueryDoesNotApplyMode)
{
    reset();
    registers regs = static_cast<registers>(SetCargoDistModeArgs(kAllCargo, DistributionMode::asymmetric));

    setCargoDistMode(regs, 0);

    EXPECT_EQ(static_cast<uint32_t>(regs.ebx), 0);
    EXPECT_EQ(getMode(0), DistributionMode::manual);
}

TEST(CargoDistCommand, RejectsInvalidArguments)
{
    registers invalidCargo = static_cast<registers>(SetCargoDistModeArgs(32, DistributionMode::manual));
    registers invalidMode = static_cast<registers>(SetCargoDistModeArgs(kAllCargo, static_cast<DistributionMode>(0xFF)));

    setCargoDistMode(invalidCargo, 0);
    setCargoDistMode(invalidMode, 0);

    EXPECT_EQ(static_cast<uint32_t>(invalidCargo.ebx), kFailure);
    EXPECT_EQ(static_cast<uint32_t>(invalidMode.ebx), kFailure);
}

TEST(CargoDistCommand, AppliesGlobalManualMode)
{
    reset();
    EntityManager::reset();
    getState().settings.modes[31] = DistributionMode::asymmetric;
    getState().graphDirty = true;
    ASSERT_EQ(getMode(31), DistributionMode::asymmetric);
    ASSERT_TRUE(getStateConst().graphDirty);
    registers regs = static_cast<registers>(SetCargoDistModeArgs(kAllCargo, DistributionMode::manual));

    setCargoDistMode(regs, Flags::apply);

    EXPECT_EQ(static_cast<uint32_t>(regs.ebx), 0);
    EXPECT_TRUE(std::ranges::all_of(getStateConst().settings.modes, [](auto mode) { return mode == DistributionMode::manual; }));
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST(CargoDistCommand, KeepsRoutingEnabledUntilTransferCreditsAreSettled)
{
    reset();
    EntityManager::reset();
    getState().settings.modes[31] = DistributionMode::asymmetric;
    getState().stationCargo[{ StationId(1), 31 }].append({ 10, StationId(1), StationId(2), 0, {}, {}, StationId(2), 50 });
    registers singleRegs = static_cast<registers>(SetCargoDistModeArgs(31, DistributionMode::manual));

    setCargoDistMode(singleRegs, Flags::apply);

    EXPECT_EQ(static_cast<uint32_t>(singleRegs.ebx), kFailure);
    EXPECT_EQ(getMode(31), DistributionMode::asymmetric);

    registers regs = static_cast<registers>(SetCargoDistModeArgs(kAllCargo, DistributionMode::manual));

    setCargoDistMode(regs, Flags::apply);

    EXPECT_EQ(static_cast<uint32_t>(regs.ebx), kFailure);
    EXPECT_EQ(getMode(31), DistributionMode::asymmetric);

    setMode(31, DistributionMode::manual);
    EXPECT_EQ(getMode(31), DistributionMode::asymmetric);
}
