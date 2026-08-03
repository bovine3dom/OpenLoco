#include "Economy/Expenditures.h"
#include "Entities/EntityManager.h"
#include "GameCommands/Terraform/RemoveWall.h"
#include "GameCommands/Undo.h"
#include "GameState.h"
#include "Map/SurfaceElement.h"
#include "Map/TileManager.h"
#include "Map/TreeElement.h"
#include "Map/WallElement.h"
#include "Scenario/ScenarioOptions.h"
#include "SceneManager.h"
#include "World/CompanyManager.h"
#include <algorithm>
#include <array>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::GameCommands;
using namespace OpenLoco::World;

namespace
{
    registers makeLandRegisters(const TilePos2 pos)
    {
        registers regs{};
        regs.ax = toWorldSpace(pos).x;
        regs.cx = toWorldSpace(pos).y;
        regs.edx = (regs.ax << 16) | static_cast<uint16_t>(regs.ax);
        regs.ebp = (regs.cx << 16) | static_cast<uint16_t>(regs.cx);
        return regs;
    }

    class UndoTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            TileManager::initialise();
            EntityManager::reset();
            Undo::clear();
            getGameState().playerCompanies[0] = CompanyId(0);
            auto* company = CompanyManager::get(CompanyId(0));
            company->cash = currency48_t{ 10000 };
            company->expenditures[0][ExpenditureType::Construction] = 0;
            company->activeEmotions[enumValue(Emotion::thinking)] = 0;
            company->observationTimeout = 0;
            getGameState().unkRng = {};
        }

        void TearDown() override
        {
            Undo::clear();
        }
    };
}

TEST_F(UndoTest, RestoresChangedTilesAndRefundsWithoutRewindingLaterCash)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* surface = TileManager::get(tilePos).surface();
    ASSERT_NE(surface, nullptr);
    const auto originalBaseZ = surface->baseZ();

    const auto regs = makeLandRegisters(tilePos);
    Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
    surface->setBaseZ(originalBaseZ + 4);
    surface->setClearZ(originalBaseZ + 4);
    Undo::commit(100, ExpenditureType::Construction, { toWorldSpace(tilePos), 32 });
    CompanyManager::applyPaymentToCompany(CompanyId(0), 100, ExpenditureType::Construction);
    CompanyManager::get(CompanyId(0))->cash += 500;
    getGameState().playerCompanies[0] = CompanyId(1);

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);

    EXPECT_EQ(TileManager::get(tilePos).surface()->baseZ(), originalBaseZ);
    EXPECT_EQ(CompanyManager::get(CompanyId(0))->cash, currency48_t{ 10500 });
    EXPECT_FALSE(Undo::isAvailable());
}

TEST_F(UndoTest, RestoresGroupedTileChangesAndRefundsTheirCombinedCost)
{
    constexpr std::array tilePositions{ TilePos2{ 20, 20 }, TilePos2{ 21, 20 } };
    std::array<coord_t, tilePositions.size()> originalBaseZ{};

    for (size_t i = 0; i < tilePositions.size(); ++i)
    {
        auto* surface = TileManager::get(tilePositions[i]).surface();
        ASSERT_NE(surface, nullptr);
        originalBaseZ[i] = surface->baseZ();
    }

    {
        Undo::Group undoGroup;
        for (size_t i = 0; i < tilePositions.size(); ++i)
        {
            auto* surface = TileManager::get(tilePositions[i]).surface();
            const auto regs = makeLandRegisters(tilePositions[i]);
            Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
            surface->setBaseZ(originalBaseZ[i] + 4);
            surface->setClearZ(originalBaseZ[i] + 4);
            Undo::commit(100, ExpenditureType::Construction, { toWorldSpace(tilePositions[i]), 32 });
            CompanyManager::applyPaymentToCompany(CompanyId(0), 100, ExpenditureType::Construction);
        }

        auto* surface = TileManager::get(tilePositions[0]).surface();
        const auto regs = makeLandRegisters(tilePositions[0]);
        Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
        surface->setBaseZ(originalBaseZ[0] + 8);
        surface->setClearZ(originalBaseZ[0] + 8);
        Undo::commit(50, ExpenditureType::Construction, { toWorldSpace(tilePositions[0]), 32 });
        CompanyManager::applyPaymentToCompany(CompanyId(0), 50, ExpenditureType::Construction);
    }

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    for (size_t i = 0; i < tilePositions.size(); ++i)
    {
        EXPECT_EQ(TileManager::get(tilePositions[i]).surface()->baseZ(), originalBaseZ[i]);
    }
    EXPECT_EQ(CompanyManager::get(CompanyId(0))->cash, currency48_t{ 10000 });
}

TEST_F(UndoTest, GroupedMapUndoSurvivesTransientUpdates)
{
    constexpr std::array tilePositions{ TilePos2{ 20, 20 }, TilePos2{ 21, 20 } };
    std::array<coord_t, tilePositions.size()> originalBaseZ{};
    std::array<uint8_t, tilePositions.size()> originalTimers{};
    for (size_t i = 0; i < tilePositions.size(); ++i)
    {
        const auto* surface = TileManager::get(tilePositions[i]).surface();
        originalBaseZ[i] = surface->baseZ();
        originalTimers[i] = surface->getUpdateTimer();
    }

    auto& random = getGameState().unkRng;
    random = Core::Prng{ 1, 2 };
    auto* company = CompanyManager::get(CompanyId(0));

    {
        Undo::Group undoGroup;
        for (size_t i = 0; i < tilePositions.size(); ++i)
        {
            auto* surface = TileManager::get(tilePositions[i]).surface();
            const auto regs = makeLandRegisters(tilePositions[i]);
            Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
            surface->setBaseZ(originalBaseZ[i] + 4);
            surface->setClearZ(originalBaseZ[i] + 4);
            if (i == 1)
            {
                surface->setUpdateTimer((originalTimers[i] + 1) & 0x7);
            }
            random.randNext();
            company->activeEmotions[enumValue(Emotion::thinking)] = 7;
            company->observationTimeout = 5;
            Undo::commit(0, ExpenditureType::Construction, {});
        }
    }

    random.randNext();
    const auto expectedRandom = random;
    company->activeEmotions[enumValue(Emotion::thinking)] = 6;
    company->observationTimeout = 4;
    std::array<uint8_t, tilePositions.size()> expectedTimers{};
    for (size_t i = 0; i < tilePositions.size(); ++i)
    {
        auto* surface = TileManager::get(tilePositions[i]).surface();
        expectedTimers[i] = (surface->getUpdateTimer() + 1) & 0x7;
        surface->setUpdateTimer(expectedTimers[i]);
    }

    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    for (size_t i = 0; i < tilePositions.size(); ++i)
    {
        const auto* surface = TileManager::get(tilePositions[i]).surface();
        EXPECT_EQ(surface->baseZ(), originalBaseZ[i]);
        EXPECT_EQ(surface->getUpdateTimer(), i == 0 ? expectedTimers[i] : originalTimers[i]);
    }
    EXPECT_EQ(random.srand_0(), expectedRandom.srand_0());
    EXPECT_EQ(random.srand_1(), expectedRandom.srand_1());
    EXPECT_EQ(company->activeEmotions[enumValue(Emotion::thinking)], 6);
    EXPECT_EQ(company->observationTimeout, 4);
}

TEST_F(UndoTest, ConstructionAndUndoPreservePlayerPause)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    constexpr auto baseZ = 4;
    auto* wallEntry = TileManager::insertElement<WallElement>(toWorldSpace(tilePos), baseZ, 0xF);
    ASSERT_NE(wallEntry, nullptr);
    wallEntry->get<WallElement>().setRotation(0);

    WallRemovalArgs args{};
    args.pos = { toWorldSpace(tilePos), baseZ * kSmallZStep };
    args.rotation = 0;
    setUpdatingCompanyId(CompanyId(0));
    const auto scenarioTicks = getGameState().scenarioTicks;
    ASSERT_EQ(SceneManager::getPauseFlags(), PauseFlags::none);
    const auto originalGameSpeed = SceneManager::getGameSpeed();
    const auto originalMadeAnyChanges = Scenario::getOptions().madeAnyChanges;
    SceneManager::setGameSpeed(GameSpeed::ExtraFastForward);
    SceneManager::setPauseFlag(PauseFlags::player);
    SceneManager::setPauseFlag(PauseFlags::objectSelection);

    EXPECT_EQ(doCommand(args, Flags::apply), kFailure);
    EXPECT_EQ(SceneManager::getPauseFlags(), PauseFlags::player | PauseFlags::objectSelection);
    EXPECT_EQ(TileManager::get(tilePos).size(), 2);
    SceneManager::unsetPauseFlag(PauseFlags::objectSelection);

    EXPECT_EQ(doCommand(args, Flags::apply), 0);
    EXPECT_EQ(SceneManager::getPauseFlags(), PauseFlags::player);
    EXPECT_EQ(SceneManager::getGameSpeed(), GameSpeed::ExtraFastForward);
    EXPECT_EQ(getGameState().scenarioTicks, scenarioTicks);
    EXPECT_EQ(TileManager::get(tilePos).size(), 1);
    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_EQ(SceneManager::getPauseFlags(), PauseFlags::player);
    EXPECT_EQ(getGameState().scenarioTicks, scenarioTicks);
    EXPECT_EQ(TileManager::get(tilePos).size(), 2);

    SceneManager::unsetPauseFlag(PauseFlags::player);
    SceneManager::setGameSpeed(originalGameSpeed);
    Scenario::getOptions().madeAnyChanges = originalMadeAnyChanges;
}

TEST_F(UndoTest, RestoresPartialChangesFromFailedGroupedCommand)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* surface = TileManager::get(tilePos).surface();
    ASSERT_NE(surface, nullptr);
    const auto originalBaseZ = surface->baseZ();

    {
        Undo::Group undoGroup;
        const auto regs = makeLandRegisters(tilePos);
        Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
        surface->setBaseZ(originalBaseZ + 4);
        surface->setClearZ(originalBaseZ + 4);
        Undo::cancel();
    }

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_EQ(TileManager::get(tilePos).surface()->baseZ(), originalBaseZ);
}

TEST_F(UndoTest, RejectsUndoWhenCommandChangedTileWasModifiedAgain)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* surface = TileManager::get(tilePos).surface();
    ASSERT_NE(surface, nullptr);

    const auto regs = makeLandRegisters(tilePos);
    Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
    surface->setBaseZ(8);
    surface->setClearZ(8);
    Undo::commit(100, ExpenditureType::Construction, { toWorldSpace(tilePos), 32 });
    surface->setBaseZ(12);
    surface->setClearZ(12);

    EXPECT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::stateChanged);
    EXPECT_EQ(TileManager::get(tilePos).surface()->baseZ(), 12);
}

TEST_F(UndoTest, RejectsUndoWhenVehicleOccupiesChangedTile)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* surface = TileManager::get(tilePos).surface();
    ASSERT_NE(surface, nullptr);

    const auto regs = makeLandRegisters(tilePos);
    Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
    surface->setBaseZ(8);
    surface->setClearZ(8);
    Undo::commit(0, ExpenditureType::Construction, {});

    auto* vehicle = EntityManager::createEntityVehicle();
    ASSERT_NE(vehicle, nullptr);
    vehicle->baseType = EntityBaseType::vehicle;
    EntityManager::moveSpatialEntry(*vehicle, { toWorldSpace(tilePos), 0 });

    EXPECT_EQ(Undo::apply(), Undo::Result::stateChanged);
    EXPECT_EQ(TileManager::get(tilePos).surface()->baseZ(), 8);
}

TEST_F(UndoTest, MapUndoPreservesTransientEffects)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* surface = TileManager::get(tilePos).surface();
    ASSERT_NE(surface, nullptr);

    const auto regs = makeLandRegisters(tilePos);
    Undo::prepare(GameCommand::raiseLand, CompanyId(0), regs, Flags::apply);
    surface->setBaseZ(8);
    surface->setClearZ(8);
    auto* constructionEffect = EntityManager::createEntityMisc();
    ASSERT_NE(constructionEffect, nullptr);
    constructionEffect->baseType = EntityBaseType::effect;
    const auto constructionEffectId = constructionEffect->id;
    Undo::commit(0, ExpenditureType::Construction, {});

    auto* paymentEffect = EntityManager::createEntityMoney();
    ASSERT_NE(paymentEffect, nullptr);
    paymentEffect->baseType = EntityBaseType::effect;
    const auto paymentEffectId = paymentEffect->id;

    ASSERT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_EQ(EntityManager::get<EntityBase>(constructionEffectId)->baseType, EntityBaseType::effect);
    EXPECT_EQ(EntityManager::get<EntityBase>(paymentEffectId)->baseType, EntityBaseType::effect);
}

TEST_F(UndoTest, RestoresRemovedTileElements)
{
    constexpr TilePos2 tilePos{ 20, 20 };
    auto* treeEntry = TileManager::insertElement<TreeElement>(toWorldSpace(tilePos), 4, 0xF);
    ASSERT_NE(treeEntry, nullptr);
    const auto expectedTree = TileManager::resolveEntry(treeEntry).rawData();
    std::array<uint8_t, kTileElementSize> expectedData{};
    std::ranges::copy(expectedTree, expectedData.begin());

    registers regs{};
    regs.ax = toWorldSpace(tilePos).x;
    regs.cx = toWorldSpace(tilePos).y;
    Undo::prepare(GameCommand::removeTree, CompanyId(0), regs, Flags::apply);
    TileManager::removeElement(*treeEntry);
    Undo::commit(0, ExpenditureType::Construction, {});

    ASSERT_TRUE(Undo::isAvailable());
    ASSERT_EQ(Undo::apply(), Undo::Result::success);
    auto tile = TileManager::get(tilePos);
    ASSERT_EQ(tile.size(), 2);
    EXPECT_TRUE(std::ranges::equal(TileManager::resolveEntry(tile[1]).rawData(), expectedData));
}

TEST_F(UndoTest, RejectsUndoWhenNewStationChangesAfterConstruction)
{
    auto& station = getGameState().stations[0];
    station = {};

    registers regs{};
    regs.ax = 20 * kTileSize;
    regs.cx = 20 * kTileSize;
    Undo::prepare(GameCommand::createTrainStation, CompanyId(0), regs, Flags::apply);
    station.name = StringId{ 1 };
    Undo::commit(0, ExpenditureType::Construction, {});
    station.cargoStats[0].quantity = 1;

    EXPECT_EQ(Undo::apply(), Undo::Result::stateChanged);
    station = {};
}

TEST_F(UndoTest, RestoresNewStationState)
{
    auto& station = getGameState().stations[0];
    station = {};

    registers regs{};
    regs.ax = 20 * kTileSize;
    regs.cx = 20 * kTileSize;
    Undo::prepare(GameCommand::createTrainStation, CompanyId(0), regs, Flags::apply);
    station.name = StringId{ 1 };
    Undo::commit(0, ExpenditureType::Construction, {});

    ASSERT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_TRUE(station.empty());
}

TEST_F(UndoTest, RestoresGroupedStationStateAfterDerivedChanges)
{
    auto& station = getGameState().stations[0];
    station = {};

    registers regs{};
    regs.ax = 20 * kTileSize;
    regs.cx = 20 * kTileSize;

    {
        Undo::Group undoGroup;
        Undo::prepare(GameCommand::createTrainStation, CompanyId(0), regs, Flags::apply);
        station.name = StringId{ 1 };
        Undo::commit(0, ExpenditureType::Construction, {});
        Undo::prepare(GameCommand::createTrainStation, CompanyId(0), regs, Flags::apply);
        station.name = StringId{ 2 };
        station.cargoStats[0].quantity = 1;
        Undo::commit(0, ExpenditureType::Construction, {});
    }

    station.labelFrame.left[0] = 1;
    station.noTilesTimeout = 1;
    station.cargoStats[0].flags |= StationCargoStatsFlags::acceptedForConsumer;
    station.cargoStats[0].industryId = IndustryId(1);
    station.var_3B0 = 1;
    station.var_3B1 = 1;

    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_TRUE(station.empty());
    station = {};
}

TEST_F(UndoTest, RestoresStateOnlyPurchaseAndRefundsCost)
{
    const auto originalVehicleType = getGameState().lastVehicleType;
    const auto laterHandedness = !getGameState().trafficHandedness;
    registers regs{};

    Undo::prepare(GameCommand::vehicleCreate, CompanyId(0), regs, Flags::apply);
    getGameState().lastVehicleType = originalVehicleType == VehicleType::aircraft ? VehicleType::ship : VehicleType::aircraft;
    Undo::commit(500, ExpenditureType::VehiclePurchases, {});
    CompanyManager::applyPaymentToCompany(CompanyId(0), 500, ExpenditureType::VehiclePurchases);
    getGameState().playerCompanies[0] = CompanyId(1);
    getGameState().trafficHandedness = laterHandedness;

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);

    EXPECT_EQ(getGameState().lastVehicleType, originalVehicleType);
    EXPECT_EQ(getGameState().trafficHandedness, laterHandedness);
    EXPECT_EQ(CompanyManager::get(CompanyId(0))->cash, currency48_t{ 10000 });
}

TEST_F(UndoTest, PreservesEntityCreatedAfterPurchase)
{
    registers regs{};
    Undo::prepare(GameCommand::vehicleCreate, CompanyId(0), regs, Flags::apply);
    auto* vehicle = EntityManager::createEntityVehicle();
    ASSERT_NE(vehicle, nullptr);
    vehicle->baseType = EntityBaseType::vehicle;
    const auto vehicleId = vehicle->id;
    Undo::commit(0, ExpenditureType::VehiclePurchases, {});

    auto* effect = EntityManager::createEntityMoney();
    ASSERT_NE(effect, nullptr);
    effect->baseType = EntityBaseType::effect;
    const auto effectId = effect->id;

    ASSERT_TRUE(Undo::isAvailable());
    EXPECT_EQ(Undo::apply(), Undo::Result::success);
    EXPECT_EQ(EntityManager::get<EntityBase>(vehicleId)->baseType, EntityBaseType::null);
    EXPECT_EQ(EntityManager::get<EntityBase>(effectId)->baseType, EntityBaseType::effect);
}
