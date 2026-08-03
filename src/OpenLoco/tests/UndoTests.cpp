#include "Economy/Expenditures.h"
#include "Entities/EntityManager.h"
#include "GameCommands/Undo.h"
#include "GameState.h"
#include "Map/SurfaceElement.h"
#include "Map/TileManager.h"
#include "Map/TreeElement.h"
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
