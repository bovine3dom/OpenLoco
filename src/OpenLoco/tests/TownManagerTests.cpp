#include "CargoDist/CargoDist.h"
#include "Config.h"
#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Map/TileManager.h"
#include "World/TownManager.h"
#include "World/TownGrowth.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::World;

namespace
{
    class TownManagerTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            TileManager::initialise();
            for (auto& town : getGameState().towns)
            {
                town = {};
                town.name = StringIds::null;
            }
            TownManager::reset();
        }

        void TearDown() override
        {
            TownManager::reset();
        }

        static Town& addTown(TownId id, Pos2 pos)
        {
            auto& town = getGameState().towns[enumValue(id)];
            town.name = StringIds::empty;
            town.x = pos.x;
            town.y = pos.y;
            town.numBuildings = 0;
            std::fill(std::begin(town.amenityCounts), std::end(town.amenityCounts), 0);
            return town;
        }
    };
}

TEST_F(TownManagerTest, ClosestTownUsesStableTownIdOrder)
{
    EXPECT_EQ(TownManager::getClosestTown({ 32, 32 }), std::nullopt);

    addTown(TownId{ 0 }, { 0, 32 });
    addTown(TownId{ 1 }, { 64, 32 });

    EXPECT_EQ(TownManager::getClosestTown({ 32, 32 }), TownId{ 0 });
    EXPECT_EQ(TownManager::getClosestTown({ 63, 32 }), TownId{ 1 });

}

TEST_F(TownManagerTest, BuildingCountRemainsExactAndLegacyCountIsClamped)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });

    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, std::numeric_limits<int16_t>::max()), nullptr);
    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, 1), nullptr);

    EXPECT_EQ(TownManager::getBuildingCount(kTownId), static_cast<uint32_t>(std::numeric_limits<int16_t>::max()) + 1);
    EXPECT_EQ(town.numBuildings, std::numeric_limits<int16_t>::max());

    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, std::numeric_limits<int16_t>::min()), nullptr);
    EXPECT_EQ(TownManager::getBuildingCount(kTownId), 0U);
    EXPECT_EQ(town.numBuildings, 0);

    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, -1), nullptr);
    EXPECT_EQ(TownManager::getBuildingCount(kTownId), 0U);
    EXPECT_EQ(town.numBuildings, 0);
}

TEST_F(TownManagerTest, AmenityCountRemainsExactAndLegacyCountIsClamped)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });

    TownManager::adjustAmenityCount(kTownId, 0, std::numeric_limits<uint8_t>::max());
    TownManager::adjustAmenityCount(kTownId, 0, 1);

    EXPECT_EQ(TownManager::getAmenityCount(kTownId, 0), static_cast<uint32_t>(std::numeric_limits<uint8_t>::max()) + 1);
    EXPECT_EQ(town.amenityCounts[0], std::numeric_limits<uint8_t>::max());

    TownManager::adjustAmenityCount(kTownId, 0, -300);
    EXPECT_EQ(TownManager::getAmenityCount(kTownId, 0), 0U);
    EXPECT_EQ(town.amenityCounts[0], 0);
}

TEST_F(TownManagerTest, DensityUsesLocalAndTransportPressureWithinTownMaturity)
{
    EXPECT_EQ(TownManager::calculateTownDensity(10000, 10000, 499), 0);
    EXPECT_EQ(TownManager::calculateTownDensity(256, 0, 500), 1);
    EXPECT_EQ(TownManager::calculateTownDensity(0, 32, 500), 1);
    EXPECT_EQ(TownManager::calculateTownDensity(1024, 0, 2500), 2);
    EXPECT_EQ(TownManager::calculateTownDensity(0, 128, 2500), 2);
    EXPECT_EQ(TownManager::calculateTownDensity(3072, 0, 10000), 3);
    EXPECT_EQ(TownManager::calculateTownDensity(0, 384, 10000), 3);
}

TEST_F(TownManagerTest, TownSizeUsesExpandedPopulationThresholds)
{
    auto& town = addTown(TownId{ 0 }, { 32, 32 });
    town.size = TownSize::hamlet;

    town.populationCapacity = 499;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::hamlet);
    town.populationCapacity = 500;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::village);
    town.populationCapacity = 2500;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::town);
    town.populationCapacity = 10000;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::city);
    town.populationCapacity = 25000;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::metropolis);

    town.populationCapacity = 399;
    town.recalculateSize();
    EXPECT_EQ(town.size, TownSize::city);
}

TEST_F(TownManagerTest, GrowthStationCandidatesMustBeActiveUsefulAndLocal)
{
    constexpr TownId kTown{ 1 };
    EXPECT_TRUE(TownGrowth::isGrowthStationCandidate(kTown, kTown, 1, StationFlags::none, 16));
    EXPECT_FALSE(TownGrowth::isGrowthStationCandidate(kTown, TownId{ 2 }, 1, StationFlags::none, 16));
    EXPECT_FALSE(TownGrowth::isGrowthStationCandidate(kTown, kTown, 0, StationFlags::none, 16));
    EXPECT_FALSE(TownGrowth::isGrowthStationCandidate(kTown, kTown, 1, StationFlags::flag_5, 16));
    EXPECT_FALSE(TownGrowth::isGrowthStationCandidate(kTown, kTown, 1, StationFlags::none, 15));
}

TEST_F(TownManagerTest, GrowthNucleusWeightsAreBoundedAndDeterministic)
{
    EXPECT_FALSE(TownGrowth::shouldUseStation(16, 0));
    EXPECT_TRUE(TownGrowth::shouldUseStation(16, std::numeric_limits<uint16_t>::max()));
    EXPECT_FALSE(TownGrowth::shouldUseStation(128, 32767));
    EXPECT_TRUE(TownGrowth::shouldUseStation(128, 32768));

    const std::array<uint32_t, 2> weighted = { 16, 48 };
    EXPECT_EQ(TownGrowth::selectStation(weighted, 16383), 0);
    EXPECT_EQ(TownGrowth::selectStation(weighted, 16384), 1);

    const std::array<uint32_t, 2> capped = { 4096, 10000 };
    EXPECT_EQ(TownGrowth::selectStation(capped, 32767), 0);
    EXPECT_EQ(TownGrowth::selectStation(capped, 32768), 1);
    EXPECT_EQ(TownGrowth::selectStation({}, 0), std::nullopt);
}

TEST_F(TownManagerTest, DensityUsesOnlyActiveNearbyTownStations)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });
    town.populationCapacity = 10000;

    auto& station = getGameState().stations[0];
    const auto previousStation = station;
    const auto previousAccessibility = CargoDist::getState().stationAccessibility.find(station.id());
    const auto hadPreviousAccessibility = previousAccessibility != CargoDist::getState().stationAccessibility.end();
    const auto previousAccessibilityValue = hadPreviousAccessibility ? previousAccessibility->second : 0;
    station = {};
    station.name = StringIds::empty;
    station.town = kTownId;
    station.x = 32;
    station.y = 32;
    station.stationTileSize = 1;
    CargoDist::getState().stationAccessibility[station.id()] = 128;

    EXPECT_EQ(TownManager::getTownDensity(kTownId, { 32, 32 }), 2);
    station.stationTileSize = 0;
    EXPECT_EQ(TownManager::getTownDensity(kTownId, { 32, 32 }), 0);

    if (hadPreviousAccessibility)
    {
        CargoDist::getState().stationAccessibility[station.id()] = previousAccessibilityValue;
    }
    else
    {
        CargoDist::getState().stationAccessibility.erase(station.id());
    }
    station = previousStation;
}

TEST_F(TownManagerTest, RoadTraversalVisitsEachPositionAndDirectionOnce)
{
    const World::Pos3 pos{ 32, 64, 8 };
    const std::array<TownGrowth::RoadTraversalState, 1> visited = { TownGrowth::RoadTraversalState{ pos, 3, false, 0 } };

    EXPECT_TRUE(TownGrowth::wasRoadStateVisited(visited, pos, 3));
    EXPECT_FALSE(TownGrowth::wasRoadStateVisited(visited, pos, 4));
    EXPECT_FALSE(TownGrowth::wasRoadStateVisited(visited, { 64, 64, 8 }, 3));

    auto bridgeVariant = visited;
    bridgeVariant[0].isBridge = true;
    EXPECT_TRUE(TownGrowth::wasRoadStateVisited(bridgeVariant, pos, 3));
}

TEST(TownGrowthDiagnostics, ClassifiesLastUpdateOutcome)
{
    TownGrowth::GrowthDiagnostics diagnostics{};
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::disabled);

    diagnostics.kind = TownGrowth::UpdateKind::maintenance;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::maintenance);

    diagnostics.initialRoadsBuilt = 1;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::initialRoadBuilt);
    diagnostics.initialRoadsBuilt = 0;

    diagnostics.kind = TownGrowth::UpdateKind::construction;
    diagnostics.growthCalls = 2;
    diagnostics.noRoadCalls = 2;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::noRoad);

    diagnostics.noRoadCalls = 0;
    diagnostics.noIdealRoadCalls = 2;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::noRoadType);

    diagnostics.noIdealRoadCalls = 0;
    diagnostics.buildingSitesAttempted = 1;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::noSuitableSite);

    diagnostics.initialRoadsBuilt = 1;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::initialRoadBuilt);

    diagnostics.buildingsConstructed = 1;
    EXPECT_EQ(TownGrowth::getOutcome(diagnostics), TownGrowth::Outcome::grew);
}

TEST_F(TownManagerTest, DisabledGrowthRecordsWhyTheTownWasSkipped)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });
    town.buildSpeed = 3;
    const auto wasDisabled = Config::get().townGrowthDisabled;
    Config::get().townGrowthDisabled = true;

    town.tick();

    Config::get().townGrowthDisabled = wasDisabled;
    const auto* diagnostics = TownGrowth::getLastGrowth(kTownId);
    ASSERT_NE(diagnostics, nullptr);
    EXPECT_EQ(diagnostics->buildSpeed, 3);
    EXPECT_EQ(TownGrowth::getOutcome(*diagnostics), TownGrowth::Outcome::disabled);

    town.name = StringIds::null;
    EXPECT_EQ(TownGrowth::getLastGrowth(kTownId), nullptr);
}

TEST_F(TownManagerTest, RuntimeMetricsRebuildPreservesPopulationAndCapacity)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });
    town.population = 123;
    town.populationCapacity = 456;
    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, 1), nullptr);
    TownManager::adjustAmenityCount(kTownId, 0, 1);

    TownManager::rebuildRuntimeMetrics();

    EXPECT_EQ(TownManager::getBuildingCount(kTownId), 0U);
    EXPECT_EQ(TownManager::getAmenityCount(kTownId, 0), 0U);
    EXPECT_EQ(town.numBuildings, 0);
    EXPECT_EQ(town.amenityCounts[0], 0);
    EXPECT_EQ(town.population, 123U);
    EXPECT_EQ(town.populationCapacity, 456U);

    TownManager::resetBuildingsInfluence();
    EXPECT_EQ(town.population, 0U);
    EXPECT_EQ(town.populationCapacity, 0U);
}

TEST_F(TownManagerTest, RuntimeMetricsRebuildClearsRemovedTownSlotForReuse)
{
    constexpr TownId kTownId{ 0 };
    auto& town = addTown(kTownId, { 32, 32 });
    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, 1), nullptr);
    TownManager::adjustAmenityCount(kTownId, 0, 1);

    town.name = StringIds::null;
    TownManager::resetBuildingsInfluence();
    addTown(kTownId, { 32, 32 });

    EXPECT_EQ(TownManager::getBuildingCount(kTownId), 0U);
    EXPECT_EQ(TownManager::getAmenityCount(kTownId, 0), 0U);
    ASSERT_NE(TownManager::updateTownInfo({ 32, 32 }, 0, 0, 0, 1), nullptr);
    TownManager::adjustAmenityCount(kTownId, 0, 1);
    EXPECT_EQ(TownManager::getBuildingCount(kTownId), 1U);
    EXPECT_EQ(TownManager::getAmenityCount(kTownId, 0), 1U);
}
