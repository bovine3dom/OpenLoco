#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Map/StationElement.h"
#include "Map/TileManager.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include <OpenLoco/CargoDist/CargoDist.h>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::World;

namespace
{
    class StationManagerTest : public ::testing::Test
    {
    protected:
        static constexpr CompanyId kOwner{ 0 };
        static constexpr StationId kStationId{ 0 };
        static constexpr TilePos2 kStationTile{ 20, 20 };
        static constexpr uint8_t kStationBaseZ = 4;

        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            CargoDist::reset();
            TileManager::initialise();
            resetStations();
            setCatchmentDisplay(nullptr, CatchmentFlags::flag_0);
            setCatchmentDisplay(nullptr, CatchmentFlags::flag_1);

            auto& station = getGameState().stations[enumValue(kStationId)];
            station.name = StringIds::new_station;
            station.owner = kOwner;
            station.stationTileSize = 1;

            auto* entry = TileManager::insertElement<StationElement>(toWorldSpace(kStationTile), kStationBaseZ, 0xF);
            ASSERT_NE(entry, nullptr);
            auto& stationElement = entry->get<StationElement>();
            stationElement.setClearZ(kStationBaseZ + 4);
            stationElement.setOwner(kOwner);
            stationElement.setStationId(kStationId);
            stationElement.setStationType(StationType::trainStation);
        }

        void TearDown() override
        {
            CargoDist::reset();
            setCatchmentDisplay(nullptr, CatchmentFlags::flag_0);
            setCatchmentDisplay(nullptr, CatchmentFlags::flag_1);
            resetStations();
        }

        static void resetStations()
        {
            for (auto& station : getGameState().stations)
            {
                station = {};
            }
        }

        static Pos3 stationPosition()
        {
            return Pos3{ toWorldSpace(kStationTile), kStationBaseZ * kSmallZStep };
        }
    };
}

TEST_F(StationManagerTest, FindsStationThreeTilesAway)
{
    const auto result = StationManager::findNearbyStation(stationPosition() + Pos3{ 3 * kTileSize, 0, 0 }, kOwner);

    EXPECT_EQ(result.id, kStationId);
    EXPECT_TRUE(result.isPhysicallyAttached);
}

TEST_F(StationManagerTest, DoesNotFindStationFourTilesAway)
{
    const auto result = StationManager::findNearbyStation(stationPosition() + Pos3{ 4 * kTileSize, 0, 0 }, kOwner);

    EXPECT_EQ(result.id, StationId::null);
    EXPECT_FALSE(result.isPhysicallyAttached);
}

TEST_F(StationManagerTest, FindsStationAcrossThreeTileHeightDifference)
{
    const auto result = StationManager::findNearbyStation(stationPosition() + Pos3{ 0, 0, 3 * kTileSize }, kOwner);

    EXPECT_EQ(result.id, kStationId);
    EXPECT_TRUE(result.isPhysicallyAttached);
}

TEST_F(StationManagerTest, CatchmentDisplayClearsPreviousBoundsAtMapEdges)
{
    constexpr TilePos2 firstTile{ 0, 0 };
    constexpr TilePos2 lastTile{ kMapColumns - 1, kMapRows - 1 };

    sub_491BF5(toWorldSpace(firstTile), CatchmentFlags::flag_0);
    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(firstTile)));
    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(TilePos2{ 4, 4 })));
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(TilePos2{ 5, 0 })));

    setCatchmentDisplay(nullptr, CatchmentFlags::flag_0);
    sub_491BF5(toWorldSpace(lastTile), CatchmentFlags::flag_0);
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(firstTile)));
    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(lastTile)));
    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(lastTile - TilePos2{ 4, 4 })));
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(lastTile - TilePos2{ 5, 0 })));
}

TEST_F(StationManagerTest, CatchmentRegionsRemainSparseAndFlagsIndependent)
{
    constexpr TilePos2 firstDisplayTile{ 20, 20 };
    constexpr TilePos2 secondDisplayTile{ 40, 20 };

    sub_491BF5(toWorldSpace(firstDisplayTile), CatchmentFlags::flag_0);
    sub_491BF5(toWorldSpace(secondDisplayTile), CatchmentFlags::flag_0);
    sub_491BF5(toWorldSpace(firstDisplayTile), CatchmentFlags::flag_1);
    setCatchmentDisplay(nullptr, CatchmentFlags::flag_1);

    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(firstDisplayTile)));
    EXPECT_TRUE(isWithinCatchmentDisplay(toWorldSpace(secondDisplayTile)));
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(TilePos2{ 30, 20 })));

    setCatchmentDisplay(nullptr, CatchmentFlags::flag_0);
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(firstDisplayTile)));
    EXPECT_FALSE(isWithinCatchmentDisplay(toWorldSpace(secondDisplayTile)));
}

TEST_F(StationManagerTest, OriginCargoAccumulatesPaymentAge)
{
    auto& station = getGameState().stations[enumValue(kStationId)];
    station.stationTileSize = 0;
    auto& cargo = station.cargoStats[0];
    cargo.quantity = 10;
    cargo.origin = kStationId;
    cargo.enrouteAge = 4;
    cargo.rating = 200;

    station.updateCargo();

    EXPECT_EQ(cargo.quantity, 10);
    EXPECT_EQ(cargo.enrouteAge, 5);
}

TEST_F(StationManagerTest, ProducedCargoPreservesWeightedPaymentAge)
{
    auto& station = getGameState().stations[enumValue(kStationId)];
    station.stationTileSize = 0;
    auto& cargo = station.cargoStats[0];
    cargo.quantity = 10;
    cargo.origin = kStationId;
    cargo.enrouteAge = 6;

    station.deliverCargoToStation(0, 10, false);

    EXPECT_EQ(cargo.quantity, 20);
    EXPECT_EQ(cargo.enrouteAge, 3);
}

TEST_F(StationManagerTest, CargoRatingUsesServiceAgeNotPaymentAge)
{
    auto& station = getGameState().stations[enumValue(kStationId)];
    station.flags |= StationFlags::flag_7;
    auto& cargo = station.cargoStats[0];
    cargo.vehicleAge = 10;
    cargo.age = 255;
    EXPECT_EQ(station.calculateCargoRating(cargo), 0);

    cargo.age = 5;
    const auto servicedRating = station.calculateCargoRating(cargo);
    EXPECT_GT(servicedRating, 0);
    cargo.enrouteAge = 255;
    EXPECT_EQ(station.calculateCargoRating(cargo), servicedRating);
}

TEST(StationCatchmentEstimate, CalculatesExpectedMonthlyBuildingProduction)
{
    EXPECT_EQ(getBuildingMonthlyProductionEstimateScaled(0, false), 0U);
    EXPECT_EQ(getBuildingMonthlyProductionEstimateScaled(8, false), 360U);
    EXPECT_EQ(getBuildingMonthlyProductionEstimateScaled(16, false), 1200U);
    EXPECT_EQ(getBuildingMonthlyProductionEstimateScaled(16, true), 720U);

    EXPECT_EQ(roundMonthlyProductionEstimate(getBuildingMonthlyProductionEstimateScaled(8, false)), 1U);
    EXPECT_EQ(roundMonthlyProductionEstimate(getBuildingMonthlyProductionEstimateScaled(16, false)), 2U);
    EXPECT_EQ(roundMonthlyProductionEstimate(getBuildingMonthlyProductionEstimateScaled(16, true)), 1U);
}

TEST(StationCatchmentEstimate, RoundsAggregateProductionOnce)
{
    EXPECT_EQ(roundMonthlyProductionEstimate(kMonthlyProductionEstimateDenominator / 2 - 1), 0U);
    EXPECT_EQ(roundMonthlyProductionEstimate(kMonthlyProductionEstimateDenominator / 2), 1U);
    EXPECT_EQ(roundMonthlyProductionEstimate(kMonthlyProductionEstimateDenominator), 1U);
}
