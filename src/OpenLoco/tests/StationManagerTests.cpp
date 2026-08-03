#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Map/StationElement.h"
#include "Map/TileManager.h"
#include "World/StationManager.h"
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
            TileManager::initialise();
            resetStations();

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
