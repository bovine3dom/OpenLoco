#include <OpenLoco/Map/SurfaceElement.h>
#include <OpenLoco/Map/TileManager.h>
#include <OpenLoco/Map/TrackElement.h>
#include <OpenLoco/Vehicles/WaterPathfinding.h>
#include <array>
#include <gtest/gtest.h>
#include <vector>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles::WaterPathfinding;
using namespace OpenLoco::World;

namespace
{
    constexpr MicroZ kWaterLevel = 5;
    constexpr SmallZ kWaterSurfaceZ = kWaterLevel * kMicroToSmallZStep;
    constexpr uint32_t kCardinalStepCost = 1000;
    constexpr uint32_t kDiagonalStepCost = 1414;
    constexpr uint8_t kWest = 0;
    constexpr uint8_t kSouthWest = 8;
    constexpr uint8_t kSouth = 16;
    constexpr uint8_t kSouthEast = 24;
    constexpr uint8_t kEast = 32;
    constexpr uint8_t kNorthEast = 40;
    constexpr uint8_t kNorth = 48;
    constexpr uint8_t kNorthWest = 56;

    class WaterPathfindingTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            TileManager::allocateMapElements();
        }

        void SetUp() override
        {
            TileManager::initialise();
        }

        static void setWater(const TilePos2 pos, const MicroZ level = kWaterLevel)
        {
            auto* surface = TileManager::get(pos).surface();
            ASSERT_NE(surface, nullptr);
            surface->setWater(level);
        }

        static void setLand(const TilePos2 pos, const SmallZ baseZ = kWaterSurfaceZ, const uint8_t slope = SurfaceSlope::flat)
        {
            auto* surface = TileManager::get(pos).surface();
            ASSERT_NE(surface, nullptr);
            surface->setBaseZ(baseZ);
            surface->setClearZ(baseZ);
            surface->setSlope(slope);
            surface->setWater(0);
        }

        static void setHorizontalWater(const tile_coord_t x1, const tile_coord_t x2, const tile_coord_t y)
        {
            for (auto x = x1; x <= x2; ++x)
            {
                setWater({ x, y });
            }
        }

        static void setVerticalWater(const tile_coord_t x, const tile_coord_t y1, const tile_coord_t y2)
        {
            for (auto y = y1; y <= y2; ++y)
            {
                setWater({ x, y });
            }
        }

        static void setWaterRectangle(const TilePos2 topLeft, const TilePos2 bottomRight)
        {
            for (auto y = topLeft.y; y <= bottomRight.y; ++y)
            {
                setHorizontalWater(topLeft.x, bottomRight.x, y);
            }
        }

        static uint8_t getStepYaw(const TilePos2 from, const TilePos2 to)
        {
            const auto delta = to - from;
            if (delta.x < 0)
            {
                return delta.y < 0 ? kNorthWest : delta.y > 0 ? kSouthWest
                                                              : kWest;
            }
            if (delta.x > 0)
            {
                return delta.y < 0 ? kNorthEast : delta.y > 0 ? kSouthEast
                                                              : kEast;
            }
            return delta.y < 0 ? kNorth : kSouth;
        }

        static bool isAdjacent(const TilePos2 lhs, const TilePos2 rhs)
        {
            return Math::Vector::chebyshevDistance2D(lhs, rhs) == 1;
        }

        static void expectPosition(const TilePos2 actual, const TilePos2 expected)
        {
            EXPECT_EQ(actual.x, expected.x);
            EXPECT_EQ(actual.y, expected.y);
        }
    };
}

TEST_F(WaterPathfindingTest, FindsRouteThatInitiallyMovesAwayFromTarget)
{
    constexpr TilePos2 start{ 20, 20 };
    constexpr TilePos2 goal{ 40, 20 };

    setHorizontalWater(10, 20, 20);
    setVerticalWater(10, 20, 30);
    setHorizontalWater(10, 40, 30);
    setVerticalWater(40, 20, 30);

    const std::array goals = { goal };
    auto current = start;
    auto direction = kEast;
    uint32_t steps = 0;
    while (steps < 100)
    {
        const auto result = findNextTile(current, kWaterLevel, goals, {}, direction);
        if (result.status == RouteStatus::arrived)
        {
            break;
        }
        ASSERT_EQ(result.status, RouteStatus::found);
        if (steps == 0)
        {
            expectPosition(result.nextTile, { 19, 20 });
            EXPECT_EQ(result.remainingDistance, 60U * kCardinalStepCost);
        }
        direction = getStepYaw(current, result.nextTile);
        current = result.nextTile;
        ++steps;
    }

    expectPosition(current, goal);
    EXPECT_EQ(steps, 60U);
}

TEST_F(WaterPathfindingTest, ReportsDisconnectedWaterAsUnreachable)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 20, 20 };
    setWater(start);
    setWater(goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast);

    EXPECT_EQ(result.status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, AmphibiousRouteCrossesLandThatBlocksOrdinaryShips)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setWater(start);
    setLand(middle);
    setWater(goal);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::unreachable);
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, middle);
}

TEST_F(WaterPathfindingTest, DoesNotShareRouteCacheBetweenNavigationModes)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setWater(start);
    setLand(middle);
    setWater(goal);
    const std::array goals = { goal };

    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious).status, RouteStatus::found);
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, AmphibiousRouteFollowsContinuousSlope)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 slope{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setWater(start);
    setLand(slope, kWaterSurfaceZ, SurfaceSlope::SideUp::northeast);
    setLand(goal, kWaterSurfaceZ + kMicroToSmallZStep);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, slope);
}

TEST_F(WaterPathfindingTest, AmphibiousRouteFollowsTerrainAboveWater)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 slope{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setWater(start);
    setLand(slope, kWaterSurfaceZ, SurfaceSlope::SideUp::northeast);
    setWater(slope);
    setLand(goal, kWaterSurfaceZ + kMicroToSmallZStep);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, slope);
}

TEST_F(WaterPathfindingTest, AmphibiousSlopeSpeedLimitsAreDirectional)
{
    constexpr TilePos2 low{ 10, 10 };
    constexpr TilePos2 slope{ 11, 10 };
    constexpr TilePos2 high{ 12, 10 };
    setWater(low);
    setLand(slope, kWaterSurfaceZ, SurfaceSlope::SideUp::northeast);
    setLand(high, kWaterSurfaceZ + kMicroToSmallZStep);
    const auto lowCentre = toWorldSpace(low) + Pos2{ 16, 16 };
    const auto slopeCentre = toWorldSpace(slope) + Pos2{ 16, 16 };
    const auto highCentre = toWorldSpace(high) + Pos2{ 16, 16 };

    EXPECT_EQ(getAmphibiousSpeedLimit(lowCentre, slopeCentre, Speed16{ 70 }), Speed16{ 20 });
    EXPECT_EQ(getAmphibiousSpeedLimit(highCentre, slopeCentre, Speed16{ 70 }), Speed16{ 25 });
    EXPECT_EQ(getAmphibiousSpeedLimit(lowCentre, lowCentre, Speed16{ 70 }), Speed16{ 70 });
    EXPECT_EQ(getAmphibiousSpeedLimit(lowCentre, slopeCentre, Speed16{ 15 }), Speed16{ 15 });
}

TEST_F(WaterPathfindingTest, AmphibiousRouteOnlyAllowsSubmergedDoubleHeightTerrain)
{
    constexpr TilePos2 slope{ 11, 10 };
    constexpr auto doubleHeightSlope = SurfaceSlope::CornerDown::west | SurfaceSlope::doubleHeight;
    setLand(slope, kWaterSurfaceZ, doubleHeightSlope);

    EXPECT_FALSE(isNavigable(slope, kWaterLevel, NavigationMode::amphibious));
    setLand(slope, 4, doubleHeightSlope);
    setWater(slope);
    EXPECT_TRUE(isNavigable(slope, kWaterLevel, NavigationMode::amphibious));
}

TEST_F(WaterPathfindingTest, AmphibiousRouteRequiresClearanceAboveExposedFloodedTerrain)
{
    constexpr TilePos2 slope{ 11, 10 };
    setLand(slope, kWaterSurfaceZ, SurfaceSlope::SideUp::northeast);
    setWater(slope);
    ASSERT_TRUE(isNavigable(slope, kWaterLevel, NavigationMode::amphibious));

    ASSERT_NE(TileManager::insertElement<TrackElement>(toWorldSpace(slope), kWaterSurfaceZ + kMicroToSmallZStep, 0xF), nullptr);

    EXPECT_FALSE(isNavigable(slope, kWaterLevel, NavigationMode::amphibious));
}

TEST_F(WaterPathfindingTest, AmphibiousRouteDoesNotCutAcrossDiscontinuousCorner)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 horizontal{ 11, 10 };
    constexpr TilePos2 vertical{ 10, 11 };
    constexpr TilePos2 goal{ 11, 11 };
    setLand(start, kWaterSurfaceZ, SurfaceSlope::CornerUp::north);
    setLand(horizontal, kWaterSurfaceZ, SurfaceSlope::CornerDown::west);
    setLand(vertical, kWaterSurfaceZ, SurfaceSlope::CornerUp::east);
    setLand(goal, kWaterSurfaceZ, SurfaceSlope::CornerUp::south);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast, NavigationMode::amphibious);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, horizontal);
}

TEST_F(WaterPathfindingTest, AmphibiousRouteDoesNotCrossCliff)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 11, 10 };
    setWater(start);
    setLand(goal, kWaterSurfaceZ + kMicroToSmallZStep);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious).status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, AmphibiousRouteDoesNotCrossSurfaceInfrastructure)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setWater(start);
    setLand(middle);
    setWater(goal);
    ASSERT_NE(TileManager::insertElement<TrackElement>(toWorldSpace(middle), kWaterSurfaceZ, 0xF), nullptr);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast, NavigationMode::amphibious).status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, DoesNotCrossDifferentWaterLevels)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 11, 10 };
    setWater(start);
    setWater(goal, kWaterLevel + 1);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast);

    EXPECT_EQ(result.status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, SelectsNearestGoal)
{
    constexpr TilePos2 start{ 10, 10 };
    setHorizontalWater(10, 30, 10);

    const std::array goals = { TilePos2{ 30, 10 }, TilePos2{ 15, 10 } };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.goal, 1U);
    EXPECT_EQ(result.remainingDistance, 5U * kCardinalStepCost);
    expectPosition(result.nextTile, { 11, 10 });
}

TEST_F(WaterPathfindingTest, ReportsArrivalAtGoal)
{
    constexpr TilePos2 goal{ 10, 10 };
    setWater(goal);

    const std::array goals = { goal };
    const auto result = findNextTile(goal, kWaterLevel, goals, {}, kEast);

    EXPECT_EQ(result.status, RouteStatus::arrived);
    EXPECT_EQ(result.goal, 0U);
    EXPECT_EQ(result.remainingDistance, 0U);
}

TEST_F(WaterPathfindingTest, UsesDiagonalRouteAcrossOpenWater)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 13, 13 };
    setWaterRectangle(start, goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 3U * kDiagonalStepCost);
    expectPosition(result.nextTile, { 11, 11 });
}

TEST_F(WaterPathfindingTest, SelectsGoalByGeometricRouteCost)
{
    constexpr TilePos2 start{ 10, 10 };
    setWaterRectangle(start, { 16, 15 });

    const std::array goals = { TilePos2{ 15, 15 }, TilePos2{ 16, 10 } };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kEast);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.goal, 1U);
    EXPECT_EQ(result.remainingDistance, 6U * kCardinalStepCost);
    expectPosition(result.nextTile, { 11, 10 });
}

TEST_F(WaterPathfindingTest, DoesNotConnectWaterAcrossDryCorner)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 11, 11 };
    setWater(start);
    setWater(goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast);

    EXPECT_EQ(result.status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, DoesNotCutPastDrySideTile)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 11, 11 };
    setWater(start);
    setWater({ 11, 10 });
    setWater(goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, { 11, 10 });
}

TEST_F(WaterPathfindingTest, SelectsFirstGoalWhenCostsAreEqual)
{
    constexpr TilePos2 start{ 11, 10 };
    setHorizontalWater(10, 12, 10);

    const std::array westFirst = { TilePos2{ 10, 10 }, TilePos2{ 12, 10 } };
    const std::array eastFirst = { TilePos2{ 12, 10 }, TilePos2{ 10, 10 } };
    const auto westResult = findNextTile(start, kWaterLevel, westFirst, {}, kEast);
    const auto eastResult = findNextTile(start, kWaterLevel, eastFirst, {}, kWest);

    EXPECT_EQ(westResult.goal, 0U);
    expectPosition(westResult.nextTile, westFirst[0]);
    EXPECT_EQ(eastResult.goal, 0U);
    expectPosition(eastResult.nextTile, eastFirst[0]);
}

TEST_F(WaterPathfindingTest, CachedRouteExpandsAfterArrivalQuery)
{
    constexpr TilePos2 goal{ 10, 10 };
    constexpr TilePos2 start{ 11, 10 };
    setHorizontalWater(goal.x, start.x, goal.y);
    const std::array goals = { goal };

    EXPECT_EQ(findNextTile(goal, kWaterLevel, goals, {}, kWest).status, RouteStatus::arrived);
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kWest);

    EXPECT_EQ(result.status, RouteStatus::found);
    expectPosition(result.nextTile, goal);
}

TEST_F(WaterPathfindingTest, PrefersCurrentHeadingBetweenEqualRoutes)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 goal{ 12, 12 };
    constexpr std::array waterTiles = {
        start,
        TilePos2{ 11, 10 },
        TilePos2{ 12, 10 },
        TilePos2{ 12, 11 },
        goal,
        TilePos2{ 10, 11 },
        TilePos2{ 10, 12 },
        TilePos2{ 11, 12 },
    };
    for (const auto tile : waterTiles)
    {
        setWater(tile);
    }
    const std::array goals = { goal };

    const auto eastResult = findNextTile(start, kWaterLevel, goals, {}, kEast);
    const auto southResult = findNextTile(start, kWaterLevel, goals, {}, kSouth);

    expectPosition(eastResult.nextTile, { 11, 10 });
    expectPosition(southResult.nextTile, { 10, 11 });
}

TEST_F(WaterPathfindingTest, UsesAnotherDescendingRouteAroundTemporaryBlockage)
{
    for (tile_coord_t y = 10; y <= 11; ++y)
    {
        setHorizontalWater(10, 11, y);
    }
    const std::array goals = { TilePos2{ 11, 11 } };
    const std::array eastBlocked = { TilePos2{ 11, 10 } };
    const std::array bothBlocked = { TilePos2{ 11, 10 }, TilePos2{ 10, 11 } };

    const auto alternate = findNextTile({ 10, 10 }, kWaterLevel, goals, eastBlocked, kEast);
    const auto waiting = findNextTile({ 10, 10 }, kWaterLevel, goals, bothBlocked, kEast);

    EXPECT_EQ(alternate.status, RouteStatus::found);
    expectPosition(alternate.nextTile, { 10, 11 });
    EXPECT_EQ(waiting.status, RouteStatus::temporarilyBlocked);
}

TEST_F(WaterPathfindingTest, TakesLongerDetourAroundTemporaryBlockage)
{
    for (tile_coord_t y = 10; y <= 11; ++y)
    {
        setHorizontalWater(10, 13, y);
    }
    const std::array goals = { TilePos2{ 13, 10 } };
    const std::array blocked = { TilePos2{ 11, 10 } };

    auto current = TilePos2{ 10, 10 };
    auto direction = kEast;
    for (auto step = 0; step < 10 && current != goals[0]; ++step)
    {
        const auto isBlockerAdjacent = isAdjacent(current, blocked[0]);
        const auto activeBlockers = isBlockerAdjacent ? std::span<const TilePos2>{ blocked } : std::span<const TilePos2>{};
        const auto result = findNextTile(current, kWaterLevel, goals, activeBlockers, direction);
        ASSERT_EQ(result.status, RouteStatus::found);
        if (step == 0)
        {
            expectPosition(result.nextTile, { 10, 11 });
        }
        direction = getStepYaw(current, result.nextTile);
        current = result.nextTile;
    }
    expectPosition(current, goals[0]);
}

TEST_F(WaterPathfindingTest, InvalidatesCachedRouteWhenObstacleChanges)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setHorizontalWater(10, 12, 10);
    const std::array goals = { goal };

    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::found);

    auto* obstacle = TileManager::insertElement<TrackElement>(toWorldSpace(middle), kWaterLevel * kMicroToSmallZStep, 0xF);
    ASSERT_NE(obstacle, nullptr);
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::unreachable);

    TileManager::removeElement(*obstacle);
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::found);
}

TEST_F(WaterPathfindingTest, AllowsSufficientClearanceAboveWater)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setHorizontalWater(10, 12, 10);

    const auto obstacleBaseZ = static_cast<SmallZ>((kWaterLevel + 1) * kMicroToSmallZStep);
    ASSERT_NE(TileManager::insertElement<TrackElement>(toWorldSpace(middle), obstacleBaseZ, 0xF), nullptr);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::found);
}

TEST_F(WaterPathfindingTest, IgnoresInfrastructureBelowSurface)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setHorizontalWater(10, 12, 10);

    ASSERT_NE(TileManager::insertElement<TrackElement>(toWorldSpace(middle), 0, 0xF), nullptr);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::found);
}

TEST_F(WaterPathfindingTest, RevalidatesRouteAfterAiElementBecomesReal)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 middle{ 11, 10 };
    constexpr TilePos2 goal{ 12, 10 };
    setHorizontalWater(10, 12, 10);

    auto* obstacle = TileManager::insertElement<TrackElement>(toWorldSpace(middle), kWaterLevel * kMicroToSmallZStep, 0xF);
    ASSERT_NE(obstacle, nullptr);
    obstacle->setAiAllocated(true);

    const std::array goals = { goal };
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::found);

    obstacle->setAiAllocated(false);
    EXPECT_EQ(findNextTile(start, kWaterLevel, goals, {}, kEast).status, RouteStatus::unreachable);
}

TEST_F(WaterPathfindingTest, RevalidatesDiagonalAfterSideElementBecomesReal)
{
    constexpr TilePos2 start{ 10, 10 };
    constexpr TilePos2 side{ 11, 10 };
    constexpr TilePos2 goal{ 11, 11 };
    setWaterRectangle(start, goal);

    auto* obstacle = TileManager::insertElement<TrackElement>(toWorldSpace(side), kWaterLevel * kMicroToSmallZStep, 0xF);
    ASSERT_NE(obstacle, nullptr);
    obstacle->setAiAllocated(true);

    const std::array goals = { goal };
    expectPosition(findNextTile(start, kWaterLevel, goals, {}, kSouthEast).nextTile, goal);

    obstacle->setAiAllocated(false);
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast);
    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, 2U * kCardinalStepCost);
    expectPosition(result.nextTile, { 10, 11 });
}

TEST_F(WaterPathfindingTest, RevalidatesBlockedDetourAfterAiElementBecomesReal)
{
    constexpr TilePos2 start{ 10, 10 };
    for (tile_coord_t y = 9; y <= 11; ++y)
    {
        setHorizontalWater(10, 14, y);
    }

    auto* obstacle = TileManager::insertElement<TrackElement>({ 11 * kTileSize, 11 * kTileSize }, kWaterLevel * kMicroToSmallZStep, 0xF);
    ASSERT_NE(obstacle, nullptr);
    obstacle->setAiAllocated(true);

    const std::array goals = { TilePos2{ 14, 10 } };
    const std::array blocked = { TilePos2{ 11, 10 } };
    auto current = findNextTile(start, kWaterLevel, goals, blocked, kSouth).nextTile;
    expectPosition(current, { 10, 11 });

    obstacle->setAiAllocated(false);
    auto direction = kSouth;
    for (auto step = 0; step < 20 && current != goals[0]; ++step)
    {
        const auto isBlockerAdjacent = isAdjacent(current, blocked[0]);
        const auto activeBlockers = isBlockerAdjacent ? std::span<const TilePos2>{ blocked } : std::span<const TilePos2>{};
        const auto result = findNextTile(current, kWaterLevel, goals, activeBlockers, direction);
        ASSERT_EQ(result.status, RouteStatus::found);
        EXPECT_NE(result.nextTile, TilePos2(11, 11));
        direction = getStepYaw(current, result.nextTile);
        current = result.nextTile;
    }
    expectPosition(current, goals[0]);
}

TEST_F(WaterPathfindingTest, HandlesMapEdge)
{
    constexpr TilePos2 start{ 0, 0 };
    constexpr TilePos2 goal{ 1, 1 };
    setWaterRectangle(start, goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouthEast);

    EXPECT_EQ(result.status, RouteStatus::found);
    EXPECT_EQ(result.remainingDistance, kDiagonalStepCost);
    expectPosition(result.nextTile, goal);
}
