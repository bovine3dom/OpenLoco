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
    constexpr uint8_t kWest = 0;
    constexpr uint8_t kSouth = 1;
    constexpr uint8_t kEast = 2;
    constexpr uint8_t kNorth = 3;

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
            EXPECT_EQ(result.remainingDistance, 60U);
        }
        direction = result.nextTile.x < current.x ? kWest
            : result.nextTile.x > current.x       ? kEast
            : result.nextTile.y < current.y       ? kNorth
                                                  : kSouth;
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
    EXPECT_EQ(result.remainingDistance, 5U);
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

TEST_F(WaterPathfindingTest, PrefersCurrentHeadingBetweenEqualRoutes)
{
    for (tile_coord_t y = 10; y <= 11; ++y)
    {
        setHorizontalWater(10, 11, y);
    }
    const std::array goals = { TilePos2{ 11, 11 } };

    const auto eastResult = findNextTile({ 10, 10 }, kWaterLevel, goals, {}, kEast);
    const auto southResult = findNextTile({ 10, 10 }, kWaterLevel, goals, {}, kSouth);

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
        const auto isBlockerAdjacent = Math::Vector::manhattanDistance2D(current, blocked[0]) == 1;
        const auto activeBlockers = isBlockerAdjacent ? std::span<const TilePos2>{ blocked } : std::span<const TilePos2>{};
        const auto result = findNextTile(current, kWaterLevel, goals, activeBlockers, direction);
        ASSERT_EQ(result.status, RouteStatus::found);
        if (step == 0)
        {
            expectPosition(result.nextTile, { 10, 11 });
        }
        direction = result.nextTile.x < current.x ? kWest
            : result.nextTile.x > current.x       ? kEast
            : result.nextTile.y < current.y       ? kNorth
                                                  : kSouth;
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
        const auto isBlockerAdjacent = Math::Vector::manhattanDistance2D(current, blocked[0]) == 1;
        const auto activeBlockers = isBlockerAdjacent ? std::span<const TilePos2>{ blocked } : std::span<const TilePos2>{};
        const auto result = findNextTile(current, kWaterLevel, goals, activeBlockers, direction);
        ASSERT_EQ(result.status, RouteStatus::found);
        EXPECT_NE(result.nextTile, TilePos2(11, 11));
        direction = result.nextTile.x < current.x ? kWest
            : result.nextTile.x > current.x       ? kEast
            : result.nextTile.y < current.y       ? kNorth
                                                  : kSouth;
        current = result.nextTile;
    }
    expectPosition(current, goals[0]);
}

TEST_F(WaterPathfindingTest, HandlesMapEdge)
{
    constexpr TilePos2 start{ 0, 0 };
    constexpr TilePos2 goal{ 0, 1 };
    setWater(start);
    setWater(goal);

    const std::array goals = { goal };
    const auto result = findNextTile(start, kWaterLevel, goals, {}, kSouth);

    EXPECT_EQ(result.status, RouteStatus::found);
    expectPosition(result.nextTile, goal);
}
