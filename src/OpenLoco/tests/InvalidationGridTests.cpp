#include "Graphics/InvalidationGrid.h"

#include <array>
#include <gtest/gtest.h>
#include <vector>

using namespace OpenLoco;

namespace
{
    using DirtyRect = std::array<int32_t, 4>;

    std::vector<DirtyRect> traverseDirtyCells(Gfx::InvalidationGrid& grid)
    {
        std::vector<DirtyRect> rectangles;
        grid.traverseDirtyCells([&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
            rectangles.push_back({ left, top, right, bottom });
        });
        return rectangles;
    }
}

TEST(InvalidationGridTests, EmptyGridHasNoDirtyRectangles)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 30, 10, 10);

    EXPECT_TRUE(traverseDirtyCells(grid).empty());
}

TEST(InvalidationGridTests, FullGridCoalescesIntoOneRectangle)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 30, 10, 10);
    grid.invalidate(0, 0, 40, 30);

    EXPECT_EQ(traverseDirtyCells(grid), (std::vector<DirtyRect>{ { 0, 0, 40, 30 } }));
}

TEST(InvalidationGridTests, RectangularRegionCoalescesInBothDimensions)
{
    Gfx::InvalidationGrid grid;
    grid.reset(50, 40, 10, 10);
    grid.invalidate(10, 10, 29, 29);

    EXPECT_EQ(traverseDirtyCells(grid), (std::vector<DirtyRect>{ { 10, 10, 30, 30 } }));
}

TEST(InvalidationGridTests, HalfOpenRectangleDoesNotDirtyTheNextBlock)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 30, 10, 10);
    grid.invalidate(0, 0, 10, 10);

    EXPECT_EQ(traverseDirtyCells(grid), (std::vector<DirtyRect>{ { 0, 0, 10, 10 } }));
}

TEST(InvalidationGridTests, OnePixelRectangleAtBlockBoundaryDirtiesOneBlock)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 30, 10, 10);
    grid.invalidate(10, 10, 11, 11);

    EXPECT_EQ(traverseDirtyCells(grid), (std::vector<DirtyRect>{ { 10, 10, 20, 20 } }));
}

TEST(InvalidationGridTests, LShapePreservesExactDirtyCoverage)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 40, 10, 10);
    grid.invalidate(0, 0, 29, 9);
    grid.invalidate(0, 0, 9, 29);

    const std::vector<DirtyRect> expected = {
        { 0, 0, 10, 30 },
        { 10, 0, 30, 10 },
    };
    EXPECT_EQ(traverseDirtyCells(grid), expected);
}

TEST(InvalidationGridTests, RectangleIsClippedAtScreenEdge)
{
    Gfx::InvalidationGrid grid;
    grid.reset(25, 17, 10, 10);
    grid.invalidate(20, 10, 40, 30);

    EXPECT_EQ(traverseDirtyCells(grid), (std::vector<DirtyRect>{ { 20, 10, 25, 17 } }));
}

TEST(InvalidationGridTests, TraversalClearsCellsAndGridCanBeInvalidatedAgain)
{
    Gfx::InvalidationGrid grid;
    grid.reset(40, 30, 10, 10);
    grid.invalidate(10, 0, 29, 19);
    const std::vector<DirtyRect> expected = { { 10, 0, 30, 20 } };

    EXPECT_EQ(traverseDirtyCells(grid), expected);
    EXPECT_TRUE(traverseDirtyCells(grid).empty());

    grid.invalidate(10, 0, 29, 19);
    EXPECT_EQ(traverseDirtyCells(grid), expected);
}

TEST(InvalidationGridTests, UploadPromotesHalfCoverageToFullFrame)
{
    Gfx::InvalidationGrid grid;
    grid.reset(100, 100, 10, 10);
    grid.invalidate(0, 0, 50, 100);

    std::vector<DirtyRect> uploads;
    EXPECT_TRUE(Gfx::Detail::uploadDirtyRegions(grid, false, 100, 100, [&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
        uploads.push_back({ left, top, right, bottom });
        return true;
    }));
    EXPECT_EQ(uploads, (std::vector<DirtyRect>{ { 0, 0, 100, 100 } }));
    EXPECT_TRUE(traverseDirtyCells(grid).empty());
}

TEST(InvalidationGridTests, UploadPromotesFragmentedDamageToFullFrame)
{
    Gfx::InvalidationGrid grid;
    grid.reset(195, 1, 1, 1);
    for (int32_t x = 0; x < 130; x += 2)
    {
        grid.invalidate(x, 0, x + 1, 1);
    }

    std::vector<DirtyRect> uploads;
    EXPECT_TRUE(Gfx::Detail::uploadDirtyRegions(grid, false, 195, 1, [&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
        uploads.push_back({ left, top, right, bottom });
        return true;
    }));
    EXPECT_EQ(uploads, (std::vector<DirtyRect>{ { 0, 0, 195, 1 } }));
}

TEST(InvalidationGridTests, FailedPartialUploadCanRetryAsFullFrame)
{
    Gfx::InvalidationGrid grid;
    grid.reset(100, 100, 10, 10);
    grid.invalidate(0, 0, 10, 10);

    EXPECT_FALSE(Gfx::Detail::uploadDirtyRegions(grid, false, 100, 100, [](int32_t, int32_t, int32_t, int32_t) {
        return false;
    }));

    std::vector<DirtyRect> uploads;
    EXPECT_TRUE(Gfx::Detail::uploadDirtyRegions(grid, true, 100, 100, [&](int32_t left, int32_t top, int32_t right, int32_t bottom) {
        uploads.push_back({ left, top, right, bottom });
        return true;
    }));
    EXPECT_EQ(uploads, (std::vector<DirtyRect>{ { 0, 0, 100, 100 } }));
    EXPECT_TRUE(traverseDirtyCells(grid).empty());
}
