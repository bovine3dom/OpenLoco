#include "Viewport.hpp"

#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Ui;

namespace
{
    void expectPoint(const Point& actual, int32_t x, int32_t y)
    {
        EXPECT_EQ(actual.x, x);
        EXPECT_EQ(actual.y, y);
    }
}

TEST(ViewportTests, ConvertsBetweenUiAndRasterCoordinates)
{
    Viewport viewport{};
    viewport.width = 320;
    viewport.height = 200;
    viewport.rasterWidth = 1920;
    viewport.rasterHeight = 1080;

    expectPoint(viewport.uiToRaster({ 0, 0 }), 0, 0);
    expectPoint(viewport.uiToRaster({ 160, 100 }), 960, 540);
    expectPoint(viewport.uiToRaster({ 320, 200 }), 1920, 1080);
    expectPoint(viewport.rasterToUi({ 960, 540 }), 160, 100);
    expectPoint(viewport.rasterToUi({ 1920, 1080 }), 320, 200);
}

TEST(ViewportTests, RoundsRasterBoundsOutward)
{
    Viewport viewport{};
    viewport.width = 3;
    viewport.height = 3;
    viewport.rasterWidth = 10;
    viewport.rasterHeight = 10;

    expectPoint(viewport.rasterToUi({ 1, 1 }), 0, 0);
    expectPoint(viewport.rasterToUiNearest({ 3, 3 }), 1, 1);
    expectPoint(viewport.rasterToUiCeil({ 1, 1 }), 1, 1);
    expectPoint(viewport.rasterToUi({ 10, 10 }), 3, 3);
    expectPoint(viewport.rasterToUiCeil({ 10, 10 }), 3, 3);

    EXPECT_EQ(ZoomLevel(ZoomLevel::half).applyInversedTo(3), 1);
    EXPECT_EQ(ZoomLevel(ZoomLevel::half).applyInversedToCeil(3), 2);
    EXPECT_EQ(ZoomLevel(ZoomLevel::half).applyInversedToCeil(-3), -1);
}

TEST(ViewportTests, HandlesMissingDimensions)
{
    const Viewport viewport{};

    expectPoint(viewport.uiToRaster({ 10, 20 }), 0, 0);
    expectPoint(viewport.rasterToUi({ 10, 20 }), 0, 0);
    expectPoint(viewport.rasterToUiNearest({ 10, 20 }), 0, 0);
    expectPoint(viewport.rasterToUiCeil({ 10, 20 }), 0, 0);
    EXPECT_FALSE(viewport.isValid());
}

TEST(ViewportTests, AppliesViewportOffsetsScaleAndZoom)
{
    Viewport viewport{};
    viewport.x = 10;
    viewport.y = 20;
    viewport.width = 300;
    viewport.height = 200;
    viewport.rasterWidth = 900;
    viewport.rasterHeight = 600;
    viewport.viewX = 1000;
    viewport.viewY = 2000;
    viewport.zoom = ZoomLevel::half;

    const Point windowPosition{ 160, 120 };
    const auto worldPosition = viewport.windowToViewport(windowPosition);
    EXPECT_EQ(worldPosition.x, 1900);
    EXPECT_EQ(worldPosition.y, 2600);
    const auto exactWorldPosition = viewport.rasterToViewport({ 451, 301 });
    EXPECT_EQ(exactWorldPosition.x, 1902);
    EXPECT_EQ(exactWorldPosition.y, 2602);
    expectPoint(viewport.viewportToWindow(worldPosition), windowPosition.x, windowPosition.y);
}
