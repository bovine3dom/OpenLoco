#include "Input/ZoomDeltaAccumulator.h"
#include "LabelFrame.h"
#include "S5/S5LabelFrame.h"
#include "Ui/WindowManager.h"
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

TEST(ViewportTests, AppliesMagnifiedRasterViewOffset)
{
    Viewport viewport{};
    viewport.width = 100;
    viewport.height = 100;
    viewport.rasterWidth = 100;
    viewport.rasterHeight = 100;
    viewport.viewX = 100;
    viewport.viewY = 200;
    viewport.zoom = ZoomLevel::quadrupled;
    viewport.viewRasterOffset = { 1, 2 };

    expectPoint(viewport.getViewOriginInRaster(), 401, 802);
    const auto viewRect = viewport.getViewRect();
    EXPECT_EQ(viewRect.left, 100);
    EXPECT_EQ(viewRect.top, 200);
    EXPECT_EQ(viewRect.right, 126);
    EXPECT_EQ(viewRect.bottom, 226);
    auto viewportPosition = viewport.rasterToViewport({ 3, 2 });
    EXPECT_EQ(viewportPosition.x, 101);
    EXPECT_EQ(viewportPosition.y, 201);
    viewportPosition = viewport.windowToViewport({ 3, 2 });
    EXPECT_EQ(viewportPosition.x, 101);
    EXPECT_EQ(viewportPosition.y, 201);
    expectPoint(viewport.viewportToWindow({ 101, 201 }), 3, 2);

    viewport.zoom = ZoomLevel::full;
    expectPoint(viewport.getViewOriginInRaster(), 100, 200);
    viewportPosition = viewport.rasterToViewport({ 3, 2 });
    EXPECT_EQ(viewportPosition.x, 103);
    EXPECT_EQ(viewportPosition.y, 202);
}

TEST(ViewportTests, RebasesMagnifiedRasterViewOffset)
{
    Viewport viewport{};
    viewport.viewX = 100;
    viewport.viewY = 200;
    viewport.zoom = ZoomLevel::quadrupled;
    viewport.viewRasterOffset = { 12, -6 };
    const auto originalOrigin = viewport.getViewOriginInRaster();

    viewport.rebaseViewRasterOffset(viewport.zoom);
    expectPoint(viewport.getViewOriginInRaster(), originalOrigin.x, originalOrigin.y);
    EXPECT_EQ(viewport.viewX, 103);
    EXPECT_EQ(viewport.viewY, 199);
    expectPoint(viewport.viewRasterOffset, 0, -2);

    const auto previousZoom = viewport.zoom;
    viewport.zoom = ZoomLevel::doubled;
    viewport.rebaseViewRasterOffset(previousZoom);
    EXPECT_EQ(viewport.viewX, 103);
    EXPECT_EQ(viewport.viewY, 199);
    expectPoint(viewport.viewRasterOffset, 0, -1);
}

TEST(ViewportTests, PreservesCameraPositionWhenUnfollowing)
{
    Window main({ 0, 0 }, { 100, 100 });
    Viewport viewport{};
    viewport.viewX = 100;
    viewport.viewY = 200;
    viewport.zoom = ZoomLevel::quadrupled;
    viewport.viewRasterOffset = { 12, -6 };
    const auto originalOrigin = viewport.getViewOriginInRaster();
    main.viewports[0] = &viewport;
    main.viewportConfigurations[0].viewportTargetSprite = static_cast<EntityId>(1);

    Windows::Main::viewportUnfocusFromEntity(main);

    EXPECT_EQ(main.viewportConfigurations[0].viewportTargetSprite, EntityId::null);
    EXPECT_EQ(main.viewportConfigurations[0].savedViewX, viewport.viewX);
    EXPECT_EQ(main.viewportConfigurations[0].savedViewY, viewport.viewY);
    expectPoint(viewport.getViewOriginInRaster(), originalOrigin.x, originalOrigin.y);
    expectPoint(viewport.viewRasterOffset, 0, -2);
}

TEST(ZoomLevelTests, SupportsEightfoldAndSixteenfoldMagnification)
{
    EXPECT_EQ(ZoomLevel::min, ZoomLevel::sixteenfold);
    EXPECT_EQ(ZoomLevel::count, 8);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::sixteenfold }.index(), 0);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::eightfold }.index(), 1);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::full }.index(), 4);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::eighth }.index(), 7);

    const ZoomLevel zoom{ ZoomLevel::sixteenfold };
    EXPECT_EQ(zoom.applyTo(160), 10);
    EXPECT_EQ(zoom.applyInversedTo(10), 160);
    EXPECT_EQ(zoom.applyInversedTo(-10), -160);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::half }.applyTo(-3), -6);

    Viewport viewport{};
    viewport.zoom = zoom;
    viewport.setDimensions({ 320, 200 }, { 1600, 960 });
    EXPECT_EQ(viewport.viewWidth, 100);
    EXPECT_EQ(viewport.viewHeight, 60);
}

TEST(ZoomLevelTests, PreservesLegacyLabelFrameMapping)
{
    LabelFrame source{};
    for (auto level = ZoomLevel::min; level <= ZoomLevel::max; ++level)
    {
        const auto index = ZoomLevel{ level }.index();
        source.left[index] = 100 + level;
        source.right[index] = 200 + level;
        source.top[index] = 300 + level;
        source.bottom[index] = 400 + level;
    }

    const auto saved = S5::exportLabelFrame(source);
    const auto restored = S5::importLabelFrame(saved);
    for (int8_t level = ZoomLevel::full; level <= ZoomLevel::max; ++level)
    {
        const auto index = ZoomLevel{ level }.index();
        EXPECT_EQ(saved.left[level], source.left[index]);
        EXPECT_EQ(saved.right[level], source.right[index]);
        EXPECT_EQ(saved.top[level], source.top[index]);
        EXPECT_EQ(saved.bottom[level], source.bottom[index]);
        EXPECT_EQ(restored.left[index], source.left[index]);
        EXPECT_EQ(restored.right[index], source.right[index]);
        EXPECT_EQ(restored.top[index], source.top[index]);
        EXPECT_EQ(restored.bottom[index], source.bottom[index]);
    }
    for (auto level = ZoomLevel::min; level < ZoomLevel::full; ++level)
    {
        const auto index = ZoomLevel{ level }.index();
        EXPECT_EQ(restored.left[index], 0);
        EXPECT_EQ(restored.right[index], 0);
        EXPECT_EQ(restored.top[index], 0);
        EXPECT_EQ(restored.bottom[index], 0);
    }
}

TEST(ZoomLevelTests, UsesSignedSaveEncoding)
{
    EXPECT_EQ(ZoomLevel{ ZoomLevel::eightfold }.toEncoded(), 0xFD);
    EXPECT_EQ(ZoomLevel{ ZoomLevel::sixteenfold }.toEncoded(), 0xFC);
    EXPECT_EQ(ZoomLevel::fromEncoded(0xFD), ZoomLevel{ ZoomLevel::eightfold });
    EXPECT_EQ(ZoomLevel::fromEncoded(0xFC), ZoomLevel{ ZoomLevel::sixteenfold });
    EXPECT_EQ(ZoomLevel::fromEncoded(0xFB), ZoomLevel{ ZoomLevel::min });
    EXPECT_EQ(ZoomLevel::fromEncoded(0x04), ZoomLevel{ ZoomLevel::max });
}

TEST(ZoomDeltaAccumulatorTests, PreservesMagnifiedScrollingSpeedAndDirection)
{
    Input::ZoomDeltaAccumulator positive;
    Input::ZoomDeltaAccumulator negative;
    int32_t positiveTotal = 0;
    int32_t negativeTotal = 0;
    for (auto i = 0; i < 4; ++i)
    {
        positiveTotal += positive.apply({ 12, 0 }, ZoomLevel::sixteenfold).x;
        negativeTotal += negative.apply({ -12, 0 }, ZoomLevel::sixteenfold).x;
    }
    EXPECT_EQ(positiveTotal, 3);
    EXPECT_EQ(negativeTotal, -3);
}

TEST(ZoomDeltaAccumulatorTests, IsIndependentOfInputBatching)
{
    Input::ZoomDeltaAccumulator individual;
    Input::ZoomDeltaAccumulator batched;
    int32_t individualTotal = 0;
    for (auto i = 0; i < 8; ++i)
    {
        individualTotal += individual.apply({ 1, 0 }, ZoomLevel::eightfold).x;
    }

    EXPECT_EQ(individualTotal, batched.apply({ 8, 0 }, ZoomLevel::eightfold).x);
}

TEST(ZoomDeltaAccumulatorTests, IsIndependentOfDirectionChangeBatching)
{
    Input::ZoomDeltaAccumulator individual;
    Input::ZoomDeltaAccumulator batched;
    const auto forward = individual.apply({ 20, 0 }, ZoomLevel::sixteenfold).x;
    const auto backward = individual.apply({ -12, 0 }, ZoomLevel::sixteenfold).x;

    EXPECT_EQ(forward + backward, batched.apply({ 8, 0 }, ZoomLevel::sixteenfold).x);
}

TEST(ZoomDeltaAccumulatorTests, ResetsRemainderOnResetAndZoomChange)
{
    Input::ZoomDeltaAccumulator accumulator;
    EXPECT_EQ(accumulator.apply({ 12, 0 }, ZoomLevel::sixteenfold).x, 0);
    accumulator.reset();
    EXPECT_EQ(accumulator.apply({ 4, 0 }, ZoomLevel::sixteenfold).x, 0);

    EXPECT_EQ(accumulator.apply({ 12, 0 }, ZoomLevel::eightfold).x, 1);
}

TEST(ZoomDeltaAccumulatorTests, ResetsAxesIndependently)
{
    Input::ZoomDeltaAccumulator accumulator;
    EXPECT_EQ(accumulator.apply({ 12, 12 }, ZoomLevel::sixteenfold), Ui::Point(0, 0));
    accumulator.resetX();

    EXPECT_EQ(accumulator.apply({ 4, 4 }, ZoomLevel::sixteenfold), Ui::Point(0, 1));
}
