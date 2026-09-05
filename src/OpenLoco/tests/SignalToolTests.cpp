#include "../src/Ui/Windows/Construction/SignalTab.cpp"
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Ui;
using namespace OpenLoco::Ui::Windows::Construction;

TEST(SignalTool, StationaryAndSmallMouseMovementsPreserveSinglePlacementPreview)
{
    const auto previousDragPosition = Input::getDragLastLocation();
    Input::setDragLastLocation({ 100, 100 });
    Window window{ { 0, 0 }, { 138, 205 } };

    for (const auto widgetIndex : { Signal::widx::single_direction, Signal::widx::both_directions })
    {
        for (const auto offset : { Point{ 0, 0 }, Point{ 3, 3 }, Point{ -3, -3 }, Point{ 4, 0 }, Point{ 0, -4 } })
        {
            SCOPED_TRACE(::testing::Message() << "widget=" << widgetIndex << " offset=" << offset.x << "," << offset.y);
            Signal::_isDragStartValid = true;
            Signal::_isDragging = false;
            // Use an object-free construction preview to detect removal before hit-testing.
            Common::setGhostVisibilityFlag(GhostVisibilityFlags::constructArrow);

            Signal::getEvents().toolDrag(window, widgetIndex, WidgetId::none, 100 + offset.x, 100 + offset.y);

            const bool withinClickTolerance = std::abs(offset.x) < 4 && std::abs(offset.y) < 4;
            EXPECT_EQ(Common::hasGhostVisibilityFlag(GhostVisibilityFlags::constructArrow), withinClickTolerance);
            EXPECT_FALSE(Signal::_isDragging);
            Common::unsetGhostVisibilityFlag(GhostVisibilityFlags::constructArrow);
        }
    }

    Signal::_isDragStartValid = false;
    Input::setDragLastLocation(previousDragPosition);
}
