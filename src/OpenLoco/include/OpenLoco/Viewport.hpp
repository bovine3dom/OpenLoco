#pragma once

#include "Graphics/Gfx.h"
#include "Location.hpp"
#include "Types.hpp"
#include "ZoomLevel.hpp"
#include <OpenLoco/Core/EnumFlags.hpp>
#include <OpenLoco/Engine/World.hpp>
#include <algorithm>

namespace OpenLoco::Gfx
{
    class DrawingContext;
}

namespace OpenLoco
{
    struct LabelFrame;
}

namespace OpenLoco::Ui
{
    struct SavedViewSimple;

    struct viewport_pos
    {
        int32_t x{};
        int32_t y{};

        viewport_pos()
            : viewport_pos(0, 0)
        {
        }
        viewport_pos(int32_t _x, int32_t _y)
            : x(_x)
            , y(_y)
        {
        }
    };

    struct ViewportRect
    {
        int32_t left = 0;
        int32_t top = 0;
        int32_t bottom = 0;
        int32_t right = 0;

        constexpr bool contains(const viewport_pos& vpos)
        {
            return (left < vpos.x && top < vpos.y && right >= vpos.x && bottom >= vpos.y);
        }
    };

    enum class ViewportFlags : uint16_t
    {
        none = 0U,
        underground_view = 1U << 0,
        seeThroughTracks = 1U << 1,
        height_marks_on_land = 1U << 2,
        height_marks_on_tracks_roads = 1U << 3,
        one_way_direction_arrows = 1U << 4,
        gridlines_on_landscape = 1U << 5,
        seeThroughScenery = 1U << 6,
        flag_7 = 1U << 7,
        flag_8 = 1U << 8,
        hideTownNames = 1U << 9,
        hideStationNames = 1U << 10,
        seeThroughRoads = 1U << 11,
        seeThroughBuildings = 1U << 12,
        seeThroughTrees = 1U << 13,
        seeThroughBridges = 1U << 14,
    };
    OPENLOCO_ENABLE_ENUM_OPERATORS(ViewportFlags);

    struct Viewport;

    namespace WindowToViewport
    {
        [[nodiscard]] constexpr Point applyTransform(const Point& uiPoint, const Viewport& vp);
    }

    namespace ViewportToWindow
    {
        [[nodiscard]] constexpr Point applyTransform(const Point& vpPoint, const Viewport& vp);
    }

    struct Viewport
    {
        int32_t width;      // 0x00
        int32_t height;     // 0x02
        int32_t x;          // 0x04
        int32_t y;          // 0x06
        int32_t viewX;      // 0x08
        int32_t viewY;      // 0x0A
        int32_t viewWidth;  // 0x0C
        int32_t viewHeight; // 0x0E
        int32_t rasterWidth;
        int32_t rasterHeight;
        Point viewRasterOffset; // Render-only phase for magnified viewports.
        ZoomLevel zoom;         // 0x10
        uint8_t pad_11;         // 0x11
        ViewportFlags flags;    // 0x12

        constexpr bool contains(const viewport_pos& vpos) const
        {
            const auto viewRect = getViewRect();
            return vpos.y >= viewRect.top && vpos.y < viewRect.bottom && vpos.x >= viewRect.left && vpos.x < viewRect.right;
        }

        constexpr bool containsWindowPos(const Point& pos) const
        {
            return (pos.x >= x && pos.x < x + width && pos.y >= y && pos.y < y + height);
        }

        Ui::Rect getUiRect() const
        {
            return Ui::Rect::fromLTRB(x, y, x + width, y + height);
        }

        void setDimensions(Ui::Size uiSize, Ui::Size rasterSize)
        {
            width = uiSize.width;
            height = uiSize.height;
            rasterWidth = rasterSize.width;
            rasterHeight = rasterSize.height;
            viewWidth = zoom.applyTo(rasterWidth);
            viewHeight = zoom.applyTo(rasterHeight);
        }

        constexpr Point uiToRaster(const Point& point) const
        {
            return {
                width == 0 ? 0 : static_cast<int32_t>(static_cast<int64_t>(point.x) * rasterWidth / width),
                height == 0 ? 0 : static_cast<int32_t>(static_cast<int64_t>(point.y) * rasterHeight / height),
            };
        }

        constexpr Point rasterToUi(const Point& point) const
        {
            return {
                rasterWidth == 0 ? 0 : static_cast<int32_t>(static_cast<int64_t>(point.x) * width / rasterWidth),
                rasterHeight == 0 ? 0 : static_cast<int32_t>(static_cast<int64_t>(point.y) * height / rasterHeight),
            };
        }

        constexpr Point rasterToUiNearest(const Point& point) const
        {
            const auto sample = [](int32_t position, int32_t sourceSize, int32_t destinationSize) {
                const auto numerator = (static_cast<int64_t>(position) * 2 + 1) * destinationSize;
                const auto denominator = static_cast<int64_t>(sourceSize) * 2;
                const auto quotient = numerator / denominator;
                return static_cast<int32_t>(quotient - (numerator % denominator < 0 ? 1 : 0));
            };
            return {
                rasterWidth == 0 ? 0 : sample(point.x, rasterWidth, width),
                rasterHeight == 0 ? 0 : sample(point.y, rasterHeight, height),
            };
        }

        constexpr Point rasterToUiCeil(const Point& point) const
        {
            const auto divideCeil = [](int64_t numerator, int32_t denominator) {
                const auto quotient = numerator / denominator;
                return static_cast<int32_t>(quotient + (numerator % denominator > 0 ? 1 : 0));
            };
            return {
                rasterWidth == 0 ? 0 : divideCeil(static_cast<int64_t>(point.x) * width, rasterWidth),
                rasterHeight == 0 ? 0 : divideCeil(static_cast<int64_t>(point.y) * height, rasterHeight),
            };
        }

        constexpr ViewportRect getViewRect() const
        {
            if (zoom >= ZoomLevel::full)
            {
                return { viewX, viewY, viewY + viewHeight, viewX + viewWidth };
            }

            const auto origin = getViewOriginInRaster();
            return {
                zoom.applyTo(origin.x),
                zoom.applyTo(origin.y),
                zoom.applyTo(origin.y + rasterHeight - 1) + 1,
                zoom.applyTo(origin.x + rasterWidth - 1) + 1,
            };
        }

        constexpr bool intersects(const ViewportRect& vpos) const
        {
            const auto viewRect = getViewRect();
            if (vpos.right <= viewRect.left)
            {
                return false;
            }

            if (vpos.bottom <= viewRect.top)
            {
                return false;
            }

            if (vpos.left >= viewRect.right)
            {
                return false;
            }

            if (vpos.top >= viewRect.bottom)
            {
                return false;
            }

            return true;
        }

        constexpr ViewportRect getIntersection(const ViewportRect& rect) const
        {
            const auto viewRect = getViewRect();
            auto out = ViewportRect();
            out.left = std::max(rect.left, viewRect.left);
            out.right = std::min(rect.right, viewRect.right);
            out.top = std::max(rect.top, viewRect.top);
            out.bottom = std::min(rect.bottom, viewRect.bottom);

            return out;
        }

        int getRotation() const;
        void setRotation(int32_t value);

        /**
         * Maps a 2D viewport position to a window relative position.
         */
        Point viewportToWindow(const viewport_pos& vpos) const
        {
            const auto vpPoint = ViewportToWindow::applyTransform({ vpos.x, vpos.y }, *this);
            return vpPoint;
        }

        /**
         * Maps a window relative position to a 2D viewport position.
         */
        viewport_pos windowToViewport(const Point& pos) const
        {
            const auto vpPoint = WindowToViewport::applyTransform(pos, *this);
            return { vpPoint.x, vpPoint.y };
        }

        viewport_pos rasterToViewport(const Point& pos) const
        {
            if (zoom < ZoomLevel::full)
            {
                return {
                    viewX + zoom.applyTo(pos.x + viewRasterOffset.x),
                    viewY + zoom.applyTo(pos.y + viewRasterOffset.y),
                };
            }
            return { viewX + zoom.applyTo(pos.x), viewY + zoom.applyTo(pos.y) };
        }

        constexpr Point getViewOriginInRaster() const
        {
            const auto rasterOffset = zoom < ZoomLevel::full ? viewRasterOffset : Point{};
            return {
                zoom.applyInversedTo(viewX) + rasterOffset.x,
                zoom.applyInversedTo(viewY) + rasterOffset.y,
            };
        }

        constexpr void rebaseViewRasterOffset(const ZoomLevel previousZoom)
        {
            const auto getRasterScale = [](const ZoomLevel value) {
                return value < ZoomLevel::full ? value.applyInversedTo(1) : 1;
            };
            const auto previousScale = getRasterScale(previousZoom);
            const auto rasterScale = getRasterScale(zoom);
            const auto rescale = [previousScale, rasterScale](const int32_t value) {
                const auto numerator = static_cast<int64_t>(value) * rasterScale;
                return static_cast<int32_t>(numerator >= 0 ? (numerator + previousScale / 2) / previousScale : -((-numerator + previousScale / 2) / previousScale));
            };

            const Point offset{ rescale(viewRasterOffset.x), rescale(viewRasterOffset.y) };
            viewX += offset.x / rasterScale;
            viewY += offset.y / rasterScale;
            viewRasterOffset = zoom < ZoomLevel::full
                ? Point{ offset.x % rasterScale, offset.y % rasterScale }
                : Point{};
        }

        /**
         * Maps a window relative rectangle to a 2D viewport rectangle.
         */
        Rect windowToViewport(const Rect& rect)
        {
            auto leftTop = windowToViewport(Point(rect.left(), rect.top()));
            auto rightBottom = windowToViewport(Point(rect.right(), rect.bottom()));
            return Rect::fromLTRB(leftTop.x, leftTop.y, rightBottom.x, rightBottom.y);
        }

        void render(Gfx::DrawingContext& drawingCtx, bool drawOverlays = true);
        void renderUiOverlays(Gfx::DrawingContext& drawingCtx);
        viewport_pos centre2dCoordinates(const World::Pos3& loc);
        SavedViewSimple toSavedView() const;

        viewport_pos getCentre() const;
        Point getWindowCentre() const;
        Rect getUiLabelRect(const LabelFrame& frame) const;
        World::Pos2 getCentreMapPosition() const;
        std::optional<World::Pos2> getCentreScreenMapPosition() const;

        constexpr bool hasFlags(ViewportFlags flagsToTest) const
        {
            return (flags & flagsToTest) != ViewportFlags::none;
        }

        constexpr bool isValid() const
        {
            return width != 0 && height != 0 && rasterWidth != 0 && rasterHeight != 0;
        }

    private:
        void paint(Gfx::DrawingContext& drawingCtx, const Ui::Rect& rect, bool drawOverlays);
    };

    struct ViewportConfig
    {
        EntityId viewportTargetSprite; // 0x0
        int32_t savedViewX;            // 0x2
        int32_t savedViewY;            // 0x4
    };

    namespace WindowToViewport
    {
        [[nodiscard]] constexpr Point windowOffsetTransform(const Point& uiPoint, const Viewport& vp)
        {
            return uiPoint - Point{ vp.x, vp.y };
        }

        [[nodiscard]] constexpr Point scaleTransform(const Point& uiPoint, const Viewport& vp)
        {
            const auto rasterPoint = vp.uiToRaster(uiPoint);
            if (vp.zoom < ZoomLevel::full)
            {
                return Point{
                    vp.zoom.applyTo(rasterPoint.x + vp.viewRasterOffset.x),
                    vp.zoom.applyTo(rasterPoint.y + vp.viewRasterOffset.y),
                };
            }
            return Point{ vp.zoom.applyTo(rasterPoint.x), vp.zoom.applyTo(rasterPoint.y) };
        }

        [[nodiscard]] constexpr Point viewOffsetTransform(const Point& point, const Viewport& vp)
        {
            return point + Point{ vp.viewX, vp.viewY };
        }

        [[nodiscard]] constexpr Point applyTransform(const Point& uiPoint, const Viewport& vp)
        {
            return viewOffsetTransform(scaleTransform(windowOffsetTransform(uiPoint, vp), vp), vp);
        }
    }

    namespace ViewportToWindow
    {
        [[nodiscard]] constexpr Point windowOffsetTransform(const Point& uiPoint, const Viewport& vp)
        {
            return uiPoint + Point{ vp.x, vp.y };
        }

        [[nodiscard]] constexpr Point scaleTransform(const Point& uiPoint, const Viewport& vp)
        {
            auto rasterPoint = Point{ vp.zoom.applyInversedTo(uiPoint.x), vp.zoom.applyInversedTo(uiPoint.y) };
            if (vp.zoom < ZoomLevel::full)
            {
                rasterPoint -= vp.viewRasterOffset;
            }
            return vp.rasterToUiNearest(rasterPoint);
        }

        [[nodiscard]] constexpr Point viewOffsetTransform(const Point& point, const Viewport& vp)
        {
            return point - Point{ vp.viewX, vp.viewY };
        }

        [[nodiscard]] constexpr Point applyTransform(const Point& vpPoint, const Viewport& vp)
        {
            return windowOffsetTransform(scaleTransform(viewOffsetTransform(vpPoint, vp), vp), vp);
        }
    }
}
