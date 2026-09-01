// SPDX-License-Identifier: MIT
#pragma once

#include "Graphics/Colour.h"
#include <OpenLoco/Engine/Ui/Point.hpp>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::Gfx
{
    class DrawingContext;
}

namespace OpenLoco::Ui
{
    struct Viewport;
    struct Window;

    namespace Windows::CargoFlowOverlay
    {
        enum class ScaleMode : uint8_t
        {
            absolute,
            percentiles,
        };

        struct ProjectedLink
        {
            Point from;
            Point to;
            PaletteIndex_t colour{};
            bool dashed{};
        };

        struct ProjectedMarker
        {
            Point position;
            PaletteIndex_t colour{};
        };

        struct Projection
        {
            std::vector<ProjectedLink> links;
            std::vector<ProjectedMarker> markers;
        };

        Window* open();
        void toggle();
        bool isOpen();
        bool hasEnabledCargo();

        uint8_t getSaturationBucket(uint64_t demand, std::optional<uint64_t> capacity);
        PaletteIndex_t getSaturationColour(uint8_t bucket);
        std::vector<uint8_t> calculateScaleBuckets(std::span<const uint64_t> values, ScaleMode mode, std::optional<uint64_t> absoluteMaximum = std::nullopt);
        Projection project(const Viewport& viewport, bool windowCoordinates);
        void drawLinks(Gfx::DrawingContext& drawingCtx, std::span<const ProjectedLink> links);
        void drawMarkers(Gfx::DrawingContext& drawingCtx, std::span<const ProjectedMarker> markers);
        bool setTooltip(const Viewport& viewport, Point cursor);
    }
}
