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
        struct ProjectedLink
        {
            Point from;
            Point to;
            PaletteIndex_t colour{};
        };

        Window* open();
        void toggle();
        bool isOpen();
        bool hasEnabledCargo();

        uint8_t getSaturationBucket(uint64_t plannedDemand, std::optional<uint32_t> capacity);
        std::vector<ProjectedLink> projectLinks(const Viewport& viewport, bool windowCoordinates);
        void drawLinks(Gfx::DrawingContext& drawingCtx, std::span<const ProjectedLink> links);
        bool setTooltip(const Viewport& viewport, Point cursor);
    }
}
