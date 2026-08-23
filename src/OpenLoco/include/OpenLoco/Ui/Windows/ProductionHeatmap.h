// SPDX-License-Identifier: MIT
#pragma once

#include "Graphics/Colour.h"
#include "Map/Tile.h"
#include <OpenLoco/Engine/Ui/Point.hpp>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace OpenLoco::Ui
{
    struct Viewport;
    struct Window;

    namespace Windows::ProductionHeatmap
    {
        constexpr uint8_t kBucketCount = 8;
        constexpr uint8_t kCatchmentRadius = 4;

        enum class Mode : uint8_t
        {
            physicalProduction,
            stationPotential,
        };

        struct ProductionSource
        {
            uint64_t monthlyProductionScaled{};
            std::vector<World::TilePos2> footprint;
        };

        struct HeatmapLayer
        {
            std::vector<uint64_t> values;
            std::vector<uint8_t> buckets;
        };

        struct HeatmapLayers
        {
            HeatmapLayer physical;
            HeatmapLayer stationPotential;
        };

        std::array<uint64_t, kBucketCount - 1> calculatePercentileThresholds(std::span<const uint64_t> values);
        uint8_t getPercentileBucket(uint64_t value, const std::array<uint64_t, kBucketCount - 1>& thresholds);
        HeatmapLayers buildProductionLayers(std::span<const ProductionSource> sources, uint16_t width, uint16_t height, uint8_t catchmentRadius = kCatchmentRadius, bool excludeNonDrawableBorder = false);

        Window* open();
        void toggle();
        bool isOpen();
        bool hasEnabledCargo();
        uint8_t getTileBucket(const World::Pos2& pos);
        uint64_t getTileValue(const World::Pos2& pos);
        Colour getBucketColour(uint8_t bucket);
        bool setTooltip(const Viewport& viewport, Point cursor);
    }
}
