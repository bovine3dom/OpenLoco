// SPDX-License-Identifier: MIT
#pragma once

#include "Graphics/Colour.h"
#include "Speed.hpp"
#include <OpenLoco/Engine/Ui/Point.hpp>
#include <OpenLoco/Engine/World.hpp>
#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace OpenLoco::World
{
    struct TrackElement;
}

namespace OpenLoco::Ui
{
    struct Viewport;
    struct Window;

    namespace Windows::RailSpeedOverlay
    {
        constexpr uint8_t kBucketCount = 8;
        using SpeedThresholds = std::array<uint64_t, kBucketCount - 1>;

        Window* open();
        void toggle();
        bool isOpen();
        std::optional<Speed16> getTrackSpeed(const World::Pos2& pos, const World::TrackElement& track);
        SpeedThresholds calculateSpeedPercentileThresholds(std::span<const Speed16> speeds);
        uint8_t getSpeedBucket(Speed16 speed);
        uint8_t getSpeedBucket(Speed16 speed, const SpeedThresholds& thresholds);
        Colour getBucketColour(uint8_t bucket);
        bool setTooltip(const Viewport& viewport, Point cursor);
    }
}
