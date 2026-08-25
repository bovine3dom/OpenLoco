// SPDX-License-Identifier: MIT
#pragma once

#include "Graphics/Colour.h"
#include "Speed.hpp"
#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <optional>

namespace OpenLoco::World
{
    struct TrackElement;
}

namespace OpenLoco::Ui
{
    struct Window;

    namespace Windows::RailSpeedOverlay
    {
        Window* open();
        void toggle();
        bool isOpen();
        std::optional<Speed16> getTrackSpeed(const World::Pos2& pos, const World::TrackElement& track);
        uint8_t getSpeedBucket(Speed16 speed);
        Colour getBucketColour(uint8_t bucket);
    }
}
