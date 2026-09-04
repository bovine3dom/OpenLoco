#pragma once

#include "Map/Track/OneWaySignalConflicts.h"

#include <cstdint>

namespace OpenLoco::World
{
    struct TrackElement;
}

namespace OpenLoco::World::Track::TrackFootprintPreview
{
    using SignalPlacement = OneWaySignalConflicts::SignalPlacement;

    void updatePreview(const SignalPlacement& placement, uint8_t lengthTiles);
    void clearPreview();
    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track);
}
