#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstdint>

namespace OpenLoco::World
{
    struct TrackElement;
}

namespace OpenLoco::World::Track::OneWaySignalConflicts
{
    struct SignalPlacement
    {
        World::Pos3 pos;
        uint16_t sides;
        uint8_t trackId;
        uint8_t rotation;
        uint8_t sequenceIndex;
        uint8_t trackObjectId;
    };

    void refreshAudit();
    void refreshAuditIfDirty();
    void invalidateAudit();
    void updatePreview(const SignalPlacement& placement);
    void clearPreview();
    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track, bool includeAudit);
}
