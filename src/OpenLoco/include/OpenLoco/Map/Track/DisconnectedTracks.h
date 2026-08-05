#pragma once

#include <OpenLoco/Engine/World.hpp>

namespace OpenLoco::World
{
    struct TrackElement;
}

namespace OpenLoco::World::Track::DisconnectedTracks
{
    void refreshAudit();
    void refreshAuditIfDirty();
    void invalidateAudit();
    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track);
}
