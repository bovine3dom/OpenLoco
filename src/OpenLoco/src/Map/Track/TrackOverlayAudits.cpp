#include "Map/Track/TrackOverlayAudits.h"
#include "Map/Track/DisconnectedTracks.h"
#include "Map/Track/OneWaySignalConflicts.h"

namespace OpenLoco::World::Track::TrackOverlayAudits
{
    void refreshAuditIfDirty()
    {
        OneWaySignalConflicts::refreshAuditIfDirty();
        DisconnectedTracks::refreshAuditIfDirty();
    }

    void invalidateAudit()
    {
        OneWaySignalConflicts::invalidateAudit();
        DisconnectedTracks::invalidateAudit();
    }
}
