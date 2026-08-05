#include "Map/Track/DisconnectedTracks.h"
#include "Map/TileLoop.hpp"
#include "Map/TileManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include <OpenLoco/Math/Vector.hpp>
#include <set>
#include <tuple>

namespace OpenLoco::World::Track::DisconnectedTracks
{
    namespace
    {
        struct TrackPiece
        {
            World::Pos3 pos;
            uint8_t trackId;
            uint8_t rotation;
            CompanyId owner;
            uint8_t trackObjectId;
        };

        struct DirectedTrack
        {
            World::Pos3 pos;
            uint16_t tad;
        };

        struct TrackElementKey
        {
            World::Pos3 pos;
            uint8_t trackId;
            uint8_t rotation;
            uint8_t sequenceIndex;
            CompanyId owner;
            uint8_t trackObjectId;

            bool operator<(const TrackElementKey& rhs) const
            {
                return std::tie(pos.x, pos.y, pos.z, trackId, rotation, sequenceIndex, owner, trackObjectId)
                    < std::tie(rhs.pos.x, rhs.pos.y, rhs.pos.z, rhs.trackId, rhs.rotation, rhs.sequenceIndex, rhs.owner, rhs.trackObjectId);
            }
        };

        std::set<TrackElementKey> _audit;
        bool _auditDirty = true;

        constexpr uint8_t getTrackId(const uint16_t tad)
        {
            return (tad >> 3) & 0x3F;
        }

        constexpr uint8_t getTrackRotation(const uint16_t tad)
        {
            return tad & 0x3;
        }

        World::Pos3 getCanonicalTrackStart(const DirectedTrack& track)
        {
            auto start = track.pos;
            if ((track.tad & (1U << 2)) != 0)
            {
                const auto& trackSize = World::TrackData::getUnkTrack(track.tad);
                start += trackSize.pos;
                if (trackSize.rotationEnd < 12)
                {
                    start -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
                }
            }
            return start;
        }

        DirectedTrack makeDirectedTrack(const TrackPiece& track, const bool reversed)
        {
            auto pos = track.pos;
            auto tad = static_cast<uint16_t>((track.trackId << 3) | track.rotation);
            if (reversed)
            {
                const auto& trackSize = World::TrackData::getUnkTrack(tad);
                pos += trackSize.pos;
                if (trackSize.rotationEnd < 12)
                {
                    pos -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
                }
                tad |= 1U << 2;
            }
            return { pos, tad };
        }

        bool isSameTrackPiece(const DirectedTrack& directed, const TrackPiece& track)
        {
            return getCanonicalTrackStart(directed) == track.pos
                && getTrackId(directed.tad) == track.trackId
                && getTrackRotation(directed.tad) == track.rotation;
        }

        bool hasExternalConnection(const TrackPiece& track, const bool reversed)
        {
            const auto directed = makeDirectedTrack(track, reversed);
            const auto [nextPos, nextRotation] = World::Track::getTrackConnectionEnd(directed.pos, directed.tad);
            if (!World::validCoords(nextPos))
            {
                return false;
            }
            const auto connections = World::Track::getTrackConnections(nextPos, nextRotation, track.owner, track.trackObjectId, 0, 0);
            for (const auto connection : connections.connections)
            {
                const DirectedTrack candidate{ nextPos, static_cast<uint16_t>(connection & World::Track::AdditionalTaDFlags::basicTaDMask) };
                if (!isSameTrackPiece(candidate, track))
                {
                    return true;
                }
            }
            return false;
        }

        void addTrackPiece(const TrackPiece& track)
        {
            for (const auto& piece : World::TrackData::getTrackPiece(track.trackId))
            {
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, track.rotation);
                const auto pos = track.pos + World::Pos3{ offset, piece.z };
                _audit.insert({ pos, track.trackId, track.rotation, piece.index, track.owner, track.trackObjectId });
            }
        }

        void invalidate(const std::set<TrackElementKey>& tracks)
        {
            for (const auto& track : tracks)
            {
                World::TileManager::mapInvalidateTileFull(track.pos);
            }
        }
    }

    void refreshAudit()
    {
        invalidate(_audit);
        _audit.clear();
        for (const auto tilePos : World::getWorldRange())
        {
            const auto pos = World::toWorldSpace(tilePos);
            for (const auto& entry : World::TileManager::get(tilePos))
            {
                const auto* track = entry.as<World::TrackElement>();
                if (track == nullptr || track->sequenceIndex() != 0 || track->isGhost() || track->isAiAllocated() || track->trackId() >= World::TrackData::kTrackPieceCount)
                {
                    continue;
                }

                const auto& piece = World::TrackData::getTrackPiece(track->trackId())[0];
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, track->rotation());
                const TrackPiece trackPiece{
                    World::Pos3{ pos, track->baseHeight() } - World::Pos3{ offset, piece.z },
                    track->trackId(),
                    track->rotation(),
                    track->owner(),
                    track->trackObjectId(),
                };
                const auto beginConnected = hasExternalConnection(trackPiece, true);
                const auto endConnected = hasExternalConnection(trackPiece, false);
                if (!beginConnected || !endConnected)
                {
                    addTrackPiece(trackPiece);
                }
            }
        }
        _auditDirty = false;
        invalidate(_audit);
    }

    void refreshAuditIfDirty()
    {
        if (_auditDirty)
        {
            refreshAudit();
        }
    }

    void invalidateAudit()
    {
        _auditDirty = true;
        invalidate(_audit);
        _audit.clear();
    }

    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track)
    {
        const TrackElementKey key{ World::Pos3{ pos, track.baseHeight() }, track.trackId(), track.rotation(), track.sequenceIndex(), track.owner(), track.trackObjectId() };
        return _audit.contains(key);
    }
}
