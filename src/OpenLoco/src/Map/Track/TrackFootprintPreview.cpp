#include "Map/Track/TrackFootprintPreview.h"

#include "Map/Tile.h"
#include "Map/TileManager.h"
#include "Map/Track/SubpositionData.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include "Vehicles/Vehicle.h"

#include <OpenLoco/Math/Vector.hpp>

#include <cstddef>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

namespace OpenLoco::World::Track::TrackFootprintPreview
{
    namespace
    {
        constexpr size_t kMaxSearchNodes = 16384;

        // A flat straight has 32 axis-only movement steps, equal to 128 object-length units.
        constexpr uint32_t kMovementDistancePerTile = 32 * Vehicles::kMovementNibbleToDistance[1];

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

        struct DirectedTrack
        {
            World::Pos3 pos;
            uint16_t tad;
            CompanyId owner;
            uint8_t trackObjectId;

            bool operator<(const DirectedTrack& rhs) const
            {
                return std::tie(pos.x, pos.y, pos.z, tad, owner, trackObjectId)
                    < std::tie(rhs.pos.x, rhs.pos.y, rhs.pos.z, rhs.tad, rhs.owner, rhs.trackObjectId);
            }
        };

        struct SearchNode
        {
            DirectedTrack track;
            uint32_t remainingDistance;

            bool operator<(const SearchNode& rhs) const
            {
                return remainingDistance < rhs.remainingDistance;
            }
        };

        std::set<TrackElementKey> _preview;

        Vehicles::TrackAndDirection::_TrackAndDirection toTad(const uint16_t tad)
        {
            return Vehicles::TrackAndDirection::_TrackAndDirection((tad >> 3) & 0x3F, tad & 0x7);
        }

        World::Pos3 getCanonicalTrackStart(const DirectedTrack& track)
        {
            auto start = track.pos;
            const auto tad = toTad(track.tad);
            if (tad.isReversed())
            {
                const auto& trackSize = World::TrackData::getUnkTrack(tad._data);
                start += trackSize.pos;
                if (trackSize.rotationEnd < 12)
                {
                    start -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
                }
            }
            return start;
        }

        DirectedTrack makeDirectedTrack(const World::Pos3& trackStart, const uint8_t trackId, const uint8_t rotation, const bool reversed, const CompanyId owner, const uint8_t trackObjectId)
        {
            auto pos = trackStart;
            auto tad = static_cast<uint16_t>((trackId << 3) | rotation);
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
            return { pos, tad, owner, trackObjectId };
        }

        bool isSameTrackPiece(const DirectedTrack& lhs, const DirectedTrack& rhs)
        {
            const auto lhsTad = toTad(lhs.tad);
            const auto rhsTad = toTad(rhs.tad);
            return getCanonicalTrackStart(lhs) == getCanonicalTrackStart(rhs)
                && lhsTad.id() == rhsTad.id()
                && lhsTad.cardinalDirection() == rhsTad.cardinalDirection()
                && lhs.owner == rhs.owner
                && lhs.trackObjectId == rhs.trackObjectId;
        }

        void addTrackPiece(const DirectedTrack& track)
        {
            const auto tad = toTad(track.tad);
            const auto trackStart = getCanonicalTrackStart(track);
            for (const auto& piece : World::TrackData::getTrackPiece(tad.id()))
            {
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, tad.cardinalDirection());
                const auto pos = trackStart + World::Pos3{ offset, piece.z };
                _preview.insert({ pos, tad.id(), tad.cardinalDirection(), piece.index, track.owner, track.trackObjectId });
            }
        }

        uint32_t getMovementDistance(const DirectedTrack& from, const DirectedTrack& to)
        {
            const auto fromSubpositions = World::TrackData::getTrackSubPositon(from.tad);
            const auto toSubpositions = World::TrackData::getTrackSubPositon(to.tad);
            if (fromSubpositions.empty() || toSubpositions.empty())
            {
                return 0;
            }

            auto previous = from.pos + fromSubpositions.back().loc;
            uint32_t distance = 0;
            for (const auto& subposition : toSubpositions)
            {
                const auto next = to.pos + subposition.loc;
                distance += Vehicles::kMovementNibbleToDistance[Vehicles::getMovementNibble(previous, next)];
                previous = next;
            }
            return distance;
        }

        bool findFootprintFrom(const DirectedTrack& source, const uint32_t distance)
        {
            std::priority_queue<SearchNode> pending;
            std::map<DirectedTrack, uint32_t> bestRemainingDistance;
            pending.push({ source, distance });
            bestRemainingDistance.emplace(source, distance);
            size_t nodeCount = 1;

            while (!pending.empty())
            {
                const auto current = pending.top();
                pending.pop();
                if (bestRemainingDistance.at(current.track) != current.remainingDistance)
                {
                    continue;
                }

                const auto [nextPos, nextRotation] = World::Track::getTrackConnectionEnd(current.track.pos, current.track.tad);
                if (!World::validCoords(nextPos))
                {
                    continue;
                }
                const auto connections = World::Track::getTrackConnections(nextPos, nextRotation, current.track.owner, current.track.trackObjectId, 0, 0);
                for (const auto connection : connections.connections)
                {
                    const DirectedTrack next{ nextPos, static_cast<uint16_t>(connection & World::Track::AdditionalTaDFlags::basicTaDMask), current.track.owner, current.track.trackObjectId };
                    if (isSameTrackPiece(next, current.track) || isSameTrackPiece(next, source))
                    {
                        continue;
                    }

                    const auto movementDistance = getMovementDistance(current.track, next);
                    if (movementDistance == 0)
                    {
                        continue;
                    }

                    addTrackPiece(next);
                    if (movementDistance >= current.remainingDistance)
                    {
                        continue;
                    }

                    const auto remainingDistance = current.remainingDistance - movementDistance;
                    const auto [best, inserted] = bestRemainingDistance.emplace(next, remainingDistance);
                    if (!inserted && best->second >= remainingDistance)
                    {
                        continue;
                    }
                    if (nodeCount >= kMaxSearchNodes)
                    {
                        return false;
                    }
                    best->second = remainingDistance;
                    pending.push({ next, remainingDistance });
                    ++nodeCount;
                }
            }
            return true;
        }

        void invalidate(const std::set<TrackElementKey>& tracks)
        {
            for (const auto& track : tracks)
            {
                World::TileManager::mapInvalidateTileFull(track.pos);
            }
        }

        std::optional<std::pair<World::Pos3, World::TrackElement*>> getPlacementTrack(const SignalPlacement& placement)
        {
            if (!World::validCoords(placement.pos) || placement.trackId >= World::TrackData::kTrackPieceCount || placement.rotation >= 4)
            {
                return std::nullopt;
            }
            const auto pieces = World::TrackData::getTrackPiece(placement.trackId);
            if (placement.sequenceIndex >= pieces.size())
            {
                return std::nullopt;
            }

            const auto& piece = pieces[placement.sequenceIndex];
            const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, placement.rotation);
            const auto trackStart = placement.pos - World::Pos3{ offset, piece.z };
            for (const auto& entry : World::TileManager::get(placement.pos))
            {
                auto* track = entry.as<World::TrackElement>();
                if (track != nullptr
                    && track->baseHeight() == placement.pos.z
                    && track->trackId() == placement.trackId
                    && track->rotation() == placement.rotation
                    && track->sequenceIndex() == placement.sequenceIndex
                    && track->trackObjectId() == placement.trackObjectId
                    && !track->isGhost()
                    && !track->isAiAllocated())
                {
                    return std::pair{ trackStart, track };
                }
            }
            return std::nullopt;
        }
    }

    void updatePreview(const SignalPlacement& placement, const uint8_t lengthTiles)
    {
        invalidate(_preview);
        _preview.clear();
        if (lengthTiles == 0)
        {
            return;
        }

        const auto trackAndStart = getPlacementTrack(placement);
        if (!trackAndStart.has_value())
        {
            return;
        }

        const auto& [trackStart, track] = *trackAndStart;
        const auto distance = static_cast<uint32_t>(lengthTiles) * kMovementDistancePerTile;
        bool complete = true;
        if ((placement.sides & 0x8000) != 0)
        {
            complete = findFootprintFrom(makeDirectedTrack(trackStart, placement.trackId, placement.rotation, true, track->owner(), placement.trackObjectId), distance);
        }
        if (complete && (placement.sides & 0x4000) != 0)
        {
            complete = findFootprintFrom(makeDirectedTrack(trackStart, placement.trackId, placement.rotation, false, track->owner(), placement.trackObjectId), distance);
        }
        if (!complete)
        {
            _preview.clear();
        }
        invalidate(_preview);
    }

    void clearPreview()
    {
        invalidate(_preview);
        _preview.clear();
    }

    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track)
    {
        const TrackElementKey key{ World::Pos3{ pos, track.baseHeight() }, track.trackId(), track.rotation(), track.sequenceIndex(), track.owner(), track.trackObjectId() };
        return _preview.contains(key);
    }
}
