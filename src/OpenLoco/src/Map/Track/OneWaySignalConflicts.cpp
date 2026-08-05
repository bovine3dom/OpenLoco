#include "Map/Track/OneWaySignalConflicts.h"
#include "Map/SignalElement.h"
#include "Map/Tile.h"
#include "Map/TileLoop.hpp"
#include "Map/TileManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include "Vehicles/Vehicle.h"
#include <OpenLoco/Math/Vector.hpp>
#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace OpenLoco::World::Track::OneWaySignalConflicts
{
    namespace
    {
        constexpr size_t kMaxSearchNodes = 16384;

        enum class SearchMode
        {
            fromSource,
            toProposedSignal,
        };

        struct TrackElementKey
        {
            World::Pos3 pos;
            uint8_t trackId;
            uint8_t rotation;
            uint8_t sequenceIndex;
            uint8_t trackObjectId;

            bool operator<(const TrackElementKey& rhs) const
            {
                return std::tie(pos.x, pos.y, pos.z, trackId, rotation, sequenceIndex, trackObjectId)
                    < std::tie(rhs.pos.x, rhs.pos.y, rhs.pos.z, rhs.trackId, rhs.rotation, rhs.sequenceIndex, rhs.trackObjectId);
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
                return std::tie(pos.x, pos.y, pos.z, tad) < std::tie(rhs.pos.x, rhs.pos.y, rhs.pos.z, rhs.tad);
            }
        };

        struct SearchNode
        {
            DirectedTrack track;
            std::vector<uint16_t> parents;
            bool isConflict;
        };

        std::set<TrackElementKey> _audit;
        std::set<TrackElementKey> _preview;
        bool _auditDirty = true;

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

        DirectedTrack makeDirectedTrack(const World::Pos3& trackStart, const uint8_t trackId, const uint8_t rotation, const bool isRight, const CompanyId owner, const uint8_t trackObjectId)
        {
            auto pos = trackStart;
            auto tad = static_cast<uint16_t>((trackId << 3) | rotation);
            if (isRight)
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
                && lhs.trackObjectId == rhs.trackObjectId;
        }

        void addTrackPiece(std::set<TrackElementKey>& result, const DirectedTrack& track)
        {
            const auto tad = toTad(track.tad);
            const auto trackStart = getCanonicalTrackStart(track);
            for (const auto& piece : World::TrackData::getTrackPiece(tad.id()))
            {
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, tad.cardinalDirection());
                const auto pos = trackStart + World::Pos3{ offset, piece.z };
                result.insert({ pos, tad.id(), tad.cardinalDirection(), piece.index, track.trackObjectId });
            }
        }

        void findConflictsFrom(std::set<TrackElementKey>& result, const DirectedTrack& source, const SearchMode mode = SearchMode::fromSource)
        {
            std::vector<SearchNode> nodes{ { source, {}, false } };
            std::map<DirectedTrack, uint16_t> nodeIndices{ { source, 0 } };
            std::vector<uint16_t> pending{ 0 };
            bool truncated = false;
            while (!pending.empty())
            {
                const auto index = pending.back();
                pending.pop_back();
                const auto current = nodes[index].track;
                const auto [nextPos, nextRotation] = World::Track::getTrackConnectionEnd(current.pos, current.tad);
                const auto connections = World::Track::getTrackConnections(nextPos, nextRotation, current.owner, current.trackObjectId, 0, 0);
                for (const auto connection : connections.connections)
                {
                    DirectedTrack next{ nextPos, static_cast<uint16_t>(connection & World::Track::AdditionalTaDFlags::basicTaDMask), current.owner, current.trackObjectId };
                    if (isSameTrackPiece(next, source))
                    {
                        continue;
                    }

                    const auto tad = toTad(next.tad);
                    const auto facingMode = Vehicles::getSignalMode(next.pos, tad, next.trackObjectId, 0);
                    const auto reverseMode = Vehicles::getSignalMode(next.pos, tad, next.trackObjectId, 1U << 31);
                    const auto isConflict = reverseMode == World::SignalMode::oneWayPath
                        && (mode == SearchMode::toProposedSignal || !facingMode.has_value());
                    const auto isPassable = mode == SearchMode::fromSource
                        ? !facingMode.has_value() && (reverseMode == World::SignalMode::path || !reverseMode.has_value())
                        : !reverseMode.has_value() && (facingMode == World::SignalMode::path || !facingMode.has_value());
                    if (!isConflict && !isPassable)
                    {
                        continue;
                    }

                    const auto existing = nodeIndices.find(next);
                    if (existing != nodeIndices.end())
                    {
                        auto& parents = nodes[existing->second].parents;
                        if (std::ranges::find(parents, index) == parents.end())
                        {
                            parents.push_back(index);
                        }
                        continue;
                    }
                    if (nodes.size() >= kMaxSearchNodes)
                    {
                        truncated = true;
                        continue;
                    }

                    const auto nextIndex = static_cast<uint16_t>(nodes.size());
                    nodeIndices.emplace(next, nextIndex);
                    nodes.push_back({ next, { index }, isConflict });
                    if (isPassable)
                    {
                        pending.push_back(nextIndex);
                    }
                }
            }
            if (truncated)
            {
                return;
            }

            std::vector<bool> marked(nodes.size());
            std::vector<uint16_t> toMark;
            for (uint16_t i = 0; i < nodes.size(); ++i)
            {
                if (nodes[i].isConflict)
                {
                    toMark.push_back(i);
                }
            }
            while (!toMark.empty())
            {
                const auto index = toMark.back();
                toMark.pop_back();
                if (marked[index])
                {
                    continue;
                }
                marked[index] = true;
                toMark.insert(toMark.end(), nodes[index].parents.begin(), nodes[index].parents.end());
            }
            for (uint16_t i = 0; i < nodes.size(); ++i)
            {
                if (marked[i])
                {
                    addTrackPiece(result, nodes[i].track);
                }
            }
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
            const auto pieces = World::TrackData::getTrackPiece(placement.trackId);
            if (placement.sequenceIndex >= pieces.size())
            {
                return std::nullopt;
            }
            const auto& piece = pieces[placement.sequenceIndex];
            const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, placement.rotation);
            const auto trackStart = placement.pos - World::Pos3{ offset, piece.z };
            const auto tile = World::TileManager::get(placement.pos);
            for (const auto& entry : tile)
            {
                auto* track = entry.as<World::TrackElement>();
                if (track != nullptr
                    && track->baseHeight() == placement.pos.z
                    && track->trackId() == placement.trackId
                    && track->rotation() == placement.rotation
                    && track->sequenceIndex() == placement.sequenceIndex
                    && track->trackObjectId() == placement.trackObjectId)
                {
                    return std::pair{ trackStart, track };
                }
            }
            return std::nullopt;
        }
    }

    void refreshAudit()
    {
        invalidate(_audit);
        _audit.clear();
        for (const auto tilePos : World::getDrawableTileRange())
        {
            const auto pos = World::toWorldSpace(tilePos);
            for (const auto& entry : World::TileManager::get(tilePos))
            {
                const auto* track = entry.as<World::TrackElement>();
                if (track == nullptr || track->sequenceIndex() != 0 || !track->hasSignal() || track->isGhost() || track->isAiAllocated())
                {
                    continue;
                }
                const auto* signal = entry.next()->as<World::SignalElement>();
                if (signal == nullptr || signal->isGhost() || signal->isAiAllocated())
                {
                    continue;
                }

                const auto& piece = World::TrackData::getTrackPiece(track->trackId())[0];
                const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, track->rotation());
                const auto trackStart = World::Pos3{ pos, track->baseHeight() } - World::Pos3{ offset, piece.z };
                if (signal->getLeft().hasSignal() && !signal->isLeftGhost() && track->leftSignalMode() == World::SignalMode::oneWayPath)
                {
                    findConflictsFrom(_audit, makeDirectedTrack(trackStart, track->trackId(), track->rotation(), false, track->owner(), track->trackObjectId()));
                }
                if (signal->getRight().hasSignal() && !signal->isRightGhost() && track->rightSignalMode() == World::SignalMode::oneWayPath)
                {
                    findConflictsFrom(_audit, makeDirectedTrack(trackStart, track->trackId(), track->rotation(), true, track->owner(), track->trackObjectId()));
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
        clearPreview();
    }

    void updatePreview(const SignalPlacement& placement)
    {
        invalidate(_preview);
        _preview.clear();

        const auto trackAndStart = getPlacementTrack(placement);
        if (!trackAndStart.has_value())
        {
            return;
        }
        const auto& [trackStart, track] = *trackAndStart;
        if ((placement.sides & 0x8000) != 0)
        {
            const auto source = makeDirectedTrack(trackStart, placement.trackId, placement.rotation, false, track->owner(), placement.trackObjectId);
            findConflictsFrom(_preview, source);
            findConflictsFrom(_preview, source, SearchMode::toProposedSignal);
        }
        if ((placement.sides & 0x4000) != 0)
        {
            const auto source = makeDirectedTrack(trackStart, placement.trackId, placement.rotation, true, track->owner(), placement.trackObjectId);
            findConflictsFrom(_preview, source);
            findConflictsFrom(_preview, source, SearchMode::toProposedSignal);
        }
        invalidate(_preview);
    }

    void clearPreview()
    {
        invalidate(_preview);
        _preview.clear();
    }

    bool isHighlighted(const World::Pos2& pos, const World::TrackElement& track, const bool includeAudit)
    {
        const TrackElementKey key{ World::Pos3{ pos, track.baseHeight() }, track.trackId(), track.rotation(), track.sequenceIndex(), track.trackObjectId() };
        return _preview.contains(key) || (includeAudit && _audit.contains(key));
    }
}
