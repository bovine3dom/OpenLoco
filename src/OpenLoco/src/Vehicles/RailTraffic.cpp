#include "Vehicles/RailTraffic.h"

#include "Date.h"
#include "Entities/EntityManager.h"
#include "GameState.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Objects/BridgeObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/TrackObject.h"
#include "Scenario/ScenarioManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleHead.h"
#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_map>

using namespace OpenLoco::Literals;

namespace OpenLoco::Vehicles::RailTraffic
{
    namespace
    {
        constexpr uint8_t kMaxConfidence = 16;
        constexpr uint8_t kPriorConfidence = 4;
        constexpr uint32_t kOverlayHistoryRetentionDays = 30;
        constexpr uint64_t kLandModeModifier = 21;
        constexpr TravelTime kMaxTraversalTime = static_cast<TravelTime>(std::numeric_limits<int32_t>::max()) << kFractionBits;
        constexpr TravelTime kTimestampMask = (TravelTime{ 1 } << 48) - 1;

        struct EdgeHash
        {
            size_t operator()(const Edge& edge) const
            {
                uint64_t value = static_cast<uint16_t>(edge.x)
                    | (static_cast<uint64_t>(static_cast<uint16_t>(edge.y)) << 16)
                    | (static_cast<uint64_t>(static_cast<uint16_t>(edge.z)) << 32)
                    | (static_cast<uint64_t>(edge.tad) << 48);
                value ^= static_cast<uint64_t>(edge.trackType) * 0x9E3779B97F4A7C15ULL;
                value ^= value >> 30;
                value *= 0xBF58476D1CE4E5B9ULL;
                value ^= value >> 27;
                return static_cast<size_t>(value ^ (value >> 31));
            }
        };

        struct ActiveAggregate
        {
            TravelTime elapsed{};
            uint16_t count{};
        };

        std::unordered_map<Edge, HistoryEntry, EdgeHash> _history;
        std::unordered_map<EntityId, ActiveTraversal> _active;
        std::unordered_map<Edge, ActiveAggregate, EdgeHash> _publishedActive;
        std::vector<std::pair<Edge, TravelTime>> _pending;
        bool _inTick{};
        uint32_t _historyRevision{};

        TravelTime add(const TravelTime lhs, const TravelTime rhs)
        {
            return rhs > std::numeric_limits<TravelTime>::max() - lhs
                ? std::numeric_limits<TravelTime>::max()
                : lhs + rhs;
        }

        TravelTime multiply(const TravelTime lhs, const uint64_t rhs)
        {
            return rhs != 0 && lhs > std::numeric_limits<TravelTime>::max() / rhs
                ? std::numeric_limits<TravelTime>::max()
                : lhs * rhs;
        }

        TravelTime currentTime(const uint32_t fraction = 0)
        {
            const auto clampedFraction = std::min<TravelTime>(fraction, kOneTick);
            const auto tick = ScenarioManager::getScenarioTicks() - static_cast<uint32_t>(_inTick) + static_cast<uint32_t>(clampedFraction / kOneTick);
            return (static_cast<TravelTime>(tick) << kFractionBits) | (clampedFraction % kOneTick);
        }

        TravelTime elapsedSince(const TravelTime enteredAt, const TravelTime now)
        {
            auto ticks = static_cast<uint32_t>(now >> kFractionBits) - static_cast<uint32_t>(enteredAt >> kFractionBits);
            const auto nowFraction = static_cast<uint16_t>(now);
            const auto enteredFraction = static_cast<uint16_t>(enteredAt);
            if (nowFraction < enteredFraction)
            {
                ticks--;
            }
            return (static_cast<TravelTime>(ticks) << kFractionBits)
                + static_cast<uint16_t>(nowFraction - enteredFraction);
        }

        VehicleHead* getHead(const Vehicle1& vehicle)
        {
            auto* base = EntityManager::get<VehicleBase>(vehicle.head);
            return base != nullptr && base->isVehicleHead() ? base->asVehicleHead() : nullptr;
        }

        bool isActive(const Vehicle1& vehicle)
        {
            const auto* head = getHead(vehicle);
            return head != nullptr && vehicle.mode == TransportMode::rail && head->isPlaced();
        }

        bool isValidEdge(const Edge& edge)
        {
            return edge.x >= 0 && edge.x < World::kMapWidth
                && edge.y >= 0 && edge.y < World::kMapHeight
                && edge.z >= 0 && edge.z <= std::numeric_limits<World::SmallZ>::max() * World::kSmallZStep
                && edge.trackType < Limits::kMaxTrackObjects
                && edge.tad < 44 * 8;
        }

        bool edgeLess(const Edge& lhs, const Edge& rhs)
        {
            return std::tie(lhs.x, lhs.y, lhs.z, lhs.trackType, lhs.tad)
                < std::tie(rhs.x, rhs.y, rhs.z, rhs.trackType, rhs.tad);
        }

        void applyTraversal(const Edge& edge, const TravelTime duration)
        {
            if (!isValidEdge(edge) || duration == 0 || duration > kMaxTraversalTime)
            {
                return;
            }

            const auto existing = _history.find(edge);
            if (existing == _history.end() && _history.size() >= kMaxHistoryEntries)
            {
                return;
            }
            auto [it, inserted] = _history.try_emplace(edge, HistoryEntry{ edge, duration, getCurrentDay(), 1 });
            if (inserted)
            {
                _historyRevision++;
                return;
            }

            auto& entry = it->second;
            if (entry.confidence < kMaxConfidence)
            {
                const auto total = add(multiply(entry.meanTraversalTime, entry.confidence), duration);
                entry.confidence++;
                entry.meanTraversalTime = total / entry.confidence;
            }
            else if (duration >= entry.meanTraversalTime)
            {
                entry.meanTraversalTime += (duration - entry.meanTraversalTime) / kMaxConfidence;
            }
            else
            {
                entry.meanTraversalTime -= (entry.meanTraversalTime - duration) / kMaxConfidence;
            }
            entry.lastObservedDay = getCurrentDay();
            _historyRevision++;
        }

        Speed16 getFreeFlowSpeed(const SpeedProfile& profile, const uint16_t routing, const uint8_t trackType)
        {
            auto speed = profile.brokenDown ? profile.maxSpeed / 4 : profile.maxSpeed;
            TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
            tad._data = routing & World::Track::AdditionalTaDFlags::basicTaDMask;
            const auto* trackObject = ObjectManager::get<TrackObject>(trackType);
            const auto applyCurveBonus = [&](Speed16 speedLimit) {
                if (profile.fasterAroundCurves && trackObject != nullptr)
                {
                    speedLimit += speedLimit / 4;
                    speedLimit = std::min(speedLimit, trackObject->curveSpeed);
                }
                return speedLimit;
            };
            if (trackObject != nullptr)
            {
                const auto fraction = World::TrackData::getTrackMiscData(tad.id()).curveSpeedFraction;
                const Speed32 fractionalSpeed{ static_cast<int32_t>(static_cast<uint32_t>(fraction) * trackObject->curveSpeed.getRaw()) };
                speed = std::min(speed, applyCurveBonus(toSpeed16(fractionalSpeed + 1.0_mph)));
            }
            if ((routing & World::Track::AdditionalTaDFlags::hasBridge) != 0)
            {
                const auto* bridge = ObjectManager::get<BridgeObject>((routing & World::Track::AdditionalTaDFlags::bridgeMask) >> 9);
                if (bridge != nullptr && bridge->maxSpeed != kSpeed16Null)
                {
                    speed = std::min(speed, applyCurveBonus(bridge->maxSpeed));
                }
            }
            if ((routing & World::Track::AdditionalTaDFlags::hasMods) != 0)
            {
                speed = std::min(speed, profile.rackRailMaxSpeed);
            }
            return speed;
        }

        TravelTime distanceToTime(const uint64_t distance, const Speed16 speed)
        {
            if (speed <= 0_mph)
            {
                return std::numeric_limits<TravelTime>::max();
            }
            const auto numerator = multiply(multiply(distance, kLandModeModifier), kOneTick);
            return add(numerator, static_cast<uint16_t>(speed.getRaw()) - 1) / static_cast<uint16_t>(speed.getRaw());
        }

        bool isCurrentTraversal(const ActiveTraversal& traversal)
        {
            auto* vehicle = EntityManager::get<VehicleBase>(traversal.vehicle);
            return vehicle != nullptr && vehicle->isVehicle1() && isActive(*vehicle->asVehicle1())
                && vehicle->head == traversal.head && getEdge(*vehicle) == traversal.edge;
        }

        void sweepActive()
        {
            std::erase_if(_active, [](const auto& item) { return !isCurrentTraversal(item.second); });
        }

        void publishActive()
        {
            _publishedActive.clear();
            const auto now = currentTime();
            for (const auto& [_, traversal] : _active)
            {
                auto& aggregate = _publishedActive[traversal.edge];
                const auto elapsed = elapsedSince(traversal.enteredAt, now);
                aggregate.elapsed = add(aggregate.elapsed, elapsed <= kMaxTraversalTime ? elapsed : 0);
                aggregate.count++;
            }
        }
    }

    Edge getEdge(const VehicleBase& vehicle)
    {
        return {
            vehicle.tileX,
            vehicle.tileY,
            static_cast<int16_t>(vehicle.tileBaseZ * World::kSmallZStep),
            static_cast<uint16_t>(vehicle.trackAndDirection.track._data & World::Track::AdditionalTaDFlags::basicTaDMask),
            vehicle.trackType,
        };
    }

    SpeedProfile getSpeedProfile(const VehicleHead& head)
    {
        const Vehicle train(head);
        return {
            train.veh2->maxSpeed,
            train.veh2->rackRailMaxSpeed,
            head.has38Flags(Flags38::fasterAroundCurves),
            train.veh2->has73Flags(Flags73::isBrokenDown),
        };
    }

    TravelTime getFreeFlowTime(const SpeedProfile& profile, const uint16_t routing, const uint8_t trackType)
    {
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & World::Track::AdditionalTaDFlags::basicTaDMask;
        return distanceToTime(World::TrackData::getTrackMiscData(tad.id()).unkWeighting, getFreeFlowSpeed(profile, routing, trackType));
    }

    TravelTime getTravelTime(const SpeedProfile& profile, const World::Pos3& pos, const uint16_t routing, const uint8_t trackType)
    {
        const auto freeFlow = getFreeFlowTime(profile, routing, trackType);
        const Edge edge{ pos.x, pos.y, pos.z, static_cast<uint16_t>(routing & World::Track::AdditionalTaDFlags::basicTaDMask), trackType };
        const auto history = _history.find(edge);
        const auto active = _publishedActive.find(edge);
        if (history == _history.end() && active == _publishedActive.end())
        {
            return freeFlow;
        }

        TravelTime weightedTime{};
        uint32_t weight{};
        if (history != _history.end())
        {
            weightedTime = multiply(history->second.meanTraversalTime, history->second.confidence);
            weight = history->second.confidence;
        }
        if (active != _publishedActive.end())
        {
            weightedTime = add(weightedTime, std::max(active->second.elapsed, multiply(freeFlow, active->second.count)));
            weight += active->second.count;
        }
        weightedTime = add(weightedTime, multiply(freeFlow, kPriorConfidence));
        weight += kPriorConfidence;
        return std::max(freeFlow, weightedTime / weight);
    }

    TravelTime getHeuristicTime(const SpeedProfile& profile, const uint32_t distance)
    {
        const auto speed = profile.brokenDown ? profile.maxSpeed / 4 : profile.maxSpeed;
        return distanceToTime(distance, speed);
    }

    TravelTime getLiveSignalPenalty(const SpeedProfile& profile, const uint16_t routing, const uint8_t trackType)
    {
        return std::min(kOneTick, getFreeFlowTime(profile, routing, trackType));
    }

    std::optional<Speed16> getAverageSpeed(const Edge& edge)
    {
        const auto it = _history.find(edge);
        if (it == _history.end())
        {
            return std::nullopt;
        }

        const auto distance = World::TrackData::getTrackMiscData(edge.tad >> 3).unkWeighting;
        const auto numerator = multiply(multiply(distance, kLandModeModifier), kOneTick);
        const auto speed = add(numerator, it->second.meanTraversalTime / 2) / it->second.meanTraversalTime;
        return Speed16(static_cast<int16_t>(std::min<uint64_t>(speed, kSpeed16Max.getRaw())));
    }

    uint32_t getHistoryRevision()
    {
        return _historyRevision;
    }

    void reset()
    {
        _history.clear();
        _active.clear();
        _publishedActive.clear();
        _pending.clear();
        _inTick = false;
        _historyRevision++;
    }

    void beginTick()
    {
        _inTick = true;
        using VehicleComponentList = EntityManager::EntityList<EntityManager::EntityListIterator<VehicleBase>, EntityManager::EntityListType::vehicle>;
        for (auto* component : VehicleComponentList{})
        {
            if (component->isVehicle1())
            {
                reconcile(*component->asVehicle1());
            }
        }
        sweepActive();
        publishActive();
    }

    void endTick()
    {
        _inTick = false;
        for (const auto& [edge, duration] : _pending)
        {
            applyTraversal(edge, duration);
        }
        _pending.clear();
        sweepActive();
        publishActive();
    }

    void updateDaily()
    {
        const auto today = getCurrentDay();
        bool erased = false;
        for (auto it = _history.begin(); it != _history.end();)
        {
            if (it->second.lastObservedDay != today && it->second.confidence != 0)
            {
                it->second.confidence--;
            }
            if (it->second.lastObservedDay > today
                || today - it->second.lastObservedDay > kOverlayHistoryRetentionDays)
            {
                it = _history.erase(it);
                erased = true;
            }
            else
            {
                ++it;
            }
        }
        _historyRevision += erased;
    }

    void reconcile(Vehicle1& vehicle)
    {
        if (!isActive(vehicle))
        {
            _active.erase(vehicle.id);
            return;
        }

        const auto edge = getEdge(vehicle);
        const auto now = currentTime();
        const auto head = vehicle.head;
        const auto it = _active.find(vehicle.id);
        if (it == _active.end())
        {
            _active.emplace(vehicle.id, ActiveTraversal{ vehicle.id, head, edge, now, false });
        }
        else if (it->second.head != head || it->second.edge != edge)
        {
            it->second = { vehicle.id, head, edge, now, false };
        }
    }

    void restart(Vehicle1& vehicle)
    {
        _active.erase(vehicle.id);
        reconcile(vehicle);
        if (!_inTick)
        {
            publishActive();
        }
    }

    void onPieceTransition(Vehicle1& vehicle, const Edge& previous, const Edge& next, const uint32_t tickFraction)
    {
        const auto when = currentTime(tickFraction);
        const auto it = _active.find(vehicle.id);
        if (it != _active.end() && it->second.head == vehicle.head && it->second.edge == previous && it->second.completeFromStart)
        {
            const auto elapsed = elapsedSince(it->second.enteredAt, when);
            if (elapsed != 0 && elapsed <= kMaxTraversalTime)
            {
                recordTraversal(previous, elapsed);
            }
        }
        _active[vehicle.id] = { vehicle.id, vehicle.head, next, when, true };
    }

    void recordTraversal(const Edge& edge, const TravelTime duration)
    {
        if (_inTick)
        {
            _pending.emplace_back(edge, duration);
        }
        else
        {
            applyTraversal(edge, duration);
        }
    }

    State captureState()
    {
        State state;
        state.history.reserve(_history.size());
        for (const auto& [_, entry] : _history)
        {
            state.history.push_back(entry);
        }
        std::ranges::sort(state.history, [](const auto& lhs, const auto& rhs) { return edgeLess(lhs.edge, rhs.edge); });
        state.active.reserve(_active.size());
        for (const auto& [_, traversal] : _active)
        {
            if (isCurrentTraversal(traversal))
            {
                state.active.push_back(traversal);
            }
        }
        std::ranges::sort(state.active, {}, &ActiveTraversal::vehicle);
        return state;
    }

    bool validateState(const State& state)
    {
        if (state.history.size() > kMaxHistoryEntries || state.active.size() > Limits::kMaxVehicles)
        {
            return false;
        }
        std::unordered_map<Edge, bool, EdgeHash> edges;
        for (const auto& entry : state.history)
        {
            if (!isValidEdge(entry.edge) || entry.meanTraversalTime == 0 || entry.meanTraversalTime > kMaxTraversalTime
                || entry.confidence > kMaxConfidence || !edges.emplace(entry.edge, true).second)
            {
                return false;
            }
        }
        std::unordered_map<EntityId, bool> vehicles;
        for (const auto& traversal : state.active)
        {
            if (traversal.vehicle == EntityId::null || traversal.head == EntityId::null
                || enumValue(traversal.vehicle) >= Limits::kMaxEntities || enumValue(traversal.head) >= Limits::kMaxEntities
                || !isValidEdge(traversal.edge) || (traversal.enteredAt & ~kTimestampMask) != 0
                || !vehicles.emplace(traversal.vehicle, true).second)
            {
                return false;
            }
        }
        return true;
    }

    bool validateState(const State& state, const GameState& gameState)
    {
        if (!validateState(state))
        {
            return false;
        }

        const auto now = static_cast<TravelTime>(gameState.scenarioTicks) << kFractionBits;
        for (const auto& entry : state.history)
        {
            if (entry.lastObservedDay > gameState.currentDay)
            {
                return false;
            }
        }
        for (const auto& traversal : state.active)
        {
            const auto* vehicle = gameState.entities[enumValue(traversal.vehicle)].asBase<VehicleBase>();
            const auto* head = gameState.entities[enumValue(traversal.head)].asBase<VehicleBase>();
            if (vehicle == nullptr || head == nullptr || !vehicle->isVehicle1() || !head->isVehicleHead()
                || vehicle->id != traversal.vehicle || vehicle->head != traversal.head
                || vehicle->mode != TransportMode::rail || head->mode != TransportMode::rail
                || !head->asVehicleHead()->isPlaced() || getEdge(*vehicle) != traversal.edge
                || elapsedSince(traversal.enteredAt, now) > kMaxTraversalTime)
            {
                return false;
            }
        }
        return true;
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state, getGameState()))
        {
            return false;
        }
        reset();
        for (const auto& entry : state.history)
        {
            _history.emplace(entry.edge, entry);
        }
        for (const auto& traversal : state.active)
        {
            _active.emplace(traversal.vehicle, traversal);
        }
        publishActive();
        return true;
    }

    bool isDefault(const State& state)
    {
        return state.history.empty() && state.active.empty();
    }
}
