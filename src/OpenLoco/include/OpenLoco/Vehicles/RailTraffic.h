#pragma once

#include "Engine/Limits.h"
#include "Speed.hpp"
#include "Types.hpp"
#include <OpenLoco/Engine/World.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles
{
    struct Vehicle1;
    struct VehicleBase;
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::RailTraffic
{
    using TravelTime = uint64_t;
    constexpr uint8_t kFractionBits = 16;
    constexpr TravelTime kOneTick = TravelTime{ 1 } << kFractionBits;
    constexpr size_t kMaxHistoryEntries = 3 * World::kMapSize;
    constexpr size_t kMaxSaveDataSize = 12 * 1024 * 1024;

    struct Edge
    {
        int16_t x{};
        int16_t y{};
        int16_t z{};
        uint16_t tad{};
        uint8_t trackType{};

        bool operator==(const Edge&) const = default;
    };

    struct SpeedProfile
    {
        Speed16 maxSpeed{ 60 };
        Speed16 rackRailMaxSpeed{ 60 };
        bool fasterAroundCurves{};
        bool brokenDown{};

        bool operator==(const SpeedProfile&) const = default;
    };

    struct HistoryEntry
    {
        Edge edge;
        TravelTime meanTraversalTime{};
        uint32_t lastObservedDay{};
        uint8_t confidence{};

        bool operator==(const HistoryEntry&) const = default;
    };

    struct ActiveTraversal
    {
        EntityId vehicle = EntityId::null;
        EntityId head = EntityId::null;
        Edge edge;
        TravelTime enteredAt{};
        bool completeFromStart{};

        bool operator==(const ActiveTraversal&) const = default;
    };

    struct State
    {
        std::vector<HistoryEntry> history;
        std::vector<ActiveTraversal> active;

        bool operator==(const State&) const = default;
    };

    Edge getEdge(const VehicleBase& vehicle);
    SpeedProfile getSpeedProfile(const VehicleHead& head);
    TravelTime getFreeFlowTime(const SpeedProfile& profile, uint16_t routing, uint8_t trackType);
    TravelTime getTravelTime(const SpeedProfile& profile, const World::Pos3& pos, uint16_t routing, uint8_t trackType);
    TravelTime getHeuristicTime(const SpeedProfile& profile, uint32_t distance);
    TravelTime getLiveSignalPenalty(const SpeedProfile& profile, uint16_t routing, uint8_t trackType);
    std::optional<Speed16> getAverageSpeed(const Edge& edge);
    std::vector<Speed16> getAverageSpeeds();
    uint32_t getHistoryRevision();

    void reset();
    void beginTick();
    void endTick();
    void updateDaily();
    void reconcile(Vehicle1& vehicle);
    void restart(Vehicle1& vehicle);
    void onPieceTransition(Vehicle1& vehicle, const Edge& previous, const Edge& next, uint32_t tickFraction);
    void recordTraversal(const Edge& edge, TravelTime duration);

    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState);
    bool restoreState(const State& state);
    bool isDefault(const State& state);
}
