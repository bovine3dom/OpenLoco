#pragma once

#include "GameCommands/Vehicles/VehiclePlace.h"
#include "Types.hpp"
#include <vector>

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::VehicleReplacement
{
    struct Request
    {
        EntityId target;
        EntityId source;

        bool operator==(const Request&) const = default;
    };

    struct State
    {
        struct PendingPlacement
        {
            GameCommands::VehiclePlacementArgs args;
            bool start = false;

            bool operator==(const PendingPlacement&) const = default;
        };

        std::vector<Request> requests;
        std::vector<PendingPlacement> pendingPlacements;

        bool operator==(const State&) const = default;
    };

    void reset();
    void remove(EntityId vehicle);
    bool schedule(EntityId source);
    bool tryReplace(VehicleHead& head);
    void tick();
    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState);
    bool restoreState(const State& state);
}
