#pragma once

#include "Types.hpp"
#include <cstddef>
#include <vector>

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::SharedOrderManager
{
    struct Group
    {
        std::vector<EntityId> members;

        bool operator==(const Group&) const = default;
    };

    struct State
    {
        std::vector<Group> groups;

        bool operator==(const State&) const = default;
    };

    void reset();
    void remove(EntityId vehicle);

    bool isShared(EntityId vehicle);
    EntityId getGroupId(EntityId vehicle);
    std::vector<EntityId> getMembers(EntityId vehicle);
    size_t getMemberCount(EntityId vehicle);

    bool areOrdersEqual(const VehicleHead& lhs, const VehicleHead& rhs);
    bool areVehiclesCompatible(const VehicleHead& target, const VehicleHead& source);

    // The caller must validate compatibility and synchronise order tables first.
    bool join(EntityId target, EntityId source);
    bool leave(EntityId vehicle);
    bool detachIfIncompatible(EntityId vehicle);
    bool joinAllMatching(EntityId source);

    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState);
    bool restoreState(const State& state);
}
