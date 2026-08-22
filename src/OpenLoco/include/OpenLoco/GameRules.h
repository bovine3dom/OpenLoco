// SPDX-License-Identifier: MIT
#pragma once

namespace OpenLoco::GameRules
{
    struct State
    {
        bool vehiclesNeverExpire{};
        bool extendedVehicleObjects{};

        bool operator==(const State&) const = default;
    };

    inline constexpr State kDefaultState{};

    void reset();
    State captureState();
    void restoreState(const State& state);

    bool vehiclesNeverExpire();
    void setVehiclesNeverExpire(bool value);

    bool extendedVehicleObjects();
    bool setExtendedVehicleObjects(bool value);
}
