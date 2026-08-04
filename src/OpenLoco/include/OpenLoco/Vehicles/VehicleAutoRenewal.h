#pragma once

#include "Engine/Limits.h"
#include "Types.hpp"
#include <array>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles::VehicleAutoRenewal
{
    constexpr uint8_t kDefaultReliabilityThreshold = 25;
    constexpr uint8_t kMinReliabilityThreshold = 0;
    constexpr uint8_t kMaxReliabilityThreshold = 100;

    struct Settings
    {
        bool enabled{};
        uint8_t reliabilityThreshold = kDefaultReliabilityThreshold;

        bool operator==(const Settings&) const = default;
    };

    struct State
    {
        std::array<Settings, Limits::kMaxCompanies> companies;

        bool operator==(const State&) const = default;
    };

    void reset();
    void reset(CompanyId company);
    const Settings& getSettings(CompanyId company);
    bool setSettings(CompanyId company, Settings settings);
    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState);
    bool restoreState(const State& state);
    bool isDefault(const State& state);

    bool tryRenew(VehicleHead& head);
}
