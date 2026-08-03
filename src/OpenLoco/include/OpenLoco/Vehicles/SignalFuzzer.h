#pragma once

#include <OpenLoco/Core/FileSystem.hpp>
#include <OpenLoco/Types.hpp>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace OpenLoco::Vehicles::SignalFuzzer
{
    struct Case
    {
        fs::path baseSave;
        std::string focusTown = "Beachtown";
        uint32_t seed{};
        uint32_t caseIndex{};
        uint32_t ticks{};
        EntityId targetVehicle = EntityId::null;
        uint32_t earliestBreakdownTick{};
        bool injectBreakdown{};
    };

    struct Options
    {
        fs::path baseSave;
        fs::path outputDirectory;
        std::string focusTown = "Beachtown";
        uint32_t cases = 100;
        uint32_t ticks = 20000;
        uint32_t seed = 1;
    };

    enum class Result
    {
        completed,
        collision,
        reservationConflict,
        invalidInput,
        loadFailure,
        runtimeFailure,
    };

    Case makeCase(const Options& options, uint32_t caseIndex, std::span<const EntityId> candidates);
    std::string serialiseCase(const Case& fuzzCase);
    std::optional<Case> deserialiseCase(std::string_view yaml);

    Result run(const Options& options);
    Result replay(const fs::path& casePath, const fs::path& outputDirectory);
}
