#pragma once

#include "Engine/Limits.h"
#include "Town.h"
#include <OpenLoco/Core/LocoFixedVector.hpp>
#include <array>
#include <cstddef>
#include <optional>

namespace OpenLoco::TownManager
{
    Town* initialiseTown(World::Pos2 pos);
    void reset();
    FixedVector<Town, Limits::kMaxTowns> towns();
    Town* get(TownId id);
    std::optional<TownId> getClosestTown(const World::Pos2& loc);
    uint8_t getTownDensity(TownId id, const World::Pos2& loc);
    uint8_t calculateTownDensity(uint32_t localPopulationPressure, uint32_t transportAccessibility, uint32_t townPopulationCapacity);
    uint32_t getBuildingCount(TownId id);
    uint32_t getAmenityCount(TownId id, size_t category);
    void adjustAmenityCount(TownId id, size_t category, int32_t delta);
    void tick();
    void updateLabels();
    void updateMonthly();
    Town* updateTownInfo(const World::Pos2& loc, uint32_t population, uint32_t populationCapacity, int16_t rating, int16_t numBuildings);
    void rebuildRuntimeMetrics();
    void resetBuildingsInfluence();
}
