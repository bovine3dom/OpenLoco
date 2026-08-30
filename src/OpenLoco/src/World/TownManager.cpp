#include "World/TownManager.h"
#include "World/TownGrowth.h"
#include "CargoDist/CargoDist.h"
#include "Game.h"
#include "GameCommands/GameCommands.h"
#include "GameState.h"
#include "GameStateFlags.h"
#include "Graphics/TextRenderer.h"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Localisation/StringManager.h"
#include "Map/BuildingElement.h"
#include "Map/SurfaceElement.h"
#include "Map/TileLoop.hpp"
#include "Map/TileManager.h"
#include "Objects/BuildingObject.h"
#include "Objects/ClimateObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/RegionObject.h"
#include "Objects/TownNamesObject.h"
#include "Scenario/ScenarioManager.h"
#include "SceneManager.h"
#include "Ui/WindowManager.h"
#include "World/CompanyManager.h"
#include "World/StationManager.h"
#include <OpenLoco/Core/EnumFlags.hpp>
#include <OpenLoco/Core/Numerics.hpp>
#include <type_traits>

using namespace OpenLoco::World;

namespace OpenLoco::TownManager
{
    using AmenityCounts = std::array<uint32_t, std::extent_v<decltype(Town::amenityCounts)>>;

    struct TownRuntimeMetrics
    {
        uint32_t buildingCount{};
        AmenityCounts amenityCounts{};
    };

    // Town's legacy counters remain clamped mirrors for S5 compatibility.
    static std::array<TownRuntimeMetrics, Limits::kMaxTowns> _runtimeMetrics{};

    static uint32_t adjustCount(uint32_t count, int32_t delta)
    {
        const auto adjusted = static_cast<int64_t>(count) + delta;
        return static_cast<uint32_t>(std::clamp<int64_t>(adjusted, 0, std::numeric_limits<uint32_t>::max()));
    }

    static void adjustBuildingCount(Town& town, int32_t delta)
    {
        auto& count = _runtimeMetrics[enumValue(town.id())].buildingCount;
        count = adjustCount(count, delta);
        town.numBuildings = static_cast<int16_t>(std::min<uint32_t>(count, std::numeric_limits<int16_t>::max()));
    }

    static uint32_t getLocalPopulationPressure(const World::Pos2& loc)
    {
        constexpr auto kRadius = 8;
        const auto centre = World::toTileSpace(loc);
        uint32_t pressure = 0;
        for (const auto& tilePos : World::getClampedRange(centre - World::TilePos2{ kRadius, kRadius }, centre + World::TilePos2{ kRadius, kRadius }))
        {
            const auto distance = std::max(std::abs(tilePos.x - centre.x), std::abs(tilePos.y - centre.y));
            const uint32_t weight = distance <= 2 ? 4 : distance <= 4 ? 2
                                                                    : 1;
            for (const auto& element : World::TileManager::get(tilePos))
            {
                const auto* building = element.as<World::BuildingElement>();
                if (building == nullptr || building->isGhost() || building->isMiscBuilding() || !building->isConstructed() || building->sequenceIndex() != 0)
                {
                    continue;
                }
                const auto* buildingObj = ObjectManager::get<BuildingObject>(building->objectId());
                pressure += buildingObj->producedQuantity[0] * weight;
            }
        }
        return pressure;
    }

    static uint32_t getTransportAccessibility(const TownId id, const World::Pos2& loc)
    {
        constexpr auto kRadius = 8 * World::kTileSize;
        uint32_t best = 0;
        uint32_t secondBest = 0;
        for (const auto& station : StationManager::stations())
        {
            if (station.town != id || station.stationTileSize == 0 || (station.flags & StationFlags::flag_5) != StationFlags::none)
            {
                continue;
            }
            const auto accessibility = CargoDist::getStationAccessibility(station.id());
            const auto distance = std::max(std::abs(station.x - loc.x), std::abs(station.y - loc.y));
            if (accessibility == 0 || distance >= kRadius)
            {
                continue;
            }
            const auto weighted = static_cast<uint32_t>(static_cast<uint64_t>(accessibility) * (kRadius - distance) / kRadius);
            if (weighted > best)
            {
                secondBest = best;
                best = weighted;
            }
            else if (weighted > secondBest)
            {
                secondBest = weighted;
            }
        }
        return static_cast<uint32_t>(std::min<uint64_t>(static_cast<uint64_t>(best) + secondBest / 2, std::numeric_limits<uint32_t>::max()));
    }

    uint8_t calculateTownDensity(const uint32_t localPopulationPressure, const uint32_t transportAccessibility, const uint32_t townPopulationCapacity)
    {
        constexpr std::array<uint32_t, 3> kDensityThresholds = { 256, 1024, 3072 };
        const auto transportPressure = std::min<uint64_t>(static_cast<uint64_t>(transportAccessibility) * 8, 4096);
        const auto pressure = static_cast<uint64_t>(localPopulationPressure) + transportPressure;
        uint8_t density = 0;
        while (density < kDensityThresholds.size() && pressure >= kDensityThresholds[density])
        {
            ++density;
        }

        const uint8_t maximumDensity = townPopulationCapacity < kTownSizeCapacityThresholds[1] ? 0
            : townPopulationCapacity < kTownSizeCapacityThresholds[2]                           ? 1
            : townPopulationCapacity < kTownSizeCapacityThresholds[3]                           ? 2
                                                                                                : 3;
        return std::min(density, maximumDensity);
    }

    uint8_t getTownDensity(const TownId id, const World::Pos2& loc)
    {
        const auto* town = get(id);
        if (town == nullptr || town->empty())
        {
            return 0;
        }
        return calculateTownDensity(getLocalPopulationPressure(loc), getTransportAccessibility(id, loc), town->populationCapacity);
    }

    // 0x0049B45F
    static uint32_t calcCargoInfluenceFlags(const Town& town)
    {
        uint32_t flags = 0;
        const auto* regionObj = ObjectManager::get<RegionObject>();
        for (auto i = 0U; i < regionObj->numCargoInflunceObjects; ++i)
        {
            bool hasInfluence = false;
            switch (regionObj->cargoInfluenceTownFilter[i])
            {
                using enum CargoInfluenceTownFilterType;
                case allTowns:
                {

                    hasInfluence = true;
                }
                break;

                case maySnow:
                {
                    auto tile = TileManager::get(town.x, town.y);
                    const auto* climageObj = ObjectManager::get<ClimateObject>();
                    const auto* surface = tile.surface();
                    hasInfluence = surface->baseZ() >= climageObj->summerSnowLine;
                }
                break;

                case inDesert:
                {
                    hasInfluence = TileManager::countSurroundingDesertTiles({ town.x, town.y }) >= 100;
                }
                break;

                default:
                    assert(false);
                    break;
            }

            if (hasInfluence)
            {
                flags |= (1U << regionObj->cargoInfluenceObjectIds[i]);
            }
        }

        return flags;
    }

    enum class LocationFlags : uint8_t
    {
        none = 0,
        adjacentToLargeWaterBody = 1 << 0,
        mountainous = 1 << 1,
        adjacentToSmallWaterBody = 1 << 2,
    };
    OPENLOCO_ENABLE_ENUM_OPERATORS(LocationFlags);

    // 0x00497D70
    static LocationFlags copyTownNameToBuffer(const TownNamesObject* namesObj, uint32_t categoryOffset, uint16_t index, char* buffer)
    {
        // Offset into the string table for the requested category, located just after the names object header.
        auto* offsetPtr = reinterpret_cast<const std::byte*>(namesObj) + categoryOffset;
        auto srcOffset = *reinterpret_cast<const int16_t*>(offsetPtr + index * 2);

        auto* srcPtr = reinterpret_cast<const char*>(offsetPtr + srcOffset);
        strcpy(buffer, srcPtr);

        // Location flags are stored at the end of the string
        LocationFlags flags = *reinterpret_cast<const LocationFlags*>(srcPtr + strlen(srcPtr) + 1);
        return flags;
    }

    // 0x00497A6A
    static LocationFlags townNameFromNamesObject(uint32_t rand, const char* buffer)
    {
        auto* namesObj = ObjectManager::get<TownNamesObject>();
        LocationFlags locationFlags = LocationFlags::none;

        // Town names are concatenated from six morpheme categories:
        // {CAT1}{CAT2}{CAT3}{CAT4}{CAT5}{CAT6}
        // Seperators are defined in each of the category strings; either '-', ' ', or nothing.
        // Categories can be completely empty, or can be skipped based on randomness.
        // e.g. "Fort " + "Apple " + "Green" for "Fort Apple Green"

        for (auto& category : namesObj->categories)
        {
            if (category.count == 0)
            {
                continue;
            }

            uint16_t ax = rand;
            uint16_t dx = category.count + category.bias;
            int16_t index = ((ax * dx) >> 16) - category.bias;

            if (index >= 0)
            {
                char* strEnd = const_cast<char*>(buffer + strlen(buffer));
                locationFlags |= copyTownNameToBuffer(namesObj, category.offset, index, strEnd);
            }

            for (auto shifts = category.count + category.bias; shifts > 0; shifts >>= 1)
            {
                rand = std::rotr(rand, 1);
            }
        }

        return locationFlags;
    }

    // 0x004978B7
    static bool generateTownName(Town* town)
    {
        for (auto attemptsLeft = 400U; attemptsLeft > 0; attemptsLeft--)
        {
            char buffer[256]{};
            auto rand = town->prng.randNext();
            auto locationFlags = townNameFromNamesObject(rand, buffer);

            if (strlen(buffer) == 0)
            {
                continue;
            }

            if (strlen(buffer) > StringManager::kUserStringSize)
            {
                continue;
            }

            if (Gfx::TextRenderer::getStringWidth(Gfx::Font::medium_bold, buffer) > 200)
            {
                continue;
            }

            // clang-format off
            auto numSurroundingWaterTilesAboveThreshold = [](Pos2 pos, uint8_t threshold) {
                return TileManager::countSurroundingWaterTiles(pos + Pos2(6 * kTileSize, 0)) > threshold ||
                    TileManager::countSurroundingWaterTiles(pos + Pos2(0, 6 * kTileSize)) > threshold ||
                    TileManager::countSurroundingWaterTiles(pos + Pos2(0 - 6 * kTileSize, 0)) > threshold ||
                    TileManager::countSurroundingWaterTiles(pos + Pos2(0, 0 - 6 * kTileSize)) > threshold;
            };
            // clang-format on

            if ((locationFlags & LocationFlags::adjacentToLargeWaterBody) != LocationFlags::none)
            {
                // Check that the town is adjacent to a large amount of water tiles on at least one side.
                auto pos = Pos2(town->x, town->y);
                if (!(numSurroundingWaterTilesAboveThreshold(pos, 65)))
                {
                    continue;
                }
            }

            if ((locationFlags & LocationFlags::mountainous) != LocationFlags::none)
            {
                auto pos = Pos2(town->x + kTileSize / 2, town->y + kTileSize / 2);
                auto height = TileManager::getHeight(pos);
                if (height.landHeight < 192)
                {
                    continue;
                }
            }

            if ((locationFlags & LocationFlags::adjacentToSmallWaterBody) != LocationFlags::none)
            {
                // Check that the town is adjacent to a low amount of water tiles on at least one side.
                auto pos = Pos2(town->x, town->y);
                if (!(numSurroundingWaterTilesAboveThreshold(pos, 15)))
                {
                    continue;
                }
            }

            bool nameInUse = false;
            for (auto& candidateTown : towns())
            {
                // Ensure the town name doesn't exist yet
                char candidateTownName[256]{};
                StringManager::formatString(candidateTownName, candidateTown.name);

                if (strcmp(buffer, candidateTownName) == 0)
                {
                    nameInUse = true;
                    break;
                }
            }

            if (nameInUse)
            {
                continue;
            }

            StringId newNameId = StringManager::userStringAllocate(buffer, true);
            if (newNameId == StringIds::empty)
            {
                continue;
            }

            town->name = newNameId;
            town->updateLabel();
            return true;
        }

        return false;
    }

    static auto& rawTowns() { return getGameState().towns; }

    // 0x00496FE7
    Town* initialiseTown(World::Pos2 pos)
    {
        Town* town = nullptr;
        for (auto& candidateTown : rawTowns())
        {
            if (candidateTown.empty())
            {
                town = &candidateTown;
                break;
            }
        }

        // No space for a new town?
        if (town == nullptr)
        {
            return nullptr;
        }

        // Initialise the new town
        const auto townIndex = enumValue(town->id());
        _runtimeMetrics[townIndex] = {};
        TownGrowth::resetLastGrowth(town->id());
        town->x = pos.x;
        town->y = pos.y;
        town->flags = TownFlags::none;
        town->population = 0;
        town->populationCapacity = 0;
        town->numBuildings = 0;
        town->size = TownSize::hamlet;
        town->historySize = 1;
        town->history[0] = 0;
        town->historyMinPopulation = 0;

        std::fill_n(&town->amenityCounts[0], std::size(town->amenityCounts), 0);

        town->var_19C[0][0] = 0;
        town->var_19C[0][1] = 0;
        town->var_19C[1][0] = 0;
        town->var_19C[1][1] = 0;
        town->numStations = 0;
        town->numberOfAirports = 0;
        town->var_1A8 = 0;

        town->prng = getGameState().rng;

        std::fill_n(&town->companyRatings[0], std::size(town->companyRatings), 500);

        town->companiesWithRating = 0;

        std::fill_n(&town->monthlyCargoDelivered[0], std::size(town->monthlyCargoDelivered), 0);

        town->cargoInfluenceFlags = calcCargoInfluenceFlags(*town);
        town->buildSpeed = 1;

        // Figure out a name for this town?
        if (!generateTownName(town))
        {
            town->name = StringIds::null;
            return nullptr;
        }

        // Figure out if we need to reset building influence
        for (auto& otherTown : towns())
        {
            if (otherTown.numBuildings == 0 && otherTown.population == 0 && otherTown.populationCapacity == 0)
            {
                continue;
            }

            resetBuildingsInfluence();
            break;
        }

        return town;
    }

    // 0x00497DC1
    // esi population
    // edi capacity
    // ebp rating | (numBuildings << 16)
    Town* updateTownInfo(const World::Pos2& loc, uint32_t population, uint32_t populationCapacity, int16_t rating, int16_t numBuildings)
    {
        auto townId = getClosestTown(loc);
        if (townId == std::nullopt)
        {
            return nullptr;
        }
        auto town = get(*townId);

        if (town == nullptr)
        {
            return nullptr;
        }

        town->populationCapacity += populationCapacity;

        if (population != 0)
        {
            town->population += population;
            Ui::WindowManager::invalidate(Ui::WindowType::townList);
            Ui::WindowManager::invalidate(Ui::WindowType::town, enumValue(town->id()));
        }
        if (rating != 0)
        {
            auto companyId = GameCommands::getUpdatingCompanyId();
            if (companyId != CompanyId::neutral)
            {
                if (!SceneManager::isEditorMode())
                {
                    town->adjustCompanyRating(companyId, rating);
                    Ui::WindowManager::invalidate(Ui::WindowType::town, enumValue(town->id()));
                }
            }
        }

        if (numBuildings != 0)
        {
            adjustBuildingCount(*town, numBuildings);
        }

        return town;
    }

    static void rebuildBuildingMetrics(bool rebuildInfluence)
    {
        _runtimeMetrics = {};
        for (auto& town : towns())
        {
            town.numBuildings = 0;
            std::fill(std::begin(town.amenityCounts), std::end(town.amenityCounts), 0);
            if (rebuildInfluence)
            {
                town.population = 0;
                town.populationCapacity = 0;
            }
        }

        for (const auto& tilePos : World::getWorldRange())
        {
            auto tile = World::TileManager::get(tilePos);
            for (auto& element : tile)
            {
                auto* building = element.as<World::BuildingElement>();
                if (building == nullptr)
                {
                    continue;
                }

                if (building->isGhost())
                {
                    continue;
                }

                if (building->isMiscBuilding())
                {
                    continue;
                }

                if (building->sequenceIndex() != 0)
                {
                    continue;
                }

                auto objectId = building->objectId();
                auto* buildingObj = ObjectManager::get<BuildingObject>(objectId);
                const auto producedQuantity = buildingObj->producedQuantity[0];
                const auto population = rebuildInfluence && building->isConstructed() ? producedQuantity : 0;
                const auto populationCapacity = rebuildInfluence ? producedQuantity : 0;
                auto* town = updateTownInfo(World::toWorldSpace(tilePos), population, populationCapacity, 0, 1);
                if (town != nullptr)
                {
                    if (buildingObj->townAmenityCategory != TownAmenityCategory::none)
                    {
                        adjustAmenityCount(town->id(), enumValue(buildingObj->townAmenityCategory), 1);
                    }
                }
            }
        }
    }

    void rebuildRuntimeMetrics()
    {
        TownGrowth::resetLastGrowth();
        rebuildBuildingMetrics(false);
    }

    // 0x00497348
    void resetBuildingsInfluence()
    {
        rebuildBuildingMetrics(true);
        Gfx::invalidateScreen();
    }

    // 0x00496B38
    void reset()
    {
        _runtimeMetrics = {};
        TownGrowth::resetLastGrowth();
        TownGrowth::resetCumulativeDiagnostics();
        for (auto& town : rawTowns())
        {
            town.name = StringIds::null;
        }
        Ui::Windows::TownList::reset();
    }

    FixedVector<Town, Limits::kMaxTowns> towns()
    {
        return FixedVector(rawTowns());
    }

    Town* get(TownId id)
    {
        if (enumValue(id) >= Limits::kMaxTowns)
        {
            return nullptr;
        }
        return &rawTowns()[enumValue(id)];
    }

    uint32_t getBuildingCount(TownId id)
    {
        const auto index = enumValue(id);
        if (index >= Limits::kMaxTowns || rawTowns()[index].empty())
        {
            return 0;
        }
        return _runtimeMetrics[index].buildingCount;
    }

    uint32_t getAmenityCount(TownId id, size_t category)
    {
        const auto index = enumValue(id);
        if (index >= Limits::kMaxTowns || category >= std::tuple_size_v<AmenityCounts> || rawTowns()[index].empty())
        {
            return 0;
        }
        return _runtimeMetrics[index].amenityCounts[category];
    }

    void adjustAmenityCount(TownId id, size_t category, int32_t delta)
    {
        const auto index = enumValue(id);
        if (index >= Limits::kMaxTowns || category >= std::tuple_size_v<AmenityCounts> || rawTowns()[index].empty())
        {
            return;
        }

        auto& count = _runtimeMetrics[index].amenityCounts[category];
        count = adjustCount(count, delta);
        rawTowns()[index].amenityCounts[category] = static_cast<uint8_t>(std::min<uint32_t>(count, std::numeric_limits<uint8_t>::max()));
    }

    // 0x00496B6D
    void tick()
    {
        if (Game::hasFlags(GameStateFlags::tileManagerLoaded) && !SceneManager::isEditorMode())
        {
            auto ticks = ScenarioManager::getScenarioTicks();
            if (ticks % 8 == 0)
            {
                const auto id = TownId((ticks / 8) % 0x7F);
                auto town = get(id);
                if (town != nullptr && !town->empty())
                {
                    GameCommands::setUpdatingCompanyId(CompanyId::neutral);
                    town->tick();
                }
            }
        }
    }

    // 0x0049771C
    void updateLabels()
    {
        for (Town& town : towns())
        {
            town.updateLabel();
        }
    }

    // 0x0049748C
    void updateMonthly()
    {
        for (Town& currTown : towns())
        {
            currTown.updateMonthly();
        }

        Ui::WindowManager::invalidate(Ui::WindowType::town);
    }

    std::optional<TownId> getClosestTown(const World::Pos2& loc)
    {
        int32_t closestDistance = std::numeric_limits<uint16_t>::max();
        auto closestTown = TownId::null; // ebx
        for (const auto& town : towns())
        {
            const auto distance = Math::Vector::manhattanDistance2D(World::Pos2(town.x, town.y), loc);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestTown = town.id();
            }
        }

        if (closestDistance == std::numeric_limits<uint16_t>::max())
        {
            return std::nullopt;
        }

        return closestTown;
    }

}

OpenLoco::TownId OpenLoco::Town::id() const
{
    // TODO check if this is stored in Town structure
    //      otherwise add it when possible
    auto index = static_cast<size_t>(this - &TownManager::rawTowns()[0]);
    if (index > Limits::kMaxTowns)
    {
        return OpenLoco::TownId::null;
    }
    return OpenLoco::TownId(index);
}
