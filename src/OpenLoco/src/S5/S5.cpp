#define DO_TITLE_SEQUENCE_CHECKS

#include "S5/S5.h"
#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/CargoDist/Simulation.h>
#include <OpenLoco/S5/SaveExtension.h>

#include "Audio/Audio.h"
#include "EditorController.h"
#include "Entities/EntityManager.h"
#include "Game.h"
#include "GameRules.h"
#include "GameState.h"
#include "GameStateFlags.h"
#include "Gui.h"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Localisation/StringManager.h"
#include "Map/BuildingElement.h"
#include "Map/IndustryElement.h"
#include "Map/RoadElement.h"
#include "Map/SignalElement.h"
#include "Map/StationElement.h"
#include "Map/SurfaceElement.h"
#include "Map/TileManager.h"
#include "Map/Track/TrackOverlayAudits.h"
#include "Map/TrackElement.h"
#include "Map/TreeElement.h"
#include "Map/WallElement.h"
#include "Objects/CargoObject.h"
#include "Objects/ObjectIndex.h"
#include "Objects/ObjectManager.h"
#include "Objects/RoadObject.h"
#include "Objects/ScenarioTextObject.h"
#include "Objects/TrackObject.h"
#include "Objects/VehicleObject.h"
#include "OpenLoco.h"
#include "S5/S5File.h"
#include "S5/S5Options.h"
#include "S5/S5TileElement.h"
#include "S5/SawyerStream.h"
#include "Scenario/Scenario.h"
#include "Scenario/ScenarioManager.h"
#include "Scenario/ScenarioOptions.h"
#include "Scenario/ScenarioPreview.h"
#include "SceneManager.h"
#include "Ui/ProgressBar.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/RailTraffic.h"
#include "Vehicles/RoutingManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleAutoRenewal.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleReplacement.h"
#include "World/CompanyManager.h"
#include "World/IndustryManager.h"
#include "World/StationManager.h"
#include "World/TownManager.h"
#include <OpenLoco/Core/Exception.hpp>
#include <OpenLoco/Core/Numerics.hpp>
#include <OpenLoco/Core/Stream.hpp>
#include <OpenLoco/Diagnostics/Logging.h>
#include <fstream>
#include <iomanip>

using namespace OpenLoco::World;
using namespace OpenLoco::Ui;
using namespace OpenLoco::Diagnostics;

namespace OpenLoco::S5
{
    constexpr uint32_t kCurrentVersion = 0x62262;

    static LoadError _lastLoadError;

    static bool exportGameState(Stream& stream, const S5File& file, const std::vector<ObjectHeader>& packedObjects);

    static constexpr size_t kVehicleObjectOffset = [] {
        size_t offset = 0;
        for (uint8_t type = 0; type < enumValue(ObjectType::vehicle); ++type)
        {
            offset += ObjectManager::getMaxObjects(static_cast<ObjectType>(type));
        }
        return offset;
    }();
    static constexpr size_t kLegacyPostVehicleObjectOffset = kVehicleObjectOffset + S5::Limits::kMaxVehicleObjects;
    static constexpr size_t kRuntimePostVehicleObjectOffset = kVehicleObjectOffset + OpenLoco::Limits::kMaxVehicleObjects;

    static_assert(kLegacyPostVehicleObjectOffset + ObjectManager::kMaxObjects - kRuntimePostVehicleObjectOffset == S5::Limits::kMaxObjectHeaders);

    RequiredObjectHeaders exportRequiredObjectHeaders(const std::span<const ObjectHeader> objects)
    {
        if (objects.size() != ObjectManager::kMaxObjects)
        {
            throw Exception::InvalidArgument("Invalid runtime required object table size");
        }

        RequiredObjectHeaders result{};
        std::copy_n(objects.begin(), kLegacyPostVehicleObjectOffset, result.begin());
        std::copy(objects.begin() + kRuntimePostVehicleObjectOffset, objects.end(), result.begin() + kLegacyPostVehicleObjectOffset);
        return result;
    }

    std::vector<ObjectHeader> importRequiredObjectHeaders(const std::span<const ObjectHeader> objects, const SaveExtension::VehicleObjectState* vehicleObjectState)
    {
        if (objects.size() != S5::Limits::kMaxObjectHeaders)
        {
            throw Exception::InvalidArgument("Invalid legacy required object table size");
        }

        std::vector<ObjectHeader> result(ObjectManager::kMaxObjects, kEmptyObjectHeader);
        std::copy_n(objects.begin(), kLegacyPostVehicleObjectOffset, result.begin());
        std::copy(objects.begin() + kLegacyPostVehicleObjectOffset, objects.end(), result.begin() + kRuntimePostVehicleObjectOffset);

        if (vehicleObjectState != nullptr)
        {
            uint16_t previousSlot{};
            for (size_t i = 0; i < vehicleObjectState->objects.size(); ++i)
            {
                const auto& object = vehicleObjectState->objects[i];
                if (object.slot < SaveExtension::kExtendedVehicleObjectStart || object.slot >= OpenLoco::Limits::kMaxVehicleObjects
                    || (i != 0 && previousSlot >= object.slot)
                    || object.header.isEmpty() || object.header.getType() != ObjectType::vehicle)
                {
                    throw Exception::InvalidArgument("Invalid extended vehicle object state");
                }
                result[kVehicleObjectOffset + object.slot] = object.header;
                previousSlot = object.slot;
            }
        }
        return result;
    }

    static std::optional<SaveExtension::VehicleObjectState> captureExtendedVehicleObjects(const std::span<const ObjectHeader> objects)
    {
        SaveExtension::VehicleObjectState state;
        for (size_t slot = SaveExtension::kExtendedVehicleObjectStart; slot < OpenLoco::Limits::kMaxVehicleObjects; ++slot)
        {
            const auto& header = objects[kVehicleObjectOffset + slot];
            if (header.isEmpty())
            {
                continue;
            }
            state.objects.push_back({ static_cast<uint16_t>(slot), header });
        }
        if (state.objects.empty())
        {
            return std::nullopt;
        }

        const auto& companies = getGameState().companies;
        for (const auto& object : state.objects)
        {
            for (size_t company = 0; company < std::size(companies); ++company)
            {
                if (companies[company].unlockedVehicles[object.slot])
                {
                    state.companyUnlocks[company].set(object.slot - SaveExtension::kExtendedVehicleObjectStart, true);
                }
            }
        }
        return state;
    }

    constexpr bool hasSaveFlags(SaveFlags flags, SaveFlags flagsToTest)
    {
        return (flags & flagsToTest) != SaveFlags::none;
    }

    constexpr bool hasLoadFlags(LoadFlags flags, LoadFlags flagsToTest)
    {
        return (flags & flagsToTest) != LoadFlags::none;
    }

    static void validateCargoDistObjects(const CargoDist::State& state, std::span<const ObjectHeader> requiredObjects)
    {
        constexpr auto kCargoOffset = [] {
            size_t offset = 0;
            for (uint8_t type = 0; type < enumValue(ObjectType::cargo); ++type)
            {
                offset += ObjectManager::getMaxObjects(static_cast<ObjectType>(type));
            }
            return offset;
        }();
        for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
        {
            const auto& header = requiredObjects[kCargoOffset + cargo];
            if (state.settings.modes[cargo] != CargoDist::DistributionMode::manual
                && (header.isEmpty() || header.getType() != ObjectType::cargo))
            {
                throw Exception::RuntimeError("CargoDist mode references unloaded cargo");
            }
        }
    }

    static void discardLegacyPathReservations(const Vehicles::RoutingManager::State& state)
    {
        for (auto* head : VehicleManager::VehicleList())
        {
            const auto vehicleRef = head->routingHandle.getVehicleRef();
            if (head->mode != TransportMode::rail || !head->isPlaced()
                || vehicleRef >= state.pathReservedRoutings.size()
                || state.pathReservedRoutings[vehicleRef] == 0)
            {
                continue;
            }
            head->discardFutureRouting();
        }
    }

    template<typename TObject>
    static const TObject* getLoadedObject(const size_t id)
    {
        return id < ObjectManager::getMaxObjects(TObject::kObjectType) ? ObjectManager::get<TObject>(id) : nullptr;
    }

    static bool hasLoadedCargoObject(const Vehicles::VehicleCargo& cargo)
    {
        return cargo.type == kCargoTypeNull || getLoadedObject<CargoObject>(cargo.type) != nullptr;
    }

    static void applyVehicleCapacityOverrides(CargoDist::State* cargoDistState)
    {
        for (auto* head : VehicleManager::VehicleList())
        {
            Vehicles::Vehicle vehicle(*head);
            bool cargoChanged = false;
            bool canUpdateTrainProperties = true;
            for (const auto& car : vehicle.cars)
            {
                auto& cargo = car.body->primaryCargo;
                const auto* vehicleObject = getLoadedObject<VehicleObject>(car.body->objectId);
                const auto* frontVehicleObject = getLoadedObject<VehicleObject>(car.front->objectId);
                canUpdateTrainProperties = canUpdateTrainProperties
                    && vehicleObject != nullptr
                    && frontVehicleObject != nullptr
                    && hasLoadedCargoObject(car.front->secondaryCargo)
                    && hasLoadedCargoObject(car.back->secondaryCargo)
                    && hasLoadedCargoObject(cargo);
                if (vehicleObject == nullptr)
                {
                    continue;
                }

                const LoadedObjectHandle handle{ ObjectType::vehicle, car.body->objectId };
                const auto& header = ObjectManager::getHeader(handle);
                auto correctedCapacity = getEffectiveVehicleCapacity(header, cargo.maxQty);
                const auto primaryCargoType = Numerics::bitScanForward(vehicleObject->compatibleCargoCategories[0]);
                const auto* primaryCargoObject = primaryCargoType == -1 ? nullptr : getLoadedObject<CargoObject>(primaryCargoType);
                const auto* cargoObject = getLoadedObject<CargoObject>(cargo.type);
                if (primaryCargoObject != nullptr && cargoObject != nullptr)
                {
                    correctedCapacity = getEffectiveVehicleCapacity(header, cargo.maxQty, primaryCargoObject->unitSize, cargoObject->unitSize);
                }
                if (correctedCapacity == cargo.maxQty)
                {
                    continue;
                }

                cargo.maxQty = correctedCapacity;
                if (cargoDistState != nullptr)
                {
                    cargoDistState->graphDirty = true;
                    cargoDistState->servicesDirty = true;
                }
                if (cargo.qty <= correctedCapacity)
                {
                    continue;
                }

                cargo.qty = correctedCapacity;
                cargoChanged = true;
                if (cargoDistState == nullptr)
                {
                    continue;
                }

                const CargoDist::VehicleCargoKey key{ car.body->id, CargoDist::VehicleCargoSlot::primary };
                const auto packetsIt = cargoDistState->vehicleCargo.find(key);
                if (packetsIt != cargoDistState->vehicleCargo.end())
                {
                    auto& packets = packetsIt->second;
                    packets.remove(packets.quantity() - correctedCapacity);
                    cargo.townFrom = packets.representativeOrigin();
                    cargo.numDays = packets.averageAge();
                }
            }
            if (head->mode == TransportMode::road)
            {
                const auto roadObjectId = head->trackType == 0xFF ? getGameState().defaultTrackTypeObjectId : head->trackType;
                canUpdateTrainProperties = canUpdateTrainProperties
                    && getLoadedObject<RoadObject>(roadObjectId) != nullptr;
            }
            else if (head->mode == TransportMode::rail)
            {
                canUpdateTrainProperties = canUpdateTrainProperties
                    && getLoadedObject<TrackObject>(head->trackType) != nullptr;
            }
            if (cargoChanged && canUpdateTrainProperties)
            {
                head->updateTrainProperties();
            }
        }
    }

    static Header prepareHeader(SaveFlags flags, size_t numPackedObjects)
    {
        Header result;
        std::memset(&result, 0, sizeof(result));

        result.type = S5Type::savedGame;
        if (hasSaveFlags(flags, SaveFlags::landscape))
        {
            result.type = S5Type::landscape;
        }
        if (hasSaveFlags(flags, SaveFlags::scenario))
        {
            result.type = S5Type::scenario;
        }

        result.numPackedObjects = static_cast<uint16_t>(numPackedObjects);
        result.version = kCurrentVersion;
        result.magic = kMagicNumber;

        if (hasSaveFlags(flags, SaveFlags::raw))
        {
            result.flags |= HeaderFlags::isRaw;
        }
        if (hasSaveFlags(flags, SaveFlags::dump))
        {
            result.flags |= HeaderFlags::isDump;
        }
        if (!hasSaveFlags(flags, SaveFlags::scenario)
            && !hasSaveFlags(flags, SaveFlags::raw)
            && !hasSaveFlags(flags, SaveFlags::dump))
        {
            result.flags |= HeaderFlags::hasSaveDetails;
        }

        return result;
    }

    // 0x004471A4
    static std::unique_ptr<SaveDetails> prepareSaveDetails(OpenLoco::GameState& gameState)
    {
        auto saveDetails = std::make_unique<SaveDetails>();

        const auto& playerCompany = gameState.companies[enumValue(gameState.playerCompanies[0])];
        StringManager::formatString(saveDetails->company, sizeof(saveDetails->company), playerCompany.name);
        StringManager::formatString(saveDetails->owner, sizeof(saveDetails->owner), playerCompany.ownerName);

        saveDetails->date = gameState.currentDay;
        saveDetails->performanceIndex = playerCompany.performanceIndex;
        saveDetails->challengeProgress = playerCompany.challengeProgress;
        saveDetails->challengeFlags = playerCompany.challengeFlags;

        std::strncpy(saveDetails->scenario, gameState.scenarioName, sizeof(saveDetails->scenario));
        Scenario::drawSavePreviewImage(saveDetails->image, { 250, 200 });

        return saveDetails;
    }

    static void loadTileElements(OpenLoco::GameState& gs, std::span<const TileElement> srcElements)
    {
        auto& ts = gs.tileState;
        ts.surface.clear();
        ts.track.clear();
        ts.station.clear();
        ts.signal.clear();
        ts.building.clear();
        ts.tree.clear();
        ts.wall.clear();
        ts.road.clear();
        ts.industry.clear();

        if (ts.entries.size() != World::TileManager::kMaxElements)
        {
            ts.entries.assign(World::TileManager::kMaxElements, World::TileElementEntry::empty());
        }
        else
        {
            std::fill(ts.entries.begin(), ts.entries.end(), World::TileElementEntry::empty());
        }

        const size_t count = std::min(srcElements.size(), ts.entries.size());
        for (size_t i = 0; i < count; ++i)
        {
            const auto& srcElem = srcElements[i];
            auto& entry = ts.entries[i];
            if (srcElem.baseZ() == 0xFFU)
            {
                entry = World::TileElementEntry::empty();
                continue;
            }

            const auto worldType = static_cast<World::ElementType>(enumValue(srcElem.type()));
            uint32_t idx = 0;
            switch (worldType)
            {
                case World::ElementType::surface:
                {
                    const auto& d = *srcElem.as<SurfaceElement>();
                    idx = ts.surface.allocate();
                    auto& dstElem = ts.surface[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setSlope(d.slope());
                    dstElem.setSnowCoverage(d.snowCoverage());
                    dstElem.setWater(d.water());
                    dstElem.setUpdateTimer(d.updateTimer());
                    dstElem.setTerrain(d.terrain());
                    dstElem.setGrowthStage(d.growthStage());
                    dstElem.setVariation(d.var7());
                    dstElem.setIsIndustrialFlag(d.isIndustrial());
                    dstElem.setType6Flag(d.type6Flag());
                    break;
                }
                case World::ElementType::track:
                {
                    const auto& d = *srcElem.as<TrackElement>();
                    idx = ts.track.allocate();
                    auto& dstElem = ts.track[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setHasSignal(d.hasSignal());
                    dstElem.setHasStationElement(d.hasStationElement());
                    dstElem.setTrackId(d.trackId());
                    dstElem.setHasGhostMods(d.hasGhostMods());
                    dstElem.setHasBridge(d.hasBridge());
                    dstElem.setSequenceIndex(d.sequenceIndex());
                    dstElem.setTrackObjectId(d.trackObjectId());
                    dstElem.setHasLevelCrossing(d.hasLevelCrossing());
                    dstElem.setSignalModes(d.signalModes());
                    dstElem.setBridgeObjectId(d.bridge());
                    dstElem.setOwner(static_cast<CompanyId>(d.owner()));
                    for (uint8_t m = 0; m < 4; ++m)
                    {
                        dstElem.setMod(m, (d.mods() >> m) & 1);
                    }
                    break;
                }
                case World::ElementType::station:
                {
                    const auto& d = *srcElem.as<StationElement>();
                    idx = ts.station.allocate();
                    auto& dstElem = ts.station[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setSequenceIndex(d.sequenceIndex());
                    dstElem.setOwner(static_cast<CompanyId>(d.owner()));
                    dstElem.setUnk4SLR4(d.unk4SLR4());
                    dstElem.setObjectId(d.objectId());
                    dstElem.setStationType(static_cast<StationType>(d.stationType()));
                    dstElem.setStationId(static_cast<StationId>(d.stationId()));
                    dstElem.setBuildingType(d.buildingType());
                    break;
                }
                case World::ElementType::signal:
                {
                    const auto& d = *srcElem.as<SignalElement>();
                    idx = ts.signal.allocate();
                    auto& dstElem = ts.signal[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setLeftGhost(d.isLeftGhost());
                    dstElem.setRightGhost(d.isRightGhost());
                    const auto copySide = [](World::SignalElement::Side& dst, const SignalElement::Side& srcSide) {
                        dst.setSignalObjectId(srcSide.signalObjectId());
                        dst.setUnk4(srcSide.unk4());
                        dst.setIsOccupied(srcSide.isOccupied());
                        dst.setHasSignal(srcSide.hasSignal());
                        dst.setFrame(srcSide.frame());
                        dst.setAllLights(srcSide.allLights());
                    };
                    copySide(dstElem.getLeft(), d.left());
                    copySide(dstElem.getRight(), d.right());
                    break;
                }
                case World::ElementType::building:
                {
                    const auto& d = *srcElem.as<BuildingElement>();
                    idx = ts.building.allocate();
                    auto& dstElem = ts.building[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setIsMiscBuilding(d.isMiscBuilding());
                    dstElem.setConstructed(d.isConstructed());
                    dstElem.setObjectId(d.objectId());
                    dstElem.setSequenceIndex(d.sequenceIndex());
                    dstElem.setUnk5u(d.unk5u());
                    dstElem.setAge(d.age());
                    dstElem.setVariation(d.variation());
                    dstElem.setColour(static_cast<Colour>(d.colour()));
                    break;
                }
                case World::ElementType::tree:
                {
                    const auto& d = *srcElem.as<TreeElement>();
                    idx = ts.tree.allocate();
                    auto& dstElem = ts.tree[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setQuadrant(d.quadrant());
                    dstElem.setTreeObjectId(d.treeObjectId());
                    dstElem.setGrowth(d.growth());
                    dstElem.setUnk5h(d.unk5h());
                    dstElem.setColour(static_cast<Colour>(d.colour()));
                    dstElem.setSnow(d.hasSnow());
                    dstElem.setIsDying(d.isDying());
                    dstElem.setUnk7l(d.unk7l());
                    dstElem.setSeason(d.season());
                    break;
                }
                case World::ElementType::wall:
                {
                    const auto& d = *srcElem.as<WallElement>();
                    idx = ts.wall.allocate();
                    auto& dstElem = ts.wall[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setSlopeFlags(static_cast<World::EdgeSlope>(d.slopeFlags()));
                    dstElem.setWallObjectId(d.wallObjectId());
                    dstElem.setPrimaryColour(static_cast<Colour>(d.primaryColour()));
                    dstElem.setSecondaryColour(static_cast<Colour>(d.secondaryColour()));
                    dstElem.setTertiaryColour(static_cast<Colour>(d.tertiaryColour()));
                    break;
                }
                case World::ElementType::road:
                {
                    const auto& d = *srcElem.as<RoadElement>();
                    idx = ts.road.allocate();
                    auto& dstElem = ts.road[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setHasStationElement(d.hasStationElement());
                    dstElem.setRoadId(d.roadId());
                    dstElem.setLaneOccupation(d.laneOccupation());
                    dstElem.setHasGhostMods(d.hasGhostMods());
                    dstElem.setHasBridge(d.hasBridge());
                    dstElem.setSequenceIndex(d.sequenceIndex());
                    dstElem.setLevelCrossingObjectId(d.levelCrossingObjectId());
                    dstElem.setRoadObjectId(d.roadObjectId());
                    dstElem.setUnk6l(d.unk6l());
                    dstElem.setBridgeObjectId(d.bridge());
                    dstElem.setOwner(static_cast<CompanyId>(d.owner()));
                    dstElem.setUnk7_10(d.unk7_10());
                    dstElem.setHasLevelCrossing(d.hasLevelCrossing());
                    dstElem.setUnk7_40(d.unk7_40());
                    dstElem.setUnk7_80(d.unk7_80());
                    break;
                }
                case World::ElementType::industry:
                {
                    const auto& d = *srcElem.as<IndustryElement>();
                    idx = ts.industry.allocate();
                    auto& dstElem = ts.industry[idx];
                    dstElem.rawData()[1] = srcElem.flags();
                    dstElem.setBaseZ(srcElem.baseZ());
                    dstElem.setClearZ(srcElem.clearZ());
                    dstElem.setRotation(d.rotation());
                    dstElem.setIsConstructed(d.isConstructed());
                    dstElem.setIndustryId(static_cast<IndustryId>(d.industryId()));
                    dstElem.setSequenceIndex(d.sequenceIndex());
                    dstElem.setSectionProgress(d.sectionProgress());
                    dstElem.setSectionsCompleted(d.var6_003F());
                    dstElem.setBuildingType(d.buildingType());
                    dstElem.setColour(static_cast<Colour>(d.colour()));
                    break;
                }
            }
            entry.setType(worldType);
            entry.setIndex(idx);
            entry.setLastFlag(srcElem.isLast());
        }

        ts.entriesEnd = static_cast<std::ptrdiff_t>(count);
        World::TileManager::updateTilePointers();
        World::Track::TrackOverlayAudits::invalidateAudit();
    }

    /**
     * Removes all tile elements that have the ghost flag set.
     * Assumes all elements are organised in tile order.
     */
    static void removeGhostElements(std::vector<TileElement>& elements)
    {
        for (size_t i = 0; i < elements.size(); i++)
        {
            if (elements[i].isGhost())
            {
                if (elements[i].isLast())
                {
                    if (i == 0 || elements[i - 1].isLast())
                    {
                        // First element of tile, can not remove...
                    }
                    else
                    {
                        elements[i - 1].setLast(true);
                        elements.erase(elements.begin() + i);
                        i--;
                    }
                }
                else
                {
                    elements.erase(elements.begin() + i);
                    i--;
                }
            }
        }
    }

    static std::unique_ptr<S5File> prepareGameState(SaveFlags flags, const RequiredObjectHeaders& requiredObjects, const std::vector<ObjectHeader>& packedObjects)
    {
        // Set saved view from main viewport
        auto mainWindow = WindowManager::getMainWindow();
        auto savedView = mainWindow != nullptr && mainWindow->viewports[0] != nullptr ? mainWindow->viewports[0]->toSavedView() : SavedViewSimple{ 0, 0, 0, 0 };

        auto file = std::make_unique<S5File>();
        auto& src = getGameState();

        // Prepare header, scenario or save details
        file->header = prepareHeader(flags, packedObjects.size());
        if (file->header.type == S5Type::scenario)
        {
            file->scenarioOptions = std::make_unique<Options>(exportOptions(Scenario::getOptions()));
        }
        if (file->header.hasFlags(HeaderFlags::hasSaveDetails))
        {
            file->saveDetails = prepareSaveDetails(src);
        }

        // Prepare required objects
        std::memcpy(file->requiredObjects, requiredObjects.data(), sizeof(file->requiredObjects));

        // Copy the source gamestate contents to the S5 gamestate, field by field
        auto& dst = file->gameState;
        dst = *exportGameState(src);
        dst.general.savedViewX = savedView.viewX;
        dst.general.savedViewY = savedView.viewY;
        dst.general.savedViewZoom = savedView.zoomLevel.toEncoded();
        dst.general.savedViewRotation = savedView.rotation;

        // Copy tile elements; remove any ghosts before saving
        const auto entries = TileManager::getEntries();
        file->tileElements.clear();
        file->tileElements.reserve(entries.size());
        for (const auto& entry : entries)
        {
            file->tileElements.push_back(toSaveElement(src, entry));
        }
        removeGhostElements(file->tileElements);

        return file;
    }

    static constexpr bool shouldPackObjects(SaveFlags flags)
    {
        return (flags & SaveFlags::raw) == SaveFlags::none
            && (flags & SaveFlags::dump) == SaveFlags::none
            && (flags & SaveFlags::packCustomObjects) != SaveFlags::none
            && !SceneManager::isNetworked();
    }

    // 0x00441C26
    bool exportGameStateToFile(const fs::path& path, SaveFlags flags)
    {
        FileStream fs(path, StreamMode::write);
        return exportGameStateToFile(fs, flags);
    }

    bool exportGameStateToFile(Stream& stream, SaveFlags flags)
    {
        if ((flags & SaveFlags::isAutosave) == SaveFlags::none)
        {
            Ui::ProgressBar::begin(StringIds::please_wait);
            Ui::ProgressBar::setProgress(20);
        }

        if ((flags & SaveFlags::noWindowClose) == SaveFlags::none
            && (flags & SaveFlags::raw) == SaveFlags::none
            && (flags & SaveFlags::dump) == SaveFlags::none)
        {
            WindowManager::closeConstructionWindows();
        }

        if ((flags & SaveFlags::raw) == SaveFlags::none)
        {
            TileManager::reorganise();
            EntityManager::resetSpatialIndex();
            EntityManager::zeroUnused();
            StationManager::zeroUnused();
            Vehicles::OrderManager::zeroUnusedOrderTable();
        }

        if ((flags & SaveFlags::isAutosave) == SaveFlags::none)
        {
            Ui::ProgressBar::setProgress(40);
        }

        bool saveResult;
        {
            const auto runtimeRequiredObjects = ObjectManager::getHeaders();
            const auto requiredObjects = exportRequiredObjectHeaders(runtimeRequiredObjects);
            auto vehicleObjectState = captureExtendedVehicleObjects(runtimeRequiredObjects);
            std::vector<ObjectHeader> packedObjects;
            if (shouldPackObjects(flags))
            {
                std::copy_if(runtimeRequiredObjects.begin(), runtimeRequiredObjects.end(), std::back_inserter(packedObjects), [](const ObjectHeader& header) {
                    return !header.isEmpty() && !header.isVanilla();
                });
            }

            auto file = prepareGameState(flags, requiredObjects, packedObjects);
            file->vehicleObjectState = std::move(vehicleObjectState);
            saveResult = exportGameState(stream, *file, packedObjects);
        }

        if ((flags & SaveFlags::isAutosave) == SaveFlags::none)
        {
            Ui::ProgressBar::setProgress(230);
        }

        if ((flags & SaveFlags::raw) == SaveFlags::none
            && (flags & SaveFlags::dump) == SaveFlags::none)
        {
            ObjectManager::reloadAll();
        }

        if ((flags & SaveFlags::isAutosave) == SaveFlags::none)
        {
            Ui::ProgressBar::end();
        }

        if (saveResult)
        {
            Gfx::invalidateScreen();
            if ((flags & SaveFlags::raw) == SaveFlags::none)
            {
                SceneManager::resetSceneAge();
            }

            return true;
        }

        return false;
    }

    static bool isDefaultTimetableState(const Vehicles::TimetableManager::State& state)
    {
        return state.ticksPerMinute == Vehicles::TimetableManager::kDefaultTicksPerMinute
            && state.clockTicks == getGameState().scenarioTicks
            && state.nextServiceId == 1 && state.nextEntryId == 1
            && state.services.empty() && state.assignments.empty() && state.vehicles.empty();
    }

    static bool exportGameState(Stream& stream, const S5File& file, const std::vector<ObjectHeader>& packedObjects)
    {
        try
        {
            std::vector<std::byte> extensionData;
            const auto supportsRuleExtension = (file.header.type == S5Type::savedGame || file.header.type == S5Type::scenario || file.header.type == S5Type::landscape)
                && !file.header.hasFlags(HeaderFlags::isRaw | HeaderFlags::isTitleSequence);
            const auto supportsGameplayExtension = (file.header.type == S5Type::savedGame || file.header.type == S5Type::landscape)
                && !file.header.hasFlags(HeaderFlags::isRaw | HeaderFlags::isDump | HeaderFlags::isTitleSequence);
            const auto gameRulesState = GameRules::captureState();
            const auto* gameRules = gameRulesState == GameRules::kDefaultState ? nullptr : &gameRulesState;
            const auto* vehicleObjects = file.vehicleObjectState ? &*file.vehicleObjectState : nullptr;
            if (vehicleObjects != nullptr && !gameRulesState.extendedVehicleObjects)
            {
                throw Exception::RuntimeError("Extended vehicle objects require the extended-object game rule");
            }
            if (supportsGameplayExtension)
            {
                const auto sharedOrderState = Vehicles::SharedOrderManager::captureState();
                if (!Vehicles::SharedOrderManager::validateState(sharedOrderState))
                {
                    throw Exception::RuntimeError("Invalid shared vehicle order state");
                }
                const auto pathReservationState = Vehicles::RoutingManager::captureState();
                if (!Vehicles::RoutingManager::validateState(pathReservationState))
                {
                    throw Exception::RuntimeError("Invalid path reservation state");
                }
                const auto hasPathReservations = Vehicles::RoutingManager::hasPathReservations(pathReservationState);
                const auto vehicleAutoRenewalState = Vehicles::VehicleAutoRenewal::captureState();
                if (!Vehicles::VehicleAutoRenewal::validateState(vehicleAutoRenewalState))
                {
                    throw Exception::RuntimeError("Invalid vehicle auto-renewal state");
                }
                const auto hasVehicleAutoRenewal = !Vehicles::VehicleAutoRenewal::isDefault(vehicleAutoRenewalState);
                const auto vehicleReplacementState = Vehicles::VehicleReplacement::captureState();
                if (!Vehicles::VehicleReplacement::validateState(vehicleReplacementState))
                {
                    throw Exception::RuntimeError("Invalid vehicle replacement state");
                }
                const auto hasVehicleReplacement = !vehicleReplacementState.requests.empty();
                const auto railTrafficState = Vehicles::RailTraffic::captureState();
                if (!Vehicles::RailTraffic::validateState(railTrafficState, getGameState()))
                {
                    throw Exception::RuntimeError("Invalid rail traffic state");
                }
                const auto hasRailTraffic = !Vehicles::RailTraffic::isDefault(railTrafficState);
                const auto timetableState = Vehicles::TimetableManager::captureState();
                if (!Vehicles::TimetableManager::validateState(timetableState, getGameState(), sharedOrderState))
                {
                    throw Exception::RuntimeError("Invalid timetable state");
                }
                const auto hasTimetable = !isDefaultTimetableState(timetableState);
                std::vector<SaveExtension::StationTileOverflow> stationTileOverflow;
                for (size_t i = 0; i < Limits::kMaxStations; ++i)
                {
                    const auto& station = getGameState().stations[i];
                    if (station.stationTileSize <= S5::kMaxStationTilesInSave)
                    {
                        continue;
                    }
                    auto& entry = stationTileOverflow.emplace_back();
                    entry.station = static_cast<StationId>(i);
                    entry.stationTileSize = station.stationTileSize;
                    entry.stationTiles.assign(std::begin(station.stationTiles), std::begin(station.stationTiles) + station.stationTileSize);
                }
                const auto hasStationTileOverflow = !stationTileOverflow.empty();
                const auto hasModernState = !sharedOrderState.groups.empty() || hasPathReservations || hasVehicleAutoRenewal
                    || hasVehicleReplacement || hasRailTraffic || hasTimetable || hasStationTileOverflow || gameRules != nullptr || vehicleObjects != nullptr;
                extensionData = !hasModernState
                    ? CargoDist::encodeState(CargoDist::getStateConst())
                    : SaveExtension::encode({
                          .cargoDistState = &CargoDist::getStateConst(),
                          .sharedOrderState = sharedOrderState.groups.empty() ? nullptr : &sharedOrderState,
                          .pathReservationState = hasPathReservations ? &pathReservationState : nullptr,
                          .vehicleAutoRenewalState = hasVehicleAutoRenewal ? &vehicleAutoRenewalState : nullptr,
                          .vehicleReplacementState = hasVehicleReplacement ? &vehicleReplacementState : nullptr,
                          .railTrafficState = hasRailTraffic ? &railTrafficState : nullptr,
                          .stationTileOverflowState = hasStationTileOverflow ? &stationTileOverflow : nullptr,
                          .gameRulesState = gameRules,
                          .vehicleObjectState = vehicleObjects,
                          .timetableState = hasTimetable ? &timetableState : nullptr,
                      });
            }
            else if (supportsRuleExtension && (gameRules != nullptr || vehicleObjects != nullptr))
            {
                extensionData = SaveExtension::encode({
                    .gameRulesState = gameRules,
                    .vehicleObjectState = vehicleObjects,
                });
            }

            SawyerStreamWriter fs(stream);
            fs.writeChunk(SawyerEncoding::rotate, file.header);
            if (file.header.hasFlags(HeaderFlags::hasSaveDetails))
            {
                fs.writeChunk(SawyerEncoding::rotate, *file.saveDetails);
            }
            if (file.header.type == S5Type::scenario)
            {
                fs.writeChunk(SawyerEncoding::rotate, *file.scenarioOptions);
            }
            if (file.header.numPackedObjects != 0)
            {
                ObjectManager::writePackedObjects(fs, packedObjects);
            }
            fs.writeChunk(SawyerEncoding::rotate, file.requiredObjects, sizeof(file.requiredObjects));

            if (file.header.type == S5Type::scenario)
            {
                fs.writeChunk(SawyerEncoding::runLengthSingle, &file.gameState.general, sizeof(S5::GeneralState));
                fs.writeChunk(SawyerEncoding::runLengthSingle, file.gameState.towns, 0x123480);
                fs.writeChunk(SawyerEncoding::runLengthSingle, file.gameState.animations, 0x79D80);
            }
            else
            {
                fs.writeChunk(SawyerEncoding::runLengthSingle, file.gameState);
            }

            const auto hasScenarioTiles = file.header.type != S5Type::scenario
                || (static_cast<GameStateFlags>(file.gameState.general.flags) & GameStateFlags::tileManagerLoaded) != GameStateFlags::none;
            if (file.header.hasFlags(HeaderFlags::isRaw))
            {
                throw Exception::NotImplemented();
            }
            else if (hasScenarioTiles)
            {
                fs.writeChunk(SawyerEncoding::runLengthMulti, file.tileElements.data(), file.tileElements.size() * sizeof(TileElement));
            }

            if (!extensionData.empty())
            {
                fs.writeChunk(SawyerEncoding::uncompressed, extensionData.data(), extensionData.size());
            }

            fs.writeChecksum();
            return true;
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to save S5: {}", e.what());
            return false;
        }
    }

    // 0x00441FC9
    std::unique_ptr<S5File> loadSave(Stream& stream)
    {
        SawyerStreamReader fs(stream);
        if (!fs.validateChecksum())
        {
            throw Exception::RuntimeError("Invalid checksum");
        }

        auto file = std::make_unique<S5File>();

        // Read header
        fs.readChunk(&file->header, sizeof(file->header));

        // Read saved details 0x00442087
        if (file->header.hasFlags(HeaderFlags::hasSaveDetails))
        {
            file->saveDetails = std::make_unique<SaveDetails>();
            fs.readChunk(file->saveDetails.get(), sizeof(SaveDetails));
        }
        if (file->header.type == S5Type::scenario)
        {
            file->scenarioOptions = std::make_unique<S5::Options>();
            fs.readChunk(&*file->scenarioOptions, sizeof(S5::Options));
        }
        // Read packed objects
        if (file->header.numPackedObjects > 0)
        {
            for (auto i = 0; i < file->header.numPackedObjects; ++i)
            {
                ObjectHeader object;
                fs.read(&object, sizeof(ObjectHeader));
                auto unownedObjectData = fs.readChunk();
                std::vector<std::byte> objectData;
                objectData.resize(unownedObjectData.size());
                std::copy(std::begin(unownedObjectData), std::end(unownedObjectData), std::begin(objectData));
                file->packedObjects.push_back(std::make_pair(object, std::move(objectData)));
            }
            // 0x004420B2
        }

        if (file->header.type == S5Type::scenario)
        {
            // Load required objects
            fs.readChunk(file->requiredObjects, sizeof(file->requiredObjects));

            // Load game state up to just before companies
            fs.readChunk(&file->gameState, sizeof(file->gameState));
            // Load game state towns industry and stations
            fs.readChunk(&file->gameState.towns, sizeof(file->gameState));
            // Load the rest of gamestate after animations
            fs.readChunk(&file->gameState.animations, sizeof(file->gameState));
            file->gameState.general.fixFlags |= enumValue(S5FixFlags::fixFlag1);
            // fixState(file->gameState); this doesn't do anything as we have set fixFlag1

            if ((static_cast<GameStateFlags>(file->gameState.general.flags) & GameStateFlags::tileManagerLoaded) != GameStateFlags::none)
            {
                // Load tile elements
                auto tileElements = fs.readChunk();
                auto numTileElements = tileElements.size() / sizeof(TileElement);
                file->tileElements.resize(numTileElements);
                std::memcpy(file->tileElements.data(), tileElements.data(), numTileElements * sizeof(TileElement));
            }
        }
        else
        {
            // Load required objects
            fs.readChunk(file->requiredObjects, sizeof(file->requiredObjects));

            // Load game state
            auto chunkData = fs.readChunk();
            const auto fixFlags = static_cast<S5FixFlags>(chunkData[0x434]);
            if (((fixFlags & S5FixFlags::fixFlag0) == S5FixFlags::none) && ((fixFlags & S5FixFlags::fixFlag1) == S5FixFlags::none))
            {
                auto oldGameState = std::make_unique<S5::GameStateType2>();
                std::memcpy(&*oldGameState, chunkData.data(), sizeof(S5::GameStateType2));
                file->gameState = *importGameStateType2(*oldGameState);
            }
            else
            {
                std::memcpy(&file->gameState, chunkData.data(), sizeof(S5::GameState));
            }
            // old fixState 0x00445A4A would set this after adjusting the data
            file->gameState.general.fixFlags |= enumValue(S5FixFlags::fixFlag1);

            // Load tile elements
            auto tileElements = fs.readChunk();
            auto numTileElements = tileElements.size() / sizeof(TileElement);
            file->tileElements.resize(numTileElements);
            std::memcpy(file->tileElements.data(), tileElements.data(), numTileElements * sizeof(TileElement));
        }

        const auto supportsExtension = (file->header.type == S5Type::savedGame || file->header.type == S5Type::scenario || file->header.type == S5Type::landscape)
            && !file->header.hasFlags(HeaderFlags::isRaw | HeaderFlags::isTitleSequence);
        if (supportsExtension)
        {
            const auto checksumPosition = stream.getLength() - sizeof(uint32_t);
            if (stream.getPosition() < checksumPosition)
            {
                const auto bytesRemaining = checksumPosition - stream.getPosition();
                if (bytesRemaining < sizeof(SawyerEncoding) + sizeof(uint32_t))
                {
                    throw Exception::RuntimeError("Truncated S5 extension");
                }

                SawyerEncoding encoding;
                uint32_t encodedLength;
                fs.read(&encoding, sizeof(encoding));
                fs.read(&encodedLength, sizeof(encodedLength));
                if (encoding != SawyerEncoding::uncompressed || encodedLength > SaveExtension::kMaxDataSize
                    || encodedLength != checksumPosition - stream.getPosition())
                {
                    throw Exception::RuntimeError("Invalid S5 extension");
                }

                std::vector<std::byte> extensionData(encodedLength);
                fs.read(extensionData.data(), extensionData.size());
                auto extensionState = SaveExtension::decode(extensionData);
                file->cargoDistState = std::move(extensionState.cargoDistState);
                file->sharedOrderState = std::move(extensionState.sharedOrderState);
                file->pathReservationState = std::move(extensionState.pathReservationState);
                file->discardPathReservationsOnLoad = extensionState.discardPathReservationsOnLoad;
                file->vehicleAutoRenewalState = std::move(extensionState.vehicleAutoRenewalState);
                file->vehicleReplacementState = std::move(extensionState.vehicleReplacementState);
                file->railTrafficState = std::move(extensionState.railTrafficState);
                file->stationTileOverflowState = std::move(extensionState.stationTileOverflowState);
                file->gameRulesState = std::move(extensionState.gameRulesState);
                file->vehicleObjectState = std::move(extensionState.vehicleObjectState);
                file->timetableState = std::move(extensionState.timetableState);
                if (file->vehicleObjectState.has_value()
                    && (!file->gameRulesState.has_value() || !file->gameRulesState->extendedVehicleObjects))
                {
                    throw Exception::RuntimeError("Extended vehicle objects require the extended-object game rule");
                }
            }
            if (stream.getPosition() != checksumPosition)
            {
                throw Exception::RuntimeError("Invalid trailing S5 data");
            }
        }

        // Reset fields that don't affect the simulation, but would cause issues when comparing game states.
        for (auto& ent : file->gameState.entities)
        {
            ent.base.spriteLeft = Location::null;
            ent.base.spriteTop = Location::null;
            ent.base.spriteRight = Location::null;
            ent.base.spriteBottom = Location::null;
        }

        return file;
    }

    const LoadError& getLastLoadError()
    {
        return _lastLoadError;
    }

    void resetLastLoadError()
    {
        _lastLoadError = {};
    }

    class LoadException : public std::runtime_error
    {
    private:
        StringId _localisedMessage;

    public:
        LoadException(const char* message, StringId localisedMessage)
            : std::runtime_error(message)
            , _localisedMessage(localisedMessage)
        {
        }

        StringId getLocalisedMessage() const
        {
            return _localisedMessage;
        }
    };

    // 0x00441FA7
    bool importSaveToGameState(const fs::path& path, LoadFlags flags)
    {
        FileStream fs(path, StreamMode::read);
        return importSaveToGameState(fs, flags);
    }

    bool importSaveToGameState(Stream& stream, LoadFlags flags)
    {
        SceneManager::setGameSpeed(GameSpeed::Normal);
        if ((flags & LoadFlags::titleSequence) == LoadFlags::none
            && (flags & LoadFlags::twoPlayer) == LoadFlags::none)
        {
            WindowManager::closeConstructionWindows();
            WindowManager::closeAllFloatingWindows();
        }

        try
        {
            Ui::ProgressBar::begin(StringIds::loading);
            Ui::ProgressBar::setProgress(10);

            auto file = loadSave(stream);

            Ui::ProgressBar::setProgress(90);

            if (file->header.version != kCurrentVersion)
            {
                throw LoadException("Unsupported S5 version", StringIds::error_file_contains_invalid_data);
            }

#ifdef DO_TITLE_SEQUENCE_CHECKS
            if ((flags & LoadFlags::titleSequence) != LoadFlags::none)
            {
                if (!file->header.hasFlags(HeaderFlags::isTitleSequence))
                {
                    throw LoadException("File was not a title sequence", StringIds::error_file_contains_invalid_data);
                }
            }
            else
            {
                if (file->header.hasFlags(HeaderFlags::isTitleSequence))
                {
                    throw LoadException("File is a title sequence", StringIds::error_file_contains_invalid_data);
                }
            }
#endif
            if (hasLoadFlags(flags, LoadFlags::landscape))
            {
                if (file->header.type != S5Type::scenario)
                {
                    _lastLoadError = LoadError{
                        .errorCode = -1,
                        .errorMessage = StringIds::error_file_contains_invalid_data,
                    };
                    Ui::ProgressBar::end();
                    return false;
                }
                if (static_cast<EditorController::Step>(file->scenarioOptions->editorStep) == EditorController::Step::null)
                {
                    file->scenarioOptions->editorStep = enumValue(EditorController::Step::landscapeEditor);
                }
            }

            Ui::ProgressBar::setProgress(100);

            // Any packed objects to install?
            if (!file->packedObjects.empty())
            {
                // For now installing objects can't be done with a progress bar
                // revert this when objects do not change the current game state
                Ui::ProgressBar::end();

                bool objectInstalled = false;

                for (auto [object, data] : file->packedObjects)
                {
                    if (ObjectManager::tryInstallObject(object, data))
                    {
                        objectInstalled = true;
                    }
                }

                if (objectInstalled)
                {
                    ObjectManager::loadIndex();
                }

                // See above. restart progress bar
                Ui::ProgressBar::begin(StringIds::loading);
            }

            Ui::ProgressBar::setProgress(150);

            auto& dst = getGameState();

            if (file->header.type == S5Type::objects)
            {
                dst.var_014A = 0;
                _lastLoadError = LoadError{
                    .errorCode = -2,
                    .errorMessage = StringIds::new_objects_installed_successfully,
                };

                Ui::ProgressBar::end();
                Game::returnToTitle();
                return false;
            }

            if (!hasLoadFlags(flags, LoadFlags::scenario | LoadFlags::landscape))
            {
                if (file->header.type == S5Type::scenario)
                {
                    throw LoadException("File is a scenario, not a saved game", StringIds::error_file_contains_invalid_data);
                }
            }

            if (file->header.hasFlags(HeaderFlags::isRaw) || file->header.hasFlags(HeaderFlags::isDump))
            {
                throw LoadException("Unsupported S5 format", StringIds::error_file_contains_invalid_data);
            }

            if (hasLoadFlags(flags, LoadFlags::twoPlayer))
            {
                if (file->header.type != S5Type::landscape)
                {
                    throw LoadException("Not a two player saved game", StringIds::error_file_is_not_two_player_save);
                }
            }
            else if (!hasLoadFlags(flags, LoadFlags::scenario) && !hasLoadFlags(flags, LoadFlags::landscape))
            {
                if (file->header.type != S5Type::savedGame)
                {
                    throw LoadException("Not a single player saved game", StringIds::error_file_is_not_single_player_save);
                }
            }

            auto importedGameState = importGameState(file->gameState);
            if (file->vehicleObjectState.has_value())
            {
                for (size_t company = 0; company < Limits::kMaxCompanies; ++company)
                {
                    for (const auto& object : file->vehicleObjectState->objects)
                    {
                        importedGameState->companies[company].unlockedVehicles.set(
                            object.slot,
                            file->vehicleObjectState->companyUnlocks[company][object.slot - SaveExtension::kExtendedVehicleObjectStart]);
                    }
                }
            }
            if (file->stationTileOverflowState.has_value())
            {
                for (const auto& overflow : *file->stationTileOverflowState)
                {
                    auto& station = importedGameState->stations[enumValue(overflow.station)];
                    station.stationTileSize = overflow.stationTileSize;
                    std::copy(std::begin(overflow.stationTiles), std::end(overflow.stationTiles), std::begin(station.stationTiles));
                }
            }
            if (file->cargoDistState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence))
            {
                validateCargoDistObjects(*file->cargoDistState, file->requiredObjects);
                // Passenger cargo types are validated after loading this save's objects.
                CargoDist::validateState(*file->cargoDistState, *importedGameState, false);
            }
            if (file->sharedOrderState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::SharedOrderManager::validateState(*file->sharedOrderState, *importedGameState))
            {
                throw LoadException("Invalid shared vehicle order state", StringIds::error_file_contains_invalid_data);
            }
            if (file->pathReservationState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::RoutingManager::validateState(*file->pathReservationState, *importedGameState))
            {
                throw LoadException("Invalid path reservation state", StringIds::error_file_contains_invalid_data);
            }
            if (file->vehicleAutoRenewalState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::VehicleAutoRenewal::validateState(*file->vehicleAutoRenewalState, *importedGameState))
            {
                throw LoadException("Invalid vehicle auto-renewal state", StringIds::error_file_contains_invalid_data);
            }
            if (file->vehicleReplacementState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::VehicleReplacement::validateState(*file->vehicleReplacementState, *importedGameState))
            {
                throw LoadException("Invalid vehicle replacement state", StringIds::error_file_contains_invalid_data);
            }
            if (file->railTrafficState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::RailTraffic::validateState(*file->railTrafficState, *importedGameState))
            {
                throw LoadException("Invalid rail traffic state", StringIds::error_file_contains_invalid_data);
            }
            const Vehicles::SharedOrderManager::State emptySharedOrders;
            const auto& sharedOrders = file->sharedOrderState.value_or(emptySharedOrders);
            if (file->timetableState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                && !Vehicles::TimetableManager::validateState(*file->timetableState, *importedGameState, sharedOrders))
            {
                throw LoadException("Invalid timetable state", StringIds::error_file_contains_invalid_data);
            }

            // Load required objects
            auto requiredObjects = importRequiredObjectHeaders(file->requiredObjects, file->vehicleObjectState ? &*file->vehicleObjectState : nullptr);
            auto loadObjectResult = ObjectManager::loadAll(requiredObjects);
            if (!loadObjectResult.success)
            {
                _lastLoadError = LoadError{
                    .errorCode = -3,
                    .errorMessage = StringIds::null,
                    .objectList = loadObjectResult.problemObjects,
                };

                if (hasLoadFlags(flags, LoadFlags::twoPlayer))
                {
                    CompanyManager::reset();
                    dst.var_014A = 0;
                    Ui::ProgressBar::end();
                    return false;
                }
                else
                {
                    Ui::ProgressBar::end();
                    Game::returnToTitle();
                    return false;
                }
            }

            ObjectManager::reloadAll();
            if (file->cargoDistState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence))
            {
                CargoDist::validateState(*file->cargoDistState, *importedGameState);
            }
            Ui::ProgressBar::setProgress(200);

            Audio::stopVehicleNoise();
            Audio::stopAmbientNoise();

            // Copy the S5 gamestate contents to the destination gamestate, field by field
            dst = std::move(*importedGameState);
            GameRules::restoreState(file->gameRulesState.value_or(GameRules::kDefaultState));
            auto* cargoDistState = file->cargoDistState.has_value() && !hasLoadFlags(flags, LoadFlags::titleSequence)
                ? &*file->cargoDistState
                : nullptr;
            applyVehicleCapacityOverrides(cargoDistState);
            CargoDist::reset();
            Vehicles::SharedOrderManager::reset();
            Vehicles::RoutingManager::resetPathReservationState();
            Vehicles::VehicleAutoRenewal::reset();
            Vehicles::VehicleReplacement::reset();
            Vehicles::RailTraffic::reset();
            Vehicles::TimetableManager::reset(dst.scenarioTicks);

            // Copy scenario options
            if (hasLoadFlags(flags, LoadFlags::scenario | LoadFlags::landscape))
            {
                Scenario::getOptions() = importOptions(*file->scenarioOptions);
            }

            // Copy tile elements
            if ((dst.flags & GameStateFlags::tileManagerLoaded) != GameStateFlags::none)
            {
                loadTileElements(dst, file->tileElements);
            }
            else
            {
                World::TileManager::initialise();
                Scenario::sub_46115C();
            }

            // Copy entity and company strings
            if (hasLoadFlags(flags, LoadFlags::landscape))
            {
                EntityManager::freeUserStrings();
            }
            if (hasLoadFlags(flags, LoadFlags::scenario | LoadFlags::landscape))
            {
                CompanyManager::reset();
                EntityManager::reset();
            }

            EntityManager::resetSpatialIndex();
            CompanyManager::updateColours();
            ObjectManager::updateTerraformObjects();
            TileManager::resetSurfaceClearance();
            IndustryManager::createAllMapAnimations();

            Ui::ProgressBar::setProgress(225);

            if (hasLoadFlags(flags, LoadFlags::landscape))
            {
                Scenario::initialiseSnowLine();
                auto* stexObj = ObjectManager::get<ScenarioTextObject>();
                if (stexObj != nullptr)
                {
                    auto header = ObjectManager::getHeader(LoadedObjectHandle{ ObjectType::scenarioText, 0 });
                    ObjectManager::unload(header);
                    ObjectManager::reloadAll();
                    ObjectManager::updateTerraformObjects();
                    auto& options = Scenario::getOptions();
                    options.editorStep = EditorController::Step::landscapeEditor;
                    options.difficulty = 3;
                    StringManager::formatString(options.scenarioDetails, StringIds::no_details_yet);
                    options.scenarioName[0] = '\0';
                }
            }
            Audio::resetSoundObjects();
            TownManager::rebuildRuntimeMetrics();

            if (hasLoadFlags(flags, LoadFlags::scenario))
            {
                dst.var_014A = 0;
                Ui::ProgressBar::end();
                return true;
            }
            if (!hasLoadFlags(flags, LoadFlags::titleSequence))
            {
                SceneManager::removeSceneFlags(SceneManager::Flags::title);
                resetSubsystems();
                Audio::resetMusic();
                if (hasLoadFlags(flags, LoadFlags::landscape))
                {
                    SceneManager::addSceneFlags(SceneManager::Flags::editor);
                    EditorController::showEditor();
                }
                else
                {
                    Gui::init();
                }
            }

            Ui::ProgressBar::setProgress(245);

            auto mainWindow = WindowManager::getMainWindow();
            if (mainWindow != nullptr)
            {
                SavedViewSimple savedView;
                savedView.viewX = file->gameState.general.savedViewX;
                savedView.viewY = file->gameState.general.savedViewY;
                savedView.zoomLevel = ZoomLevel::fromEncoded(file->gameState.general.savedViewZoom);
                savedView.rotation = file->gameState.general.savedViewRotation;
                mainWindow->viewportFromSavedView(savedView);
                mainWindow->invalidate();
            }

            EntityManager::updateSpatialIndex();
            TownManager::updateLabels();
            StationManager::updateLabels();
            Ui::Windows::Terraform::resetDefaultObjectIds();
            WindowManager::resetThousandthTickCounter();
            Gfx::invalidateScreen();
            if (!hasLoadFlags(flags, LoadFlags::landscape))
            {
                Scenario::loadPreferredCurrencyAlways();
            }
            Gfx::loadCurrency();
            dst.var_014A = 0;

            if (hasLoadFlags(flags, LoadFlags::titleSequence))
            {
                ScenarioManager::setScenarioTicks(ScenarioManager::getScenarioTicks() - 1);
                ScenarioManager::setScenarioTicks2(ScenarioManager::getScenarioTicks2() - 1);
                World::TileManager::disablePeriodicDefrag();
            }
            else
            {
                if (file->sharedOrderState.has_value()
                    && !Vehicles::SharedOrderManager::restoreState(*file->sharedOrderState))
                {
                    throw Exception::RuntimeError("Invalid shared vehicle order state");
                }
                if (file->pathReservationState.has_value() && file->discardPathReservationsOnLoad)
                {
                    discardLegacyPathReservations(*file->pathReservationState);
                }
                else if (file->pathReservationState.has_value() && !Vehicles::RoutingManager::restoreState(*file->pathReservationState))
                {
                    throw Exception::RuntimeError("Invalid path reservation state");
                }
                if (file->cargoDistState.has_value())
                {
                    CargoDist::restoreState(std::move(*file->cargoDistState));
                }
                else
                {
                    CargoDist::State legacyState;
                    legacyState.requiresStationMetadataRefresh = true;
                    CargoDist::restoreState(std::move(legacyState));
                }
                if (file->vehicleAutoRenewalState.has_value()
                    && !Vehicles::VehicleAutoRenewal::restoreState(*file->vehicleAutoRenewalState))
                {
                    throw Exception::RuntimeError("Invalid vehicle auto-renewal state");
                }
                if (file->vehicleReplacementState.has_value()
                    && !Vehicles::VehicleReplacement::restoreState(*file->vehicleReplacementState))
                {
                    throw Exception::RuntimeError("Invalid vehicle replacement state");
                }
                if (file->railTrafficState.has_value()
                    && !Vehicles::RailTraffic::restoreState(*file->railTrafficState))
                {
                    throw Exception::RuntimeError("Invalid rail traffic state");
                }
                if (file->timetableState.has_value()
                    && !Vehicles::TimetableManager::restoreState(*file->timetableState))
                {
                    throw Exception::RuntimeError("Invalid timetable state");
                }
            }

            Ui::ProgressBar::end();

            if (!hasLoadFlags(flags, LoadFlags::titleSequence) && !hasLoadFlags(flags, LoadFlags::twoPlayer) && !hasLoadFlags(flags, LoadFlags::landscape))
            {
                SceneManager::resetSceneAge();
            }

            return true;
        }
        catch (const LoadException& e)
        {
            Logging::error("Unable to load S5: {}", e.what());
            _lastLoadError = LoadError{
                .errorCode = -4,
                .errorMessage = e.getLocalisedMessage(),
            };
            Ui::ProgressBar::end();
            return false;
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to load S5: {}", e.what());
            _lastLoadError = LoadError{
                .errorCode = -5,
                .errorMessage = StringIds::null,
            };
            Ui::ProgressBar::end();
            return false;
        }
    }

    // 0x00442403
    std::unique_ptr<SaveDetails> readSaveDetails(const fs::path& path)
    {
        FileStream stream(path, StreamMode::read);
        SawyerStreamReader fs(stream);
        if (!fs.validateChecksum())
        {
            return nullptr;
        }

        Header s5Header{};

        // Read header
        fs.readChunk(&s5Header, sizeof(s5Header));

        if (s5Header.version != kCurrentVersion)
        {
            return nullptr;
        }

        if (s5Header.hasFlags(HeaderFlags::isTitleSequence | HeaderFlags::isDump | HeaderFlags::isRaw))
        {
            return nullptr;
        }

        if (s5Header.hasFlags(HeaderFlags::hasSaveDetails))
        {
            // 0x0050AEA8
            auto ret = std::make_unique<SaveDetails>();
            fs.readChunk(ret.get(), sizeof(*ret));
            return ret;
        }
        return nullptr;
    }

    // 0x00442AFC
    std::unique_ptr<Scenario::Options> readScenarioOptions(const fs::path& path)
    {
        FileStream stream(path, StreamMode::read);
        SawyerStreamReader fs(stream);
        if (!fs.validateChecksum())
        {
            return nullptr;
        }

        Header s5Header{};

        // Read header
        fs.readChunk(&s5Header, sizeof(s5Header));

        if (s5Header.version != kCurrentVersion)
        {
            return nullptr;
        }

        if (s5Header.type == S5Type::scenario)
        {
            // 0x009DA285 = 1
            // 0x009CCA54 _previewOptions

            auto s5Options = std::make_unique<S5::Options>();
            fs.readChunk(s5Options.get(), sizeof(S5::Options));
            return std::make_unique<Scenario::Options>(importOptions(*s5Options));
        }
        return nullptr;
    }
}
