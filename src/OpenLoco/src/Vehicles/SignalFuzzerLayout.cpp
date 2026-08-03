#include "Vehicles/SignalFuzzerLayout.h"

#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Terraform/ClearLand.h"
#include "GameCommands/Track/CreateSignal.h"
#include "GameCommands/Track/CreateTrack.h"
#include "GameCommands/Vehicles/CreateVehicle.h"
#include "GameCommands/Vehicles/VehicleChangeRunningMode.h"
#include "GameCommands/Vehicles/VehiclePlace.h"
#include "Logging.h"
#include "Map/BuildingElement.h"
#include "Map/SurfaceElement.h"
#include "Map/TileManager.h"
#include "Map/Track/TrackEnum.h"
#include "Objects/BuildingObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/ObjectUtils.h"
#include "Objects/TrackObject.h"
#include "Objects/VehicleObject.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include <algorithm>
#include <vector>

using namespace OpenLoco::Diagnostics;

namespace OpenLoco::Vehicles::SignalFuzzer::Layouts
{
    namespace
    {
        constexpr int16_t kSiteSize = 29;
        constexpr int16_t kSiteMargin = kSiteSize / 2;
        constexpr uint16_t kLeftSignal = 0x8000;
        constexpr uint16_t kRightSignal = 0x4000;
        constexpr uint8_t kCommandFlags = GameCommands::Flags::apply | GameCommands::Flags::allowNegativeCashFlow | GameCommands::Flags::noErrorWindow | GameCommands::Flags::noPayment;

        struct Assets
        {
            CompanyId owner;
            uint16_t vehicleObject;
            uint8_t trackObject;
            uint8_t requiredMods;
            uint8_t signalObject;
        };

        struct FlatSite
        {
            World::TilePos2 origin;
            World::SmallZ baseZ;
        };

        struct TrackSpec
        {
            int8_t x;
            int8_t y;
            World::Track::TrackId id;
            uint8_t rotation;
        };

        struct SignalSpec
        {
            int8_t x;
            int8_t y;
            uint8_t rotation;
            uint16_t side;
        };

        struct TrainSpec
        {
            int8_t x;
            int8_t y;
            uint16_t trackAndDirection;
        };

        struct Template
        {
            std::vector<TrackSpec> tracks;
            std::vector<SignalSpec> signals;
            std::vector<TrainSpec> trains;
        };

        struct UpdatingCompanyScope
        {
            CompanyId previous = GameCommands::getUpdatingCompanyId();

            explicit UpdatingCompanyScope(const CompanyId company)
            {
                GameCommands::setUpdatingCompanyId(company);
            }

            ~UpdatingCompanyScope()
            {
                GameCommands::setUpdatingCompanyId(previous);
            }
        };

        static bool isSiteBlocked(const World::TilePos2 pos)
        {
            const auto tile = World::TileManager::get(pos);
            const auto* surface = tile.surface();
            if (surface == nullptr || surface->isIndustrial())
            {
                return true;
            }
            return std::ranges::any_of(tile, [](const auto& entry) {
                const auto type = entry.type();
                if (type == World::ElementType::building)
                {
                    const auto* object = entry.template as<World::BuildingElement>()->getObject();
                    return object->hasFlags(BuildingObjectFlags::isHeadquarters | BuildingObjectFlags::indestructible);
                }
                return type != World::ElementType::surface && type != World::ElementType::tree && type != World::ElementType::wall;
            });
        }

        static bool flattenSite(const FlatSite& site, const CompanyId owner)
        {
            const auto pointA = World::toWorldSpace(site.origin);
            const auto pointB = World::toWorldSpace(World::TilePos2{ static_cast<tile_coord_t>(site.origin.x + kSiteSize - 1), static_cast<tile_coord_t>(site.origin.y + kSiteSize - 1) });
            GameCommands::ClearLandArgs clearArgs{};
            clearArgs.centre = (pointA + pointB) / 2;
            clearArgs.pointA = pointA;
            clearArgs.pointB = pointB;
            UpdatingCompanyScope companyScope(owner);
            if (GameCommands::doCommand(clearArgs, kCommandFlags) == GameCommands::kFailure)
            {
                Logging::error("Unable to clear generated signal fuzz site");
                return false;
            }

            for (auto y = site.origin.y; y < site.origin.y + kSiteSize; ++y)
            {
                for (auto x = site.origin.x; x < site.origin.x + kSiteSize; ++x)
                {
                    auto* surface = World::TileManager::get(World::TilePos2{ x, y }).surface();
                    if (surface == nullptr)
                    {
                        return false;
                    }
                    surface->setBaseZ(site.baseZ);
                    surface->setClearZ(site.baseZ);
                    surface->setSlope(World::SurfaceSlope::flat);
                    surface->setWater(0);
                    World::TileManager::mapInvalidateTileFull(World::toWorldSpace(World::TilePos2{ x, y }));
                }
            }
            World::TileManager::resetSurfaceClearance();
            return true;
        }

        static std::optional<Assets> findAssets()
        {
            for (const auto* head : VehicleManager::VehicleList())
            {
                if (head->mode != TransportMode::rail || head->tileX == -1)
                {
                    continue;
                }
                const auto* track = ObjectManager::get<TrackObject>(head->trackType);
                if (track == nullptr || !track->hasTraitFlags(World::Track::TrackTraitFlags::smallCurve))
                {
                    continue;
                }

                const auto signals = getAvailableCompatibleSignals(head->trackType);
                if (signals.empty())
                {
                    continue;
                }

                const Vehicle train(*head);
                for (const auto& car : train.cars)
                {
                    const auto* vehicle = ObjectManager::get<VehicleObject>(car.front->objectId);
                    if (vehicle != nullptr && vehicle->power != 0)
                    {
                        return Assets{ head->owner, car.front->objectId, head->trackType, head->var_53, signals.front() };
                    }
                }
            }
            return std::nullopt;
        }

        static Template makeTemplate(const Layout layout)
        {
            Template result;
            for (int8_t x = -11; x <= 11; ++x)
            {
                result.tracks.push_back({ x, 0, World::Track::TrackId::straight, 0 });
            }

            std::vector<int8_t> northBranches;
            std::vector<int8_t> southBranches;
            switch (layout)
            {
                case Layout::flatMerge:
                    northBranches = { 0 };
                    break;
                case Layout::flatFan:
                    northBranches = { -4, 0, 4 };
                    break;
                case Layout::flatInterchange:
                    northBranches = { -6, 0, 6 };
                    southBranches = { -3, 3 };
                    break;
                default:
                    return result;
            }

            for (const auto x : northBranches)
            {
                const auto branchX = static_cast<int8_t>(x - 1);
                result.tracks.push_back({ x, 0, World::Track::TrackId::leftCurveSmall, 0 });
                for (int8_t y = -2; y >= -11; --y)
                {
                    result.tracks.push_back({ branchX, y, World::Track::TrackId::straight, 3 });
                }
                result.signals.push_back({ branchX, -7, 3, kRightSignal });
                result.trains.push_back({ branchX, -9, 7 });
            }
            for (const auto x : southBranches)
            {
                const auto branchX = static_cast<int8_t>(x - 1);
                result.tracks.push_back({ x, 0, World::Track::TrackId::rightCurveSmall, 0 });
                for (int8_t y = 2; y <= 11; ++y)
                {
                    result.tracks.push_back({ branchX, y, World::Track::TrackId::straight, 1 });
                }
                result.signals.push_back({ branchX, 7, 1, kRightSignal });
                result.trains.push_back({ branchX, 9, 5 });
            }

            result.signals.push_back({ -8, 0, 0, kRightSignal });
            result.signals.push_back({ 8, 0, 0, kLeftSignal });
            result.trains.push_back({ -9, 0, 4 });
            result.trains.push_back({ 9, 0, 0 });
            return result;
        }

        static World::Pos3 getPosition(const FlatSite& site, const int8_t x, const int8_t y)
        {
            const auto centre = World::toWorldSpace(World::TilePos2{ static_cast<tile_coord_t>(site.origin.x + kSiteMargin), static_cast<tile_coord_t>(site.origin.y + kSiteMargin) });
            return { static_cast<coord_t>(centre.x + x * World::kTileSize), static_cast<coord_t>(centre.y + y * World::kTileSize), static_cast<coord_t>(site.baseZ * World::kSmallZStep) };
        }

        static bool buildTrack(const Template& layout, const FlatSite& site, const Assets& assets)
        {
            for (const auto& spec : layout.tracks)
            {
                GameCommands::TrackPlacementArgs args{};
                args.pos = getPosition(site, spec.x, spec.y);
                args.rotation = spec.rotation;
                args.trackId = enumValue(spec.id);
                args.mods = assets.requiredMods;
                args.unkFlags = 0;
                args.bridge = 0xFF;
                args.trackObjectId = assets.trackObject;
                args.unk = false;
                if (GameCommands::doCommand(args, kCommandFlags) == GameCommands::kFailure)
                {
                    Logging::error("Unable to place generated track at ({}, {}) with piece {}", args.pos.x, args.pos.y, args.trackId);
                    return false;
                }
            }
            for (const auto& spec : layout.signals)
            {
                GameCommands::SignalPlacementArgs args{};
                args.pos = getPosition(site, spec.x, spec.y);
                args.rotation = spec.rotation;
                args.trackId = enumValue(World::Track::TrackId::straight);
                args.index = 0;
                args.type = assets.signalObject;
                args.mode = World::SignalMode::path;
                args.trackObjType = assets.trackObject;
                args.sides = spec.side;
                if (GameCommands::doCommand(args, kCommandFlags) == GameCommands::kFailure)
                {
                    Logging::error("Unable to place generated signal at ({}, {})", args.pos.x, args.pos.y);
                    return false;
                }
            }
            return true;
        }

        static std::optional<EntityId> createTrain(const TrainSpec& spec, const FlatSite& site, const Assets& assets)
        {
            GameCommands::VehicleCreateArgs createArgs{};
            createArgs.vehicleId = EntityId::null;
            createArgs.vehicleType = assets.vehicleObject;
            if (GameCommands::doCommand(createArgs, kCommandFlags) == GameCommands::kFailure)
            {
                return std::nullopt;
            }
            auto* component = EntityManager::get<VehicleBase>(GameCommands::getLegacyReturnState().lastCreatedVehicleId);
            auto* head = component == nullptr ? nullptr : EntityManager::get<VehicleHead>(component->getHead());
            if (head == nullptr)
            {
                return std::nullopt;
            }

            GameCommands::VehiclePlacementArgs placementArgs{};
            placementArgs.pos = getPosition(site, spec.x, spec.y);
            placementArgs.trackAndDirection = spec.trackAndDirection;
            placementArgs.trackProgress = 0;
            placementArgs.head = head->id;
            if (GameCommands::doCommand(placementArgs, kCommandFlags) == GameCommands::kFailure)
            {
                return std::nullopt;
            }

            GameCommands::VehicleChangeRunningModeArgs startArgs{};
            startArgs.head = head->id;
            startArgs.mode = GameCommands::VehicleChangeRunningModeArgs::Mode::startVehicle;
            if (GameCommands::doCommand(startArgs, kCommandFlags) == GameCommands::kFailure)
            {
                return std::nullopt;
            }
            return head->id;
        }

        static std::optional<FlatSite> findFlatSite()
        {
            constexpr auto pitch = World::kMapColumns + 1;
            std::vector<uint32_t> blocked(static_cast<size_t>(pitch) * (World::kMapRows + 1));
            const auto at = [&blocked](const int32_t x, const int32_t y) -> uint32_t& {
                return blocked[static_cast<size_t>(y) * pitch + x];
            };
            for (int32_t y = 0; y < World::kMapRows; ++y)
            {
                for (int32_t x = 0; x < World::kMapColumns; ++x)
                {
                    const auto value = isSiteBlocked(World::TilePos2{ static_cast<tile_coord_t>(x), static_cast<tile_coord_t>(y) }) ? 1U : 0U;
                    at(x + 1, y + 1) = value + at(x, y + 1) + at(x + 1, y) - at(x, y);
                }
            }
            for (int32_t y = 3; y + kSiteSize < World::kMapRows - 3; ++y)
            {
                for (int32_t x = 3; x + kSiteSize < World::kMapColumns - 3; ++x)
                {
                    const auto x2 = x + kSiteSize;
                    const auto y2 = y + kSiteSize;
                    if (at(x2, y2) - at(x, y2) - at(x2, y) + at(x, y) != 0)
                    {
                        continue;
                    }
                    World::SmallZ baseZ = 0;
                    for (auto tileY = y; tileY < y2; ++tileY)
                    {
                        for (auto tileX = x; tileX < x2; ++tileX)
                        {
                            baseZ = std::max(baseZ, World::TileManager::get(World::TilePos2{ static_cast<tile_coord_t>(tileX), static_cast<tile_coord_t>(tileY) }).surface()->baseZ());
                        }
                    }
                    return FlatSite{ { static_cast<tile_coord_t>(x), static_cast<tile_coord_t>(y) }, baseZ };
                }
            }
            return std::nullopt;
        }
    }

    std::optional<PreparedLayout> generate(const Layout layout)
    {
        const auto generated = makeTemplate(layout);
        if (generated.tracks.empty())
        {
            return std::nullopt;
        }
        const auto site = findFlatSite();
        if (!site.has_value())
        {
            Logging::error("Unable to find a clear site for generated signal fuzz layout '{}'", layoutName(layout));
            return std::nullopt;
        }
        const auto assets = findAssets();
        if (!assets.has_value())
        {
            Logging::error("Unable to find compatible train, track, and signal objects for generated signal fuzz layouts");
            return std::nullopt;
        }
        if (!flattenSite(*site, assets->owner))
        {
            return std::nullopt;
        }

        UpdatingCompanyScope companyScope(assets->owner);
        if (!buildTrack(generated, *site, *assets))
        {
            return std::nullopt;
        }

        PreparedLayout result;
        const auto centre = getPosition(*site, 0, 0);
        result.centre = { centre.x, centre.y };
        result.radius = kSiteMargin * World::kTileSize;
        for (const auto& train : generated.trains)
        {
            const auto vehicle = createTrain(train, *site, *assets);
            if (!vehicle.has_value())
            {
                Logging::error("Unable to create generated signal fuzz train at ({}, {})", train.x, train.y);
                return std::nullopt;
            }
            result.vehicles.push_back(*vehicle);
        }
        return result;
    }
}
