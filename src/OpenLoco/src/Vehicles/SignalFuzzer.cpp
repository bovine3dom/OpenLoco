#include "Vehicles/SignalFuzzer.h"

#include "Config.h"
#include "Entities/EntityManager.h"
#include "Localisation/Formatting.h"
#include "Logging.h"
#include "Map/SignalElement.h"
#include "Map/TileManager.h"
#include "Map/TrackElement.h"
#include "Objects/ObjectManager.h"
#include "Objects/VehicleObject.h"
#include "OpenLoco.h"
#include "S5/S5.h"
#include "Scenario/ScenarioManager.h"
#include "SceneManager.h"
#include "Scenes/BootScene.h"
#include "Scenes/GameScene.h"
#include "Vehicles/PathSignals.h"
#include "Vehicles/RoutingManager.h"
#include "Vehicles/SignalFuzzerLayout.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleCollision.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleTail.h"
#include "World/StationManager.h"
#include "World/TownManager.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <deque>
#include <fstream>
#include <locale>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace OpenLoco::Diagnostics;

namespace OpenLoco::Vehicles::SignalFuzzer
{
    namespace
    {
        constexpr auto kFocusRadius = 160 * World::kTileSize;
        constexpr size_t kTraceLength = 256;
        constexpr std::array kGeneratedLayouts{ Layout::flatMerge, Layout::flatFan, Layout::flatInterchange };

        struct FocusArea
        {
            World::Pos2 centre;
            int32_t radius;
        };

        struct Collision
        {
            EntityId source = EntityId::null;
            EntityId target = EntityId::null;
            uint32_t tick{};
            bool pathReservationIncursion{};
            bool overlappingPathReservations{};
        };

        struct RouteConflict
        {
            EntityId first = EntityId::null;
            EntityId second = EntityId::null;
            World::Pos3 pos{};
            uint8_t quarters{};
            uint32_t tick{};
            bool firstOccupied{};
            bool secondOccupied{};
            bool firstPathReserved{};
            bool secondPathReserved{};

            bool isPathReservationConflict() const
            {
                return firstPathReserved && secondPathReserved;
            }

            bool involvesPathReservation() const
            {
                return firstPathReserved || secondPathReserved;
            }
        };

        struct VehicleTrace
        {
            EntityId id;
            int16_t ordinalNumber;
            World::Pos3 headPos;
            World::Pos3 tailPos;
            World::Pos3 tilePos;
            int32_t speed;
            int32_t targetSpeed;
            uint16_t trackAndDirection;
            uint16_t subPosition;
            std::array<uint16_t, 5> routings;
            uint8_t status;
            uint8_t breakdownFlags;
            uint16_t reservedPieces;
        };

        struct TraceFrame
        {
            uint32_t tick;
            std::vector<VehicleTrace> vehicles;
        };

        struct RunContext
        {
            uint32_t tick{};
            std::optional<Collision> collision;
        };

        struct CaseResult
        {
            std::optional<Collision> collision;
            std::optional<RouteConflict> routeOverlap;
            std::optional<RouteConflict> reservationConflict;
            bool breakdownInjected{};
            std::optional<uint32_t> breakdownTick;
            std::deque<TraceFrame> trace;
        };

        struct PreparedCase
        {
            FocusArea focus;
            std::vector<EntityId> candidates;
        };

        static RunContext* _runContext;

        static void onCollision(const EntityId sourceHead, const EntityId collidedComponent)
        {
            if (_runContext == nullptr || _runContext->collision.has_value())
            {
                return;
            }

            auto targetHead = EntityId::null;
            const auto* target = EntityManager::get<VehicleBase>(collidedComponent);
            if (target != nullptr)
            {
                targetHead = target->getHead();
            }
            _runContext->collision = Collision{ sourceHead, targetHead, _runContext->tick };
        }

        struct RunMonitor
        {
            explicit RunMonitor(RunContext& context)
            {
                _runContext = &context;
                VehicleCollision::setCallback(onCollision);
            }

            ~RunMonitor()
            {
                VehicleCollision::setCallback(nullptr);
                _runContext = nullptr;
            }
        };

        static std::string formatName(const StringId id)
        {
            char buffer[256]{};
            StringManager::formatString(buffer, sizeof(buffer), id);
            return buffer;
        }

        static bool equalsIgnoreCase(const std::string_view lhs, const std::string_view rhs)
        {
            return lhs.size() == rhs.size() && std::ranges::equal(lhs, rhs, [](const char a, const char b) {
                       return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
                   });
        }

        static Town* findTown(const std::string_view name)
        {
            for (auto& town : TownManager::towns())
            {
                if (equalsIgnoreCase(formatName(town.name), name))
                {
                    return &town;
                }
            }
            return nullptr;
        }

        static bool isNear(const VehicleHead& head, const FocusArea& focus)
        {
            return std::abs(static_cast<int32_t>(head.tileX) - focus.centre.x) + std::abs(static_cast<int32_t>(head.tileY) - focus.centre.y) <= focus.radius;
        }

        static std::vector<EntityId> getRailVehicles()
        {
            std::vector<EntityId> result;
            for (const auto* head : VehicleManager::VehicleList())
            {
                if (head->mode == TransportMode::rail && head->tileX != -1 && head->status != Status::crashed && head->status != Status::stuck)
                {
                    result.push_back(head->id);
                }
            }
            return result;
        }

        static bool hasBreakdown(const Vehicle& train)
        {
            for (const auto& car : train.cars)
            {
                if (car.front->hasBreakdownFlags(BreakdownFlags::brokenDown | BreakdownFlags::breakdownPending))
                {
                    return true;
                }
            }
            return false;
        }

        static bool injectBreakdown(VehicleHead& head)
        {
            Vehicle train(head);
            if (hasBreakdown(train))
            {
                return false;
            }
            for (auto& car : train.cars)
            {
                const auto* object = ObjectManager::get<VehicleObject>(car.front->objectId);
                if (object != nullptr && object->power != 0)
                {
                    car.front->breakdownFlags |= BreakdownFlags::breakdownPending;
                    return true;
                }
            }
            return false;
        }

        static uint8_t getBreakdownFlags(const Vehicle& train)
        {
            auto flags = BreakdownFlags::none;
            for (const auto& car : train.cars)
            {
                flags |= car.front->breakdownFlags & (BreakdownFlags::brokenDown | BreakdownFlags::breakdownPending);
            }
            return enumValue(flags);
        }

        static uint16_t getReservedPieces(const VehicleHead& head)
        {
            uint16_t count = 0;
            for ([[maybe_unused]] const auto handle : RoutingManager::RingView(head.routingHandle))
            {
                count++;
            }
            return (count == 0 ? 0 : count - 1) + static_cast<uint16_t>(RoutingManager::getReservedContinuation(head.routingHandle).size());
        }

        static std::array<uint16_t, 5> getRoutings(const VehicleHead& head)
        {
            std::array<uint16_t, 5> result{};
            auto handle = head.routingHandle;
            for (auto& routing : result)
            {
                routing = RoutingManager::getRouting(handle);
                handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            }
            return result;
        }

        static std::optional<RouteConflict> findRouteConflict(
            const uint32_t tick,
            const FocusArea& focus,
            const std::optional<std::pair<EntityId, EntityId>>& vehiclePair = std::nullopt)
        {
            struct Claim
            {
                uint32_t mask{};
                uint32_t occupiedMask{};
                uint32_t pathReservedMask{};
            };
            std::map<std::tuple<coord_t, coord_t, coord_t>, std::map<EntityId, Claim>> claims;
            for (const auto& resource : PathSignals::getClaimedResources())
            {
                if (std::abs(static_cast<int32_t>(resource.pos.x) - focus.centre.x) + std::abs(static_cast<int32_t>(resource.pos.y) - focus.centre.y) > focus.radius)
                {
                    continue;
                }
                auto& claim = claims[{ resource.pos.x, resource.pos.y, resource.pos.z }][resource.vehicle];
                claim.mask |= resource.conflictMask;
                claim.occupiedMask |= resource.occupied ? resource.conflictMask : 0;
                claim.pathReservedMask |= resource.pathReserved ? resource.conflictMask : 0;
            }

            std::optional<RouteConflict> unprotectedConflict;
            for (const auto& [pos, claimsAtPosition] : claims)
            {
                for (auto first = claimsAtPosition.begin(); first != claimsAtPosition.end(); ++first)
                {
                    auto second = first;
                    for (++second; second != claimsAtPosition.end(); ++second)
                    {
                        if (vehiclePair.has_value()
                            && !((first->first == vehiclePair->first && second->first == vehiclePair->second) || (first->first == vehiclePair->second && second->first == vehiclePair->first)))
                        {
                            continue;
                        }
                        const auto overlapMask = first->second.mask & second->second.mask;
                        uint8_t overlappingQuarters = 0;
                        for (uint8_t connection = 0; connection < 8; ++connection)
                        {
                            overlappingQuarters |= static_cast<uint8_t>((overlapMask >> (connection * 4)) & 0xF);
                        }
                        if (overlappingQuarters == 0)
                        {
                            continue;
                        }
                        const auto& [x, y, z] = pos;
                        RouteConflict conflict{
                            first->first,
                            second->first,
                            { x, y, z },
                            overlappingQuarters,
                            tick,
                            (first->second.occupiedMask & overlapMask) != 0,
                            (second->second.occupiedMask & overlapMask) != 0,
                            (first->second.pathReservedMask & overlapMask) != 0,
                            (second->second.pathReservedMask & overlapMask) != 0,
                        };
                        if (conflict.isPathReservationConflict())
                        {
                            return conflict;
                        }
                        if (!unprotectedConflict.has_value()
                            || (!unprotectedConflict->involvesPathReservation() && conflict.involvesPathReservation()))
                        {
                            unprotectedConflict = conflict;
                        }
                    }
                }
            }
            return unprotectedConflict;
        }

        static TraceFrame captureTrace(const uint32_t tick, const FocusArea& focus)
        {
            TraceFrame result{ tick, {} };
            for (const auto* head : VehicleManager::VehicleList())
            {
                if (head->mode != TransportMode::rail || head->tileX == -1 || !isNear(*head, focus))
                {
                    continue;
                }
                const Vehicle train(*head);
                result.vehicles.push_back({
                    head->id,
                    head->ordinalNumber,
                    head->position,
                    train.tail->position,
                    { head->tileX, head->tileY, static_cast<coord_t>(head->tileBaseZ * World::kSmallZStep) },
                    train.veh2->currentSpeed.getRaw(),
                    train.veh1->targetSpeed.getRaw(),
                    head->trackAndDirection.track._data,
                    head->subPosition,
                    getRoutings(*head),
                    enumValue(head->status),
                    getBreakdownFlags(train),
                    getReservedPieces(*head),
                });
            }
            return result;
        }

        static bool canInject(const VehicleHead& head, const FocusArea& focus)
        {
            if (!isNear(head, focus) || (head.status != Status::travelling && head.status != Status::approaching))
            {
                return false;
            }
            const Vehicle train(head);
            return train.veh2->currentSpeed.getRaw() > 0 && !hasBreakdown(train);
        }

        static bool loadInitialSave(const fs::path& path)
        {
            try
            {
                initialise();
                Scenes::BootScene::loadFile(path);
                SceneManager::applySceneTransition();
            }
            catch (const std::exception& e)
            {
                Logging::error("Unable to load signal fuzz save: {}", e.what());
                return false;
            }
            return SceneManager::getCurrentScene() == SceneManager::SceneId::gameplay;
        }

        static bool reloadSave(const fs::path& path)
        {
            try
            {
                return S5::importSaveToGameState(path, S5::LoadFlags::none);
            }
            catch (const std::exception& e)
            {
                Logging::error("Unable to reload signal fuzz save: {}", e.what());
                return false;
            }
        }

        static Layout resolveLayout(const Layout layout, const uint32_t caseIndex)
        {
            if (layout != Layout::flatAll)
            {
                return layout;
            }
            return kGeneratedLayouts[caseIndex % kGeneratedLayouts.size()];
        }

        static std::optional<PreparedCase> prepareCase(const Case& fuzzCase)
        {
            if (fuzzCase.layout == Layout::fixture)
            {
                const auto* town = findTown(fuzzCase.focusTown);
                if (town == nullptr)
                {
                    Logging::error("Unable to find focus town '{}'", fuzzCase.focusTown);
                    return std::nullopt;
                }

                const FocusArea focus{ { town->x, town->y }, kFocusRadius };
                size_t stationCount = 0;
                for (const auto& station : StationManager::stations())
                {
                    stationCount += station.town == town->id() ? 1 : 0;
                }
                auto candidates = getRailVehicles();
                Logging::info("Signal fuzz focus: {} at ({}, {}), {} stations, {} rail vehicles", formatName(town->name), town->x, town->y, stationCount, candidates.size());
                return PreparedCase{ focus, std::move(candidates) };
            }

            auto layout = Layouts::generate(fuzzCase.layout);
            if (!layout.has_value())
            {
                return std::nullopt;
            }
            Logging::info("Signal fuzz layout: {} at ({}, {}) with {} rail vehicles", layoutName(fuzzCase.layout), layout->centre.x, layout->centre.y, layout->vehicles.size());
            return PreparedCase{ { layout->centre, layout->radius }, std::move(layout->vehicles) };
        }

        static CaseResult runCase(const Case& fuzzCase, const FocusArea& focus)
        {
            CaseResult result{};
            RunContext context{};
            RunMonitor monitor(context);

            for (uint32_t tick = 0; tick < fuzzCase.ticks; ++tick)
            {
                context.tick = tick;
                if (result.trace.size() == kTraceLength)
                {
                    result.trace.pop_front();
                }
                result.trace.push_back(captureTrace(tick, focus));

                if (!result.reservationConflict.has_value())
                {
                    const auto conflict = findRouteConflict(tick, focus);
                    if (conflict.has_value())
                    {
                        if (conflict->isPathReservationConflict())
                        {
                            result.reservationConflict = conflict;
                        }
                        else if (!result.routeOverlap.has_value() || (!result.routeOverlap->involvesPathReservation() && conflict->involvesPathReservation()))
                        {
                            result.routeOverlap = conflict;
                        }
                    }
                }

                if (fuzzCase.injectBreakdown && !result.breakdownInjected && tick >= fuzzCase.earliestBreakdownTick)
                {
                    auto* target = EntityManager::get<VehicleHead>(fuzzCase.targetVehicle);
                    if (target != nullptr && canInject(*target, focus))
                    {
                        result.breakdownInjected = injectBreakdown(*target);
                        if (result.breakdownInjected)
                        {
                            result.breakdownTick = tick;
                            Logging::info("Case {}: injected breakdown into vehicle {} at tick {}", fuzzCase.caseIndex, enumValue(target->id), tick);
                        }
                    }
                }

                Scenes::GameScene::tick();
                if (context.collision.has_value())
                {
                    if (result.trace.size() == kTraceLength)
                    {
                        result.trace.pop_front();
                    }
                    result.trace.push_back(captureTrace(tick + 1, focus));
                    const auto collisionConflict = findRouteConflict(
                        tick + 1,
                        focus,
                        std::pair{ context.collision->source, context.collision->target });
                    context.collision->pathReservationIncursion = collisionConflict.has_value() && collisionConflict->involvesPathReservation();
                    context.collision->overlappingPathReservations = collisionConflict.has_value() && collisionConflict->isPathReservationConflict();
                    result.collision = context.collision;
                    break;
                }
            }

            return result;
        }

        static std::ofstream openOutputFile(const fs::path& path)
        {
            std::ofstream stream;
            stream.exceptions(std::ios::failbit | std::ios::badbit);
            stream.open(path, std::ios::out | std::ios::trunc);
            stream.imbue(std::locale::classic());
            return stream;
        }

        static void writeTextFile(const fs::path& path, const std::string_view contents)
        {
            auto stream = openOutputFile(path);
            stream << contents;
        }

        static void writeTrace(const fs::path& path, const std::deque<TraceFrame>& trace)
        {
            auto stream = openOutputFile(path);
            stream << "tick,vehicle,ordinal,status,x,y,z,tail_x,tail_y,tail_z,tile_x,tile_y,tile_z,speed,target_speed,track_and_direction,subposition,breakdown,reserved_pieces,routing_0,routing_1,routing_2,routing_3,routing_4\n";
            for (const auto& frame : trace)
            {
                for (const auto& vehicle : frame.vehicles)
                {
                    stream << frame.tick << ',' << enumValue(vehicle.id) << ',' << vehicle.ordinalNumber << ',' << static_cast<int32_t>(vehicle.status)
                           << ',' << vehicle.headPos.x << ',' << vehicle.headPos.y << ',' << vehicle.headPos.z
                           << ',' << vehicle.tailPos.x << ',' << vehicle.tailPos.y << ',' << vehicle.tailPos.z
                           << ',' << vehicle.tilePos.x << ',' << vehicle.tilePos.y << ',' << vehicle.tilePos.z
                           << ',' << vehicle.speed << ',' << vehicle.targetSpeed << ',' << vehicle.trackAndDirection << ',' << vehicle.subPosition
                           << ',' << static_cast<int32_t>(vehicle.breakdownFlags) << ',' << static_cast<int32_t>(vehicle.reservedPieces);
                    for (const auto routing : vehicle.routings)
                    {
                        stream << ',' << routing;
                    }
                    stream << '\n';
                }
            }
        }

        static void writeTrackContext(const fs::path& path, const CaseResult& result)
        {
            std::vector<std::pair<std::string_view, World::Pos3>> centres;
            if (result.routeOverlap.has_value())
            {
                centres.emplace_back("route_overlap", result.routeOverlap->pos);
            }
            if (result.reservationConflict.has_value())
            {
                centres.emplace_back("reservation", result.reservationConflict->pos);
            }
            if (result.collision.has_value())
            {
                if (const auto* source = EntityManager::get<VehicleHead>(result.collision->source); source != nullptr)
                {
                    centres.emplace_back("collision", source->position);
                }
            }

            auto stream = openOutputFile(path);
            stream << "context,tile_x,tile_y,base_z,track_id,rotation,sequence,track_object,owner,has_signal,left_signal,right_signal,left_mode,right_mode,left_occupied,right_occupied\n";
            for (const auto& [context, centre] : centres)
            {
                const auto centreTile = World::toTileSpace(centre);
                for (auto y = centreTile.y - 3; y <= centreTile.y + 3; ++y)
                {
                    for (auto x = centreTile.x - 3; x <= centreTile.x + 3; ++x)
                    {
                        for (auto& entry : World::TileManager::get(World::TilePos2{ x, y }))
                        {
                            const auto* track = entry.as<World::TrackElement>();
                            if (track == nullptr)
                            {
                                continue;
                            }
                            const World::SignalElement* signal = nullptr;
                            if (track->hasSignal())
                            {
                                signal = entry.next()->as<World::SignalElement>();
                            }
                            stream << context << ',' << x << ',' << y << ',' << static_cast<int32_t>(track->baseZ())
                                   << ',' << static_cast<int32_t>(track->trackId()) << ',' << static_cast<int32_t>(track->rotation())
                                   << ',' << static_cast<int32_t>(track->sequenceIndex()) << ',' << static_cast<int32_t>(track->trackObjectId())
                                   << ',' << static_cast<int32_t>(enumValue(track->owner())) << ',' << track->hasSignal()
                                   << ',' << (signal != nullptr && signal->getLeft().hasSignal()) << ',' << (signal != nullptr && signal->getRight().hasSignal())
                                   << ',' << static_cast<int32_t>(track->leftSignalMode()) << ',' << static_cast<int32_t>(track->rightSignalMode())
                                   << ',' << (signal != nullptr && signal->getLeft().isOccupied()) << ',' << (signal != nullptr && signal->getRight().isOccupied()) << '\n';
                        }
                    }
                }
            }
        }

        static void writeRouteConflict(const fs::path& path, const RouteConflict& conflict)
        {
            const auto classification = conflict.isPathReservationConflict()
                ? "overlapping_path_reservations"
                : conflict.involvesPathReservation() ? "unprotected_route_overlap_with_path_reservation"
                                                     : "unprotected_route_overlap";
            YAML::Emitter out;
            out << YAML::BeginMap
                << YAML::Key << "classification" << YAML::Value << classification
                << YAML::Key << "tick" << YAML::Value << std::to_string(conflict.tick)
                << YAML::Key << "first_vehicle" << YAML::Value << std::to_string(enumValue(conflict.first))
                << YAML::Key << "second_vehicle" << YAML::Value << std::to_string(enumValue(conflict.second))
                << YAML::Key << "x" << YAML::Value << std::to_string(conflict.pos.x)
                << YAML::Key << "y" << YAML::Value << std::to_string(conflict.pos.y)
                << YAML::Key << "z" << YAML::Value << std::to_string(conflict.pos.z)
                << YAML::Key << "quarters" << YAML::Value << std::to_string(conflict.quarters)
                << YAML::Key << "first_occupied" << YAML::Value << conflict.firstOccupied
                << YAML::Key << "second_occupied" << YAML::Value << conflict.secondOccupied
                << YAML::Key << "first_path_reserved" << YAML::Value << conflict.firstPathReserved
                << YAML::Key << "second_path_reserved" << YAML::Value << conflict.secondPathReserved
                << YAML::EndMap;
            writeTextFile(path, out.c_str());
        }

        static fs::path getCaseDirectory(const fs::path& outputRoot, const uint32_t caseIndex)
        {
            return outputRoot / ("case-" + std::to_string(caseIndex));
        }

        static void clearCaseArtifacts(const fs::path& caseDirectory)
        {
            constexpr std::array filenames{
                "case.yml", "collision.yml", "execution.yml", "failure.SV5", "reservation-conflict.yml", "route-overlap.yml", "trace.csv", "track-context.csv"
            };
            for (const auto* filename : filenames)
            {
                fs::remove(caseDirectory / filename);
            }
        }

        static void writeFailureArtifacts(const fs::path& outputRoot, const Case& fuzzCase, const CaseResult& result)
        {
            const auto caseDirectory = getCaseDirectory(outputRoot, fuzzCase.caseIndex);
            fs::create_directories(caseDirectory);
            writeTextFile(caseDirectory / "case.yml", serialiseCase(fuzzCase));
            writeTrace(caseDirectory / "trace.csv", result.trace);
            writeTrackContext(caseDirectory / "track-context.csv", result);
            S5::exportGameStateToFile(caseDirectory / "failure.SV5", S5::SaveFlags::none);

            YAML::Emitter execution;
            execution << YAML::BeginMap
                      << YAML::Key << "breakdown_injected" << YAML::Value << result.breakdownInjected;
            if (result.breakdownTick.has_value())
            {
                execution << YAML::Key << "breakdown_tick" << YAML::Value << std::to_string(*result.breakdownTick);
            }
            execution << YAML::EndMap;
            writeTextFile(caseDirectory / "execution.yml", execution.c_str());

            if (result.collision.has_value())
            {
                const auto classification = result.collision->overlappingPathReservations
                    ? "overlapping_path_reservations"
                    : result.collision->pathReservationIncursion ? "unprotected_incursion_into_path_reservation"
                                                                 : "unprotected_route_collision";
                YAML::Emitter out;
                out << YAML::BeginMap
                    << YAML::Key << "tick" << YAML::Value << std::to_string(result.collision->tick)
                    << YAML::Key << "source_vehicle" << YAML::Value << std::to_string(enumValue(result.collision->source))
                    << YAML::Key << "target_vehicle" << YAML::Value << std::to_string(enumValue(result.collision->target))
                    << YAML::Key << "classification" << YAML::Value << classification
                    << YAML::EndMap;
                writeTextFile(caseDirectory / "collision.yml", out.c_str());
            }
            if (result.routeOverlap.has_value())
            {
                writeRouteConflict(caseDirectory / "route-overlap.yml", *result.routeOverlap);
            }
            if (result.reservationConflict.has_value())
            {
                writeRouteConflict(caseDirectory / "reservation-conflict.yml", *result.reservationConflict);
            }
            Logging::error("Signal fuzz failure artifacts written to {}", caseDirectory.u8string());
        }

        static Result runCases(const Options& options, const std::optional<Case>& replayCase)
        {
            if (!loadInitialSave(options.baseSave))
            {
                return Result::loadFailure;
            }

            auto& config = Config::get();
            config.autosaveFrequency = 0;
            config.breakdownsDisabled = false;

            const auto outputDirectory = options.outputDirectory.empty() ? fs::temp_directory_path() / "openloco-signal-fuzz" : options.outputDirectory;
            const auto caseCount = replayCase.has_value() ? 1U : options.cases;
            std::set<Layout> exportedLayouts;
            uint32_t injectedCount = 0;
            uint32_t collisionCount = 0;
            uint32_t routeOverlapCount = 0;
            uint32_t reservationConflictCount = 0;
            for (uint32_t index = 0; index < caseCount; ++index)
            {
                if (index != 0 && !reloadSave(options.baseSave))
                {
                    return Result::loadFailure;
                }
                auto fuzzCase = replayCase.has_value() ? *replayCase : makeCase(options, index, std::span<const EntityId>{});
                fuzzCase.layout = resolveLayout(fuzzCase.layout, fuzzCase.caseIndex);
                auto prepared = prepareCase(fuzzCase);
                if (!prepared.has_value())
                {
                    return Result::invalidInput;
                }
                if (prepared->candidates.empty())
                {
                    Logging::error("No running rail vehicles found for signal fuzz layout '{}'", layoutName(fuzzCase.layout));
                    return Result::invalidInput;
                }
                if (!replayCase.has_value())
                {
                    fuzzCase = makeCase(options, index, prepared->candidates);
                }

                if (fuzzCase.layout != Layout::fixture && exportedLayouts.insert(fuzzCase.layout).second)
                {
                    const auto layoutDirectory = outputDirectory / "layouts";
                    fs::create_directories(layoutDirectory);
                    const auto layoutPath = layoutDirectory / (std::string(layoutName(fuzzCase.layout)) + ".SV5");
                    S5::exportGameStateToFile(layoutPath, S5::SaveFlags::none);
                    Logging::info("Generated signal fuzz layout written to {}", layoutPath.u8string());
                }

                clearCaseArtifacts(getCaseDirectory(outputDirectory, fuzzCase.caseIndex));
                Logging::info("Signal fuzz case {}/{} (layout {}, seed {}, target {}, earliest tick {})", index + 1, caseCount, layoutName(fuzzCase.layout), fuzzCase.seed, enumValue(fuzzCase.targetVehicle), fuzzCase.earliestBreakdownTick);
                auto result = runCase(fuzzCase, prepared->focus);
                injectedCount += result.breakdownInjected ? 1 : 0;
                auto writeArtifacts = false;
                if (result.routeOverlap.has_value())
                {
                    Logging::warn("Unprotected route overlap in case {} at tick {} between vehicles {} and {}", fuzzCase.caseIndex, result.routeOverlap->tick, enumValue(result.routeOverlap->first), enumValue(result.routeOverlap->second));
                    routeOverlapCount++;
                }
                if (result.collision.has_value())
                {
                    Logging::error("Collision in case {} at tick {} between vehicles {} and {}", fuzzCase.caseIndex, result.collision->tick, enumValue(result.collision->source), enumValue(result.collision->target));
                    collisionCount++;
                    writeArtifacts = true;
                }
                if (result.reservationConflict.has_value())
                {
                    Logging::error("Reservation conflict in case {} at tick {} between vehicles {} and {}", fuzzCase.caseIndex, result.reservationConflict->tick, enumValue(result.reservationConflict->first), enumValue(result.reservationConflict->second));
                    reservationConflictCount++;
                    writeArtifacts = true;
                }
                if (writeArtifacts)
                {
                    writeFailureArtifacts(outputDirectory, fuzzCase, result);
                }
            }

            Logging::info("Signal fuzz completed {} cases with {} injected breakdowns, {} unprotected route overlaps, {} reservation conflicts, and {} collisions", caseCount, injectedCount, routeOverlapCount, reservationConflictCount, collisionCount);
            if (collisionCount != 0)
            {
                return Result::collision;
            }
            return reservationConflictCount == 0 ? Result::completed : Result::reservationConflict;
        }
    }

    std::string_view layoutName(const Layout layout)
    {
        switch (layout)
        {
            case Layout::fixture:
                return "fixture";
            case Layout::flatMerge:
                return "flat-merge";
            case Layout::flatFan:
                return "flat-fan";
            case Layout::flatInterchange:
                return "flat-interchange";
            case Layout::flatAll:
                return "flat-all";
        }
        return {};
    }

    std::optional<Layout> parseLayout(const std::string_view value)
    {
        constexpr std::array layouts{ Layout::fixture, Layout::flatMerge, Layout::flatFan, Layout::flatInterchange, Layout::flatAll };
        for (const auto layout : layouts)
        {
            if (value == layoutName(layout))
            {
                return layout;
            }
        }
        return std::nullopt;
    }

    Case makeCase(const Options& options, const uint32_t caseIndex, const std::span<const EntityId> candidates)
    {
        Case result{};
        result.baseSave = fs::absolute(options.baseSave);
        result.focusTown = options.focusTown;
        result.layout = resolveLayout(options.layout, caseIndex);
        result.seed = options.seed;
        result.caseIndex = caseIndex;
        result.ticks = options.ticks;
        const auto isBaseline = caseIndex == 0 || (options.layout == Layout::flatAll && caseIndex < kGeneratedLayouts.size());
        if (isBaseline || candidates.empty())
        {
            return result;
        }

        std::mt19937 random(options.seed ^ (caseIndex * 0x9E3779B9U));
        result.injectBreakdown = true;
        result.targetVehicle = candidates[random() % candidates.size()];
        result.earliestBreakdownTick = random() % std::max(1U, options.ticks / 2);
        return result;
    }

    std::string serialiseCase(const Case& fuzzCase)
    {
        YAML::Emitter out;
        out << YAML::BeginMap
            << YAML::Key << "version" << YAML::Value << 2
            << YAML::Key << "base_save" << YAML::Value << fuzzCase.baseSave.u8string()
            << YAML::Key << "focus_town" << YAML::Value << fuzzCase.focusTown
            << YAML::Key << "layout" << YAML::Value << std::string(layoutName(fuzzCase.layout))
            << YAML::Key << "seed" << YAML::Value << std::to_string(fuzzCase.seed)
            << YAML::Key << "case_index" << YAML::Value << std::to_string(fuzzCase.caseIndex)
            << YAML::Key << "ticks" << YAML::Value << std::to_string(fuzzCase.ticks)
            << YAML::Key << "target_vehicle" << YAML::Value << std::to_string(enumValue(fuzzCase.targetVehicle))
            << YAML::Key << "earliest_breakdown_tick" << YAML::Value << std::to_string(fuzzCase.earliestBreakdownTick)
            << YAML::Key << "inject_breakdown" << YAML::Value << fuzzCase.injectBreakdown
            << YAML::EndMap;
        return out.c_str();
    }

    std::optional<Case> deserialiseCase(const std::string_view yaml)
    {
        try
        {
            const auto node = YAML::Load(std::string(yaml));
            const auto version = node["version"].as<uint32_t>(0);
            if (version < 1 || version > 2)
            {
                return std::nullopt;
            }
            Case result{};
            result.baseSave = fs::u8path(node["base_save"].as<std::string>());
            result.focusTown = node["focus_town"].as<std::string>("Beachtown");
            if (version >= 2)
            {
                const auto layout = parseLayout(node["layout"].as<std::string>());
                if (!layout.has_value())
                {
                    return std::nullopt;
                }
                result.layout = *layout;
            }
            result.seed = node["seed"].as<uint32_t>();
            result.caseIndex = node["case_index"].as<uint32_t>();
            result.ticks = node["ticks"].as<uint32_t>();
            result.targetVehicle = EntityId(node["target_vehicle"].as<uint16_t>());
            result.earliestBreakdownTick = node["earliest_breakdown_tick"].as<uint32_t>();
            result.injectBreakdown = node["inject_breakdown"].as<bool>();
            return result;
        }
        catch (const std::exception&)
        {
            return std::nullopt;
        }
    }

    Result run(const Options& options)
    {
        if (options.baseSave.empty() || (options.layout == Layout::fixture && options.focusTown.empty()) || options.cases == 0 || options.ticks == 0)
        {
            return Result::invalidInput;
        }
        try
        {
            return runCases(options, std::nullopt);
        }
        catch (const std::exception& e)
        {
            Logging::error("Signal fuzz failed: {}", e.what());
            return Result::runtimeFailure;
        }
    }

    Result replay(const fs::path& casePath, const fs::path& outputDirectory)
    {
        try
        {
            std::ifstream stream(casePath, std::ios::in | std::ios::binary);
            if (!stream)
            {
                Logging::error("Unable to read signal fuzz case: {}", casePath.u8string());
                return Result::invalidInput;
            }
            std::stringstream contents;
            contents << stream.rdbuf();
            const auto fuzzCase = deserialiseCase(contents.str());
            if (!fuzzCase.has_value() || fuzzCase->baseSave.empty() || (fuzzCase->layout == Layout::fixture && fuzzCase->focusTown.empty()) || fuzzCase->ticks == 0
                || (fuzzCase->injectBreakdown && fuzzCase->targetVehicle == EntityId::null))
            {
                return Result::invalidInput;
            }
            Options options{};
            options.baseSave = fuzzCase->baseSave;
            options.outputDirectory = outputDirectory;
            options.focusTown = fuzzCase->focusTown;
            options.layout = fuzzCase->layout;
            options.cases = 1;
            options.ticks = fuzzCase->ticks;
            options.seed = fuzzCase->seed;
            return runCases(options, fuzzCase);
        }
        catch (const std::exception& e)
        {
            Logging::error("Signal replay failed: {}", e.what());
            return Result::runtimeFailure;
        }
    }
}
