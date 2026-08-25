// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Date.h"
#include "GameState.h"
#include "Map/SurfaceElement.h"
#include "Map/TileManager.h"
#include "Objects/CargoObject.h"
#include "Objects/IndustryObject.h"
#include "Objects/ObjectManager.h"
#include "S5/Limits.h"
#include "Scenario/ScenarioManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "World/IndustryManager.h"
#include "World/Station.h"
#include "World/StationManager.h"
#include "World/TownManager.h"
#include <OpenLoco/Math/Vector.hpp>
#include <OpenLoco/Math/Bound.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

namespace OpenLoco::CargoDist
{
    namespace
    {
        struct RouteOrder
        {
            uint64_t raw{};
            uint16_t offset{};
            std::optional<World::Pos2> position;
            StationId stop = StationId::null;
            uint8_t unloadCargo = 0xFF;
            uint8_t waitForCargo = 0xFF;
        };

        struct EdgeAccumulator
        {
            uint64_t capacityFrequency{};
            uint64_t weightedTravelTime{};
            uint64_t departureFrequency{};
            uint64_t fleetCapacity{};
        };

        struct MemberEdgeAccumulator
        {
            uint64_t departures{};
            uint64_t capacity{};
            uint64_t travelTime{};
        };

        struct CargoCapacityInput
        {
            uint32_t acceptedTypes{};
            uint8_t type{};
            uint16_t maxQty{};
        };

        struct VehicleServiceInput
        {
            EntityId id{};
            CompanyId owner{};
            VehicleType vehicleType{};
            TransportMode mode{};
            uint8_t trackType{};
            bool active{};
            bool express{};
            int32_t maxSpeed{};
            std::vector<RouteOrder> orders;
            std::vector<CargoCapacityInput> compartments;
        };

        struct VehicleRoute
        {
            EntityId vehicle{};
            std::vector<uint64_t> canonicalOrders;
            std::vector<VehicleServiceLeg> legs;
            std::vector<uint32_t> legTimes;
            std::vector<uint32_t> unloadMasks;
            std::vector<std::array<uint32_t, 32>> legCapacities;
            uint16_t occurrenceCount{};
            uint64_t cycleTime{};
            bool active{};
            bool express{};
        };

        struct ServiceCalculationResult
        {
            std::map<ServiceEdgeKey, ServiceEdgeStats> serviceEdges;
            std::map<EntityId, std::vector<VehicleServiceLeg>> vehicleServiceLegs;
        };

        struct FlowCalculationInput
        {
            RoutingSettings settings;
            std::array<std::optional<RoutingGraph>, 32> graphs;
            std::array<std::vector<RoutingDemand>, 32> holidayDemands;
            uint32_t flowCargoMask{};
        };

        struct FlowCalculationResult
        {
            std::map<FlowKey, std::vector<FlowOption>> flows;
            std::map<DestinationFlowKey, std::vector<DestinationOption>> destinationFlows;
            std::map<FlowKey, std::vector<FlowOption>> holidayFlows;
            std::map<DestinationFlowKey, std::vector<DestinationOption>> holidayDestinationFlows;
            std::map<StationId, uint32_t> stationAccessibility;
            std::vector<uint8_t> computedCargoes;
            uint64_t solveNanoseconds{};
        };

        struct ServiceGroupKey
        {
            CompanyId owner{};
            VehicleType vehicleType{};
            TransportMode mode{};
            uint8_t trackType{};
            bool express{};
            std::vector<uint64_t> orders;

            auto operator<=>(const ServiceGroupKey&) const = default;
        };

        struct JourneyCache
        {
            uint64_t revision = std::numeric_limits<uint64_t>::max();
            std::map<uint8_t, RoutingGraph> graphs;
            std::map<std::tuple<uint8_t, StationId, ServicePoint>, std::vector<StationJourneyCost>> costs;
            std::map<ServiceEdgeKey, uint64_t> committedDemand;
        };

        struct AlternativeBoarding
        {
            StationId destination = StationId::null;
            StationId nextHop = StationId::null;
            ServicePoint departure{};
            ServicePoint arrival{};
            ServiceEdgeKey edge{};
            uint32_t quantity{};
            uint64_t firstEligible{};
            uint64_t saving{};
        };

        JourneyCache _journeyCache;
        std::map<FlowKey, std::vector<FlowOption>> _holidayFlows;
        std::map<DestinationFlowKey, std::vector<DestinationOption>> _holidayDestinationFlows;

        void clearHolidayRouting()
        {
            _holidayFlows.clear();
            _holidayDestinationFlows.clear();
        }

        void clearJourneyCache()
        {
            _journeyCache = {};
        }

        void synchroniseJourneyCacheRevision()
        {
            const auto revision = getStateConst().routingRevision;
            if (_journeyCache.revision != revision)
            {
                _journeyCache = {};
                _journeyCache.revision = revision;
            }
        }

        void markCargoChanged()
        {
            ++getState().cargoRevision;
        }

        bool isPassengerCargo(uint8_t cargo)
        {
            const auto* cargoObject = ObjectManager::get<CargoObject>(cargo);
            return cargoObject != nullptr && cargoObject->cargoCategory == CargoCategory::passengers;
        }

        uint32_t saturatedAdd(uint32_t lhs, uint32_t rhs)
        {
            return rhs > std::numeric_limits<uint32_t>::max() - lhs
                ? std::numeric_limits<uint32_t>::max()
                : lhs + rhs;
        }

        uint64_t saturatedAdd(uint64_t lhs, uint64_t rhs)
        {
            return rhs > std::numeric_limits<uint64_t>::max() - lhs
                ? std::numeric_limits<uint64_t>::max()
                : lhs + rhs;
        }

        uint64_t saturatedMultiply(uint64_t lhs, uint64_t rhs)
        {
            return lhs != 0 && rhs > std::numeric_limits<uint64_t>::max() / lhs
                ? std::numeric_limits<uint64_t>::max()
                : lhs * rhs;
        }

        bool seedStationCargo(StationId station, uint8_t cargo, const StationCargoStats& nativeCargo)
        {
            if (nativeCargo.quantity == 0)
            {
                return false;
            }
            const auto origin = nativeCargo.origin == StationId::null ? station : nativeCargo.origin;
            getOrCreateStationCargo(station, cargo).append({ nativeCargo.quantity, origin, StationId::null, nativeCargo.enrouteAge });
            return true;
        }

        bool seedVehicleCargo(VehicleCargoKey key, const Vehicles::VehicleCargo& nativeCargo, StationId nextHop, ServicePoint departure = {}, ServicePoint arrival = {})
        {
            if (nativeCargo.qty == 0)
            {
                return false;
            }
            getOrCreateVehicleCargo(key).append({ nativeCargo.qty, nativeCargo.townFrom, nextHop, nativeCargo.numDays, departure, arrival });
            return true;
        }

        template<typename TFunc>
        void forEachVehicleCargo(TFunc&& func)
        {
            for (auto* head : VehicleManager::VehicleList())
            {
                Vehicles::Vehicle train(*head);
                for (auto& car : train.cars)
                {
                    for (auto& component : car)
                    {
                        func(*head, VehicleCargoKey{ component.front->id, VehicleCargoSlot::secondary }, component.front->secondaryCargo);
                        func(*head, VehicleCargoKey{ component.back->id, VehicleCargoSlot::secondary }, component.back->secondaryCargo);
                        func(*head, VehicleCargoKey{ component.body->id, VehicleCargoSlot::primary }, component.body->primaryCargo);
                    }
                }
            }
        }

        template<typename TFunc>
        void forEachVehicleCargo(const GameState& gameState, TFunc&& func)
        {
            for (const auto& entity : gameState.entities)
            {
                const auto* vehicle = entity.asBase<Vehicles::VehicleBase>();
                if (vehicle == nullptr)
                {
                    continue;
                }

                switch (vehicle->getSubType())
                {
                    case Vehicles::VehicleEntityType::bogie:
                    {
                        const auto& bogie = *reinterpret_cast<const Vehicles::VehicleBogie*>(&entity);
                        func(VehicleCargoKey{ bogie.id, VehicleCargoSlot::secondary }, bogie.secondaryCargo);
                        break;
                    }
                    case Vehicles::VehicleEntityType::body_start:
                    case Vehicles::VehicleEntityType::body_continued:
                    {
                        const auto& body = *reinterpret_cast<const Vehicles::VehicleBody*>(&entity);
                        func(VehicleCargoKey{ body.id, VehicleCargoSlot::primary }, body.primaryCargo);
                        break;
                    }
                    default:
                        break;
                }
            }
        }

        void synchroniseAllCargo()
        {
            for (auto& station : StationManager::stations())
            {
                if (station.empty())
                {
                    continue;
                }
                for (uint8_t cargo = 0; cargo < std::size(station.cargoStats); ++cargo)
                {
                    if (isEnabled(cargo))
                    {
                        synchroniseStationCargo(station.id(), cargo, station.cargoStats[cargo]);
                    }
                }
            }

            forEachVehicleCargo([](const auto&, VehicleCargoKey key, auto& cargo) {
                if (isEnabled(cargo.type))
                {
                    synchroniseVehicleCargo(key, cargo);
                }
            });
        }

        void addCargoCapacity(std::array<uint32_t, 32>& capacities, const CargoCapacityInput& cargo, uint32_t routeWaitForMask, uint32_t departureWaitForMask)
        {
            if (cargo.maxQty == 0)
            {
                return;
            }
            auto acceptedTypes = cargo.acceptedTypes;
            if (routeWaitForMask != 0)
            {
                const auto waitedTypes = acceptedTypes & routeWaitForMask;
                acceptedTypes = waitedTypes != 0 ? waitedTypes : acceptedTypes & (1U << cargo.type);
            }
            const auto requiredTypes = acceptedTypes & departureWaitForMask;
            acceptedTypes = requiredTypes != 0 ? std::bit_floor(requiredTypes) : acceptedTypes;
            for (uint8_t cargoType = 0; cargoType < capacities.size(); ++cargoType)
            {
                if ((acceptedTypes & (1U << cargoType)) != 0)
                {
                    capacities[cargoType] = saturatedAdd(capacities[cargoType], cargo.maxQty);
                }
            }
        }

        size_t getCanonicalRotation(const std::vector<RouteOrder>& orders)
        {
            size_t best = 0;
            for (size_t candidate = 1; candidate < orders.size(); ++candidate)
            {
                for (size_t i = 0; i < orders.size(); ++i)
                {
                    const auto candidateRaw = orders[(candidate + i) % orders.size()].raw;
                    const auto bestRaw = orders[(best + i) % orders.size()].raw;
                    if (candidateRaw != bestRaw)
                    {
                        if (candidateRaw < bestRaw)
                        {
                            best = candidate;
                        }
                        break;
                    }
                }
            }
            return best;
        }

        size_t getPrimitivePeriod(const std::vector<RouteOrder>& orders)
        {
            for (size_t period = 1; period < orders.size(); ++period)
            {
                if (orders.size() % period != 0)
                {
                    continue;
                }
                if (std::equal(orders.begin() + period, orders.end(), orders.begin(), [](const auto& lhs, const auto& rhs) {
                        return lhs.raw == rhs.raw;
                    }))
                {
                    return period;
                }
            }
            return orders.size();
        }

        uint32_t estimateTravelTime(uint64_t distance, TransportMode mode, int32_t speed)
        {
            if (speed <= 0)
            {
                return 0;
            }
            const uint32_t modeModifier = [mode]() {
                switch (mode)
                {
                    case TransportMode::air:
                        return 36U;
                    case TransportMode::water:
                        return 31U;
                    default:
                        return 21U;
                }
            }();
            const auto maximumNumerator = std::numeric_limits<uint64_t>::max() - speed;
            const auto numerator = distance > maximumNumerator / modeModifier ? maximumNumerator : distance * modeModifier;
            return static_cast<uint32_t>(std::min<uint64_t>(std::max<uint64_t>(1, (numerator + speed - 1) / speed), std::numeric_limits<uint32_t>::max()));
        }

        std::optional<VehicleRoute> getVehicleRoute(const VehicleServiceInput& vehicle)
        {
            if (vehicle.orders.empty() || vehicle.maxSpeed <= 0)
            {
                return std::nullopt;
            }

            std::vector<RouteOrder> orders = vehicle.orders;
            uint32_t routeWaitForMask = 0;
            for (const auto& order : orders)
            {
                if (order.waitForCargo != 0xFF)
                {
                    routeWaitForMask |= 1U << order.waitForCargo;
                }
            }
            if (orders.empty())
            {
                return std::nullopt;
            }

            const auto rotation = getCanonicalRotation(orders);
            std::rotate(orders.begin(), orders.begin() + rotation, orders.end());

            std::vector<size_t> stops;
            for (size_t i = 0; i < orders.size(); ++i)
            {
                if (orders[i].stop != StationId::null)
                {
                    stops.push_back(i);
                }
            }
            if (stops.size() < 2)
            {
                return std::nullopt;
            }

            VehicleRoute route;
            route.vehicle = vehicle.id;
            const auto period = getPrimitivePeriod(orders);
            route.canonicalOrders.reserve(period);
            for (size_t i = 0; i < period; ++i)
            {
                route.canonicalOrders.push_back(orders[i].raw);
                if (orders[i].stop != StationId::null)
                {
                    ++route.occurrenceCount;
                }
            }
            route.active = vehicle.active;
            route.express = vehicle.express;

            for (size_t stopIndex = 0; stopIndex < stops.size(); ++stopIndex)
            {
                const auto fromIndex = stops[stopIndex];
                const auto toIndex = stops[(stopIndex + 1) % stops.size()];

                uint64_t distance = 0;
                auto previous = *orders[fromIndex].position;
                for (auto orderIndex = (fromIndex + 1) % orders.size();; orderIndex = (orderIndex + 1) % orders.size())
                {
                    if (orders[orderIndex].position.has_value())
                    {
                        distance += Math::Vector::distance2D(previous, *orders[orderIndex].position);
                        previous = *orders[orderIndex].position;
                    }
                    if (orderIndex == toIndex)
                    {
                        break;
                    }
                }

                const auto travelTime = estimateTravelTime(distance, vehicle.mode, vehicle.maxSpeed);
                uint32_t unloadMask = 0;
                uint32_t departureWaitForMask = 0;
                for (auto orderIndex = (fromIndex + 1) % orders.size(); orderIndex != toIndex; orderIndex = (orderIndex + 1) % orders.size())
                {
                    if (orders[orderIndex].position.has_value())
                    {
                        break;
                    }
                    if (orders[orderIndex].unloadCargo != 0xFF)
                    {
                        unloadMask |= 1U << orders[orderIndex].unloadCargo;
                    }
                    if (orders[orderIndex].waitForCargo != 0xFF)
                    {
                        departureWaitForMask |= 1U << orders[orderIndex].waitForCargo;
                    }
                }
                std::array<uint32_t, 32> capacities{};
                for (const auto& cargo : vehicle.compartments)
                {
                    addCargoCapacity(capacities, cargo, routeWaitForMask, departureWaitForMask);
                }
                route.legs.push_back({
                    orders[(fromIndex + 1) % orders.size()].offset,
                    orders[fromIndex].stop,
                    orders[toIndex].stop,
                });
                route.legTimes.push_back(travelTime);
                route.unloadMasks.push_back(unloadMask);
                route.legCapacities.push_back(capacities);
                route.cycleTime += travelTime;
            }
            return route;
        }

        const VehicleServiceLeg* findCurrentServiceLeg(const Vehicles::VehicleHead& head, bool allowDirty = false)
        {
            if (getStateConst().servicesDirty && !allowDirty)
            {
                return nullptr;
            }
            const auto found = getStateConst().vehicleServiceLegs.find(head.id);
            if (found == getStateConst().vehicleServiceLegs.end() || found->second.empty())
            {
                return nullptr;
            }

            const auto& legs = found->second;
            const auto atStation = head.stationId != StationId::null
                && (head.status == Vehicles::Status::unloading || head.status == Vehicles::Status::loading);
            const auto direct = std::find_if(legs.begin(), legs.end(), [&](const auto& leg) {
                return leg.currentOrder == head.currentOrder && (!atStation || leg.from == head.stationId);
            });
            if (direct != legs.end())
            {
                return &*direct;
            }
            const auto anyDirect = std::find_if(legs.begin(), legs.end(), [&](const auto& leg) {
                return leg.currentOrder == head.currentOrder;
            });
            if (anyDirect != legs.end())
            {
                return &*anyDirect;
            }

            if (head.sizeOfOrderTable <= sizeof(Vehicles::OrderEnd))
            {
                return nullptr;
            }
            const auto ringSize = static_cast<uint16_t>(head.sizeOfOrderTable - sizeof(Vehicles::OrderEnd));
            const auto findClosest = [&](bool requireCurrentStation) {
                const VehicleServiceLeg* best = nullptr;
                uint16_t bestDistance = std::numeric_limits<uint16_t>::max();
                for (const auto& leg : legs)
                {
                    if (requireCurrentStation && leg.from != head.stationId)
                    {
                        continue;
                    }
                    const auto distance = static_cast<uint16_t>((head.currentOrder + ringSize - leg.currentOrder) % ringSize);
                    if (distance < bestDistance)
                    {
                        best = &leg;
                        bestDistance = distance;
                    }
                }
                return best;
            };
            const auto* stationLeg = atStation ? findClosest(true) : nullptr;
            return stationLeg != nullptr ? stationLeg : findClosest(false);
        }

        const VehicleServiceLeg* findPriorServiceLeg(const Vehicles::VehicleHead& head, const VehicleServiceLeg& onward)
        {
            const auto found = getStateConst().vehicleServiceLegs.find(head.id);
            if (found == getStateConst().vehicleServiceLegs.end())
            {
                return nullptr;
            }
            const auto prior = std::find_if(found->second.begin(), found->second.end(), [&](const auto& leg) {
                return leg.to == head.stationId && leg.arrival == onward.departure;
            });
            return prior == found->second.end() ? nullptr : &*prior;
        }

        void retagVehiclePackets()
        {
            forEachVehicleCargo([](const auto& head, VehicleCargoKey key, const auto&) {
                auto* vehiclePackets = getVehicleCargo(key);
                if (vehiclePackets == nullptr)
                {
                    return;
                }

                const auto* current = findCurrentServiceLeg(head, true);
                const auto* prior = current != nullptr
                        && head.status == Vehicles::Status::unloading
                        && head.stationId != StationId::null
                        && current->from == head.stationId
                    ? findPriorServiceLeg(head, *current)
                    : nullptr;

                vehiclePackets->transform([&](CargoPacket& packet) {
                    const auto* packetLeg = current != nullptr && packet.nextHop == current->to
                        ? current
                        : (prior != nullptr && packet.nextHop == head.stationId ? prior : nullptr);
                    packet.departure = packetLeg != nullptr ? packetLeg->departure : ServicePoint{};
                    packet.arrival = packetLeg != nullptr ? packetLeg->arrival : ServicePoint{};
                });
            });
        }

        uint32_t getServiceCargoMask()
        {
            uint32_t mask = 0;
            const auto& state = getStateConst();
            for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
            {
                if (state.settings.modes[cargo] != DistributionMode::manual || isPassengerCargo(cargo))
                {
                    mask |= 1U << cargo;
                }
            }
            return mask;
        }

        std::vector<VehicleServiceInput> captureServiceCalculationInput()
        {
            std::vector<VehicleServiceInput> vehicles;
            for (const auto* head : VehicleManager::VehicleList())
            {
                Vehicles::Vehicle train(*head);
                if (train.cars.empty() || train.veh2->maxSpeed.getRaw() <= 0)
                {
                    continue;
                }

                VehicleServiceInput vehicle;
                vehicle.id = head->id;
                vehicle.owner = head->owner;
                vehicle.vehicleType = head->vehicleType;
                vehicle.mode = head->mode;
                vehicle.trackType = head->trackType;
                vehicle.active = head->isPlaced()
                    && !head->hasVehicleFlags(Vehicles::VehicleFlags::commandStop | Vehicles::VehicleFlags::manualControl)
                    && head->status != Vehicles::Status::crashed
                    && head->status != Vehicles::Status::stuck;
                vehicle.express = (train.veh1->var_48 & Vehicles::Flags48::expressMode) != Vehicles::Flags48::none;
                vehicle.maxSpeed = train.veh2->maxSpeed.getRaw();

                for (const auto& order : Vehicles::OrderRingView(head->orderTableOffset))
                {
                    RouteOrder routeOrder{ order.getRaw(), static_cast<uint16_t>(order.getOffset() - head->orderTableOffset), std::nullopt, StationId::null };
                    if (const auto* stop = order.as<Vehicles::OrderStopAt>())
                    {
                        const auto* station = StationManager::get(stop->getStation());
                        if (station == nullptr || station->empty())
                        {
                            continue;
                        }
                        routeOrder.position = World::Pos2{ station->x, station->y };
                        routeOrder.stop = stop->getStation();
                    }
                    else if (const auto* through = order.as<Vehicles::OrderRouteThrough>())
                    {
                        const auto* station = StationManager::get(through->getStation());
                        if (station == nullptr || station->empty())
                        {
                            continue;
                        }
                        routeOrder.position = World::Pos2{ station->x, station->y };
                    }
                    else if (const auto* waypoint = order.as<Vehicles::OrderRouteWaypoint>())
                    {
                        routeOrder.position = World::Pos2{ waypoint->getWaypoint() };
                    }
                    else if (const auto* unload = order.as<Vehicles::OrderUnloadAll>())
                    {
                        routeOrder.unloadCargo = unload->getCargo();
                    }
                    else if (const auto* waitFor = order.as<Vehicles::OrderWaitFor>())
                    {
                        routeOrder.waitForCargo = waitFor->getCargo();
                    }
                    vehicle.orders.push_back(std::move(routeOrder));
                }
                if (vehicle.orders.empty())
                {
                    continue;
                }

                vehicle.compartments.reserve(train.cars.size() * 2);
                for (const auto& car : train.cars)
                {
                    vehicle.compartments.push_back({ car.body->primaryCargo.acceptedTypes, car.body->primaryCargo.type, car.body->primaryCargo.maxQty });
                    vehicle.compartments.push_back({ car.front->secondaryCargo.acceptedTypes, car.front->secondaryCargo.type, car.front->secondaryCargo.maxQty });
                }
                vehicles.push_back(std::move(vehicle));
            }
            return vehicles;
        }

        ServiceCalculationResult calculateServiceEdges(const std::vector<VehicleServiceInput>& vehicles)
        {
            ServiceCalculationResult result;
            std::map<ServiceGroupKey, std::vector<VehicleRoute>> groups;
            for (const auto& input : vehicles)
            {
                auto route = getVehicleRoute(input);
                if (!route.has_value())
                {
                    continue;
                }
                ServiceGroupKey key{
                    input.owner,
                    input.vehicleType,
                    input.mode,
                    input.trackType,
                    route->express,
                    std::move(route->canonicalOrders),
                };
                groups[std::move(key)].push_back(std::move(*route));
            }

            const auto serviceCargoMask = getServiceCargoMask();
            std::map<ServiceEdgeKey, EdgeAccumulator> accumulators;
            // Q32.32 leaves enough headroom to accumulate capacity-weighted frequencies.
            constexpr uint64_t kFrequencyOne = uint64_t{ 1 } << 32;
            for (auto& group : groups)
            {
                auto& members = group.second;
                auto serviceValue = EntityId::null;
                for (const auto& member : members)
                {
                    serviceValue = std::min(serviceValue, member.vehicle);
                }
                const auto service = static_cast<ServiceId>(static_cast<uint16_t>(serviceValue));
                for (auto& member : members)
                {
                    for (size_t i = 0; i < member.legs.size(); ++i)
                    {
                        const auto occurrence = static_cast<uint16_t>(i % member.occurrenceCount);
                        member.legs[i].departure = { service, occurrence };
                        member.legs[i].arrival = { service, static_cast<uint16_t>((occurrence + 1) % member.occurrenceCount) };
                    }
                    result.vehicleServiceLegs[member.vehicle] = member.legs;
                    if (!member.active)
                    {
                        continue;
                    }

                    for (uint8_t cargo = 0; cargo < member.legCapacities.front().size(); ++cargo)
                    {
                        if ((serviceCargoMask & (1U << cargo)) == 0)
                        {
                            continue;
                        }
                        std::map<ServiceEdgeKey, MemberEdgeAccumulator> memberEdges;
                        for (size_t i = 0; i < member.legs.size(); ++i)
                        {
                            if ((member.unloadMasks[i] & (1U << cargo)) != 0)
                            {
                                continue;
                            }
                            const auto capacity = member.legCapacities[i][cargo];
                            if (capacity == 0)
                            {
                                continue;
                            }
                            const auto& leg = member.legs[i];
                            const ServiceEdgeKey key{ cargo, leg.from, leg.to, leg.departure, leg.arrival };
                            auto& edge = memberEdges[key];
                            ++edge.departures;
                            edge.capacity += capacity;
                            edge.travelTime += member.legTimes[i];
                        }
                        for (const auto& [key, memberEdge] : memberEdges)
                        {
                            const auto frequencyNumerator = kFrequencyOne * memberEdge.departures;
                            const auto frequency = std::max<uint64_t>(1, frequencyNumerator / member.cycleTime + (frequencyNumerator % member.cycleTime != 0));
                            const auto capacity = memberEdge.capacity / memberEdge.departures
                                + (memberEdge.capacity % memberEdge.departures * 2 >= memberEdge.departures);
                            const auto travelTime = memberEdge.travelTime / memberEdge.departures
                                + (memberEdge.travelTime % memberEdge.departures * 2 >= memberEdge.departures);
                            auto& edge = accumulators[key];
                            edge.departureFrequency = saturatedAdd(edge.departureFrequency, frequency);
                            edge.capacityFrequency = saturatedAdd(edge.capacityFrequency, saturatedMultiply(capacity, frequency));
                            edge.weightedTravelTime = saturatedAdd(edge.weightedTravelTime, saturatedMultiply(travelTime, frequency));
                            edge.fleetCapacity = saturatedAdd(edge.fleetCapacity, capacity);
                        }
                    }
                }
            }

            for (const auto& [key, edge] : accumulators)
            {
                const auto averageCapacity = edge.capacityFrequency / edge.departureFrequency
                    + (edge.capacityFrequency % edge.departureFrequency >= edge.departureFrequency / 2 + edge.departureFrequency % 2);
                const auto capacity = std::clamp<uint64_t>(averageCapacity, 1, std::numeric_limits<uint32_t>::max());
                const auto headway = std::max<uint64_t>(1, kFrequencyOne / edge.departureFrequency + (kFrequencyOne % edge.departureFrequency != 0));
                const auto waitTime = headway / 2 + (headway % 2 != 0);
                result.serviceEdges[key] = {
                    static_cast<uint32_t>(capacity),
                    static_cast<uint32_t>(std::min<uint64_t>(edge.weightedTravelTime / edge.departureFrequency + (edge.weightedTravelTime % edge.departureFrequency >= edge.departureFrequency / 2 + edge.departureFrequency % 2), std::numeric_limits<uint32_t>::max())),
                    static_cast<uint32_t>(std::min<uint64_t>(waitTime, std::numeric_limits<uint32_t>::max())),
                    static_cast<uint32_t>(std::min<uint64_t>(headway, std::numeric_limits<uint32_t>::max())),
                    static_cast<uint32_t>(std::min<uint64_t>(edge.fleetCapacity, std::numeric_limits<uint32_t>::max())),
                };
            }
            return result;
        }

        void rebuildServiceEdges()
        {
            const auto serviceEdges = calculateServiceEdges(captureServiceCalculationInput());
            getState().serviceEdges = serviceEdges.serviceEdges;
            getState().vehicleServiceLegs = serviceEdges.vehicleServiceLegs;
            getState().servicesDirty = false;
            retagVehiclePackets();
        }

        bool hasValidServicePlans()
        {
            const auto& state = getStateConst();
            std::set<std::tuple<uint8_t, StationId, ServicePoint>> arrivals;
            for (const auto& [key, edge] : state.serviceEdges)
            {
                arrivals.emplace(key.cargo, key.to, key.arrival);
            }
            for (const auto& [key, packets] : state.stationCargo)
            {
                for (const auto& packet : packets.packets())
                {
                    if (packet.nextHop != StationId::null
                        && !state.serviceEdges.contains({ key.cargo, key.station, packet.nextHop, packet.departure, packet.arrival }))
                    {
                        return false;
                    }
                }
            }
            for (const auto& [key, options] : state.flows)
            {
                if (!key.incoming.empty() && !arrivals.contains({ key.cargo, key.station, key.incoming }))
                {
                    return false;
                }
                for (const auto& option : options)
                {
                    if (option.via != key.station
                        && !state.serviceEdges.contains({ key.cargo, key.station, option.via, option.departure, option.arrival }))
                    {
                        return false;
                    }
                }
            }
            for (const auto& [key, options] : state.destinationFlows)
            {
                std::vector<std::pair<const DestinationOption*, uint64_t>> expectedWeights;
                uint64_t expectedTotal = 0;
                for (const auto& option : options)
                {
                    const auto flow = state.flows.find({ key.cargo, key.station, key.origin, key.incoming, option.destination });
                    if (flow == state.flows.end())
                    {
                        return false;
                    }
                    uint64_t expectedWeight = 0;
                    for (const auto& route : flow->second)
                    {
                        expectedWeight += route.weight;
                    }
                    expectedWeights.emplace_back(&option, expectedWeight);
                    expectedTotal += expectedWeight;
                }
                const auto maximum = std::numeric_limits<uint32_t>::max();
                const auto divisor = expectedTotal > maximum ? expectedTotal / maximum + (expectedTotal % maximum != 0) : 1;
                uint64_t representableTotal = 0;
                for (auto& expected : expectedWeights)
                {
                    auto& expectedWeight = expected.second;
                    expectedWeight = std::max<uint64_t>(1, expectedWeight / divisor);
                    representableTotal += expectedWeight;
                }
                for (auto& [option, expectedWeight] : expectedWeights)
                {
                    if (representableTotal > maximum)
                    {
                        const auto reduction = std::min<uint64_t>(expectedWeight - 1, representableTotal - maximum);
                        expectedWeight -= reduction;
                        representableTotal -= reduction;
                    }
                    if (option->weight != expectedWeight)
                    {
                        return false;
                    }
                }
            }
            for (const auto& [key, options] : state.flows)
            {
                const auto destinations = state.destinationFlows.find({ key.cargo, key.station, key.origin, key.incoming });
                if (destinations == state.destinationFlows.end()
                    || std::none_of(destinations->second.begin(), destinations->second.end(), [&](const auto& option) { return option.destination == key.destination; }))
                {
                    return false;
                }
            }
            return true;
        }

        StationId findResortStation(const PendingHolidayReturn& pending)
        {
            const auto* preferred = StationManager::get(pending.resortStation);
            if (preferred != nullptr && !preferred->empty())
            {
                return pending.resortStation;
            }
            for (const auto& station : StationManager::stations())
            {
                if (!station.empty() && station.cargoStats[pending.cargo].industryId == pending.resort)
                {
                    return station.id();
                }
            }
            return StationId::null;
        }

        StationId findHomeStation(const PendingHolidayReturn& pending)
        {
            const auto* preferred = StationManager::get(pending.homeStation);
            if (preferred != nullptr && !preferred->empty() && preferred->town == pending.homeTown && preferred->cargoStats[pending.cargo].isAccepted())
            {
                return pending.homeStation;
            }
            for (const auto& station : StationManager::stations())
            {
                if (!station.empty() && station.town == pending.homeTown && station.cargoStats[pending.cargo].isAccepted())
                {
                    return station.id();
                }
            }
            return StationId::null;
        }

        std::map<std::tuple<StationId, StationId, ServicePoint, StationId>, uint32_t> getRoutingDemands(uint8_t cargo)
        {
            const auto& state = getStateConst();
            using DemandKey = std::tuple<StationId, StationId, ServicePoint, StationId>;
            std::map<DemandKey, uint32_t> demands;
            std::map<DemandKey, uint32_t> outstanding;
            const auto addPackets = [&outstanding, releaseDestinations = isPassengerCargo(cargo)](StationId source, const PacketList* packets) {
                if (packets == nullptr)
                {
                    return;
                }
                for (const auto& packet : packets->packets())
                {
                    if (packet.tripKind == PassengerTripKind::ordinary && packet.origin != StationId::null)
                    {
                        const auto destination = releaseDestinations && packet.origin == source ? StationId::null : packet.destination;
                        auto& amount = outstanding[{ source, packet.origin, {}, destination }];
                        amount = saturatedAdd(amount, packet.quantity);
                    }
                }
            };
            const auto addVehiclePackets = [&outstanding](const PacketList* packets) {
                if (packets == nullptr)
                {
                    return;
                }
                for (const auto& packet : packets->packets())
                {
                    if (packet.tripKind == PassengerTripKind::ordinary && packet.origin != StationId::null && packet.nextHop != StationId::null)
                    {
                        auto& amount = outstanding[{ packet.nextHop, packet.origin, packet.arrival, packet.destination }];
                        amount = saturatedAdd(amount, packet.quantity);
                    }
                }
            };
            for (const auto& [key, packets] : state.stationCargo)
            {
                if (key.cargo == cargo)
                {
                    addPackets(key.station, &packets);
                }
            }
            forEachVehicleCargo([&](const auto&, VehicleCargoKey key, const auto& nativeCargo) {
                if (nativeCargo.type == cargo)
                {
                    addVehiclePackets(getVehicleCargoConst(key));
                }
            });
            for (const auto& [key, amount] : outstanding)
            {
                demands[key] = amount;
            }
            for (const auto& [key, amount] : state.supply)
            {
                if (key.first != cargo)
                {
                    continue;
                }
                uint64_t represented = 0;
                for (const auto& [outstandingKey, outstandingAmount] : outstanding)
                {
                    const auto& [source, origin, incoming, destination] = outstandingKey;
                    if (source == key.second && origin == key.second && incoming.empty())
                    {
                        represented += outstandingAmount;
                    }
                }
                if (represented < amount)
                {
                    auto& unassigned = demands[{ key.second, key.second, {}, StationId::null }];
                    unassigned = saturatedAdd(unassigned, amount - static_cast<uint32_t>(represented));
                }
            }
            return demands;
        }

        std::vector<RoutingDemand> getHolidayRoutingDemands(const uint8_t cargo)
        {
            using DemandKey = std::tuple<StationId, StationId, ServicePoint, StationId>;
            std::map<DemandKey, uint32_t> amounts;
            const auto addPacket = [&amounts, cargo](const StationId source, const ServicePoint incoming, const CargoPacket& packet) {
                if (packet.tripKind == PassengerTripKind::ordinary || packet.origin == StationId::null || packet.destination == StationId::null)
                {
                    return;
                }
                auto& amount = amounts[{ source, packet.origin, incoming, packet.destination }];
                amount = saturatedAdd(amount, packet.quantity);
                if (packet.tripKind == PassengerTripKind::holidayOutbound)
                {
                    const PendingHolidayReturn futureReturn{ 0, packet.quantity, packet.destination, packet.origin, packet.homeTown, packet.holidayIndustry, cargo };
                    const auto resortStation = findResortStation(futureReturn);
                    const auto homeStation = findHomeStation(futureReturn);
                    if (resortStation != StationId::null && homeStation != StationId::null)
                    {
                        auto& returnAmount = amounts[{ resortStation, resortStation, {}, homeStation }];
                        returnAmount = saturatedAdd(returnAmount, packet.quantity);
                    }
                }
            };
            const auto addPackets = [&addPacket](const StationId source, const ServicePoint incoming, const PacketList* packets) {
                if (packets == nullptr)
                {
                    return;
                }
                for (const auto& packet : packets->packets())
                {
                    addPacket(source, incoming, packet);
                }
            };
            for (const auto& [key, packets] : getStateConst().stationCargo)
            {
                if (key.cargo == cargo)
                {
                    addPackets(key.station, {}, &packets);
                }
            }
            forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, const auto& nativeCargo) {
                if (nativeCargo.type != cargo)
                {
                    return;
                }
                const auto* packets = getVehicleCargoConst(key);
                if (packets == nullptr)
                {
                    return;
                }
                for (const auto& packet : packets->packets())
                {
                    if (packet.nextHop != StationId::null)
                    {
                        addPacket(packet.nextHop, packet.arrival, packet);
                    }
                }
            });
            for (const auto& pending : getStateConst().pendingHolidayReturns)
            {
                if (pending.cargo == cargo)
                {
                    const auto resortStation = findResortStation(pending);
                    const auto homeStation = findHomeStation(pending);
                    if (resortStation != StationId::null && homeStation != StationId::null)
                    {
                        auto& amount = amounts[{ resortStation, resortStation, {}, homeStation }];
                        amount = saturatedAdd(amount, pending.quantity);
                    }
                }
            }

            std::vector<RoutingDemand> demands;
            demands.reserve(amounts.size());
            for (const auto& [key, amount] : amounts)
            {
                const auto& [source, origin, incoming, destination] = key;
                demands.push_back({ source, origin, amount, incoming, destination });
            }
            return demands;
        }

        struct PassengerIndustryActivity
        {
            uint32_t previousMonthVisitors{};
        };

        std::optional<PassengerIndustryActivity> getPassengerIndustryActivity(const IndustryId industryId, const uint8_t cargo)
        {
            const auto* industry = IndustryManager::get(industryId);
            if (industry == nullptr || industry->empty())
            {
                return std::nullopt;
            }
            const auto* object = industry->getObject();
            if (object == nullptr || std::find(std::begin(object->producedCargoType), std::end(object->producedCargoType), cargo) == std::end(object->producedCargoType))
            {
                return std::nullopt;
            }

            PassengerIndustryActivity activity{};
            auto requiresPassengers = false;
            for (uint8_t i = 0; i < 3; ++i)
            {
                if (object->requiredCargoType[i] == cargo)
                {
                    requiresPassengers = true;
                    activity.previousMonthVisitors = std::max<uint32_t>(activity.previousMonthVisitors, industry->receivedCargoQuantityPreviousMonth[i]);
                }
            }
            return requiresPassengers ? std::optional<PassengerIndustryActivity>{ activity } : std::nullopt;
        }

        bool isHolidayResortObject(const IndustryObject& object, const uint8_t cargo)
        {
            return isPassengerCargo(cargo)
                && object.hasFlags(IndustryObjectFlags::farmTilesDrawAboveSnow)
                && object.hasFlags(IndustryObjectFlags::farmTilesPartialCoverage)
                && std::find(std::begin(object.requiredCargoType), std::end(object.requiredCargoType), cargo) != std::end(object.requiredCargoType)
                && std::find(std::begin(object.producedCargoType), std::end(object.producedCargoType), cargo) != std::end(object.producedCargoType);
        }

        uint32_t getHolidayGuests(const State& state, const IndustryId industry)
        {
            uint32_t guests = 0;
            for (const auto& pending : state.pendingHolidayReturns)
            {
                if (!pending.released && pending.resort == industry)
                {
                    guests = saturatedAdd(guests, pending.quantity);
                }
            }
            const auto addPackets = [&](const PacketList& packets) {
                for (const auto& packet : packets.packets())
                {
                    if (packet.tripKind == PassengerTripKind::holidayOutbound && packet.holidayIndustry == industry)
                    {
                        guests = saturatedAdd(guests, packet.quantity);
                    }
                }
            };
            for (const auto& [key, packets] : state.stationCargo)
            {
                addPackets(packets);
            }
            for (const auto& [key, packets] : state.vehicleCargo)
            {
                addPackets(packets);
            }
            return guests;
        }

        uint32_t getHolidayDemand(const State& state, const Station& station, const uint8_t cargo)
        {
            const auto supply = state.supply.find({ cargo, station.id() });
            const auto* town = TownManager::get(station.town);
            if (supply == state.supply.end() || supply->second == 0 || !state.holidaySources.contains({ station.id(), cargo }) || town == nullptr || town->empty())
            {
                return 0;
            }
            return std::max<uint32_t>(1, static_cast<uint64_t>(supply->second) * getHolidayPercentage(town->size) / 100);
        }

        RoutingGraph buildGraph(uint8_t cargo, bool includeDemands = true)
        {
            struct PassengerIndustryGroup
            {
                uint32_t bonus{};
                uint32_t holidayCapacity{};
                uint8_t holidayPopularity{};
                bool hasOutboundSupply{};
                bool holidayResort{};
                std::vector<size_t> nodes;
            };

            RoutingGraph graph;
            graph.timeSensitive = true;
            graph.passengerRouting = isPassengerCargo(cargo);
            std::array<PassengerIndustryGroup, Limits::kMaxIndustries> passengerIndustries{};
            const auto& state = getStateConst();
            for (const auto& station : StationManager::stations())
            {
                if (station.empty() || (graph.passengerRouting && (station.stationTileSize == 0 || (station.flags & StationFlags::flag_5) != StationFlags::none)))
                {
                    continue;
                }
                const auto accepts = station.cargoStats[cargo].isAccepted();
                uint32_t attraction = 1;
                if (accepts)
                {
                    const auto found = state.stationAttraction.find({ station.id(), cargo });
                    const auto recordedAttraction = found == state.stationAttraction.end() ? 0 : found->second;
                    attraction = getRoutingAttraction(graph.passengerRouting, station.cargoStats[cargo].industryId != IndustryId::null, recordedAttraction);
                }
                const auto industryId = station.cargoStats[cargo].industryId;
                const auto industryPassengerSink = graph.passengerRouting && industryId != IndustryId::null;
                const auto industryIndex = enumValue(industryId);
                const auto hasValidPassengerIndustry = industryPassengerSink && industryIndex < passengerIndustries.size();
                const auto industryActivity = hasValidPassengerIndustry ? getPassengerIndustryActivity(industryId, cargo) : std::nullopt;
                const auto* industry = hasValidPassengerIndustry ? IndustryManager::get(industryId) : nullptr;
                const auto holidayResort = industry != nullptr && !industry->empty() && isHolidayResortObject(*industry->getObject(), cargo);
                graph.nodes.push_back({
                    station.id(),
                    station.x,
                    station.y,
                    0,
                    accepts,
                    attraction,
                    industryPassengerSink && !hasValidPassengerIndustry,
                    station.town,
                    graph.passengerRouting ? getHolidayDemand(state, station, cargo) : 0,
                    0,
                    0,
                    IndustryId::null,
                });
                if (hasValidPassengerIndustry)
                {
                    auto& group = passengerIndustries[industryIndex];
                    group.nodes.push_back(graph.nodes.size() - 1);
                    group.holidayResort |= holidayResort;
                    if (industryActivity.has_value())
                    {
                        group.bonus = getPassengerIndustryBonus(industryActivity->previousMonthVisitors);
                        const auto supply = state.supply.find({ cargo, station.id() });
                        group.hasOutboundSupply |= supply != state.supply.end() && supply->second != 0;
                    }
                    if (holidayResort && group.nodes.size() == 1)
                    {
                        const auto activity = state.resorts.find(industryId);
                        if (activity != state.resorts.end())
                        {
                            group.holidayPopularity = activity->second.popularity;
                            const auto guests = getHolidayGuests(state, industryId);
                            group.holidayCapacity = activity->second.capacity > guests
                                ? activity->second.capacity - guests
                                : 0;
                        }
                    }
                }
            }
            for (size_t industryIndex = 0; industryIndex < passengerIndustries.size(); ++industryIndex)
            {
                auto& group = passengerIndustries[industryIndex];
                std::sort(group.nodes.begin(), group.nodes.end(), [&graph](const auto lhs, const auto rhs) {
                    return enumValue(graph.nodes[lhs].station) < enumValue(graph.nodes[rhs].station);
                });
                const auto stationCount = static_cast<uint32_t>(group.nodes.size());
                const auto producesPassengers = group.bonus != 0;
                for (uint32_t i = 0; i < stationCount; ++i)
                {
                    auto& node = graph.nodes[group.nodes[i]];
                    if (group.holidayResort)
                    {
                        node.holidayIndustry = IndustryId(static_cast<uint8_t>(industryIndex));
                        node.holidayCapacity = group.holidayCapacity;
                        node.holidayPopularity = group.holidayPopularity;
                    }
                    node.passengerSink = isPassengerIndustrySink(producesPassengers, group.hasOutboundSupply);
                    if (!producesPassengers)
                    {
                        continue;
                    }
                    const auto bonus = getSharedPassengerIndustryBonus(group.bonus, stationCount, i);
                    node.attraction = getPassengerIndustryAttraction(node.attraction, bonus);
                    node.accepts &= node.attraction != 0;
                }
            }
            if (includeDemands)
            {
                for (const auto& [key, amount] : getRoutingDemands(cargo))
                {
                    const auto& [source, origin, incoming, destination] = key;
                    graph.demands.push_back({ source, origin, amount, incoming, destination });
                }
            }
            for (const auto& [key, edge] : state.serviceEdges)
            {
                if (key.cargo == cargo)
                {
                    graph.edges.push_back({ key.from, key.to, edge.capacity, edge.travelTime, key.departure, key.arrival, edge.waitTime, edge.headway });
                }
            }
            return graph;
        }

        struct HolidayFlowCalculation
        {
            std::vector<FlowShare> shares;
            std::map<DestinationFlowKey, std::vector<DestinationOption>> destinations;
        };

        HolidayFlowCalculation calculateHolidayFlows(const uint8_t cargo, const RoutingGraph& graph, const std::vector<RoutingDemand>& existingDemands, const RoutingSettings& settings)
        {
            struct Candidate
            {
                size_t source{};
                size_t destination{};
                IndustryId industry = IndustryId::null;
                uint32_t weight{};
                uint32_t amount{};
                uint64_t remainder{};
            };

            const auto getCost = [](const std::vector<StationJourneyCost>& costs, const StationId station) {
                const auto found = std::find_if(costs.begin(), costs.end(), [station](const auto& item) { return item.destination == station; });
                return found == costs.end() ? kUnreachableJourneyCost : found->cost;
            };

            std::map<IndustryId, uint32_t> capacities;
            for (const auto& node : graph.nodes)
            {
                if (node.holidayIndustry != IndustryId::null)
                {
                    capacities[node.holidayIndustry] = std::max(capacities[node.holidayIndustry], node.holidayCapacity);
                }
            }
            if (capacities.empty() && existingDemands.empty())
            {
                return {};
            }

            std::map<StationId, std::vector<StationJourneyCost>> reverseCosts;
            std::vector<Candidate> candidates;
            for (size_t source = 0; source < graph.nodes.size(); ++source)
            {
                if (graph.nodes[source].holidayDemand == 0)
                {
                    continue;
                }
                const auto outwardCosts = calculateJourneyCosts(graph, graph.nodes[source].station);
                std::map<IndustryId, Candidate> bestByIndustry;
                for (size_t destination = 0; destination < graph.nodes.size(); ++destination)
                {
                    const auto& destinationNode = graph.nodes[destination];
                    if (destinationNode.holidayIndustry == IndustryId::null || destinationNode.holidayCapacity == 0 || !destinationNode.accepts || source == destination)
                    {
                        continue;
                    }
                    const auto outward = getCost(outwardCosts, destinationNode.station);
                    if (outward == kUnreachableJourneyCost)
                    {
                        continue;
                    }
                    auto [reverse, inserted] = reverseCosts.try_emplace(destinationNode.station);
                    if (inserted)
                    {
                        reverse->second = calculateJourneyCosts(graph, destinationNode.station);
                    }
                    const auto returnCost = getCost(reverse->second, graph.nodes[source].station);
                    if (returnCost == kUnreachableJourneyCost)
                    {
                        continue;
                    }
                    const auto averageCost = outward / 2 + returnCost / 2 + (outward % 2 + returnCost % 2) / 2;
                    const auto scaledWeight = static_cast<uint64_t>(std::max<uint8_t>(1, destinationNode.holidayPopularity)) * 65536;
                    const auto weight = static_cast<uint32_t>(std::clamp<uint64_t>(scaledWeight / std::max<uint64_t>(1, averageCost), 1, std::numeric_limits<uint32_t>::max()));
                    Candidate candidate{ source, destination, destinationNode.holidayIndustry, weight };
                    const auto found = bestByIndustry.find(candidate.industry);
                    if (found == bestByIndustry.end() || std::tie(candidate.weight, destinationNode.station) > std::tie(found->second.weight, graph.nodes[found->second.destination].station))
                    {
                        bestByIndustry[candidate.industry] = candidate;
                    }
                }

                uint64_t totalWeight = 0;
                for (const auto& [industry, candidate] : bestByIndustry)
                {
                    totalWeight += candidate.weight;
                }
                if (totalWeight == 0)
                {
                    continue;
                }
                const auto first = candidates.size();
                uint32_t allocated = 0;
                for (auto& [industry, candidate] : bestByIndustry)
                {
                    const auto scaled = static_cast<uint64_t>(graph.nodes[source].holidayDemand) * candidate.weight;
                    candidate.amount = static_cast<uint32_t>(scaled / totalWeight);
                    candidate.remainder = scaled % totalWeight;
                    allocated += candidate.amount;
                    candidates.push_back(candidate);
                }
                auto remaining = graph.nodes[source].holidayDemand - allocated;
                std::vector<size_t> order(candidates.size() - first);
                std::iota(order.begin(), order.end(), first);
                std::sort(order.begin(), order.end(), [&](const auto lhs, const auto rhs) {
                    return std::tie(candidates[lhs].remainder, candidates[lhs].industry) > std::tie(candidates[rhs].remainder, candidates[rhs].industry);
                });
                for (size_t i = 0; remaining != 0 && !order.empty(); ++i, --remaining)
                {
                    ++candidates[order[i % order.size()]].amount;
                }
            }

            for (const auto& [industry, capacity] : capacities)
            {
                uint64_t requested = 0;
                for (const auto& candidate : candidates)
                {
                    if (candidate.industry == industry)
                    {
                        requested += candidate.amount;
                    }
                }
                if (requested <= capacity || requested == 0)
                {
                    continue;
                }
                uint32_t allocated = 0;
                std::vector<size_t> order;
                for (size_t i = 0; i < candidates.size(); ++i)
                {
                    auto& candidate = candidates[i];
                    if (candidate.industry != industry)
                    {
                        continue;
                    }
                    const auto scaled = static_cast<uint64_t>(candidate.amount) * capacity;
                    candidate.amount = static_cast<uint32_t>(scaled / requested);
                    candidate.remainder = scaled % requested;
                    allocated += candidate.amount;
                    order.push_back(i);
                }
                std::sort(order.begin(), order.end(), [&](const auto lhs, const auto rhs) {
                    return std::tie(candidates[lhs].remainder, candidates[lhs].source) > std::tie(candidates[rhs].remainder, candidates[rhs].source);
                });
                auto remaining = capacity - allocated;
                for (size_t i = 0; remaining != 0 && !order.empty(); ++i, --remaining)
                {
                    ++candidates[order[i % order.size()]].amount;
                }
            }

            auto holidayGraph = graph;
            holidayGraph.demands = existingDemands;
            for (auto& node : holidayGraph.nodes)
            {
                node.supply = 0;
                node.accepts = true;
                node.passengerSink = false;
            }
            HolidayFlowCalculation result;
            for (const auto& candidate : candidates)
            {
                if (candidate.amount == 0)
                {
                    continue;
                }
                const auto source = graph.nodes[candidate.source].station;
                const auto destination = graph.nodes[candidate.destination].station;
                result.destinations[{ cargo, source, source, {} }].push_back({ destination, candidate.amount, 0 });
                holidayGraph.demands.push_back({ source, source, candidate.amount, {}, destination });
                holidayGraph.demands.push_back({ destination, destination, candidate.amount, {}, source });
            }
            for (auto& [key, options] : result.destinations)
            {
                std::sort(options.begin(), options.end(), [](const auto& lhs, const auto& rhs) { return lhs.destination < rhs.destination; });
            }
            result.shares = calculateAsymmetricFlows(holidayGraph, settings);
            return result;
        }

        std::map<ServiceEdgeKey, CommittedServiceDemand> calculateCommittedServiceDemands(uint8_t cargo)
        {
            const auto& state = getStateConst();
            std::map<ServiceEdgeKey, CommittedServiceDemand> result;
            for (const auto& [key, packets] : state.stationCargo)
            {
                if (key.cargo != cargo)
                {
                    continue;
                }
                for (const auto& packet : packets.packets())
                {
                    if (packet.nextHop == StationId::null || packet.nextHop == key.station || packet.departure.empty())
                    {
                        continue;
                    }
                    auto& waiting = result[{ cargo, key.station, packet.nextHop, packet.departure, packet.arrival }].waiting;
                    waiting = saturatedAdd(waiting, static_cast<uint64_t>(packet.quantity));
                }
            }

            std::map<std::pair<FlowKey, bool>, uint32_t> inbound;
            if (!state.vehicleCargo.empty())
            {
                forEachVehicleCargo([&](const auto&, VehicleCargoKey key, const auto& nativeCargo) {
                    if (nativeCargo.type != cargo)
                    {
                        return;
                    }
                    const auto* packets = getVehicleCargoConst(key);
                    if (packets == nullptr)
                    {
                        return;
                    }
                    for (const auto& packet : packets->packets())
                    {
                        if (packet.nextHop == StationId::null)
                        {
                            continue;
                        }
                        const FlowKey flow{ cargo, packet.nextHop, packet.origin, packet.arrival, packet.destination };
                        auto& quantity = inbound[{ flow, packet.tripKind != PassengerTripKind::ordinary }];
                        quantity = saturatedAdd(quantity, packet.quantity);
                    }
                });
            }
            for (const auto& [entry, quantity] : inbound)
            {
                const auto& [key, holiday] = entry;
                auto shares = holiday
                    ? previewFixedVia(_holidayFlows, cargo, key.station, key.origin, key.destination, quantity, key.incoming)
                    : previewVia(cargo, key.station, key.origin, key.destination, quantity, key.incoming);
                if (shares.empty() && !key.incoming.empty())
                {
                    shares = holiday
                        ? previewFixedVia(_holidayFlows, cargo, key.station, key.origin, key.destination, quantity)
                        : previewVia(cargo, key.station, key.origin, key.destination, quantity);
                }
                for (const auto& share : shares)
                {
                    if (share.via == StationId::null || share.via == key.station || share.departure.empty())
                    {
                        continue;
                    }
                    auto& incoming = result[{ cargo, key.station, share.via, share.departure, share.arrival }].incoming;
                    incoming = saturatedAdd(incoming, static_cast<uint64_t>(share.amount));
                }
            }
            return result;
        }

        const RoutingGraph& getJourneyGraph(uint8_t cargo)
        {
            synchroniseJourneyCacheRevision();
            const auto cached = _journeyCache.graphs.find(cargo);
            if (cached != _journeyCache.graphs.end())
            {
                return cached->second;
            }

            std::erase_if(_journeyCache.committedDemand, [cargo](const auto& item) { return item.first.cargo == cargo; });
            for (const auto& [key, demand] : calculateCommittedServiceDemands(cargo))
            {
                _journeyCache.committedDemand.emplace(key, demand.total());
            }

            auto graph = buildGraph(cargo, false);
            for (auto& edge : graph.edges)
            {
                const auto demand = _journeyCache.committedDemand[{ cargo, edge.from, edge.to, edge.departure, edge.arrival }];
                const auto queuedDepartures = demand == 0 ? 0 : (demand - 1) / std::max<uint32_t>(1, edge.capacity);
                const auto waitRoom = static_cast<uint64_t>(std::numeric_limits<uint32_t>::max() - edge.waitTime);
                const auto extraWait = edge.headway != 0 && queuedDepartures > waitRoom / edge.headway
                    ? waitRoom
                    : static_cast<uint64_t>(edge.headway) * queuedDepartures;
                edge.waitTime += static_cast<uint32_t>(extraWait);
                edge.headway = 0;
            }
            return _journeyCache.graphs.emplace(cargo, std::move(graph)).first->second;
        }

        void invalidateJourneyGraph(uint8_t cargo)
        {
            _journeyCache.graphs.erase(cargo);
            std::erase_if(_journeyCache.costs, [cargo](const auto& item) { return std::get<0>(item.first) == cargo; });
            std::erase_if(_journeyCache.committedDemand, [cargo](const auto& item) { return item.first.cargo == cargo; });
        }

        uint64_t getJourneyCost(uint8_t cargo, StationId station, StationId destination, ServicePoint departure = {})
        {
            const auto& graph = getJourneyGraph(cargo);
            const auto key = std::tuple{ cargo, station, departure };
            auto cached = _journeyCache.costs.find(key);
            if (cached == _journeyCache.costs.end())
            {
                cached = _journeyCache.costs.emplace(key, calculateJourneyCosts(graph, station, departure)).first;
            }
            const auto found = std::lower_bound(cached->second.begin(), cached->second.end(), destination, [](const auto& item, StationId value) {
                return item.destination < value;
            });
            return found != cached->second.end() && found->destination == destination
                ? found->cost
                : kUnreachableJourneyCost;
        }

        std::vector<AlternativeBoarding> getAlternativeBoarding(StationId station, uint8_t cargo, const VehicleServiceLeg& serviceLeg, const PacketList& packets)
        {
            const auto& state = getStateConst();
            if (!state.serviceEdges.contains({ cargo, station, serviceLeg.to, serviceLeg.departure, serviceLeg.arrival }))
            {
                return {};
            }

            getJourneyGraph(cargo);
            using PacketKey = std::tuple<StationId, StationId, ServicePoint, ServicePoint>;
            std::map<PacketKey, uint32_t> quantities;
            for (const auto& packet : packets.packets())
            {
                const auto exact = packet.nextHop == serviceLeg.to && packet.departure == serviceLeg.departure;
                if (exact || packet.destination == StationId::null || packet.nextHop == StationId::null || packet.departure.empty())
                {
                    continue;
                }
                auto& quantity = quantities[{ packet.destination, packet.nextHop, packet.departure, packet.arrival }];
                quantity = saturatedAdd(quantity, packet.quantity);
            }

            std::vector<AlternativeBoarding> candidates;
            candidates.reserve(quantities.size());
            for (const auto& [packetKey, quantity] : quantities)
            {
                const auto& [destination, nextHop, departure, arrival] = packetKey;
                const ServiceEdgeKey edgeKey{ cargo, station, nextHop, departure, arrival };
                const auto edge = state.serviceEdges.find(edgeKey);
                const auto demand = _journeyCache.committedDemand.find(edgeKey);
                if (edge == state.serviceEdges.end() || demand == _journeyCache.committedDemand.end() || demand->second == 0)
                {
                    continue;
                }

                const auto continuationCost = getJourneyCost(cargo, station, destination, departure);
                const auto presentCost = getJourneyCost(cargo, station, destination, serviceLeg.departure);
                if (continuationCost == kUnreachableJourneyCost || presentCost == kUnreachableJourneyCost)
                {
                    continue;
                }
                const auto baseCost = saturatedAdd(continuationCost, static_cast<uint64_t>(edge->second.waitTime));
                auto firstEligible = std::numeric_limits<uint64_t>::max();
                if (presentCost < baseCost)
                {
                    firstEligible = 0;
                }
                else if (edge->second.headway != 0)
                {
                    const auto queuedDepartures = (presentCost - baseCost) / edge->second.headway + 1;
                    firstEligible = saturatedMultiply(queuedDepartures, std::max<uint32_t>(1, edge->second.capacity));
                }
                const auto lastQueuedDeparture = (demand->second - 1) / std::max<uint32_t>(1, edge->second.capacity);
                const auto marginalCost = saturatedAdd(baseCost, saturatedMultiply(lastQueuedDeparture, edge->second.headway));
                candidates.push_back({ destination, nextHop, departure, arrival, edgeKey, quantity, firstEligible, marginalCost > presentCost ? marginalCost - presentCost : 0 });
            }

            std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
                if (lhs.saving != rhs.saving)
                {
                    return lhs.saving > rhs.saving;
                }
                return std::tie(lhs.destination, lhs.nextHop, lhs.departure, lhs.arrival)
                    < std::tie(rhs.destination, rhs.nextHop, rhs.departure, rhs.arrival);
            });
            std::map<ServiceEdgeKey, uint64_t> remainingDemand;
            for (auto& candidate : candidates)
            {
                auto [remaining, inserted] = remainingDemand.try_emplace(candidate.edge);
                if (inserted)
                {
                    remaining->second = _journeyCache.committedDemand.at(candidate.edge);
                }
                const auto assigned = std::min<uint64_t>(candidate.quantity, remaining->second);
                const auto first = remaining->second - assigned;
                const auto eligible = remaining->second > std::max(first, candidate.firstEligible)
                    ? remaining->second - std::max(first, candidate.firstEligible)
                    : 0;
                candidate.quantity = static_cast<uint32_t>(std::min<uint64_t>(eligible, candidate.quantity));
                remaining->second = first;
            }
            std::erase_if(candidates, [](const auto& candidate) { return candidate.quantity == 0; });
            return candidates;
        }

        std::vector<ViaShare> allocatePacketVia(const uint8_t cargo, const StationId station, const CargoPacket& packet, const ServicePoint incoming = {}, const StationId excluded = StationId::null)
        {
            if (packet.tripKind == PassengerTripKind::ordinary)
            {
                return allocateVia(cargo, station, packet.origin, packet.destination, packet.quantity, incoming, excluded);
            }
            return allocateFixedVia(_holidayFlows, cargo, station, packet.origin, packet.destination, packet.quantity, incoming, excluded);
        }

        void rerouteWaitingCargo(uint8_t cargo)
        {
            auto& state = getState();
            for (auto& [key, packets] : state.stationCargo)
            {
                if (key.cargo != cargo)
                {
                    continue;
                }
                PacketList::Container rerouted;
                rerouted.reserve(packets.size());
                for (auto packet : packets.packets())
                {
                    const auto requested = packet.quantity;
                    const auto releaseDestination = isPassengerCargo(cargo) && packet.origin == key.station && packet.tripKind == PassengerTripKind::ordinary;
                    const auto destination = releaseDestination ? StationId::null : packet.destination;
                    packet.destination = destination;
                    const auto shares = allocatePacketVia(cargo, key.station, packet, ServicePoint{}, key.station);
                    if (shares.empty())
                    {
                        const auto existingRoute = ServiceEdgeKey{ cargo, key.station, packet.nextHop, packet.departure, packet.arrival };
                        if (packet.destination != StationId::null
                            && packet.nextHop != StationId::null
                            && state.serviceEdges.contains(existingRoute)
                            && getJourneyCost(cargo, key.station, packet.destination, packet.departure) != kUnreachableJourneyCost)
                        {
                            rerouted.push_back(packet);
                            continue;
                        }
                        packet.destination = destination;
                        packet.nextHop = StationId::null;
                        packet.departure = {};
                        packet.arrival = {};
                        rerouted.push_back(packet);
                        continue;
                    }
                    uint32_t allocated = 0;
                    auto remainingPacket = packet;
                    for (const auto& share : shares)
                    {
                        allocated += share.amount;
                        auto routedPacket = remainingPacket.extract(static_cast<uint16_t>(share.amount));
                        routedPacket.nextHop = share.via;
                        routedPacket.departure = share.departure;
                        routedPacket.arrival = share.arrival;
                        routedPacket.destination = share.destination;
                        rerouted.push_back(routedPacket);
                    }
                    if (allocated < requested)
                    {
                        remainingPacket.destination = destination;
                        remainingPacket.nextHop = StationId::null;
                        remainingPacket.departure = {};
                        remainingPacket.arrival = {};
                        rerouted.push_back(remainingPacket);
                    }
                }
                packets = PacketList::fromPackets(std::move(rerouted));
                if (auto* station = StationManager::get(key.station); station != nullptr && !station->empty())
                {
                    synchroniseStationCargo(key.station, cargo, station->cargoStats[cargo]);
                }
            }
        }

        StationId findHolidayDestination(const CargoPacket& packet, const uint8_t cargo)
        {
            const auto matches = [&](const Station& station) {
                if (station.empty() || !station.cargoStats[cargo].isAccepted())
                {
                    return false;
                }
                return packet.tripKind == PassengerTripKind::holidayOutbound
                    ? station.cargoStats[cargo].industryId == packet.holidayIndustry
                    : station.town == packet.homeTown;
            };
            const auto* current = StationManager::get(packet.destination);
            if (current != nullptr && matches(*current))
            {
                return packet.destination;
            }
            for (const auto& station : StationManager::stations())
            {
                if (matches(station))
                {
                    return station.id();
                }
            }
            return StationId::null;
        }

        bool repairHolidayDestination(CargoPacket& packet, const uint8_t cargo, const bool clearRoute)
        {
            if (packet.tripKind == PassengerTripKind::ordinary)
            {
                return false;
            }
            const auto destination = findHolidayDestination(packet, cargo);
            if (destination == packet.destination)
            {
                return false;
            }
            packet.destination = destination;
            if (clearRoute)
            {
                packet.nextHop = StationId::null;
                packet.departure = {};
                packet.arrival = {};
            }
            return true;
        }

        void releaseRejectedDestinations()
        {
            bool changed = false;
            const auto release = [&changed](PacketList& packets, uint8_t cargo, bool clearRoute) {
                packets.transform([&changed, cargo, clearRoute](CargoPacket& packet) {
                    if (packet.tripKind != PassengerTripKind::ordinary)
                    {
                        changed |= repairHolidayDestination(packet, cargo, clearRoute);
                        return;
                    }
                    if (packet.destination != StationId::null)
                    {
                        const auto* destination = StationManager::get(packet.destination);
                        if (destination == nullptr || destination->empty() || !destination->cargoStats[cargo].isAccepted())
                        {
                            changed = true;
                            packet.destination = StationId::null;
                            if (clearRoute)
                            {
                                packet.nextHop = StationId::null;
                                packet.departure = {};
                                packet.arrival = {};
                            }
                        }
                    }
                });
            };

            for (auto& [key, packets] : getState().stationCargo)
            {
                if (isEnabled(key.cargo))
                {
                    release(packets, key.cargo, true);
                }
            }
            forEachVehicleCargo([&](const auto&, VehicleCargoKey key, const auto& nativeCargo) {
                if (!isEnabled(nativeCargo.type))
                {
                    return;
                }
                if (auto* packets = getVehicleCargo(key); packets != nullptr)
                {
                    release(*packets, nativeCargo.type, false);
                }
            });
            if (changed)
            {
                ++getState().routingRevision;
            }
        }

        void clearRoutingState(uint8_t cargo)
        {
            auto& state = getState();
            std::erase_if(state.supply, [cargo](const auto& item) { return item.first.first == cargo; });
            std::erase_if(state.serviceEdges, [cargo](const auto& item) { return item.first.cargo == cargo; });
            std::erase_if(state.flows, [cargo](const auto& item) { return item.first.cargo == cargo; });
            std::erase_if(state.destinationFlows, [cargo](const auto& item) { return item.first.cargo == cargo; });
            std::erase_if(_holidayFlows, [cargo](const auto& item) { return item.first.cargo == cargo; });
            std::erase_if(_holidayDestinationFlows, [cargo](const auto& item) { return item.first.cargo == cargo; });
        }

        FlowCalculationInput captureFlowCalculationInput(bool includeFlowGraphs = true)
        {
            FlowCalculationInput input;
            input.settings = getStateConst().settings.routing;
            for (uint8_t cargo = 0; cargo < input.graphs.size(); ++cargo)
            {
                const auto flow = includeFlowGraphs && isEnabled(cargo);
                const auto passenger = isPassengerCargo(cargo);
                if (flow || passenger)
                {
                    input.graphs[cargo] = buildGraph(cargo, flow);
                    if (flow && passenger)
                    {
                        input.holidayDemands[cargo] = getHolidayRoutingDemands(cargo);
                    }
                    input.flowCargoMask |= static_cast<uint32_t>(flow) << cargo;
                }
            }
            return input;
        }

        std::map<StationId, uint32_t> solveStationAccessibility(const FlowCalculationInput& input)
        {
            std::map<StationId, uint32_t> result;
            for (const auto& graph : input.graphs)
            {
                if (!graph.has_value() || !graph->passengerRouting)
                {
                    continue;
                }
                for (const auto& accessibility : calculateStationAccessibility(*graph))
                {
                    auto& score = result[accessibility.station];
                    score = std::max(score, accessibility.score);
                }
            }
            return result;
        }

        FlowCalculationResult solveFlowCalculation(const FlowCalculationInput& input)
        {
            FlowCalculationResult result;
            for (uint8_t cargo = 0; cargo < input.graphs.size(); ++cargo)
            {
                if ((input.flowCargoMask & (1U << cargo)) != 0)
                {
                    buildFlowMaps(result.flows, result.destinationFlows, cargo, calculateAsymmetricFlows(*input.graphs[cargo], input.settings));
                    if (input.graphs[cargo]->passengerRouting)
                    {
                        auto holiday = calculateHolidayFlows(cargo, *input.graphs[cargo], input.holidayDemands[cargo], input.settings);
                        std::map<DestinationFlowKey, std::vector<DestinationOption>> ignoredDestinations;
                        buildFlowMaps(result.holidayFlows, ignoredDestinations, cargo, holiday.shares);
                        result.holidayDestinationFlows.merge(holiday.destinations);
                    }
                    result.computedCargoes.push_back(cargo);
                }
            }
            result.stationAccessibility = solveStationAccessibility(input);
            return result;
        }

        class FlowCalculationWorker
        {
        public:
            FlowCalculationWorker()
                : _thread([this] { run(); })
            {
            }

            ~FlowCalculationWorker()
            {
                {
                    const std::scoped_lock lock(_mutex);
                    _stopping = true;
                }
                _condition.notify_all();
                _thread.join();
            }

            void submit(uint64_t generation, FlowCalculationInput input)
            {
                {
                    const std::scoped_lock lock(_mutex);
                    _request = { generation, std::move(input) };
                    _result.reset();
                }
                _condition.notify_all();
            }

            std::optional<FlowCalculationResult> take(uint64_t generation, bool wait)
            {
                std::unique_lock lock(_mutex);
                if (wait)
                {
                    _condition.wait(lock, [&] {
                        return _stopping || (_result.has_value() && _result->generation == generation);
                    });
                }
                if (!_result.has_value() || _result->generation != generation)
                {
                    return std::nullopt;
                }
                auto result = std::move(_result->flows);
                _result.reset();
                return result;
            }

        private:
            struct Request
            {
                uint64_t generation;
                FlowCalculationInput input;
            };

            struct Result
            {
                uint64_t generation;
                FlowCalculationResult flows;
            };

            void run()
            {
                while (true)
                {
                    Request request{};
                    {
                        std::unique_lock lock(_mutex);
                        _condition.wait(lock, [&] { return _stopping || _request.has_value(); });
                        if (_stopping)
                        {
                            return;
                        }
                        request = std::move(*_request);
                        _request.reset();
                    }

                    const auto solveStart = std::chrono::steady_clock::now();
                    auto result = solveFlowCalculation(request.input);
                    result.solveNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - solveStart).count();
                    {
                        const std::scoped_lock lock(_mutex);
                        if (_stopping)
                        {
                            return;
                        }
                        if (!_request.has_value())
                        {
                            _result = { request.generation, std::move(result) };
                        }
                    }
                    _condition.notify_all();
                }
            }

            std::mutex _mutex;
            std::condition_variable _condition;
            std::optional<Request> _request;
            std::optional<Result> _result;
            bool _stopping{};
            std::thread _thread;
        };

        struct PendingFlowCalculation
        {
            uint64_t generation;
            uint32_t deadline;
            bool scheduled;
            bool blocksTransfers;
            uint64_t dirtyEpoch;
        };

        std::unique_ptr<FlowCalculationWorker> _flowWorker;
        std::optional<PendingFlowCalculation> _pendingFlowCalculation;
        uint64_t _flowCalculationGeneration;
        uint64_t _dirtyEpoch;
        RecalculationMetrics _recalculationMetrics;

        bool isDeadlineReached(uint32_t deadline)
        {
            return static_cast<int32_t>(ScenarioManager::getScenarioTicks() - deadline) >= 0;
        }

        uint32_t nextDayDeadline()
        {
            constexpr uint32_t kDayCounterIncrement = 682;
            const auto ticksUntilNextDay = (std::numeric_limits<uint16_t>::max() - getDayProgression()) / kDayCounterIncrement + 1;
            return ScenarioManager::getScenarioTicks() + ticksUntilNextDay;
        }

        void startFlowCalculation(bool scheduled, bool nextDay)
        {
            const auto captureStart = std::chrono::steady_clock::now();
            auto& state = getState();
            const auto blocksTransfers = state.servicesDirty;
            rebuildServiceEdges();
            releaseRejectedDestinations();
            const auto graphStart = std::chrono::steady_clock::now();
            if (_flowWorker == nullptr)
            {
                _flowWorker = std::make_unique<FlowCalculationWorker>();
            }
            _flowWorker->submit(_flowCalculationGeneration, captureFlowCalculationInput());
            const auto captureEnd = std::chrono::steady_clock::now();
            _recalculationMetrics.preparationNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(graphStart - captureStart).count();
            _recalculationMetrics.graphNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(captureEnd - graphStart).count();
            _pendingFlowCalculation = {
                _flowCalculationGeneration,
                nextDay ? nextDayDeadline() : ScenarioManager::getScenarioTicks() + 48,
                scheduled,
                blocksTransfers,
                _dirtyEpoch,
            };
        }

        void finishScheduledRecalculation()
        {
            auto& state = getState();
            for (auto it = state.supply.begin(); it != state.supply.end();)
            {
                it->second /= 2;
                if (it->second == 0)
                {
                    it = state.supply.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            state.nextRecalculationDay = getCurrentDay() + std::max<uint16_t>(1, state.settings.recalculationInterval);
        }

        void commitFlowCalculation(FlowCalculationResult&& result)
        {
            auto& state = getState();
            state.flows = std::move(result.flows);
            state.destinationFlows = std::move(result.destinationFlows);
            _holidayFlows = std::move(result.holidayFlows);
            _holidayDestinationFlows = std::move(result.holidayDestinationFlows);
            state.stationAccessibility = std::move(result.stationAccessibility);
            state.hasStationAccessibilitySnapshot = true;
            ++state.routingRevision;
            for (const auto cargo : result.computedCargoes)
            {
                rerouteWaitingCargo(cargo);
                invalidateJourneyGraph(cargo);
            }
        }

        void rebuildStationAccessibility()
        {
            const auto input = captureFlowCalculationInput(false);
            getState().stationAccessibility = solveStationAccessibility(input);
            getState().hasStationAccessibilitySnapshot = true;
        }

        void recalculateFlows()
        {
            rebuildServiceEdges();
            releaseRejectedDestinations();
            auto input = captureFlowCalculationInput();
            commitFlowCalculation(solveFlowCalculation(input));
        }

        void recalculateHolidayFlows()
        {
            clearHolidayRouting();
            for (uint8_t cargo = 0; cargo < getStateConst().settings.modes.size(); ++cargo)
            {
                if (!isEnabled(cargo) || !isPassengerCargo(cargo))
                {
                    continue;
                }
                const auto graph = buildGraph(cargo, false);
                auto holiday = calculateHolidayFlows(cargo, graph, getHolidayRoutingDemands(cargo), getStateConst().settings.routing);
                std::map<DestinationFlowKey, std::vector<DestinationOption>> ignoredDestinations;
                buildFlowMaps(_holidayFlows, ignoredDestinations, cargo, holiday.shares);
                _holidayDestinationFlows.merge(holiday.destinations);
            }
        }
    }

    void synchroniseStationCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo)
    {
        const auto* packets = getStationCargoConst(station, cargo);
        nativeCargo.quantity = packets == nullptr
            ? 0
            : static_cast<uint16_t>(std::min<uint32_t>(packets->quantity(), std::numeric_limits<uint16_t>::max()));
        if (nativeCargo.quantity != 0)
        {
            nativeCargo.origin = packets->representativeOrigin();
            nativeCargo.enrouteAge = packets->averageAge();
        }
    }

    void synchroniseVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo)
    {
        const auto* packets = getVehicleCargoConst(key);
        nativeCargo.qty = packets == nullptr
            ? 0
            : static_cast<uint8_t>(std::min<uint32_t>(packets->quantity(), std::numeric_limits<uint8_t>::max()));
        if (nativeCargo.qty != 0)
        {
            nativeCargo.townFrom = packets->representativeOrigin();
            nativeCargo.numDays = packets->averageAge();
        }
    }

    void setStationAttraction(StationId station, uint8_t cargo, uint32_t attraction)
    {
        auto& state = getState();
        const StationCargoKey key{ station, cargo };
        auto found = state.stationAttraction.find(key);
        if (found != state.stationAttraction.end() && found->second == attraction)
        {
            return;
        }
        if (attraction == 0)
        {
            if (found == state.stationAttraction.end())
            {
                return;
            }
            state.stationAttraction.erase(found);
        }
        else
        {
            state.stationAttraction.insert_or_assign(key, attraction);
        }
        if (isEnabled(cargo) || isPassengerCargo(cargo))
        {
            markGraphDirty();
        }
    }

    bool isHolidayResort(const IndustryId industryId, const uint8_t cargo)
    {
        const auto* industry = IndustryManager::get(industryId);
        return industry != nullptr && !industry->empty() && industry->getObject() != nullptr && isHolidayResortObject(*industry->getObject(), cargo);
    }

    static uint16_t countLiveResortSlopes(const Industry& industry)
    {
        const auto* object = industry.getObject();
        if (object == nullptr)
        {
            return 0;
        }
        constexpr auto kRadius = 18;
        const auto centre = World::toTileSpace(World::Pos2{ industry.x, industry.y });
        const auto topLeft = centre - World::TilePos2{ kRadius, kRadius };
        const auto bottomRight = centre + World::TilePos2{ kRadius, kRadius };
        uint16_t count = 0;
        for (const auto& tilePos : World::getClampedRange(topLeft, bottomRight))
        {
            const auto* surface = World::TileManager::get(tilePos).surface();
            if (surface == nullptr || !surface->isIndustrial() || surface->industryId() != industry.id() || surface->snowCoverage() < 4)
            {
                continue;
            }
            const auto stage = surface->getGrowthStage();
            if (stage == 0 || stage != object->farmTileGrowthStageNoProduction)
            {
                count = Math::Bound::add(count, 1);
            }
        }
        return count;
    }

    static void refreshResorts(const bool advanceMonth)
    {
        auto& state = getState();
        std::set<IndustryId> active;
        for (const auto& industry : IndustryManager::industries())
        {
            if (industry.empty() || industry.getObject() == nullptr)
            {
                continue;
            }
            bool activeResort = false;
            for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
            {
                if (getMode(cargo) == DistributionMode::asymmetric && isHolidayResortObject(*industry.getObject(), cargo))
                {
                    activeResort = true;
                    break;
                }
            }
            if (!activeResort)
            {
                continue;
            }
            active.insert(industry.id());
            const auto [found, inserted] = state.resorts.try_emplace(industry.id());
            auto& activity = found->second;
            const auto liveSlopes = countLiveResortSlopes(industry);
            const auto baseCapacity = static_cast<uint32_t>(liveSlopes) * 4;
            const auto occupancyScore = baseCapacity == 0 ? 0 : std::min<uint64_t>(100, static_cast<uint64_t>(activity.guestDays) * 100 / (baseCapacity * 30));
            const auto slopeScore = static_cast<uint8_t>(std::min<uint32_t>(100, static_cast<uint32_t>(liveSlopes) * 4));
            if (inserted)
            {
                activity.popularity = slopeScore / 2;
            }
            else if (advanceMonth)
            {
                activity.popularity = updateResortPopularity(activity.popularity, slopeScore, static_cast<uint8_t>(occupancyScore));
            }
            activity.liveSlopes = liveSlopes;
            activity.capacity = getResortCapacity(liveSlopes, activity.popularity);
            if (advanceMonth)
            {
                activity.guestDays = 0;
            }
        }
        std::erase_if(state.resorts, [&](const auto& item) { return !active.contains(item.first); });
    }

    void updateResortsMonthly()
    {
        refreshResorts(true);
    }

    void scheduleHolidayReturn(const uint8_t cargo, const StationId resortStation, const CargoPacket& packet)
    {
        const auto* station = StationManager::get(resortStation);
        const auto resort = station == nullptr || station->empty() ? IndustryId::null : station->cargoStats[cargo].industryId;
        if (packet.tripKind != PassengerTripKind::holidayOutbound || packet.quantity == 0 || packet.origin == StationId::null || packet.homeTown == TownId::null
            || resort != packet.holidayIndustry || !isHolidayResort(resort, cargo))
        {
            return;
        }
        auto& state = getState();
        const auto firstDay = getCurrentDay() + 7;
        const auto each = packet.quantity / 8;
        const auto remainder = static_cast<uint16_t>(packet.quantity % 8);
        const auto rotation = (enumValue(packet.origin) + enumValue(resortStation) + getCurrentDay()) % 8;
        for (uint8_t i = 0; i < 8; ++i)
        {
            const auto offset = static_cast<uint16_t>((i + 8 - rotation) % 8);
            const auto quantity = static_cast<uint16_t>(each + (offset < remainder));
            if (quantity == 0)
            {
                continue;
            }
            state.pendingHolidayReturns.push_back({ firstDay + i, quantity, resortStation, packet.origin, packet.homeTown, resort, cargo });
        }
        std::sort(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end());
        markCargoChanged();
        markGraphDirty();
    }

    static std::optional<CargoPacket> createHolidayPacket(const StationId source, const uint8_t cargo, HolidaySourceState& sourceState)
    {
        auto& state = getState();
        const auto options = _holidayDestinationFlows.find({ cargo, source, source, {} });
        const auto* sourceStation = StationManager::get(source);
        if (options == _holidayDestinationFlows.end() || sourceStation == nullptr || sourceStation->empty())
        {
            return std::nullopt;
        }

        struct Candidate
        {
            const DestinationOption* option;
            IndustryId industry;
        };
        std::vector<Candidate> candidates;
        uint64_t totalWeight = 0;
        for (const auto& option : options->second)
        {
            const auto* destination = StationManager::get(option.destination);
            if (destination == nullptr || destination->empty())
            {
                continue;
            }
            const auto industry = destination->cargoStats[cargo].industryId;
            const auto activity = state.resorts.find(industry);
            if (!isHolidayResort(industry, cargo) || activity == state.resorts.end() || getHolidayGuests(state, industry) >= activity->second.capacity
                || !_holidayFlows.contains({ cargo, source, source, {}, option.destination }))
            {
                continue;
            }
            candidates.push_back({ &option, industry });
            totalWeight += option.weight;
        }
        if (candidates.empty() || totalWeight == 0)
        {
            return std::nullopt;
        }

        auto ticket = static_cast<uint64_t>(sourceState.sequence++) % totalWeight;
        const Candidate* selected = nullptr;
        for (const auto& candidate : candidates)
        {
            if (ticket < candidate.option->weight)
            {
                selected = &candidate;
                break;
            }
            ticket -= candidate.option->weight;
        }
        if (selected == nullptr)
        {
            return std::nullopt;
        }
        const auto route = allocateFixedVia(_holidayFlows, cargo, source, source, selected->option->destination, 1);
        if (route.empty())
        {
            return std::nullopt;
        }
        const auto& share = route.front();
        CargoPacket packet{ 1, source, share.via, 0, share.departure, share.arrival, share.destination };
        packet.tripKind = PassengerTripKind::holidayOutbound;
        packet.holidayIndustry = selected->industry;
        packet.homeTown = sourceStation->town;
        return packet;
    }

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity, bool generateHolidays)
    {
        if (quantity == 0)
        {
            return;
        }
        auto& packets = getOrCreateStationCargo(station, cargo);
        const auto room = std::numeric_limits<uint32_t>::max() - packets.quantity();
        const auto added = static_cast<uint16_t>(std::min<uint32_t>(quantity, room));
        if (added != 0)
        {
            const auto shares = allocateVia(cargo, station, station, StationId::null, added);
            if (shares.empty())
            {
                packets.append({ added, station, StationId::null, 0 });
            }
            else
            {
                for (const auto& share : shares)
                {
                    packets.append({ static_cast<uint16_t>(share.amount), station, share.via, 0, share.departure, share.arrival, share.destination });
                }
            }
            markCargoChanged();
        }
        if (generateHolidays && isPassengerCargo(cargo) && getMode(cargo) == DistributionMode::asymmetric)
        {
            const auto* sourceStation = StationManager::get(station);
            const auto* town = sourceStation == nullptr ? nullptr : TownManager::get(sourceStation->town);
            if (town != nullptr && !town->empty())
            {
                auto& holidaySource = getState().holidaySources[{ station, cargo }];
                const auto scaled = static_cast<uint32_t>(holidaySource.remainder) + static_cast<uint32_t>(quantity) * getHolidayPercentage(town->size);
                auto holidayQuantity = scaled / 100;
                holidaySource.remainder = static_cast<uint8_t>(scaled % 100);
                while (holidayQuantity-- != 0 && packets.quantity() != std::numeric_limits<uint32_t>::max())
                {
                    const auto packet = createHolidayPacket(station, cargo, holidaySource);
                    if (!packet.has_value())
                    {
                        break;
                    }
                    packets.append(*packet);
                    markCargoChanged();
                }
            }
        }
        auto& supply = getState().supply[{ cargo, station }];
        const auto isNewSource = supply == 0;
        supply = saturatedAdd(supply, quantity);
        if (isNewSource)
        {
            markGraphDirty();
        }
        invalidateJourneyGraph(cargo);
        synchroniseStationCargo(station, cargo, nativeCargo);
    }

    void updateStationCargoDaily(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantityBeforeUpdate)
    {
        auto* packets = getStationCargo(station, cargo);
        if (packets == nullptr)
        {
            if (seedStationCargo(station, cargo, nativeCargo))
            {
                invalidateJourneyGraph(cargo);
                markCargoChanged();
            }
            return;
        }
        packets->ageAtStation(station);
        if (nativeCargo.quantity < quantityBeforeUpdate)
        {
            if (packets->removeForRating(quantityBeforeUpdate - nativeCargo.quantity) != 0)
            {
                invalidateJourneyGraph(cargo);
                markCargoChanged();
            }
        }
        synchroniseStationCargo(station, cargo, nativeCargo);
    }

    void updateVehicleCargoDaily(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo)
    {
        auto* packets = getVehicleCargo(key);
        if (packets == nullptr)
        {
            if (seedVehicleCargo(key, nativeCargo, StationId::null))
            {
                invalidateJourneyGraph(nativeCargo.type);
                markCargoChanged();
            }
            return;
        }
        packets->ageInVehicle();
        synchroniseVehicleCargo(key, nativeCargo);
    }

    uint32_t getLoadableQuantity(StationId station, uint8_t cargo, const VehicleServiceLeg& serviceLeg)
    {
        if (serviceLeg.from != station || serviceLeg.to == StationId::null)
        {
            return 0;
        }
        const auto* packets = getStationCargoConst(station, cargo);
        if (packets == nullptr)
        {
            return 0;
        }
        uint32_t quantity = packets->quantityFor(serviceLeg.to, serviceLeg.departure);
        const auto alternatives = getAlternativeBoarding(station, cargo, serviceLeg, *packets);
        for (const auto& alternative : alternatives)
        {
            quantity = saturatedAdd(quantity, alternative.quantity);
        }
        return quantity;
    }

    std::map<ServiceEdgeKey, CommittedServiceDemand> getCommittedServiceDemands(uint8_t cargo)
    {
        return calculateCommittedServiceDemands(cargo);
    }

    currency32_t accrueTransferCredit(CargoPacket& packet, const currency32_t projectedPayment)
    {
        const auto uncreditedPayment = static_cast<int64_t>(projectedPayment) - packet.transferCredit;
        if (uncreditedPayment <= 0)
        {
            return 0;
        }
        const auto credit = static_cast<currency32_t>(uncreditedPayment);
        packet.transferCredit += credit;
        return credit;
    }

    int64_t calculateFinalDeliveryIncome(const int64_t transferCredit, const currency32_t grossPayment)
    {
        return static_cast<int64_t>(grossPayment) - transferCredit;
    }

    void addVehicleRevenueAdjustment(const EntityId vehicle, const int64_t adjustment)
    {
        if (adjustment == 0)
        {
            return;
        }
        auto& adjustments = getState().pendingVehicleRevenueAdjustments;
        auto& current = adjustments[vehicle];
        if (adjustment > 0 && current > std::numeric_limits<int64_t>::max() - adjustment)
        {
            current = std::numeric_limits<int64_t>::max();
        }
        else if (adjustment < 0 && current < std::numeric_limits<int64_t>::min() - adjustment)
        {
            current = std::numeric_limits<int64_t>::min();
        }
        else
        {
            current += adjustment;
        }
        if (current == 0)
        {
            adjustments.erase(vehicle);
        }
    }

    std::optional<int64_t> consumeVehicleRevenueAdjustment(const EntityId vehicle)
    {
        auto& adjustments = getState().pendingVehicleRevenueAdjustments;
        const auto found = adjustments.find(vehicle);
        if (found == adjustments.end())
        {
            return std::nullopt;
        }
        const auto adjustment = found->second;
        adjustments.erase(found);
        return adjustment;
    }

    uint16_t loadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, const VehicleServiceLeg& serviceLeg)
    {
        if (serviceLeg.from != station || serviceLeg.to == StationId::null)
        {
            return 0;
        }
        bool cargoChanged = false;
        if (getStationCargoConst(station, nativeCargo.type) == nullptr)
        {
            cargoChanged |= seedStationCargo(station, nativeCargo.type, nativeStationCargo);
        }
        if (getVehicleCargoConst(key) == nullptr)
        {
            cargoChanged |= seedVehicleCargo(key, nativeCargo, serviceLeg.to, serviceLeg.departure, serviceLeg.arrival);
        }
        if (cargoChanged)
        {
            invalidateJourneyGraph(nativeCargo.type);
        }

        auto& stationPackets = getOrCreateStationCargo(station, nativeCargo.type);
        auto& vehiclePackets = getOrCreateVehicleCargo(key);
        const auto freeCapacity = nativeCargo.maxQty - std::min<uint32_t>(vehiclePackets.quantity(), nativeCargo.maxQty);
        auto loaded = stationPackets.takeFor(serviceLeg.to, serviceLeg.departure, freeCapacity);
        auto remainingCapacity = freeCapacity - loaded.quantity();
        if (remainingCapacity != 0)
        {
            const auto alternatives = getAlternativeBoarding(station, nativeCargo.type, serviceLeg, stationPackets);
            for (const auto& alternative : alternatives)
            {
                auto diverted = stationPackets.takeForJourney(alternative.destination, alternative.nextHop, alternative.departure, std::min(remainingCapacity, alternative.quantity));
                const auto moved = diverted.quantity();
                for (auto packet : diverted.packets())
                {
                    packet.nextHop = serviceLeg.to;
                    packet.departure = serviceLeg.departure;
                    packet.arrival = serviceLeg.arrival;
                    loaded.append(packet);
                }
                remainingCapacity -= moved;
                if (remainingCapacity == 0)
                {
                    break;
                }
            }
        }
        const auto quantity = static_cast<uint16_t>(loaded.quantity());
        vehiclePackets.append(std::move(loaded));
        if (quantity != 0)
        {
            invalidateJourneyGraph(nativeCargo.type);
        }
        if (cargoChanged || quantity != 0)
        {
            markCargoChanged();
        }
        synchroniseStationCargo(station, nativeCargo.type, nativeStationCargo);
        synchroniseVehicleCargo(key, nativeCargo);
        return quantity;
    }

    UnloadResult unloadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, std::span<const StationId> remainingStops, bool forceUnload, std::optional<VehicleServiceLeg> onwardLeg, TransferPaymentCalculator transferPayment)
    {
        if (getVehicleCargoConst(key) == nullptr)
        {
            seedVehicleCargo(key, nativeCargo, station);
        }
        auto* vehiclePackets = getVehicleCargo(key);
        if (vehiclePackets == nullptr)
        {
            return {};
        }

        auto allPackets = vehiclePackets->take(std::numeric_limits<uint32_t>::max());
        PacketList arriving;
        for (auto packet : allPackets.packets())
        {
            const auto nextHopStillServed = std::find(remainingStops.begin(), remainingStops.end(), packet.nextHop) != remainingStops.end();
            if (forceUnload || packet.nextHop == station || packet.nextHop == StationId::null || !nextHopStillServed)
            {
                arriving.append(packet);
            }
            else
            {
                vehiclePackets->append(packet);
            }
        }

        UnloadResult result;
        PacketList transferred;
        bool needsRecalculation = false;
        for (auto packet : arriving.packets())
        {
            if (forceUnload && nativeStationCargo.isAccepted() && packet.tripKind == PassengerTripKind::ordinary)
            {
                result.delivered.append(packet);
                continue;
            }
            if (packet.destination == station && nativeStationCargo.isAccepted())
            {
                result.delivered.append(packet);
                continue;
            }
            const auto rejectedDestination = packet.destination == station;
            if (rejectedDestination)
            {
                packet.destination = StationId::null;
                needsRecalculation = true;
            }
            const auto excluded = rejectedDestination ? station : StationId::null;
            auto shares = allocatePacketVia(nativeCargo.type, station, packet, packet.arrival, excluded);
            if (shares.empty() && !packet.arrival.empty())
            {
                shares = allocatePacketVia(nativeCargo.type, station, packet, ServicePoint{}, excluded);
            }
            const auto returnsToPreviousOccurrence = !forceUnload && onwardLeg.has_value()
                && onwardLeg->from == station
                && packet.arrival == onwardLeg->departure
                && !packet.departure.empty()
                && onwardLeg->arrival == packet.departure
                && onwardLeg->to != packet.destination;
            if (returnsToPreviousOccurrence)
            {
                std::erase_if(shares, [&](const auto& share) {
                    return share.via == onwardLeg->to
                        && share.departure == onwardLeg->departure
                        && share.arrival == onwardLeg->arrival;
                });
            }
            if (shares.empty())
            {
                shares.push_back({ StationId::null, packet.quantity, {}, {}, packet.destination });
            }
            const auto hasPlannedContinuation = onwardLeg.has_value() && std::any_of(shares.begin(), shares.end(), [&](const auto& share) {
                                                    return onwardLeg->from == station
                                                        && packet.arrival == onwardLeg->departure
                                                        && share.via == onwardLeg->to
                                                        && share.departure == onwardLeg->departure
                                                        && share.arrival == onwardLeg->arrival;
                                                });
            if (!forceUnload && onwardLeg.has_value() && !returnsToPreviousOccurrence && !hasPlannedContinuation
                && onwardLeg->from == station
                && packet.destination != StationId::null
                && packet.arrival == onwardLeg->departure
                && getStateConst().serviceEdges.contains({ nativeCargo.type, station, onwardLeg->to, onwardLeg->departure, onwardLeg->arrival })
                && getJourneyCost(nativeCargo.type, station, packet.destination, onwardLeg->departure)
                    < getJourneyCost(nativeCargo.type, station, packet.destination))
            {
                packet.nextHop = onwardLeg->to;
                packet.departure = onwardLeg->departure;
                packet.arrival = onwardLeg->arrival;
                vehiclePackets->append(packet);
                continue;
            }
            uint32_t allocated = 0;
            auto remainingPacket = packet;
            for (const auto& share : shares)
            {
                allocated += share.amount;
                auto routedPacket = remainingPacket.extract(static_cast<uint16_t>(share.amount));
                if (share.destination != StationId::null)
                {
                    routedPacket.destination = share.destination;
                }
                const auto via = share.via;
                if (!forceUnload && onwardLeg.has_value()
                    && onwardLeg->from == station
                    && routedPacket.arrival == onwardLeg->departure
                    && via == onwardLeg->to
                    && share.departure == onwardLeg->departure
                    && share.arrival == onwardLeg->arrival)
                {
                    routedPacket.nextHop = via;
                    routedPacket.departure = share.departure;
                    routedPacket.arrival = share.arrival;
                    vehiclePackets->append(routedPacket);
                    continue;
                }
                if (via == station && routedPacket.destination == station && nativeStationCargo.isAccepted())
                {
                    result.delivered.append(routedPacket);
                    continue;
                }
                routedPacket.nextHop = via == station ? StationId::null : via;
                routedPacket.departure = share.departure;
                routedPacket.arrival = share.arrival;
                needsRecalculation |= routedPacket.nextHop == StationId::null;
                transferred.append(routedPacket);
            }
            if (allocated < packet.quantity)
            {
                remainingPacket.nextHop = StationId::null;
                remainingPacket.departure = {};
                remainingPacket.arrival = {};
                transferred.append(remainingPacket);
                needsRecalculation = true;
            }
        }

        if (!transferred.empty())
        {
            auto& stationPackets = getOrCreateStationCargo(station, nativeCargo.type);
            const auto room = std::numeric_limits<uint32_t>::max() - stationPackets.quantity();
            auto retained = transferred.take(room);
            result.transferred = static_cast<uint16_t>(retained.quantity());
            if (transferPayment)
            {
                retained.transform([&](CargoPacket& packet) {
                    const auto credit = accrueTransferCredit(packet, transferPayment(packet));
                    if (credit != 0)
                    {
                        result.transferCredits.push_back({ packet, credit });
                    }
                });
            }
            stationPackets.append(std::move(retained));
            transferred.transform([](CargoPacket& packet) {
                packet.nextHop = StationId::null;
                packet.departure = {};
                packet.arrival = {};
            });
            needsRecalculation |= !transferred.empty();
            vehiclePackets->append(std::move(transferred));
        }
        if (needsRecalculation)
        {
            markGraphDirty();
        }
        invalidateJourneyGraph(nativeCargo.type);
        if (!allPackets.empty())
        {
            markCargoChanged();
        }
        synchroniseStationCargo(station, nativeCargo.type, nativeStationCargo);
        synchroniseVehicleCargo(key, nativeCargo);
        return result;
    }

    std::optional<VehicleServiceLeg> getCurrentServiceLeg(const Vehicles::VehicleHead& head)
    {
        if (getStateConst().servicesDirty || isServiceRecalculationPending())
        {
            return std::nullopt;
        }
        const auto* leg = findCurrentServiceLeg(head);
        return leg == nullptr ? std::nullopt : std::optional{ *leg };
    }

    StationId getNextStop(const Vehicles::VehicleHead& head)
    {
        for (const auto& order : head.getCurrentOrders())
        {
            if (const auto* stop = order.as<Vehicles::OrderStopAt>())
            {
                return stop->getStation();
            }
        }
        return StationId::null;
    }

    bool hasOutstandingTransferCredits(const uint8_t cargo)
    {
        const auto hasCredits = [](const PacketList& packets) {
            return std::any_of(packets.packets().begin(), packets.packets().end(), [](const auto& packet) { return packet.transferCredit != 0; });
        };
        const auto& state = getStateConst();
        if (std::any_of(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end(), [cargo](const auto& pending) { return pending.cargo == cargo && pending.transferCredit != 0; }))
        {
            return true;
        }
        if (std::any_of(state.stationCargo.begin(), state.stationCargo.end(), [&](const auto& item) { return item.first.cargo == cargo && hasCredits(item.second); }))
        {
            return true;
        }
        bool found = false;
        forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, const auto& nativeCargo) {
            const auto* packets = getVehicleCargoConst(key);
            found |= nativeCargo.type == cargo && packets != nullptr && hasCredits(*packets);
        });
        return found;
    }

    bool canDisableDistribution(const uint8_t cargo)
    {
        const auto& state = getStateConst();
        const auto& stationCargo = state.stationCargo;
        const auto hasHolidayPackets = [cargo](const auto& item) {
            return item.first.cargo == cargo && std::any_of(item.second.packets().begin(), item.second.packets().end(), [](const auto& packet) { return packet.tripKind != PassengerTripKind::ordinary; });
        };
        bool hasVehicleHolidayPackets = false;
        forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, const auto& nativeCargo) {
            const auto* packets = getVehicleCargoConst(key);
            hasVehicleHolidayPackets |= nativeCargo.type == cargo && packets != nullptr
                && std::any_of(packets->packets().begin(), packets->packets().end(), [](const auto& packet) { return packet.tripKind != PassengerTripKind::ordinary; });
        });
        return std::none_of(stationCargo.begin(), stationCargo.end(), [cargo](const auto& item) {
                   return item.first.cargo == cargo && item.second.quantity() > std::numeric_limits<uint16_t>::max();
               })
            && std::none_of(stationCargo.begin(), stationCargo.end(), hasHolidayPackets)
            && !hasVehicleHolidayPackets
            && std::none_of(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end(), [cargo](const auto& pending) { return pending.cargo == cargo; })
            && !hasOutstandingTransferCredits(cargo);
    }

    void setMode(uint8_t cargo, DistributionMode mode)
    {
        auto& state = getState();
        if (cargo >= state.settings.modes.size() || state.settings.modes[cargo] == mode)
        {
            return;
        }
        if (mode == DistributionMode::manual && !canDisableDistribution(cargo))
        {
            return;
        }

        if (mode == DistributionMode::asymmetric)
        {
            state.settings.modes[cargo] = mode;
            for (auto& station : StationManager::stations())
            {
                if (station.empty())
                {
                    continue;
                }
                const auto& nativeCargo = station.cargoStats[cargo];
                seedStationCargo(station.id(), cargo, nativeCargo);
                if (nativeCargo.quantity != 0 && nativeCargo.origin == station.id())
                {
                    state.supply[{ cargo, station.id() }] = nativeCargo.quantity;
                }
            }
            forEachVehicleCargo([cargo](const auto& head, VehicleCargoKey key, const auto& nativeCargo) {
                if (nativeCargo.type == cargo)
                {
                    const auto nextHop = head.status == Vehicles::Status::unloading && head.stationId != StationId::null
                        ? head.stationId
                        : getNextStop(head);
                    seedVehicleCargo(key, nativeCargo, nextHop);
                }
            });
        }
        else
        {
            synchroniseAllCargo();
            std::erase_if(state.stationCargo, [cargo](const auto& item) { return item.first.cargo == cargo; });
            forEachVehicleCargo([cargo](const auto&, VehicleCargoKey key, const auto& nativeCargo) {
                if (nativeCargo.type == cargo)
                {
                    eraseVehicleCargo(key);
                }
            });
            clearRoutingState(cargo);
            std::erase_if(state.holidaySources, [cargo](const auto& item) { return item.first.cargo == cargo; });
            state.settings.modes[cargo] = mode;
        }
        if (isPassengerCargo(cargo))
        {
            refreshResorts(false);
        }
        state.nextRecalculationDay = 0;
        state.graphDirty = true;
        state.servicesDirty = true;
        ++state.routingRevision;
        notifyRecalculationDirty();
    }

    void recalculateNow()
    {
        cancelPendingRecalculation();
        auto& state = getState();
        if (getServiceCargoMask() == 0)
        {
            clearHolidayRouting();
            state.stationAccessibility.clear();
            state.hasStationAccessibilitySnapshot = true;
            state.graphDirty = false;
            state.servicesDirty = false;
            return;
        }
        recalculateFlows();
        state.nextRecalculationDay = getCurrentDay() + std::max<uint16_t>(1, state.settings.recalculationInterval);
        state.graphDirty = false;
    }

    void update()
    {
        auto& state = getState();
        if (_pendingFlowCalculation.has_value() && _pendingFlowCalculation->generation != _flowCalculationGeneration)
        {
            _pendingFlowCalculation.reset();
        }
        if (_pendingFlowCalculation.has_value() && _pendingFlowCalculation->dirtyEpoch != _dirtyEpoch)
        {
            const auto pending = *_pendingFlowCalculation;
            notifyRecalculationDirty();
            _pendingFlowCalculation.reset();
            state.servicesDirty |= pending.blocksTransfers;
            startFlowCalculation(pending.scheduled, !pending.blocksTransfers);
            return;
        }
        if (_pendingFlowCalculation.has_value())
        {
            const auto pending = *_pendingFlowCalculation;
            if (!isDeadlineReached(pending.deadline))
            {
                return;
            }
            const auto waitStart = std::chrono::steady_clock::now();
            auto result = _flowWorker->take(pending.generation, true);
            const auto waitEnd = std::chrono::steady_clock::now();
            if (pending.generation != _flowCalculationGeneration || pending.dirtyEpoch != _dirtyEpoch || !result.has_value())
            {
                _pendingFlowCalculation.reset();
                return;
            }
            const auto commitStart = waitEnd;
            commitFlowCalculation(std::move(*result));
            _recalculationMetrics.solveNanoseconds += result->solveNanoseconds;
            _recalculationMetrics.waitNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(waitEnd - waitStart).count();
            _recalculationMetrics.commitNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - commitStart).count();
            ++_recalculationMetrics.calculations;
            if (pending.scheduled)
            {
                finishScheduledRecalculation();
            }
            state.graphDirty = false;
            _pendingFlowCalculation.reset();
            return;
        }
        if (state.servicesDirty)
        {
            startFlowCalculation(false, false);
        }
        else if (state.nextRecalculationDay != 0 && getCurrentDay() + 1 >= state.nextRecalculationDay)
        {
            startFlowCalculation(true, true);
        }
    }

    bool isServiceRecalculationPending()
    {
        return getStateConst().servicesDirty
            || (_pendingFlowCalculation.has_value() && _pendingFlowCalculation->blocksTransfers);
    }

    void notifyRecalculationDirty()
    {
        ++_flowCalculationGeneration;
    }

    void notifyGraphDirty()
    {
        ++_dirtyEpoch;
    }

    void cancelPendingRecalculation()
    {
        ++_flowCalculationGeneration;
        _pendingFlowCalculation.reset();
        _flowWorker.reset();
        clearHolidayRouting();
    }

    RecalculationMetrics getRecalculationMetrics()
    {
        return _recalculationMetrics;
    }

    void validateState(const State& state, const GameState& gameState)
    {
        const auto isActiveStation = [&gameState](StationId id) {
            const auto index = enumValue(id);
            return index < std::size(gameState.stations) && !gameState.stations[index].empty();
        };
        const auto isEnabled = [&state](uint8_t cargo) {
            return cargo < state.settings.modes.size() && state.settings.modes[cargo] != DistributionMode::manual;
        };

        const auto validatePackets = [&](const PacketList& packets, const uint8_t cargo) {
            return std::all_of(packets.packets().begin(), packets.packets().end(), [&](const auto& packet) {
                const auto validKind = packet.tripKind == PassengerTripKind::ordinary || packet.tripKind == PassengerTripKind::holidayOutbound || packet.tripKind == PassengerTripKind::holidayReturn;
                const auto town = enumValue(packet.homeTown);
                const auto validHoliday = packet.tripKind == PassengerTripKind::ordinary
                    ? packet.holidayIndustry == IndustryId::null && packet.homeTown == TownId::null
                    : isPassengerCargo(cargo) && enumValue(packet.holidayIndustry) < std::size(gameState.industries) && town < std::size(gameState.towns) && !gameState.towns[town].empty();
                return validKind && validHoliday
                    && isActiveStation(packet.origin)
                    && (packet.destination == StationId::null || isActiveStation(packet.destination))
                    && (packet.nextHop == StationId::null || isActiveStation(packet.nextHop));
            });
        };
        for (const auto& [key, attraction] : state.stationAttraction)
        {
            if (key.cargo >= state.settings.modes.size() || !isActiveStation(key.station) || attraction == 0)
            {
                throw std::runtime_error("Invalid CargoDist station attraction state");
            }
        }
        if (!state.hasStationAccessibilitySnapshot && !state.stationAccessibility.empty())
        {
            throw std::runtime_error("CargoDist station accessibility has no committed snapshot");
        }
        for (const auto& entry : state.stationAccessibility)
        {
            if (!isActiveStation(entry.first) || entry.second == 0)
            {
                throw std::runtime_error("Invalid CargoDist station accessibility state");
            }
        }
        for (const auto& [industry, activity] : state.resorts)
        {
            const auto index = enumValue(industry);
            if (index >= std::size(gameState.industries) || activity.popularity > 100)
            {
                throw std::runtime_error("Invalid CargoDist resort activity state");
            }
        }
        for (const auto& [key, source] : state.holidaySources)
        {
            if (!isEnabled(key.cargo) || !isPassengerCargo(key.cargo) || !isActiveStation(key.station) || source.remainder >= 100)
            {
                throw std::runtime_error("Invalid CargoDist holiday source state");
            }
        }
        if (!std::is_sorted(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end()))
        {
            throw std::runtime_error("Non-canonical CargoDist pending holiday return state");
        }
        for (const auto& pending : state.pendingHolidayReturns)
        {
            const auto industry = enumValue(pending.resort);
            const auto town = enumValue(pending.homeTown);
            if (pending.quantity == 0 || !isEnabled(pending.cargo) || !isPassengerCargo(pending.cargo) || industry >= std::size(gameState.industries)
                || town >= std::size(gameState.towns) || gameState.towns[town].empty()
                || (pending.resortStation != StationId::null && !isActiveStation(pending.resortStation))
                || (pending.homeStation != StationId::null && !isActiveStation(pending.homeStation))
                || pending.transferCredit < 0 || pending.transferCredit > static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * pending.quantity
                || (!pending.released && (pending.age != 0 || pending.transferCredit != 0)))
            {
                throw std::runtime_error("Invalid CargoDist pending holiday return state");
            }
        }
        for (const auto& [key, packets] : state.stationCargo)
        {
            if (!isEnabled(key.cargo) || !isActiveStation(key.station) || !validatePackets(packets, key.cargo))
            {
                throw std::runtime_error("Invalid CargoDist station cargo state");
            }
        }
        for (const auto& [key, amount] : state.supply)
        {
            if (!isEnabled(key.first) || !isActiveStation(key.second) || amount == 0)
            {
                throw std::runtime_error("Invalid CargoDist supply state");
            }
        }
        for (const auto& [key, options] : state.flows)
        {
            if (!isEnabled(key.cargo) || !isActiveStation(key.station) || !isActiveStation(key.origin) || !isActiveStation(key.destination)
                || std::any_of(options.begin(), options.end(), [&](const auto& option) { return !isActiveStation(option.via); }))
            {
                throw std::runtime_error("Invalid CargoDist flow state");
            }
        }
        for (const auto& [key, options] : state.destinationFlows)
        {
            if (!isEnabled(key.cargo) || !isActiveStation(key.station) || !isActiveStation(key.origin)
                || std::any_of(options.begin(), options.end(), [&](const auto& option) { return !isActiveStation(option.destination); }))
            {
                throw std::runtime_error("Invalid CargoDist destination flow state");
            }
        }
        for (const auto& [vehicleId, adjustment] : state.pendingVehicleRevenueAdjustments)
        {
            const auto index = enumValue(vehicleId);
            const auto* vehicle = index < std::size(gameState.entities) ? gameState.entities[index].asBase<Vehicles::VehicleBase>() : nullptr;
            if (adjustment == 0 || vehicle == nullptr || vehicle->getSubType() != Vehicles::VehicleEntityType::head || vehicle->id != vehicleId)
            {
                throw std::runtime_error("Invalid CargoDist pending vehicle revenue");
            }
        }

        std::set<VehicleCargoKey> liveVehicleCargo;
        forEachVehicleCargo(gameState, [&](VehicleCargoKey key, const auto& nativeCargo) {
            liveVehicleCargo.insert(key);
            const auto packets = state.vehicleCargo.find(key);
            if (isEnabled(nativeCargo.type))
            {
                const auto quantity = packets == state.vehicleCargo.end() ? 0 : packets->second.quantity();
                if (nativeCargo.qty != quantity
                    || (quantity != 0 && (nativeCargo.townFrom != packets->second.representativeOrigin() || nativeCargo.numDays != packets->second.averageAge()))
                    || (packets != state.vehicleCargo.end() && !validatePackets(packets->second, nativeCargo.type)))
                {
                    throw std::runtime_error("CargoDist vehicle cargo does not match native state");
                }
            }
            else if (packets != state.vehicleCargo.end())
            {
                throw std::runtime_error("CargoDist vehicle packets use manual cargo");
            }
        });
        if (std::any_of(state.vehicleCargo.begin(), state.vehicleCargo.end(), [&](const auto& item) { return !liveVehicleCargo.contains(item.first); }))
        {
            throw std::runtime_error("CargoDist vehicle cargo references missing component");
        }

        for (uint16_t stationIndex = 0; stationIndex < std::size(gameState.stations); ++stationIndex)
        {
            const auto& station = gameState.stations[stationIndex];
            if (station.empty())
            {
                continue;
            }
            for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
            {
                const auto packets = state.stationCargo.find({ StationId(stationIndex), cargo });
                if (isEnabled(cargo))
                {
                    const auto quantity = packets == state.stationCargo.end() ? 0 : packets->second.quantity();
                    const auto& nativeCargo = station.cargoStats[cargo];
                    const auto nativeQuantity = static_cast<uint16_t>(std::min<uint32_t>(quantity, std::numeric_limits<uint16_t>::max()));
                    if (nativeCargo.quantity != nativeQuantity
                        || (quantity != 0 && (nativeCargo.origin != packets->second.representativeOrigin() || nativeCargo.enrouteAge != packets->second.averageAge())))
                    {
                        throw std::runtime_error("CargoDist station cargo does not match native state");
                    }
                }
                else if (packets != state.stationCargo.end())
                {
                    throw std::runtime_error("CargoDist station packets use manual cargo");
                }
            }
        }
    }

    void restoreState(State state)
    {
        cancelPendingRecalculation();
        for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
        {
            if (ObjectManager::get<CargoObject>(cargo) == nullptr && state.settings.modes[cargo] != DistributionMode::manual)
            {
                throw std::runtime_error("CargoDist mode references unloaded cargo");
            }
        }
        validateState(state, getGameState());
        const auto hasSavedStationAccessibility = state.hasStationAccessibilitySnapshot;
        auto savedStationAccessibility = std::move(state.stationAccessibility);
        state.stationAccessibility.clear();
        state.hasStationAccessibilitySnapshot = false;
        const auto refreshStationMetadata = state.requiresStationMetadataRefresh;
        state.requiresStationMetadataRefresh = false;
        state.serviceEdges.clear();
        state.vehicleServiceLegs.clear();
        state.routingRevision = getStateConst().routingRevision + 1;
        state.cargoRevision = getStateConst().cargoRevision + 1;

        getState() = std::move(state);
        if (refreshStationMetadata)
        {
            for (auto& station : StationManager::stations())
            {
                station.refreshCargoRoutingMetadata();
            }
        }
        refreshResorts(false);
        rebuildServiceEdges();
        const auto needsRecalculation = getStateConst().graphDirty || !hasValidServicePlans();
        if (needsRecalculation)
        {
            recalculateFlows();
            // Keep the committed snapshot until the normal worker can replace it.
            getState().graphDirty = hasSavedStationAccessibility;
        }
        else
        {
            recalculateHolidayFlows();
            if (!hasSavedStationAccessibility)
            {
                rebuildStationAccessibility();
            }
        }
        if (hasSavedStationAccessibility)
        {
            getState().stationAccessibility = std::move(savedStationAccessibility);
            getState().hasStationAccessibilitySnapshot = true;
        }
    }

    static void processHolidayReturns(const bool accrueGuestDays)
    {
        auto& state = getState();
        bool repaired = false;
        for (auto& [key, packets] : state.stationCargo)
        {
            packets.transform([&](auto& packet) { repaired |= repairHolidayDestination(packet, key.cargo, true); });
        }
        forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, const auto& nativeCargo) {
            if (auto* packets = getVehicleCargo(key); packets != nullptr)
            {
                packets->transform([&](auto& packet) { repaired |= repairHolidayDestination(packet, nativeCargo.type, false); });
            }
        });
        if (repaired)
        {
            markGraphDirty();
        }
        bool pendingChanged = false;
        if (accrueGuestDays)
        {
            for (auto& pending : state.pendingHolidayReturns)
            {
                if (pending.released)
                {
                    pendingChanged |= pending.age != std::numeric_limits<uint8_t>::max();
                    pending.age = Math::Bound::add(pending.age, 1);
                    continue;
                }
                const auto activity = state.resorts.find(pending.resort);
                if (activity != state.resorts.end())
                {
                    activity->second.guestDays = saturatedAdd(activity->second.guestDays, pending.quantity);
                }
            }
        }

        bool changed = false;
        for (auto it = state.pendingHolidayReturns.begin(); it != state.pendingHolidayReturns.end();)
        {
            if (it->releaseDay > getCurrentDay())
            {
                break;
            }
            const auto resortStation = findResortStation(*it);
            const auto homeStation = findHomeStation(*it);
            if (resortStation == StationId::null || homeStation == StationId::null)
            {
                ++it;
                continue;
            }
            if (resortStation == homeStation)
            {
                invalidateJourneyGraph(it->cargo);
                changed = true;
                it = state.pendingHolidayReturns.erase(it);
                continue;
            }

            auto& packets = getOrCreateStationCargo(resortStation, it->cargo);
            const auto available = std::numeric_limits<uint32_t>::max() - packets.quantity();
            const auto releaseQuantity = static_cast<uint16_t>(std::min<uint32_t>(available, it->quantity));
            if (releaseQuantity == 0)
            {
                ++it;
                continue;
            }

            CargoPacket remainingPacket{ it->quantity, resortStation, StationId::null, it->age, {}, {}, homeStation, it->transferCredit };
            remainingPacket.tripKind = PassengerTripKind::holidayReturn;
            remainingPacket.holidayIndustry = it->resort;
            remainingPacket.homeTown = it->homeTown;
            auto returnPacket = remainingPacket.extract(releaseQuantity);
            const auto shares = allocateFixedVia(_holidayFlows, it->cargo, resortStation, resortStation, homeStation, releaseQuantity);
            if (shares.empty())
            {
                packets.append(returnPacket);
            }
            else
            {
                for (const auto& share : shares)
                {
                    auto packet = returnPacket.extract(static_cast<uint16_t>(share.amount));
                    packet.nextHop = share.via;
                    packet.departure = share.departure;
                    packet.arrival = share.arrival;
                    packets.append(packet);
                }
            }
            auto* station = StationManager::get(resortStation);
            synchroniseStationCargo(resortStation, it->cargo, station->cargoStats[it->cargo]);
            station->updateCargoDistribution();
            auto* industry = IndustryManager::get(it->resort);
            if (!it->released && industry != nullptr && !industry->empty() && industry->getObject() != nullptr)
            {
                for (uint8_t output = 0; output < 2; ++output)
                {
                    if (industry->getObject()->producedCargoType[output] == it->cargo)
                    {
                        industry->producedCargoQuantityMonthlyTotal[output] = Math::Bound::add(industry->producedCargoQuantityMonthlyTotal[output], releaseQuantity);
                        industry->producedCargoQuantityDeliveredMonthlyTotal[output] = Math::Bound::add(industry->producedCargoQuantityDeliveredMonthlyTotal[output], releaseQuantity);
                    }
                }
            }
            invalidateJourneyGraph(it->cargo);
            changed = true;
            if (remainingPacket.quantity == 0)
            {
                it = state.pendingHolidayReturns.erase(it);
            }
            else
            {
                it->quantity = remainingPacket.quantity;
                it->transferCredit = remainingPacket.transferCredit;
                pendingChanged = true;
                ++it;
            }
        }
        if (pendingChanged)
        {
            std::sort(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end());
        }
        if (changed)
        {
            markCargoChanged();
            markGraphDirty();
        }
    }

    void updateDaily()
    {
        processHolidayReturns(true);
        if (getServiceCargoMask() == 0)
        {
            return;
        }

        auto& state = getState();
        const auto currentDay = getCurrentDay();
        const auto scheduled = state.nextRecalculationDay == 0 || currentDay >= state.nextRecalculationDay;
        if (!state.graphDirty && !scheduled)
        {
            return;
        }

        if (_pendingFlowCalculation.has_value())
        {
            if (!scheduled || _pendingFlowCalculation->scheduled)
            {
                return;
            }
            notifyRecalculationDirty();
            _pendingFlowCalculation.reset();
        }
        startFlowCalculation(scheduled, true);
    }

    void removeIndustry(const IndustryId industry)
    {
        auto& state = getState();
        for (auto& pending : state.pendingHolidayReturns)
        {
            if (pending.resort == industry)
            {
                pending.resortStation = findResortStation(pending);
                pending.releaseDay = std::min(pending.releaseDay, getCurrentDay());
            }
        }
        std::sort(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end());
        processHolidayReturns(false);
        bool pendingChanged = std::erase_if(state.pendingHolidayReturns, [industry](const auto& pending) {
            return pending.resort == industry && pending.resortStation == StationId::null;
        }) != 0;
        for (auto& pending : state.pendingHolidayReturns)
        {
            if (pending.resort == industry)
            {
                pendingChanged |= !pending.released;
                pending.released = true;
            }
        }
        std::sort(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end());

        bool cargoChanged = false;
        const auto cancelOutbound = [&](PacketList& packets, const uint8_t cargo, const StationId currentStation) {
            packets.transform([&](auto& packet) {
                if (packet.tripKind == PassengerTripKind::holidayOutbound && packet.holidayIndustry == industry)
                {
                    cargoChanged = true;
                    const auto* current = StationManager::get(currentStation);
                    if (current != nullptr && !current->empty() && current->town == packet.homeTown)
                    {
                        packet.quantity = 0;
                        return;
                    }
                    packet.tripKind = PassengerTripKind::holidayReturn;
                    repairHolidayDestination(packet, cargo, true);
                }
            });
        };
        for (auto& [key, packets] : state.stationCargo)
        {
            cancelOutbound(packets, key.cargo, key.station);
            if (auto* station = StationManager::get(key.station); station != nullptr && !station->empty())
            {
                synchroniseStationCargo(key.station, key.cargo, station->cargoStats[key.cargo]);
            }
        }
        forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, auto& nativeCargo) {
            if (auto* packets = getVehicleCargo(key); packets != nullptr)
            {
                cancelOutbound(*packets, nativeCargo.type, StationId::null);
                synchroniseVehicleCargo(key, nativeCargo);
            }
        });
        std::erase_if(state.stationCargo, [](const auto& item) { return item.second.empty(); });
        std::erase_if(state.vehicleCargo, [](const auto& item) { return item.second.empty(); });
        state.resorts.erase(industry);
        clearHolidayRouting();
        clearJourneyCache();
        markGraphDirty();
        if (cargoChanged || pendingChanged)
        {
            markCargoChanged();
        }
    }

    void removeTown(const TownId town)
    {
        auto& state = getState();
        bool changed = std::erase_if(state.pendingHolidayReturns, [town](const auto& pending) { return pending.homeTown == town; }) != 0;
        const auto removePackets = [town](PacketList& packets) {
            bool removed = false;
            packets.transform([&](auto& packet) {
                if (packet.tripKind != PassengerTripKind::ordinary && packet.homeTown == town)
                {
                    packet.quantity = 0;
                    removed = true;
                }
            });
            return removed;
        };
        for (auto& [key, packets] : state.stationCargo)
        {
            if (removePackets(packets))
            {
                changed = true;
                if (auto* station = StationManager::get(key.station); station != nullptr && !station->empty())
                {
                    synchroniseStationCargo(key.station, key.cargo, station->cargoStats[key.cargo]);
                }
            }
        }
        forEachVehicleCargo([&](const auto&, const VehicleCargoKey key, auto& nativeCargo) {
            if (auto* packets = getVehicleCargo(key); packets != nullptr)
            {
                if (removePackets(*packets))
                {
                    changed = true;
                    synchroniseVehicleCargo(key, nativeCargo);
                }
            }
        });
        std::erase_if(state.stationCargo, [](const auto& item) { return item.second.empty(); });
        std::erase_if(state.vehicleCargo, [](const auto& item) { return item.second.empty(); });
        changed |= std::erase_if(state.holidaySources, [town](const auto& item) {
            const auto* station = StationManager::get(item.first.station);
            return station == nullptr || station->empty() || station->town == town;
        }) != 0;
        if (changed)
        {
            clearHolidayRouting();
            clearJourneyCache();
            markCargoChanged();
            markGraphDirty();
        }
    }

    void removeStation(StationId station)
    {
        auto& state = getState();
        std::set<uint8_t> affectedCargo;
        bool preservedReturns = false;
        for (const auto& [key, packets] : state.stationCargo)
        {
            if (key.station != station)
            {
                continue;
            }
            for (const auto& packet : packets.packets())
            {
                if (packet.tripKind == PassengerTripKind::holidayReturn)
                {
                    PendingHolidayReturn pending;
                    pending.releaseDay = getCurrentDay();
                    pending.quantity = packet.quantity;
                    pending.resortStation = packet.origin == station ? StationId::null : packet.origin;
                    pending.homeStation = packet.destination;
                    pending.homeTown = packet.homeTown;
                    pending.resort = packet.holidayIndustry;
                    pending.cargo = key.cargo;
                    pending.age = packet.age;
                    pending.released = true;
                    pending.transferCredit = packet.transferCredit;
                    state.pendingHolidayReturns.push_back(pending);
                    preservedReturns = true;
                }
            }
        }
        if (preservedReturns)
        {
            std::sort(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end());
            markCargoChanged();
        }
        std::erase_if(state.stationCargo, [station](const auto& item) { return item.first.station == station; });
        for (auto& [key, packets] : state.stationCargo)
        {
            packets.removeStationReferences(station);
        }
        for (auto& [key, packets] : state.vehicleCargo)
        {
            packets.removeStationReferences(station);
        }
        std::erase_if(state.stationCargo, [](const auto& item) { return item.second.empty(); });
        std::erase_if(state.vehicleCargo, [](const auto& item) { return item.second.empty(); });
        std::erase_if(state.supply, [station](const auto& item) { return item.first.second == station; });
        std::erase_if(state.stationAttraction, [station](const auto& item) { return item.first.station == station; });
        std::erase_if(state.holidaySources, [station](const auto& item) { return item.first.station == station; });
        for (auto& pending : state.pendingHolidayReturns)
        {
            if (pending.resortStation == station)
            {
                pending.resortStation = StationId::null;
            }
            if (pending.homeStation == station)
            {
                pending.homeStation = StationId::null;
            }
        }
        clearHolidayRouting();
        state.stationAccessibility.erase(station);
        std::erase_if(state.serviceEdges, [station](const auto& item) { return item.first.from == station || item.first.to == station; });
        for (auto it = state.flows.begin(); it != state.flows.end();)
        {
            if (it->first.station == station || it->first.origin == station || it->first.destination == station)
            {
                affectedCargo.insert(it->first.cargo);
                it = state.flows.erase(it);
                continue;
            }
            if (std::erase_if(it->second, [station](const auto& option) { return option.via == station; }) != 0)
            {
                affectedCargo.insert(it->first.cargo);
                for (auto& option : it->second)
                {
                    option.current = 0;
                }
            }
            if (it->second.empty())
            {
                it = state.flows.erase(it);
            }
            else
            {
                ++it;
            }
        }
        for (const auto cargo : affectedCargo)
        {
            rebuildDestinationFlows(cargo);
        }
        synchroniseAllCargo();
        state.graphDirty = true;
        state.servicesDirty = true;
        notifyRecalculationDirty();
        ++state.routingRevision;
    }

    void eraseVehicleCargoForComponent(EntityId component)
    {
        eraseVehicleCargo({ component, VehicleCargoSlot::primary });
        eraseVehicleCargo({ component, VehicleCargoSlot::secondary });
        markServicesDirty();
    }

    void removeVehicleService(EntityId vehicle)
    {
        auto& state = getState();
        state.pendingVehicleRevenueAdjustments.erase(vehicle);
        const auto found = state.vehicleServiceLegs.find(vehicle);
        if (found == state.vehicleServiceLegs.end() || found->second.empty())
        {
            markServicesDirty();
            return;
        }

        const auto service = found->second.front().departure.service;
        state.vehicleServiceLegs.erase(found);
        if (service != static_cast<ServiceId>(static_cast<uint16_t>(vehicle)))
        {
            markServicesDirty();
            return;
        }

        for (auto& [key, packets] : state.stationCargo)
        {
            packets.removeServiceReferences(service);
        }
        for (auto& [key, packets] : state.vehicleCargo)
        {
            packets.removeServiceReferences(service, true);
        }
        std::erase_if(state.serviceEdges, [service](const auto& item) {
            return item.first.departure.service == service || item.first.arrival.service == service;
        });
        std::set<uint8_t> affectedCargo;
        for (auto it = state.flows.begin(); it != state.flows.end();)
        {
            if (it->first.incoming.service == service)
            {
                affectedCargo.insert(it->first.cargo);
                it = state.flows.erase(it);
                continue;
            }
            const auto removed = std::erase_if(it->second, [service](const auto& option) {
                return option.departure.service == service || option.arrival.service == service;
            });
            if (removed != 0)
            {
                affectedCargo.insert(it->first.cargo);
                for (auto& option : it->second)
                {
                    option.current = 0;
                }
            }
            if (it->second.empty())
            {
                it = state.flows.erase(it);
            }
            else
            {
                ++it;
            }
        }
        for (const auto cargo : affectedCargo)
        {
            rebuildDestinationFlows(cargo);
        }
        clearHolidayRouting();
        state.graphDirty = true;
        state.servicesDirty = true;
        ++state.routingRevision;
    }

    void moveVehicleCargo(VehicleCargoKey source, VehicleCargoKey destination)
    {
        if (source == destination)
        {
            return;
        }
        auto& cargo = getState().vehicleCargo;
        cargo.erase(destination);
        const auto found = cargo.find(source);
        if (found != cargo.end())
        {
            auto packets = std::move(found->second);
            cargo.erase(found);
            cargo.emplace(destination, std::move(packets));
        }
    }
}
