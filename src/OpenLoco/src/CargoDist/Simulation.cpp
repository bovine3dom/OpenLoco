// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Date.h"
#include "GameState.h"
#include "Objects/CargoObject.h"
#include "Objects/ObjectManager.h"
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
#include "World/Station.h"
#include "World/StationManager.h"
#include <OpenLoco/Math/Vector.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
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
        };

        struct FlowCalculationResult
        {
            std::map<FlowKey, std::vector<FlowOption>> flows;
            std::map<DestinationFlowKey, std::vector<DestinationOption>> destinationFlows;
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

        uint32_t getEnabledCargoMask()
        {
            uint32_t mask = 0;
            const auto& state = getStateConst();
            for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
            {
                if (state.settings.modes[cargo] != DistributionMode::manual)
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

            const auto enabledCargoMask = getEnabledCargoMask();
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
                        if ((enabledCargoMask & (1U << cargo)) == 0)
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
                    if (packet.origin != StationId::null)
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
                    if (packet.origin != StationId::null && packet.nextHop != StationId::null)
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

        RoutingGraph buildGraph(uint8_t cargo, bool includeDemands = true)
        {
            RoutingGraph graph;
            graph.timeSensitive = true;
            graph.passengerRouting = isPassengerCargo(cargo);
            const auto& state = getStateConst();
            for (const auto& station : StationManager::stations())
            {
                if (station.empty())
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
                graph.nodes.push_back({
                    station.id(),
                    station.x,
                    station.y,
                    0,
                    accepts,
                    attraction,
                });
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

            std::map<FlowKey, uint32_t> inbound;
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
                        auto& quantity = inbound[{ cargo, packet.nextHop, packet.origin, packet.arrival, packet.destination }];
                        quantity = saturatedAdd(quantity, packet.quantity);
                    }
                });
            }
            for (const auto& [key, quantity] : inbound)
            {
                auto shares = previewVia(cargo, key.station, key.origin, key.destination, quantity, key.incoming);
                if (shares.empty() && !key.incoming.empty())
                {
                    shares = previewVia(cargo, key.station, key.origin, key.destination, quantity);
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
                    const auto releaseDestination = isPassengerCargo(cargo) && packet.origin == key.station;
                    const auto destination = releaseDestination ? StationId::null : packet.destination;
                    const auto shares = allocateVia(cargo, key.station, packet.origin, destination, packet.quantity, ServicePoint{}, key.station);
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

        void releaseRejectedDestinations()
        {
            bool changed = false;
            const auto release = [&changed](PacketList& packets, uint8_t cargo, bool clearRoute) {
                packets.transform([&changed, cargo, clearRoute](CargoPacket& packet) {
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
        }

        FlowCalculationInput captureFlowCalculationInput()
        {
            FlowCalculationInput input;
            input.settings = getStateConst().settings.routing;
            for (uint8_t cargo = 0; cargo < input.graphs.size(); ++cargo)
            {
                if (isEnabled(cargo))
                {
                    input.graphs[cargo] = buildGraph(cargo);
                }
            }
            return input;
        }

        FlowCalculationResult solveFlowCalculation(const FlowCalculationInput& input)
        {
            FlowCalculationResult result;
            for (uint8_t cargo = 0; cargo < input.graphs.size(); ++cargo)
            {
                if (input.graphs[cargo].has_value())
                {
                    buildFlowMaps(result.flows, result.destinationFlows, cargo, calculateAsymmetricFlows(*input.graphs[cargo], input.settings));
                    result.computedCargoes.push_back(cargo);
                }
            }
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
            ++state.routingRevision;
            for (const auto cargo : result.computedCargoes)
            {
                rerouteWaitingCargo(cargo);
                invalidateJourneyGraph(cargo);
            }
        }

        void recalculateFlows()
        {
            rebuildServiceEdges();
            releaseRejectedDestinations();
            auto input = captureFlowCalculationInput();
            commitFlowCalculation(solveFlowCalculation(input));
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
        if (isEnabled(cargo))
        {
            markGraphDirty();
        }
    }

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity)
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
            const auto removed = packets->take(quantityBeforeUpdate - nativeCargo.quantity);
            if (!removed.empty())
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
            if (forceUnload && nativeStationCargo.isAccepted())
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
            auto shares = allocateVia(nativeCargo.type, station, packet.origin, packet.destination, packet.quantity, packet.arrival, excluded);
            if (shares.empty() && !packet.arrival.empty())
            {
                shares = allocateVia(nativeCargo.type, station, packet.origin, packet.destination, packet.quantity, ServicePoint{}, excluded);
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
        const auto& stationCargo = getStateConst().stationCargo;
        return std::none_of(stationCargo.begin(), stationCargo.end(), [cargo](const auto& item) {
            return item.first.cargo == cargo && item.second.quantity() > std::numeric_limits<uint16_t>::max();
        }) && !hasOutstandingTransferCredits(cargo);
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
            state.settings.modes[cargo] = mode;
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
        if (std::none_of(state.settings.modes.begin(), state.settings.modes.end(), [](auto mode) { return mode != DistributionMode::manual; }))
        {
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
            if (pending.generation != _flowCalculationGeneration || !result.has_value())
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
            if (pending.dirtyEpoch == _dirtyEpoch)
            {
                state.graphDirty = false;
            }
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

        const auto validatePackets = [&](const PacketList& packets) {
            return std::all_of(packets.packets().begin(), packets.packets().end(), [&](const auto& packet) {
                return isActiveStation(packet.origin)
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
        for (const auto& [key, packets] : state.stationCargo)
        {
            if (!isEnabled(key.cargo) || !isActiveStation(key.station) || !validatePackets(packets))
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
                    || (packets != state.vehicleCargo.end() && !validatePackets(packets->second)))
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
        state.serviceEdges.clear();
        state.vehicleServiceLegs.clear();
        state.routingRevision = getStateConst().routingRevision + 1;
        state.cargoRevision = getStateConst().cargoRevision + 1;

        const auto needsRecalculation = state.graphDirty;
        getState() = std::move(state);
        rebuildServiceEdges();
        if (needsRecalculation || !hasValidServicePlans())
        {
            recalculateFlows();
            getState().graphDirty = false;
        }
    }

    void updateDaily()
    {
        const auto anyEnabled = std::any_of(getStateConst().settings.modes.begin(), getStateConst().settings.modes.end(), [](auto mode) {
            return mode != DistributionMode::manual;
        });
        if (!anyEnabled)
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

    void removeStation(StationId station)
    {
        auto& state = getState();
        std::set<uint8_t> affectedCargo;
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
