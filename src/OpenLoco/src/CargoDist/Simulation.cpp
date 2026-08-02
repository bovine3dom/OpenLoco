// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Date.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/Vehicle.h"
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
#include <limits>
#include <map>
#include <vector>

namespace OpenLoco::CargoDist
{
    namespace
    {
        struct RoutePoint
        {
            World::Pos2 position;
            StationId stop = StationId::null;
        };

        struct EdgeAccumulator
        {
            uint64_t capacity{};
            uint64_t weightedTravelTime{};
        };

        uint32_t saturatedAdd(uint32_t lhs, uint32_t rhs)
        {
            return rhs > std::numeric_limits<uint32_t>::max() - lhs
                ? std::numeric_limits<uint32_t>::max()
                : lhs + rhs;
        }

        void seedStationCargo(StationId station, uint8_t cargo, const StationCargoStats& nativeCargo)
        {
            if (nativeCargo.quantity == 0)
            {
                return;
            }
            const auto origin = nativeCargo.origin == StationId::null ? station : nativeCargo.origin;
            getOrCreateStationCargo(station, cargo).append({ nativeCargo.quantity, origin, StationId::null, nativeCargo.enrouteAge });
        }

        void seedVehicleCargo(VehicleCargoKey key, const Vehicles::VehicleCargo& nativeCargo, StationId nextHop)
        {
            if (nativeCargo.qty == 0)
            {
                return;
            }
            getOrCreateVehicleCargo(key).append({ nativeCargo.qty, nativeCargo.townFrom, nextHop, nativeCargo.numDays });
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

        void addCargoCapacity(std::array<uint32_t, 32>& capacities, const Vehicles::VehicleCargo& cargo)
        {
            if (cargo.maxQty == 0)
            {
                return;
            }
            for (uint8_t cargoType = 0; cargoType < capacities.size(); ++cargoType)
            {
                if ((cargo.acceptedTypes & (1U << cargoType)) != 0)
                {
                    capacities[cargoType] = saturatedAdd(capacities[cargoType], cargo.maxQty);
                }
            }
        }

        std::vector<RoutePoint> getRoutePoints(const Vehicles::VehicleHead& head)
        {
            std::vector<RoutePoint> points;
            for (const auto& order : Vehicles::OrderRingView(head.orderTableOffset))
            {
                if (const auto* stop = order.as<Vehicles::OrderStopAt>())
                {
                    const auto* station = StationManager::get(stop->getStation());
                    if (station != nullptr && !station->empty())
                    {
                        points.push_back({ { station->x, station->y }, stop->getStation() });
                    }
                }
                else if (const auto* through = order.as<Vehicles::OrderRouteThrough>())
                {
                    const auto* station = StationManager::get(through->getStation());
                    if (station != nullptr && !station->empty())
                    {
                        points.push_back({ { station->x, station->y }, StationId::null });
                    }
                }
                else if (const auto* waypoint = order.as<Vehicles::OrderRouteWaypoint>())
                {
                    points.push_back({ World::Pos2{ waypoint->getWaypoint() }, StationId::null });
                }
            }
            return points;
        }

        uint32_t estimateTravelTime(uint64_t distance, const Vehicles::VehicleHead& head, const Vehicles::Vehicle& train)
        {
            const auto speed = train.veh2->maxSpeed.getRaw();
            if (speed <= 0)
            {
                return 0;
            }
            const uint32_t modeModifier = [&head]() {
                switch (head.mode)
                {
                    case TransportMode::air:
                        return 36U;
                    case TransportMode::water:
                        return 31U;
                    default:
                        return 21U;
                }
            }();
            const auto numerator = std::min<uint64_t>(distance * modeModifier, std::numeric_limits<uint64_t>::max() - speed);
            return static_cast<uint32_t>(std::min<uint64_t>(std::max<uint64_t>(1, (numerator + speed - 1) / speed), std::numeric_limits<uint32_t>::max()));
        }

        void addVehicleEdges(const Vehicles::VehicleHead& head, std::map<ServiceEdgeKey, EdgeAccumulator>& accumulators)
        {
            Vehicles::Vehicle train(head);
            if (train.cars.empty())
            {
                return;
            }

            std::array<uint32_t, 32> capacities{};
            for (const auto& car : train.cars)
            {
                addCargoCapacity(capacities, car.body->primaryCargo);
                addCargoCapacity(capacities, car.front->secondaryCargo);
            }

            const auto points = getRoutePoints(head);
            std::vector<size_t> stops;
            for (size_t i = 0; i < points.size(); ++i)
            {
                if (points[i].stop != StationId::null)
                {
                    stops.push_back(i);
                }
            }
            if (stops.size() < 2)
            {
                return;
            }

            for (size_t stopIndex = 0; stopIndex < stops.size(); ++stopIndex)
            {
                const auto fromIndex = stops[stopIndex];
                const auto toIndex = stops[(stopIndex + 1) % stops.size()];
                const auto from = points[fromIndex].stop;
                const auto to = points[toIndex].stop;
                if (from == to)
                {
                    continue;
                }

                uint64_t distance = 0;
                auto previous = points[fromIndex].position;
                for (auto pointIndex = (fromIndex + 1) % points.size();; pointIndex = (pointIndex + 1) % points.size())
                {
                    distance += Math::Vector::distance2D(previous, points[pointIndex].position);
                    previous = points[pointIndex].position;
                    if (pointIndex == toIndex)
                    {
                        break;
                    }
                }

                const auto travelTime = estimateTravelTime(distance, head, train);
                if (travelTime == 0)
                {
                    continue;
                }
                for (uint8_t cargo = 0; cargo < capacities.size(); ++cargo)
                {
                    if (!isEnabled(cargo) || capacities[cargo] == 0)
                    {
                        continue;
                    }
                    auto& edge = accumulators[{ cargo, from, to }];
                    edge.capacity += capacities[cargo];
                    edge.weightedTravelTime += static_cast<uint64_t>(capacities[cargo]) * travelTime;
                }
            }
        }

        void rebuildServiceEdges()
        {
            std::map<ServiceEdgeKey, EdgeAccumulator> accumulators;
            for (const auto* head : VehicleManager::VehicleList())
            {
                addVehicleEdges(*head, accumulators);
            }

            auto& state = getState();
            state.serviceEdges.clear();
            for (const auto& [key, edge] : accumulators)
            {
                const auto capacity = std::min<uint64_t>(edge.capacity, std::numeric_limits<uint32_t>::max());
                state.serviceEdges[key] = {
                    static_cast<uint32_t>(capacity),
                    static_cast<uint32_t>(std::min<uint64_t>(edge.weightedTravelTime / edge.capacity, std::numeric_limits<uint32_t>::max())),
                };
            }
        }

        std::map<std::pair<StationId, StationId>, uint32_t> getRoutingDemands(uint8_t cargo)
        {
            const auto& state = getStateConst();
            std::map<std::pair<StationId, StationId>, uint32_t> demands;
            for (const auto& [key, amount] : state.supply)
            {
                if (key.first == cargo)
                {
                    demands[{ key.second, key.second }] = amount;
                }
            }

            std::map<std::pair<StationId, StationId>, uint32_t> outstanding;
            const auto addPackets = [&outstanding](StationId source, const PacketList* packets) {
                if (packets == nullptr)
                {
                    return;
                }
                for (const auto& packet : packets->packets())
                {
                    if (packet.origin != StationId::null)
                    {
                        outstanding[{ source, packet.origin }] = saturatedAdd(outstanding[{ source, packet.origin }], packet.quantity);
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
                        outstanding[{ packet.nextHop, packet.origin }] = saturatedAdd(outstanding[{ packet.nextHop, packet.origin }], packet.quantity);
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
                demands[key] = std::max(demands[key], amount);
            }
            return demands;
        }

        RoutingGraph buildGraph(uint8_t cargo)
        {
            RoutingGraph graph;
            graph.timeSensitive = true;
            const auto& state = getStateConst();
            const auto demands = getRoutingDemands(cargo);
            for (const auto& station : StationManager::stations())
            {
                if (station.empty())
                {
                    continue;
                }
                graph.nodes.push_back({
                    station.id(),
                    station.x,
                    station.y,
                    0,
                    station.cargoStats[cargo].isAccepted(),
                });
            }
            for (const auto& [key, amount] : demands)
            {
                graph.demands.push_back({ key.first, key.second, amount });
            }
            for (const auto& [key, edge] : state.serviceEdges)
            {
                if (key.cargo == cargo)
                {
                    graph.edges.push_back({ key.from, key.to, edge.capacity, edge.travelTime });
                }
            }
            return graph;
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
                PacketList rerouted;
                for (auto packet : packets.packets())
                {
                    const auto shares = allocateVia(cargo, key.station, packet.origin, packet.quantity, key.station);
                    if (shares.empty())
                    {
                        if (packet.nextHop == key.station)
                        {
                            packet.nextHop = StationId::null;
                        }
                        rerouted.append(packet);
                        continue;
                    }
                    for (const auto& share : shares)
                    {
                        packet.quantity = static_cast<uint16_t>(share.amount);
                        packet.nextHop = share.via;
                        rerouted.append(packet);
                    }
                }
                packets = std::move(rerouted);
                if (auto* station = StationManager::get(key.station); station != nullptr && !station->empty())
                {
                    synchroniseStationCargo(key.station, cargo, station->cargoStats[cargo]);
                }
            }
        }

        void clearRoutingState(uint8_t cargo)
        {
            auto& state = getState();
            std::erase_if(state.supply, [cargo](const auto& item) { return item.first.first == cargo; });
            std::erase_if(state.serviceEdges, [cargo](const auto& item) { return item.first.cargo == cargo; });
            std::erase_if(state.flows, [cargo](const auto& item) { return item.first.cargo == cargo; });
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

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity)
    {
        auto& packets = getOrCreateStationCargo(station, cargo);
        const auto room = std::numeric_limits<uint16_t>::max() - std::min<uint32_t>(packets.quantity(), std::numeric_limits<uint16_t>::max());
        const auto added = static_cast<uint16_t>(std::min<uint32_t>(quantity, room));
        if (added != 0)
        {
            const auto shares = allocateVia(cargo, station, station, added);
            if (shares.empty())
            {
                packets.append({ added, station, StationId::null, 0 });
            }
            else
            {
                for (const auto& share : shares)
                {
                    packets.append({ static_cast<uint16_t>(share.amount), station, share.via, 0 });
                }
            }
        }
        auto& supply = getState().supply[{ cargo, station }];
        supply = saturatedAdd(supply, quantity);
        synchroniseStationCargo(station, cargo, nativeCargo);
    }

    void updateStationCargoDaily(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantityBeforeUpdate)
    {
        auto* packets = getStationCargo(station, cargo);
        if (packets == nullptr)
        {
            seedStationCargo(station, cargo, nativeCargo);
            return;
        }
        packets->ageAtStation(station);
        if (nativeCargo.quantity < quantityBeforeUpdate)
        {
            packets->remove(quantityBeforeUpdate - nativeCargo.quantity);
        }
        synchroniseStationCargo(station, cargo, nativeCargo);
    }

    void updateVehicleCargoDaily(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo)
    {
        auto* packets = getVehicleCargo(key);
        if (packets == nullptr)
        {
            seedVehicleCargo(key, nativeCargo, StationId::null);
            return;
        }
        packets->ageInVehicle();
        synchroniseVehicleCargo(key, nativeCargo);
    }

    uint32_t getLoadableQuantity(StationId station, uint8_t cargo, StationId nextStop)
    {
        if (nextStop == StationId::null)
        {
            return 0;
        }
        const auto* packets = getStationCargoConst(station, cargo);
        return packets == nullptr ? 0 : packets->quantityFor(nextStop);
    }

    uint16_t loadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, StationId nextStop)
    {
        if (nextStop == StationId::null)
        {
            return 0;
        }
        if (getStationCargoConst(station, nativeCargo.type) == nullptr)
        {
            seedStationCargo(station, nativeCargo.type, nativeStationCargo);
        }
        if (getVehicleCargoConst(key) == nullptr)
        {
            seedVehicleCargo(key, nativeCargo, nextStop);
        }

        auto& stationPackets = getOrCreateStationCargo(station, nativeCargo.type);
        auto& vehiclePackets = getOrCreateVehicleCargo(key);
        const auto freeCapacity = nativeCargo.maxQty - std::min<uint32_t>(vehiclePackets.quantity(), nativeCargo.maxQty);
        auto loaded = stationPackets.takeFor(nextStop, freeCapacity);
        const auto quantity = static_cast<uint16_t>(loaded.quantity());
        vehiclePackets.append(std::move(loaded));
        synchroniseStationCargo(station, nativeCargo.type, nativeStationCargo);
        synchroniseVehicleCargo(key, nativeCargo);
        return quantity;
    }

    UnloadResult unloadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, std::span<const StationId> remainingStops, bool forceUnload)
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
            auto shares = allocateVia(nativeCargo.type, station, packet.origin, packet.quantity);
            if (shares.empty())
            {
                shares.push_back({ StationId::null, packet.quantity });
            }
            for (const auto& share : shares)
            {
                packet.quantity = static_cast<uint16_t>(share.amount);
                const auto via = share.via;
                if ((via == station || via == StationId::null) && nativeStationCargo.isAccepted())
                {
                    result.delivered.append(packet);
                    continue;
                }
                packet.nextHop = via == station ? StationId::null : via;
                needsRecalculation |= packet.nextHop == StationId::null;
                transferred.append(packet);
            }
        }

        result.transferred = static_cast<uint16_t>(transferred.quantity());
        if (!transferred.empty())
        {
            auto& stationPackets = getOrCreateStationCargo(station, nativeCargo.type);
            const auto room = std::numeric_limits<uint16_t>::max() - std::min<uint32_t>(stationPackets.quantity(), std::numeric_limits<uint16_t>::max());
            stationPackets.append(transferred.take(room));
        }
        if (needsRecalculation)
        {
            markGraphDirty();
        }
        synchroniseStationCargo(station, nativeCargo.type, nativeStationCargo);
        synchroniseVehicleCargo(key, nativeCargo);
        return result;
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

    void setMode(uint8_t cargo, DistributionMode mode)
    {
        auto& state = getState();
        if (cargo >= state.settings.modes.size() || state.settings.modes[cargo] == mode)
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
    }

    void enableAll()
    {
        for (uint8_t cargo = 0; cargo < getStateConst().settings.modes.size(); ++cargo)
        {
            setMode(cargo, DistributionMode::asymmetric);
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

        rebuildServiceEdges();
        for (uint8_t cargo = 0; cargo < state.settings.modes.size(); ++cargo)
        {
            if (!isEnabled(cargo))
            {
                continue;
            }
            const auto graph = buildGraph(cargo);
            setFlows(cargo, calculateAsymmetricFlows(graph, state.settings.routing));
            rerouteWaitingCargo(cargo);
        }
        if (scheduled)
        {
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
            state.nextRecalculationDay = currentDay + std::max<uint16_t>(1, state.settings.recalculationInterval);
        }
        state.graphDirty = false;
    }

    void removeStation(StationId station)
    {
        auto& state = getState();
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
        std::erase_if(state.serviceEdges, [station](const auto& item) { return item.first.from == station || item.first.to == station; });
        for (auto it = state.flows.begin(); it != state.flows.end();)
        {
            if (it->first.station == station || it->first.origin == station)
            {
                it = state.flows.erase(it);
                continue;
            }
            std::erase_if(it->second, [station](const auto& option) { return option.via == station; });
            if (it->second.empty())
            {
                it = state.flows.erase(it);
            }
            else
            {
                ++it;
            }
        }
        synchroniseAllCargo();
        state.graphDirty = true;
    }

    void eraseVehicleCargoForComponent(EntityId component)
    {
        eraseVehicleCargo({ component, VehicleCargoSlot::primary });
        eraseVehicleCargo({ component, VehicleCargoSlot::secondary });
        markGraphDirty();
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
