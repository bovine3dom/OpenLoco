// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/CargoDist.h>

#include <algorithm>
#include <limits>
#include <tuple>

namespace OpenLoco::CargoDist
{
    namespace
    {
        State _state;

        uint16_t idValue(StationId id)
        {
            return static_cast<uint16_t>(id);
        }
    }

    uint32_t PacketList::quantity() const
    {
        uint32_t total = 0;
        for (const auto& packet : _packets)
        {
            total += packet.quantity;
        }
        return total;
    }

    PacketList PacketList::fromPackets(Container packets)
    {
        PacketList result;
        result._packets = std::move(packets);
        result.canonicalise();
        return result;
    }

    uint32_t PacketList::quantityFor(StationId nextHop) const
    {
        uint32_t total = 0;
        for (const auto& packet : _packets)
        {
            if (packet.nextHop == nextHop)
            {
                total += packet.quantity;
            }
        }
        return total;
    }

    uint32_t PacketList::quantityFor(StationId nextHop, ServicePoint departure) const
    {
        uint32_t total = 0;
        for (const auto& packet : _packets)
        {
            if (packet.nextHop == nextHop && packet.departure == departure)
            {
                total += packet.quantity;
            }
        }
        return total;
    }

    StationId PacketList::representativeOrigin() const
    {
        std::map<StationId, uint32_t> quantities;
        for (const auto& packet : _packets)
        {
            quantities[packet.origin] += packet.quantity;
        }

        StationId result = StationId::null;
        uint32_t largest = 0;
        for (const auto& [origin, amount] : quantities)
        {
            if (amount > largest || (amount == largest && idValue(origin) < idValue(result)))
            {
                result = origin;
                largest = amount;
            }
        }
        return result;
    }

    uint8_t PacketList::averageAge() const
    {
        uint64_t weightedAge = 0;
        uint32_t total = 0;
        for (const auto& packet : _packets)
        {
            weightedAge += static_cast<uint64_t>(packet.age) * packet.quantity;
            total += packet.quantity;
        }
        return total == 0 ? 0 : static_cast<uint8_t>(weightedAge / total);
    }

    void PacketList::canonicalise()
    {
        std::sort(_packets.begin(), _packets.end(), [](const auto& lhs, const auto& rhs) {
            return std::tie(lhs.nextHop, lhs.origin, lhs.age, lhs.departure, lhs.arrival)
                < std::tie(rhs.nextHop, rhs.origin, rhs.age, rhs.departure, rhs.arrival);
        });
        Container canonical;
        canonical.reserve(_packets.size());
        for (auto packet : _packets)
        {
            if (packet.quantity == 0)
            {
                continue;
            }
            if (!canonical.empty())
            {
                auto& previous = canonical.back();
                if (previous.origin == packet.origin && previous.nextHop == packet.nextHop && previous.age == packet.age
                    && previous.departure == packet.departure && previous.arrival == packet.arrival)
                {
                    const auto room = static_cast<uint32_t>(std::numeric_limits<uint16_t>::max() - previous.quantity);
                    const auto merged = std::min<uint32_t>(room, packet.quantity);
                    previous.quantity += merged;
                    packet.quantity -= merged;
                }
            }
            if (packet.quantity != 0)
            {
                canonical.push_back(packet);
            }
        }
        _packets = std::move(canonical);
    }

    void PacketList::append(CargoPacket packet)
    {
        if (packet.quantity == 0)
        {
            return;
        }
        _packets.push_back(packet);
        canonicalise();
    }

    void PacketList::append(PacketList packets)
    {
        _packets.insert(_packets.end(), packets._packets.begin(), packets._packets.end());
        canonicalise();
    }

    PacketList PacketList::takeImpl(uint32_t requested, std::optional<StationId> nextHop, std::optional<ServicePoint> departure)
    {
        PacketList result;
        for (auto it = _packets.begin(); it != _packets.end() && requested != 0;)
        {
            if ((nextHop.has_value() && it->nextHop != *nextHop) || (departure.has_value() && it->departure != *departure))
            {
                ++it;
                continue;
            }

            const auto moved = static_cast<uint16_t>(std::min<uint32_t>(requested, it->quantity));
            auto packet = *it;
            packet.quantity = moved;
            result._packets.push_back(packet);
            requested -= moved;
            it->quantity -= moved;
            if (it->quantity == 0)
            {
                it = _packets.erase(it);
            }
            else
            {
                ++it;
            }
        }
        result.canonicalise();
        return result;
    }

    PacketList PacketList::take(uint32_t quantity)
    {
        return takeImpl(quantity, std::nullopt, std::nullopt);
    }

    PacketList PacketList::takeFor(StationId nextHop, uint32_t quantity)
    {
        return takeImpl(quantity, nextHop, std::nullopt);
    }

    PacketList PacketList::takeFor(StationId nextHop, ServicePoint departure, uint32_t quantity)
    {
        return takeImpl(quantity, nextHop, departure);
    }

    uint32_t PacketList::remove(uint32_t requested)
    {
        const auto before = quantity();
        take(requested);
        return before - quantity();
    }

    void PacketList::removeStationReferences(StationId station)
    {
        std::erase_if(_packets, [station](const auto& packet) {
            return packet.origin == station;
        });
        for (auto& packet : _packets)
        {
            if (packet.nextHop == station)
            {
                packet.nextHop = StationId::null;
                packet.departure = {};
                packet.arrival = {};
            }
        }
        canonicalise();
    }

    void PacketList::removeServiceReferences(ServiceId service)
    {
        if (service == ServiceId::null)
        {
            return;
        }
        for (auto& packet : _packets)
        {
            if (packet.departure.service == service || packet.arrival.service == service)
            {
                packet.nextHop = StationId::null;
                packet.departure = {};
                packet.arrival = {};
            }
        }
        canonicalise();
    }

    void PacketList::ageAtStation(StationId station)
    {
        for (auto& packet : _packets)
        {
            if (packet.origin != station && packet.age != std::numeric_limits<uint8_t>::max())
            {
                ++packet.age;
            }
        }
        canonicalise();
    }

    void PacketList::ageInVehicle()
    {
        for (auto& packet : _packets)
        {
            if (packet.age != std::numeric_limits<uint8_t>::max())
            {
                ++packet.age;
            }
        }
        canonicalise();
    }

    State& getState()
    {
        return _state;
    }

    const State& getStateConst()
    {
        return _state;
    }

    void reset()
    {
        const auto routingRevision = _state.routingRevision + 1;
        _state = {};
        _state.routingRevision = routingRevision;
    }

    DistributionMode getMode(uint8_t cargo)
    {
        return cargo < _state.settings.modes.size() ? _state.settings.modes[cargo] : DistributionMode::manual;
    }

    bool isEnabled(uint8_t cargo)
    {
        return getMode(cargo) != DistributionMode::manual;
    }

    PacketList* getStationCargo(StationId station, uint8_t cargo)
    {
        const auto it = _state.stationCargo.find({ station, cargo });
        return it == _state.stationCargo.end() ? nullptr : &it->second;
    }

    const PacketList* getStationCargoConst(StationId station, uint8_t cargo)
    {
        const auto it = _state.stationCargo.find({ station, cargo });
        return it == _state.stationCargo.end() ? nullptr : &it->second;
    }

    PacketList& getOrCreateStationCargo(StationId station, uint8_t cargo)
    {
        return _state.stationCargo[{ station, cargo }];
    }

    PacketList* getVehicleCargo(VehicleCargoKey key)
    {
        const auto it = _state.vehicleCargo.find(key);
        return it == _state.vehicleCargo.end() ? nullptr : &it->second;
    }

    const PacketList* getVehicleCargoConst(VehicleCargoKey key)
    {
        const auto it = _state.vehicleCargo.find(key);
        return it == _state.vehicleCargo.end() ? nullptr : &it->second;
    }

    PacketList& getOrCreateVehicleCargo(VehicleCargoKey key)
    {
        return _state.vehicleCargo[key];
    }

    void eraseVehicleCargo(VehicleCargoKey key)
    {
        _state.vehicleCargo.erase(key);
    }

    void setFlows(uint8_t cargo, std::span<const FlowShare> shares)
    {
        std::erase_if(_state.flows, [cargo](const auto& item) { return item.first.cargo == cargo; });
        for (const auto& share : shares)
        {
            if (share.amount == 0)
            {
                continue;
            }
            auto& options = _state.flows[{ cargo, share.station, share.origin, share.incoming }];
            options.push_back({ share.via, share.amount, 0, share.departure, share.arrival });
        }
        for (auto& [key, options] : _state.flows)
        {
            if (key.cargo != cargo)
            {
                continue;
            }
            std::sort(options.begin(), options.end(), [](const auto& lhs, const auto& rhs) {
                return std::tie(lhs.via, lhs.departure, lhs.arrival) < std::tie(rhs.via, rhs.departure, rhs.arrival);
            });
        }
    }

    std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, uint32_t quantity, ServicePoint incoming, StationId excluded, StationId excluded2)
    {
        const auto flow = _state.flows.find({ cargo, station, origin, incoming });
        if (flow == _state.flows.end() || quantity == 0)
        {
            return {};
        }

        struct Candidate
        {
            FlowOption* option;
            uint32_t amount{};
        };
        std::vector<Candidate> candidates;
        uint64_t totalWeight = 0;
        for (auto& option : flow->second)
        {
            if (option.via == excluded || option.via == excluded2)
            {
                continue;
            }
            candidates.push_back({ &option });
            totalWeight += option.weight;
        }
        if (candidates.empty() || totalWeight == 0)
        {
            return {};
        }

        uint64_t allocated = 0;
        const auto total = static_cast<int64_t>(totalWeight);
        for (auto& candidate : candidates)
        {
            auto& option = *candidate.option;
            const auto target = option.current + static_cast<int64_t>(quantity) * option.weight;
            if (target > 0)
            {
                candidate.amount = static_cast<uint32_t>(target / total);
                allocated += candidate.amount;
            }
            option.current = target - static_cast<int64_t>(candidate.amount) * total;
        }

        while (allocated < quantity)
        {
            const auto chosen = std::max_element(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
                if (lhs.option->current != rhs.option->current)
                {
                    return lhs.option->current < rhs.option->current;
                }
                return std::tie(lhs.option->via, lhs.option->departure, lhs.option->arrival)
                    > std::tie(rhs.option->via, rhs.option->departure, rhs.option->arrival);
            });
            ++chosen->amount;
            chosen->option->current -= total;
            ++allocated;
        }
        while (allocated > quantity)
        {
            auto chosen = candidates.end();
            for (auto it = candidates.begin(); it != candidates.end(); ++it)
            {
                if (it->amount != 0
                    && (chosen == candidates.end() || it->option->current < chosen->option->current
                        || (it->option->current == chosen->option->current
                            && std::tie(it->option->via, it->option->departure, it->option->arrival)
                                < std::tie(chosen->option->via, chosen->option->departure, chosen->option->arrival))))
                {
                    chosen = it;
                }
            }
            --chosen->amount;
            chosen->option->current += total;
            --allocated;
        }

        std::vector<ViaShare> result;
        for (const auto& candidate : candidates)
        {
            if (candidate.amount != 0)
            {
                result.push_back({ candidate.option->via, candidate.amount, candidate.option->departure, candidate.option->arrival });
            }
        }
        return result;
    }

    void markGraphDirty()
    {
        _state.graphDirty = true;
    }

    void markServicesDirty()
    {
        _state.graphDirty = true;
        _state.servicesDirty = true;
    }

    std::vector<PlannedServiceEdge> getPlannedServiceEdges(uint8_t cargo)
    {
        struct EdgeStats
        {
            uint64_t plannedDemand{};
            uint64_t capacity{};
            bool hasCapacity{};
        };

        std::map<std::pair<StationId, StationId>, EdgeStats> edges;
        for (const auto& [key, stats] : _state.serviceEdges)
        {
            if (key.cargo == cargo && key.from != StationId::null && key.to != StationId::null && key.from != key.to)
            {
                auto& edge = edges[{ key.from, key.to }];
                edge.capacity += stats.capacity;
                edge.hasCapacity = true;
            }
        }

        for (const auto& [key, options] : _state.flows)
        {
            if (key.cargo != cargo || key.station == StationId::null)
            {
                continue;
            }
            for (const auto& option : options)
            {
                if (option.weight == 0 || option.via == StationId::null || option.via == key.station)
                {
                    continue;
                }

                auto& plannedDemand = edges[{ key.station, option.via }].plannedDemand;
                plannedDemand += std::min<uint64_t>(option.weight, std::numeric_limits<uint64_t>::max() - plannedDemand);
            }
        }

        std::vector<PlannedServiceEdge> result;
        result.reserve(edges.size());
        for (const auto& [key, stats] : edges)
        {
            const auto capacity = stats.hasCapacity
                ? std::optional<uint32_t>{ static_cast<uint32_t>(std::min<uint64_t>(stats.capacity, std::numeric_limits<uint32_t>::max())) }
                : std::nullopt;
            result.push_back({ key.first, key.second, stats.plannedDemand, capacity });
        }
        return result;
    }
}
