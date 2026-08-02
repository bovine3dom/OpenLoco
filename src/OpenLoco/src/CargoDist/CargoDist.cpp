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
            return std::tie(lhs.nextHop, lhs.origin, lhs.age) < std::tie(rhs.nextHop, rhs.origin, rhs.age);
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
                if (previous.origin == packet.origin && previous.nextHop == packet.nextHop && previous.age == packet.age)
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

    PacketList PacketList::takeImpl(uint32_t requested, StationId nextHop, bool filterByNextHop)
    {
        PacketList result;
        for (auto it = _packets.begin(); it != _packets.end() && requested != 0;)
        {
            if (filterByNextHop && it->nextHop != nextHop)
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
        return takeImpl(quantity, StationId::null, false);
    }

    PacketList PacketList::takeFor(StationId nextHop, uint32_t quantity)
    {
        return takeImpl(quantity, nextHop, true);
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
        _state = {};
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
            auto& options = _state.flows[{ cargo, share.station, share.origin }];
            options.push_back({ share.via, share.amount, 0 });
        }
        for (auto& [key, options] : _state.flows)
        {
            if (key.cargo != cargo)
            {
                continue;
            }
            std::sort(options.begin(), options.end(), [](const auto& lhs, const auto& rhs) { return idValue(lhs.via) < idValue(rhs.via); });
        }
    }

    std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, uint32_t quantity, StationId excluded, StationId excluded2)
    {
        const auto flow = _state.flows.find({ cargo, station, origin });
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
                return idValue(lhs.option->via) > idValue(rhs.option->via);
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
                        || (it->option->current == chosen->option->current && idValue(it->option->via) < idValue(chosen->option->via))))
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
                result.push_back({ candidate.option->via, candidate.amount });
            }
        }
        return result;
    }

    StationId chooseVia(uint8_t cargo, StationId station, StationId origin, StationId excluded, StationId excluded2)
    {
        const auto shares = allocateVia(cargo, station, origin, 1, excluded, excluded2);
        return shares.empty() ? StationId::null : shares.front().via;
    }

    void markGraphDirty()
    {
        _state.graphDirty = true;
    }
}
