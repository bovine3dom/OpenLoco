// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include <algorithm>
#include <cassert>
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

        struct CargoRouteNodeBuilder
        {
            uint64_t quantity{};
            std::map<StationId, CargoRouteNodeBuilder> children;
        };

        StationId getRouteField(const CargoRouteSummary& route, CargoRouteField field)
        {
            switch (field)
            {
                case CargoRouteField::origin:
                    return route.origin;
                case CargoRouteField::destination:
                    return route.destination;
                case CargoRouteField::nextHop:
                    return route.nextHop;
            }
            return StationId::null;
        }

        std::vector<CargoRouteNode> makeRouteTree(const std::map<StationId, CargoRouteNodeBuilder>& builders)
        {
            std::vector<CargoRouteNode> nodes;
            nodes.reserve(builders.size());
            for (const auto& [station, builder] : builders)
            {
                nodes.push_back({ station, builder.quantity, makeRouteTree(builder.children) });
            }
            return nodes;
        }

        template<typename TOption>
        struct AllocationCandidate
        {
            TOption* option;
            uint32_t amount{};
            int64_t current{};
            uint64_t allocationWeight{};
            uint64_t remainder{};
        };

        template<typename TRange, typename TGetCurrent>
        void normaliseCursors(TRange& options, int64_t limit, TGetCurrent&& getCurrent)
        {
            int64_t total = 0;
            for (auto& option : options)
            {
                auto& current = getCurrent(option);
                current = std::clamp(current, -limit, limit);
                total += current;
            }
            for (auto& option : options)
            {
                if (total == 0)
                {
                    break;
                }
                auto& current = getCurrent(option);
                if (total > 0 && current > 0)
                {
                    const auto adjustment = std::min(total, current);
                    current -= adjustment;
                    total -= adjustment;
                }
                else if (total < 0 && current < 0)
                {
                    const auto adjustment = std::min(-total, -current);
                    current += adjustment;
                    total += adjustment;
                }
            }
        }

        template<typename TOption>
        uint64_t normaliseCursors(std::vector<TOption>& options)
        {
            uint64_t totalWeight = 0;
            for (const auto& option : options)
            {
                totalWeight += option.weight;
            }
            if (totalWeight > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / kFlowCursorScale))
            {
                for (auto& option : options)
                {
                    option.current = 0;
                }
                return totalWeight;
            }
            normaliseCursors(options, static_cast<int64_t>(totalWeight * kFlowCursorScale), [](auto& option) -> int64_t& { return option.current; });
            return totalWeight;
        }

        template<typename TOption, typename TPredicate, typename TKey>
        std::vector<AllocationCandidate<TOption>> allocateWeighted(std::vector<TOption>& options, uint32_t quantity, TPredicate&& include, TKey&& key)
        {
            const auto fullWeight = normaliseCursors(options);
            std::vector<AllocationCandidate<TOption>> candidates;
            uint64_t totalWeight = 0;
            for (auto& option : options)
            {
                if (!include(option))
                {
                    continue;
                }
                candidates.push_back({ &option, 0, option.current, static_cast<uint64_t>(option.weight) * kFlowCursorScale });
                totalWeight += option.weight;
            }
            if (candidates.empty() || totalWeight == 0 || quantity == 0 || fullWeight > std::numeric_limits<uint32_t>::max())
            {
                return {};
            }

            const auto filtered = totalWeight != fullWeight;
            const auto cursorRange = fullWeight * kFlowCursorScale;
            if (filtered)
            {
                uint64_t assignedWeight = 0;
                std::vector<AllocationCandidate<TOption>*> remainders;
                remainders.reserve(candidates.size());
                for (auto& candidate : candidates)
                {
                    const auto quotient = cursorRange / totalWeight;
                    const auto remainder = cursorRange % totalWeight;
                    candidate.allocationWeight = quotient * candidate.option->weight + remainder * candidate.option->weight / totalWeight;
                    candidate.remainder = remainder * candidate.option->weight % totalWeight;
                    assignedWeight += candidate.allocationWeight;
                    remainders.push_back(&candidate);
                }
                std::sort(remainders.begin(), remainders.end(), [&](const auto* lhs, const auto* rhs) {
                    if (lhs->remainder != rhs->remainder)
                    {
                        return lhs->remainder > rhs->remainder;
                    }
                    if (lhs->current != rhs->current)
                    {
                        return lhs->current > rhs->current;
                    }
                    return key(*lhs->option) < key(*rhs->option);
                });
                for (uint64_t remainder = cursorRange - assignedWeight; remainder != 0; --remainder)
                {
                    ++remainders[remainder - 1]->allocationWeight;
                }
            }

            uint64_t allocated = 0;
            const auto total = static_cast<int64_t>(cursorRange);
            for (auto& candidate : candidates)
            {
                auto target = static_cast<uint64_t>(quantity) * candidate.allocationWeight;
                if (candidate.current >= 0)
                {
                    target += static_cast<uint64_t>(candidate.current);
                }
                else
                {
                    const auto negativeCurrent = static_cast<uint64_t>(-(candidate.current + 1)) + 1;
                    if (target < negativeCurrent)
                    {
                        candidate.current = -static_cast<int64_t>(negativeCurrent - target);
                        continue;
                    }
                    target -= negativeCurrent;
                }
                if (target > 0)
                {
                    candidate.amount = static_cast<uint32_t>(target / total);
                    allocated += candidate.amount;
                }
                candidate.current = static_cast<int64_t>(target % cursorRange);
            }

            while (allocated < quantity)
            {
                const auto chosen = std::max_element(candidates.begin(), candidates.end(), [&](const auto& lhs, const auto& rhs) {
                    if (lhs.current != rhs.current)
                    {
                        return lhs.current < rhs.current;
                    }
                    return key(*lhs.option) > key(*rhs.option);
                });
                ++chosen->amount;
                chosen->current -= total;
                ++allocated;
            }
            while (allocated > quantity)
            {
                auto chosen = candidates.end();
                for (auto it = candidates.begin(); it != candidates.end(); ++it)
                {
                    if (it->amount != 0
                        && (chosen == candidates.end() || it->current < chosen->current
                            || (it->current == chosen->current && key(*it->option) < key(*chosen->option))))
                    {
                        chosen = it;
                    }
                }
                --chosen->amount;
                chosen->current += total;
                --allocated;
            }
            for (auto& candidate : candidates)
            {
                candidate.option->current = candidate.current;
            }
            if (filtered)
            {
                normaliseCursors(options);
            }
            return candidates;
        }
    }

    uint32_t PacketList::quantity() const
    {
        uint32_t total = 0;
        for (const auto& packet : _packets)
        {
            total = packet.quantity > std::numeric_limits<uint32_t>::max() - total
                ? std::numeric_limits<uint32_t>::max()
                : total + packet.quantity;
        }
        return total;
    }

    CargoPacket CargoPacket::extract(const uint16_t amount)
    {
        assert(amount <= quantity);
        auto extracted = *this;
        extracted.quantity = amount;
        const auto quotient = transferCredit / quantity;
        const auto remainder = transferCredit % quantity;
        extracted.transferCredit = quotient * amount + (remainder * amount) / quantity;
        quantity -= amount;
        transferCredit -= extracted.transferCredit;
        return extracted;
    }

    std::vector<CargoRouteSummary> getRouteSummaries(const PacketList& packets)
    {
        std::map<std::tuple<StationId, StationId, StationId>, uint64_t> quantities;
        for (const auto& packet : packets.packets())
        {
            quantities[{ packet.origin, packet.destination, packet.nextHop }] += packet.quantity;
        }

        std::vector<CargoRouteSummary> summaries;
        summaries.reserve(quantities.size());
        for (const auto& [route, quantity] : quantities)
        {
            const auto& [origin, destination, nextHop] = route;
            summaries.push_back({ origin, destination, nextHop, quantity });
        }
        return summaries;
    }

    std::vector<CargoRouteNode> getRouteTree(const std::span<const CargoRouteSummary> summaries, const std::array<CargoRouteField, 3>& order)
    {
        constexpr std::array kFields = { CargoRouteField::origin, CargoRouteField::destination, CargoRouteField::nextHop };
        if (!std::is_permutation(order.begin(), order.end(), kFields.begin()))
        {
            return {};
        }

        std::map<StationId, CargoRouteNodeBuilder> roots;
        for (const auto& route : summaries)
        {
            auto* level = &roots;
            for (const auto field : order)
            {
                auto& node = (*level)[getRouteField(route, field)];
                node.quantity = route.quantity > std::numeric_limits<uint64_t>::max() - node.quantity
                    ? std::numeric_limits<uint64_t>::max()
                    : node.quantity + route.quantity;
                level = &node.children;
            }
        }
        return makeRouteTree(roots);
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
            return std::tie(lhs.destination, lhs.nextHop, lhs.origin, lhs.age, lhs.departure, lhs.arrival, lhs.tripKind, lhs.holidayIndustry, lhs.homeTown, lhs.transferCredit)
                < std::tie(rhs.destination, rhs.nextHop, rhs.origin, rhs.age, rhs.departure, rhs.arrival, rhs.tripKind, rhs.holidayIndustry, rhs.homeTown, rhs.transferCredit);
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
                if (previous.origin == packet.origin && previous.destination == packet.destination && previous.nextHop == packet.nextHop && previous.age == packet.age
                    && previous.departure == packet.departure && previous.arrival == packet.arrival && previous.tripKind == packet.tripKind
                    && previous.holidayIndustry == packet.holidayIndustry && previous.homeTown == packet.homeTown)
                {
                    const auto room = static_cast<uint32_t>(std::numeric_limits<uint16_t>::max() - previous.quantity);
                    const auto merged = std::min<uint32_t>(room, packet.quantity);
                    const auto extracted = packet.extract(static_cast<uint16_t>(merged));
                    previous.quantity += extracted.quantity;
                    previous.transferCredit += extracted.transferCredit;
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

    PacketList PacketList::takeImpl(uint32_t requested, std::optional<StationId> nextHop, std::optional<ServicePoint> departure, std::optional<StationId> destination)
    {
        PacketList result;
        for (auto it = _packets.begin(); it != _packets.end() && requested != 0;)
        {
            if ((nextHop.has_value() && it->nextHop != *nextHop)
                || (departure.has_value() && it->departure != *departure)
                || (destination.has_value() && it->destination != *destination))
            {
                ++it;
                continue;
            }

            const auto moved = static_cast<uint16_t>(std::min<uint32_t>(requested, it->quantity));
            result._packets.push_back(it->extract(moved));
            requested -= moved;
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

    PacketList PacketList::takeForJourney(StationId destination, StationId nextHop, ServicePoint departure, uint32_t quantity)
    {
        return takeImpl(quantity, nextHop, departure, destination);
    }

    uint32_t PacketList::remove(uint32_t requested)
    {
        const auto before = quantity();
        take(requested);
        return before - quantity();
    }

    uint32_t PacketList::removeForRating(uint32_t requested)
    {
        const auto before = quantity();
        for (auto it = _packets.begin(); it != _packets.end() && requested != 0;)
        {
            if (it->tripKind == PassengerTripKind::holidayReturn)
            {
                ++it;
                continue;
            }
            const auto removed = static_cast<uint16_t>(std::min<uint32_t>(requested, it->quantity));
            it->extract(removed);
            requested -= removed;
            if (it->quantity == 0)
            {
                it = _packets.erase(it);
            }
            else
            {
                ++it;
            }
        }
        const auto removed = before - quantity();
        canonicalise();
        return removed;
    }

    void PacketList::removeStationReferences(StationId station)
    {
        for (auto& packet : _packets)
        {
            if (packet.origin == station)
            {
                if (packet.tripKind == PassengerTripKind::ordinary)
                {
                    packet.quantity = 0;
                    continue;
                }
                const auto replacement = packet.nextHop != StationId::null && packet.nextHop != station
                    ? packet.nextHop
                    : packet.destination;
                if (replacement == StationId::null || replacement == station)
                {
                    packet.quantity = 0;
                    continue;
                }
                packet.origin = replacement;
            }
            if (packet.nextHop == station || packet.destination == station)
            {
                if (packet.destination == station)
                {
                    packet.destination = StationId::null;
                }
                packet.nextHop = StationId::null;
                packet.departure = {};
                packet.arrival = {};
            }
        }
        std::erase_if(_packets, [](const auto& packet) { return packet.quantity == 0; });
        canonicalise();
    }

    void PacketList::removeServiceReferences(ServiceId service, bool preserveNextHop)
    {
        if (service == ServiceId::null)
        {
            return;
        }
        for (auto& packet : _packets)
        {
            if (packet.departure.service == service || packet.arrival.service == service)
            {
                if (!preserveNextHop)
                {
                    packet.nextHop = StationId::null;
                }
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

    uint32_t getStationAccessibility(const StationId station)
    {
        const auto found = _state.stationAccessibility.find(station);
        return found == _state.stationAccessibility.end() ? 0 : found->second;
    }

    void reset()
    {
        cancelPendingRecalculation();
        const auto routingRevision = _state.routingRevision + 1;
        const auto cargoRevision = _state.cargoRevision + 1;
        _state = {};
        _state.routingRevision = routingRevision;
        _state.cargoRevision = cargoRevision;
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

    void buildFlowMaps(std::map<FlowKey, std::vector<FlowOption>>& flows, std::map<DestinationFlowKey, std::vector<DestinationOption>>& destinationFlows, uint8_t cargo, std::span<const FlowShare> shares)
    {
        std::vector<FlowKey> touched;
        for (const auto& share : shares)
        {
            if (share.amount == 0)
            {
                continue;
            }
            const FlowKey key{ cargo, share.station, share.origin, share.incoming, share.destination };
            auto& options = flows[key];
            if (options.empty())
            {
                touched.push_back(key);
            }
            options.push_back({ share.via, share.amount, 0, share.departure, share.arrival });
        }
        for (const auto& key : touched)
        {
            auto& options = flows.at(key);
            std::sort(options.begin(), options.end(), [](const auto& lhs, const auto& rhs) {
                return std::tie(lhs.via, lhs.departure, lhs.arrival) < std::tie(rhs.via, rhs.departure, rhs.arrival);
            });
        }

        std::map<DestinationFlowKey, std::map<StationId, uint64_t>> destinationWeights;
        for (const auto& key : touched)
        {
            auto& weight = destinationWeights[{ cargo, key.station, key.origin, key.incoming }][key.destination];
            for (const auto& route : flows.at(key))
            {
                weight += std::min<uint64_t>(route.weight, std::numeric_limits<uint64_t>::max() - weight);
            }
        }
        for (const auto& [key, weights] : destinationWeights)
        {
            uint64_t total = 0;
            for (const auto& [destination, weight] : weights)
            {
                total += weight;
            }
            const auto maximum = std::numeric_limits<uint32_t>::max();
            const auto divisor = total > maximum ? total / maximum + (total % maximum != 0) : 1;
            auto& options = destinationFlows[key];
            uint64_t representableTotal = 0;
            for (const auto& [destination, weight] : weights)
            {
                const auto representableWeight = static_cast<uint32_t>(std::max<uint64_t>(1, weight / divisor));
                options.push_back({ destination, representableWeight, 0 });
                representableTotal += representableWeight;
            }
            for (auto& option : options)
            {
                if (representableTotal <= maximum)
                {
                    break;
                }
                const auto reduction = std::min<uint64_t>(option.weight - 1, representableTotal - maximum);
                option.weight -= static_cast<uint32_t>(reduction);
                representableTotal -= reduction;
            }
        }
    }

    void setFlows(uint8_t cargo, std::span<const FlowShare> shares)
    {
        std::erase_if(_state.flows, [cargo](const auto& item) { return item.first.cargo == cargo; });
        std::erase_if(_state.destinationFlows, [cargo](const auto& item) { return item.first.cargo == cargo; });
        buildFlowMaps(_state.flows, _state.destinationFlows, cargo, shares);
        ++_state.routingRevision;
    }

    void rebuildDestinationFlows(uint8_t cargo)
    {
        std::erase_if(_state.destinationFlows, [cargo](const auto& item) { return item.first.cargo == cargo; });
        std::map<DestinationFlowKey, std::map<StationId, uint64_t>> destinationWeights;
        for (const auto& [key, routes] : _state.flows)
        {
            if (key.cargo != cargo)
            {
                continue;
            }
            auto& weight = destinationWeights[{ cargo, key.station, key.origin, key.incoming }][key.destination];
            for (const auto& route : routes)
            {
                weight += std::min<uint64_t>(route.weight, std::numeric_limits<uint64_t>::max() - weight);
            }
        }
        for (const auto& [key, weights] : destinationWeights)
        {
            uint64_t total = 0;
            for (const auto& [destination, weight] : weights)
            {
                total += weight;
            }
            const auto maximum = std::numeric_limits<uint32_t>::max();
            const auto divisor = total > maximum ? total / maximum + (total % maximum != 0) : 1;
            auto& options = _state.destinationFlows[key];
            uint64_t representableTotal = 0;
            for (const auto& [destination, weight] : weights)
            {
                const auto representableWeight = static_cast<uint32_t>(std::max<uint64_t>(1, weight / divisor));
                options.push_back({ destination, representableWeight, 0 });
                representableTotal += representableWeight;
            }
            for (auto& option : options)
            {
                if (representableTotal <= maximum)
                {
                    break;
                }
                const auto reduction = std::min<uint64_t>(option.weight - 1, representableTotal - maximum);
                option.weight -= static_cast<uint32_t>(reduction);
                representableTotal -= reduction;
            }
        }
    }

    static std::vector<ViaShare> allocateViaImpl(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming, StationId excluded, StationId excluded2, bool updateCursors)
    {
        if (quantity == 0)
        {
            return {};
        }

        const auto flexibleOrigin = destination == StationId::null && station == origin && incoming.empty();
        const auto isOutsideOption = [&](StationId selectedDestination) {
            return flexibleOrigin && selectedDestination == station;
        };
        std::vector<std::pair<StationId, uint32_t>> destinations;
        if (destination == StationId::null)
        {
            const auto choices = _state.destinationFlows.find({ cargo, station, origin, incoming });
            if (choices == _state.destinationFlows.end())
            {
                if (_state.flows.contains({ cargo, station, origin, incoming, StationId::null }))
                {
                    destinations.emplace_back(StationId::null, quantity);
                }
                else
                {
                    return {};
                }
            }
            else
            {
                auto optionsCopy = updateCursors ? std::vector<DestinationOption>{} : choices->second;
                auto& options = updateCursors ? choices->second : optionsCopy;
                const auto allocated = allocateWeighted(options, quantity, [&](const auto& option) {
                    const auto flow = _state.flows.find({ cargo, station, origin, incoming, option.destination });
                    return flow != _state.flows.end() && std::any_of(flow->second.begin(), flow->second.end(), [&](const auto& route) {
                        return isOutsideOption(option.destination)
                            ? route.via == station
                            : option.destination != excluded && option.destination != excluded2 && route.via != excluded && route.via != excluded2;
                    }); }, [](const auto& option) { return option.destination; });
                for (const auto& choice : allocated)
                {
                    if (choice.amount != 0)
                    {
                        destinations.emplace_back(choice.option->destination, choice.amount);
                    }
                }
            }
        }
        else
        {
            destinations.emplace_back(destination, quantity);
        }

        std::vector<ViaShare> result;
        for (const auto& [selectedDestination, amount] : destinations)
        {
            if (isOutsideOption(selectedDestination))
            {
                result.push_back({ StationId::null, amount });
                continue;
            }
            const auto flow = _state.flows.find({ cargo, station, origin, incoming, selectedDestination });
            if (flow == _state.flows.end())
            {
                continue;
            }
            auto optionsCopy = updateCursors ? std::vector<FlowOption>{} : flow->second;
            auto& options = updateCursors ? flow->second : optionsCopy;
            const auto allocated = allocateWeighted(options, amount, [&](const auto& option) { return option.via != excluded && option.via != excluded2; }, [](const auto& option) { return std::tie(option.via, option.departure, option.arrival); });
            for (const auto& candidate : allocated)
            {
                if (candidate.amount != 0)
                {
                    result.push_back({ candidate.option->via, candidate.amount, candidate.option->departure, candidate.option->arrival, selectedDestination });
                }
            }
        }
        return result;
    }

    static std::vector<ViaShare> allocateFixedOptions(std::vector<FlowOption>& options, const StationId destination, const uint32_t quantity, const StationId excluded, const StationId excluded2)
    {
        std::vector<ViaShare> result;
        for (const auto& candidate : allocateWeighted(options, quantity, [&](const auto& option) { return option.via != excluded && option.via != excluded2; }, [](const auto& option) { return std::tie(option.via, option.departure, option.arrival); }))
        {
            if (candidate.amount != 0)
            {
                result.push_back({ candidate.option->via, candidate.amount, candidate.option->departure, candidate.option->arrival, destination });
            }
        }
        return result;
    }

    std::vector<ViaShare> allocateFixedVia(std::map<FlowKey, std::vector<FlowOption>>& flows, const uint8_t cargo, const StationId station, const StationId origin, const StationId destination, const uint32_t quantity, const ServicePoint incoming, const StationId excluded, const StationId excluded2)
    {
        if (quantity == 0 || destination == StationId::null)
        {
            return {};
        }
        const auto flow = flows.find({ cargo, station, origin, incoming, destination });
        if (flow == flows.end())
        {
            return {};
        }
        return allocateFixedOptions(flow->second, destination, quantity, excluded, excluded2);
    }

    std::vector<ViaShare> previewFixedVia(const std::map<FlowKey, std::vector<FlowOption>>& flows, const uint8_t cargo, const StationId station, const StationId origin, const StationId destination, const uint32_t quantity, const ServicePoint incoming, const StationId excluded, const StationId excluded2)
    {
        const auto flow = flows.find({ cargo, station, origin, incoming, destination });
        if (quantity == 0 || destination == StationId::null || flow == flows.end())
        {
            return {};
        }
        auto options = flow->second;
        return allocateFixedOptions(options, destination, quantity, excluded, excluded2);
    }

    std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming, StationId excluded, StationId excluded2)
    {
        return allocateViaImpl(cargo, station, origin, destination, quantity, incoming, excluded, excluded2, true);
    }

    std::vector<ViaShare> previewVia(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming, StationId excluded, StationId excluded2)
    {
        return allocateViaImpl(cargo, station, origin, destination, quantity, incoming, excluded, excluded2, false);
    }

    void markGraphDirty()
    {
        _state.graphDirty = true;
        notifyGraphDirty();
    }

    void markServicesDirty()
    {
        _state.graphDirty = true;
        _state.servicesDirty = true;
        notifyRecalculationDirty();
    }

    std::vector<PlannedServiceEdge> getPlannedServiceEdges(uint8_t cargo)
    {
        struct EdgeStats
        {
            uint64_t plannedDemand{};
            uint64_t waitingDemand{};
            uint64_t incomingDemand{};
            uint64_t capacity{};
            bool hasCapacity{};
            ServicePoint departure{};
            ServicePoint arrival{};

            uint64_t committedDemand() const { return waitingDemand + incomingDemand; }
        };

        std::map<std::pair<StationId, StationId>, EdgeStats> edges;
        std::map<ServiceEdgeKey, EdgeStats> serviceEdges;
        for (const auto& [key, stats] : _state.serviceEdges)
        {
            if (key.cargo == cargo && key.from != StationId::null && key.to != StationId::null && key.from != key.to)
            {
                auto& edge = edges[{ key.from, key.to }];
                edge.capacity += stats.fleetCapacity;
                edge.hasCapacity = true;
                auto& service = serviceEdges[key];
                service.capacity += stats.fleetCapacity;
                service.hasCapacity = true;
                service.departure = key.departure;
                service.arrival = key.arrival;
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
                auto& service = serviceEdges[{ cargo, key.station, option.via, option.departure, option.arrival }];
                service.departure = option.departure;
                service.arrival = option.arrival;
                service.plannedDemand += std::min<uint64_t>(option.weight, std::numeric_limits<uint64_t>::max() - service.plannedDemand);
            }
        }

        for (const auto& [key, demand] : getCommittedServiceDemands(cargo))
        {
            auto& edge = edges[{ key.from, key.to }];
            edge.waitingDemand += demand.waiting;
            edge.incomingDemand += demand.incoming;
            auto& service = serviceEdges[key];
            service.waitingDemand += demand.waiting;
            service.incomingDemand += demand.incoming;
            service.departure = key.departure;
            service.arrival = key.arrival;
        }

        const auto saturationScore = [](const EdgeStats& stats, uint64_t demand) {
            if (demand == 0)
            {
                return uint64_t{ 0 };
            }
            if (!stats.hasCapacity || stats.capacity == 0 || demand >= stats.capacity * 2)
            {
                return uint64_t{ 12 * 20'001 };
            }
            const auto doubledCapacity = stats.capacity * 2;
            const auto bucket = (demand * 12 - 1) / doubledCapacity;
            return bucket * 20'001 + demand * 10'000 / stats.capacity;
        };
        const auto serviceScore = [&](const EdgeStats& stats) {
            return std::pair{ saturationScore(stats, stats.committedDemand()), saturationScore(stats, stats.plannedDemand) };
        };
        std::map<std::pair<StationId, StationId>, EdgeStats> busiestServices;
        for (const auto& [key, stats] : serviceEdges)
        {
            const auto [it, inserted] = busiestServices.try_emplace({ key.from, key.to });
            if (inserted || serviceScore(stats) > serviceScore(it->second))
            {
                it->second = stats;
            }
        }

        std::vector<PlannedServiceEdge> result;
        result.reserve(edges.size());
        for (const auto& [key, stats] : edges)
        {
            const auto capacity = stats.hasCapacity
                ? std::optional<uint32_t>{ static_cast<uint32_t>(std::min<uint64_t>(stats.capacity, std::numeric_limits<uint32_t>::max())) }
                : std::nullopt;
            const auto& busiest = busiestServices.at(key);
            const auto serviceCapacity = busiest.hasCapacity
                ? std::optional<uint32_t>{ static_cast<uint32_t>(std::min<uint64_t>(busiest.capacity, std::numeric_limits<uint32_t>::max())) }
                : std::nullopt;
            result.push_back({ key.first, key.second, stats.plannedDemand, capacity, busiest.plannedDemand, busiest.committedDemand(), busiest.waitingDemand, busiest.incomingDemand, serviceCapacity, busiest.departure, busiest.arrival });
        }
        return result;
    }
}
