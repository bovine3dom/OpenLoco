// SPDX-License-Identifier: MIT
#include "CargoDist/FlowAnalytics.h"

#include "CargoDist/CargoDist.h"
#include "Date.h"
#include "Game.h"
#include "GameState.h"
#include "GameStateFlags.h"
#include "Map/BuildingElement.h"
#include "Map/TileManager.h"
#include "Objects/BuildingObject.h"
#include "Objects/CargoObject.h"
#include "Objects/IndustryObject.h"
#include "Objects/ObjectManager.h"
#include "World/IndustryManager.h"
#include "World/StationManager.h"
#include "World/TownManager.h"
#include <OpenLoco/Math/Vector.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>

namespace OpenLoco::CargoDist::FlowAnalytics
{
    namespace
    {
        using DailyMetrics = std::map<ServiceKey, ServiceMetric>;

        std::map<uint32_t, DailyMetrics> _days;
        uint32_t _revision{};

        uint64_t saturatedAdd(const uint64_t lhs, const uint64_t rhs)
        {
            return rhs > std::numeric_limits<uint64_t>::max() - lhs
                ? std::numeric_limits<uint64_t>::max()
                : lhs + rhs;
        }

        uint64_t getDailyCapacityQ16(const ServiceEdgeStats& stats)
        {
            if (stats.capacity == 0 || stats.headway == 0)
            {
                return 0;
            }

            // A game day advances 65536 day-counter units in steps of 682 per tick.
            const auto numerator = static_cast<uint64_t>(stats.capacity) << 32;
            const auto denominator = static_cast<uint64_t>(stats.headway) * 682;
            return numerator / denominator;
        }

        uint64_t saturatedMultiply(const uint64_t lhs, const uint64_t rhs)
        {
            return rhs != 0 && lhs > std::numeric_limits<uint64_t>::max() / rhs
                ? std::numeric_limits<uint64_t>::max()
                : lhs * rhs;
        }

        ServiceMetric* getMetric(DailyMetrics& metrics, const ServiceKey key)
        {
            if (const auto found = metrics.find(key); found != metrics.end())
            {
                return &found->second;
            }
            if (metrics.size() >= kMaxDailyRecords)
            {
                return nullptr;
            }
            return &metrics.emplace(key, ServiceMetric{ key }).first->second;
        }

        State makeState()
        {
            State state;
            state.days.reserve(_days.size());
            for (const auto& [day, metrics] : _days)
            {
                auto& record = state.days.emplace_back();
                record.day = day;
                record.services.reserve(metrics.size());
                for (const auto& [_, metric] : metrics)
                {
                    record.services.push_back(metric);
                }
            }
            return state;
        }
    }

    void reset()
    {
        _days.clear();
        _revision++;
    }

    void recordDeparture(const uint8_t cargo, const StationId from, const StationId to, const uint32_t quantity, const uint32_t capacity)
    {
        if (capacity == 0 || cargo >= 32 || from == StationId::null || to == StationId::null || from == to)
        {
            return;
        }
        if (auto* metric = getMetric(_days[getCurrentDay()], { cargo, from, to }); metric != nullptr)
        {
            metric->throughput = saturatedAdd(metric->throughput, quantity);
            metric->observedThroughput = saturatedAdd(metric->observedThroughput, std::min(quantity, capacity));
            metric->offeredCapacity = saturatedAdd(metric->offeredCapacity, capacity);
        }
    }

    void updateDaily()
    {
        const auto currentDay = getCurrentDay();
        if (currentDay == 0)
        {
            return;
        }
        const auto completedDay = currentDay - 1;
        auto& metrics = _days[completedDay];
        for (auto& [_, metric] : metrics)
        {
            metric.plannedDemand = 0;
            metric.capacityQ16 = 0;
        }

        const auto& cargoDistState = getStateConst();
        if (!cargoDistState.servicesDirty)
        {
            for (uint8_t cargo = 0; cargo < 32; ++cargo)
            {
                if (!isEnabled(cargo))
                {
                    continue;
                }
                for (const auto& edge : getPlannedServiceEdges(cargo))
                {
                    if (auto* metric = getMetric(metrics, { cargo, edge.from, edge.to }); metric != nullptr)
                    {
                        metric->plannedDemand = edge.plannedDemand;
                    }
                }
            }
            for (const auto& [key, stats] : cargoDistState.serviceEdges)
            {
                if (auto* metric = getMetric(metrics, { key.cargo, key.from, key.to }); metric != nullptr)
                {
                    metric->capacityQ16 = saturatedAdd(metric->capacityQ16, getDailyCapacityQ16(stats));
                }
            }
        }

        while (!_days.empty() && _days.begin()->first < currentDay && currentDay - _days.begin()->first > kMaximumHorizonDays)
        {
            _days.erase(_days.begin());
        }
        _revision++;
    }

    uint32_t getRevision()
    {
        return _revision;
    }

    std::vector<ServiceSummary> summarise(const State& state, const uint8_t cargo, const uint16_t horizonDays, const uint32_t currentDay)
    {
        const auto horizon = std::clamp<uint16_t>(horizonDays, 1, kMaximumHorizonDays);
        const auto firstDay = currentDay > horizon ? currentDay - horizon : 0;
        std::map<std::pair<StationId, StationId>, ServiceSummary> summaries;
        for (const auto& day : state.days)
        {
            if (day.day < firstDay || day.day >= currentDay)
            {
                continue;
            }
            for (const auto& metric : day.services)
            {
                if (metric.key.cargo != cargo)
                {
                    continue;
                }
                auto& summary = summaries[{ metric.key.from, metric.key.to }];
                summary.from = metric.key.from;
                summary.to = metric.key.to;
                summary.throughput = saturatedAdd(summary.throughput, metric.throughput);
                summary.observedThroughput = saturatedAdd(summary.observedThroughput, metric.observedThroughput);
                summary.offeredCapacity = saturatedAdd(summary.offeredCapacity, metric.offeredCapacity);
                summary.plannedDemand = saturatedAdd(summary.plannedDemand, metric.plannedDemand);
                summary.capacityQ16 = saturatedAdd(summary.capacityQ16, metric.capacityQ16);
            }
        }

        std::vector<ServiceSummary> result;
        result.reserve(summaries.size());
        for (const auto& [_, summary] : summaries)
        {
            result.push_back(summary);
        }
        return result;
    }

    std::vector<ServiceSummary> getServiceSummaries(const uint8_t cargo, const uint16_t horizonDays)
    {
        return summarise(makeState(), cargo, horizonDays, getCurrentDay());
    }

    uint64_t roundCapacity(const uint64_t capacityQ16)
    {
        return saturatedAdd(capacityQ16, uint64_t{ 1 } << 15) >> 16;
    }

    std::vector<DestinationFlow> allocateLatentDemand(std::vector<Endpoint>& endpoints, const uint8_t distanceEffect)
    {
        std::map<EndpointKey, size_t> endpointIndices;
        for (size_t i = 0; i < endpoints.size(); ++i)
        {
            endpoints[i].localDemand = 0;
            if (endpoints[i].supply != 0 || endpoints[i].attraction != 0)
            {
                endpointIndices.emplace(endpoints[i].key, i);
            }
        }

        std::vector<DestinationFlow> flows;
        for (const auto& origin : endpoints)
        {
            if (origin.supply == 0 || !endpointIndices.contains(origin.key))
            {
                continue;
            }
            struct Candidate
            {
                EndpointKey key;
                uint64_t score{};
                uint64_t amount{};
                long double remainder{};
            };
            std::vector<std::pair<const Endpoint*, uint64_t>> distances;
            uint64_t minimumDistance = std::numeric_limits<uint64_t>::max();
            for (const auto& destination : endpoints)
            {
                if (destination.attraction == 0 || !endpointIndices.contains(destination.key))
                {
                    continue;
                }
                constexpr uint64_t kMinimumDistance = 32 * World::kTileSize;
                const auto distance = std::max<uint64_t>(kMinimumDistance, Math::Vector::distance2D(origin.position, destination.position));
                distances.emplace_back(&destination, distance);
                minimumDistance = std::min(minimumDistance, distance);
            }

            std::vector<Candidate> candidates;
            long double totalScore = 0;
            for (const auto& [destination, distance] : distances)
            {
                const auto effectiveDistance = saturatedAdd(minimumDistance, (distance - minimumDistance) * distanceEffect / 100);
                const auto proximity = std::min<uint64_t>(uint64_t{ 1 } << 16, (minimumDistance << 16) / std::max<uint64_t>(1, effectiveDistance));
                const auto score = saturatedMultiply(destination->attraction, proximity * proximity);
                candidates.push_back({ destination->key, std::max<uint64_t>(1, score) });
                totalScore += candidates.back().score;
            }
            if (candidates.empty() || totalScore == 0)
            {
                continue;
            }

            uint64_t allocated = 0;
            for (auto& candidate : candidates)
            {
                const auto exact = static_cast<long double>(origin.supply) * candidate.score / totalScore;
                candidate.amount = exact >= origin.supply ? origin.supply : static_cast<uint64_t>(exact);
                candidate.amount = std::min(candidate.amount, origin.supply - allocated);
                candidate.remainder = exact - candidate.amount;
                allocated += candidate.amount;
            }
            std::ranges::sort(candidates, [](const auto& lhs, const auto& rhs) {
                return lhs.remainder != rhs.remainder ? lhs.remainder > rhs.remainder : lhs.key < rhs.key;
            });
            auto remainder = origin.supply - allocated;
            const auto sharedRemainder = remainder / candidates.size();
            if (sharedRemainder != 0)
            {
                for (auto& candidate : candidates)
                {
                    candidate.amount += sharedRemainder;
                }
                remainder %= candidates.size();
            }
            for (size_t i = 0; i < remainder; ++i)
            {
                candidates[i].amount++;
            }
            std::ranges::sort(candidates, {}, &Candidate::key);
            for (const auto& candidate : candidates)
            {
                if (candidate.amount == 0)
                {
                    continue;
                }
                if (candidate.key == origin.key)
                {
                    auto& localDemand = endpoints[endpointIndices.at(origin.key)].localDemand;
                    localDemand = saturatedAdd(localDemand, candidate.amount);
                    continue;
                }
                flows.push_back({ origin.key, candidate.key, candidate.amount, 0, GapReason::noStation });
            }
        }
        return flows;
    }

    DestinationModel getDestinationModel(const uint8_t cargo, const uint16_t horizonDays)
    {
        struct EndpointData
        {
            Endpoint endpoint;
            std::vector<StationId> sourceStations;
            std::vector<StationId> destinationStations;
        };

        const auto horizon = std::clamp<uint16_t>(horizonDays, 1, kMaximumHorizonDays);
        const auto* cargoObject = ObjectManager::get<CargoObject>(cargo);
        if (cargoObject == nullptr)
        {
            return {};
        }
        const auto townCargo = isTownCargoCategory(cargoObject->cargoCategory);
        std::map<EndpointKey, EndpointData> endpoints;
        const auto getTownEndpoint = [&](const TownId townId) -> EndpointData* {
            auto* town = TownManager::get(townId);
            if (town == nullptr || town->empty())
            {
                return nullptr;
            }
            const EndpointKey key{ EndpointKind::town, static_cast<uint16_t>(townId) };
            auto [entry, inserted] = endpoints.try_emplace(key);
            auto& data = entry->second;
            if (inserted)
            {
                data.endpoint.key = key;
                data.endpoint.position = { town->x, town->y };
                data.endpoint.name = town->name;
                data.endpoint.town = townId;
                const auto height = World::TileManager::getHeight(data.endpoint.position);
                data.endpoint.z = std::max(height.landHeight, height.waterHeight);
            }
            return &data;
        };
        const auto getIndustryEndpoint = [&](Industry& industry) -> EndpointData& {
            const EndpointKey key{ EndpointKind::industry, static_cast<uint16_t>(industry.id()) };
            auto [entry, inserted] = endpoints.try_emplace(key);
            auto& data = entry->second;
            if (inserted)
            {
                data.endpoint.key = key;
                data.endpoint.position = { industry.x, industry.y };
                data.endpoint.name = industry.name;
                data.endpoint.town = industry.town;
                const auto height = World::TileManager::getHeight(data.endpoint.position);
                data.endpoint.z = std::max(height.landHeight, height.waterHeight);
            }
            return data;
        };

        if (townCargo)
        {
            for (auto& town : TownManager::towns())
            {
                if (auto* endpoint = getTownEndpoint(town.id()); endpoint != nullptr)
                {
                    endpoint->endpoint.attraction = std::max<uint64_t>(1, town.population);
                }
            }
        }

        const auto reducedProduction = Game::hasFlags(GameStateFlags::unk2);
        constexpr uint64_t kBuildingProductionDenominator = kMonthlyProductionEstimateDenominator * 30;
        for (tile_coord_t y = 0; y < World::kMapRows; ++y)
        {
            for (tile_coord_t x = 0; x < World::kMapColumns; ++x)
            {
                const World::TilePos2 tilePos{ x, y };
                for (const auto& element : World::TileManager::get(tilePos))
                {
                    if (element.isGhost() || element.type() != World::ElementType::building)
                    {
                        continue;
                    }
                    const auto& building = element.get<World::BuildingElement>();
                    if (building.sequenceIndex() != 0 || building.isMiscBuilding() || !building.isConstructed())
                    {
                        continue;
                    }
                    const auto* object = building.getObject();
                    if (object == nullptr)
                    {
                        continue;
                    }
                    uint64_t supply = 0;
                    uint64_t attraction = 0;
                    for (uint8_t slot = 0; slot < 2; ++slot)
                    {
                        if (object->producedCargoType[slot] == cargo)
                        {
                            const auto scaled = getBuildingMonthlyProductionEstimateScaled(object->producedQuantity[slot], reducedProduction);
                            const auto production = (scaled * horizon + kBuildingProductionDenominator / 2) / kBuildingProductionDenominator;
                            supply = saturatedAdd(supply, production);
                        }
                        if (!townCargo && object->consumedCargoType[slot] == cargo)
                        {
                            attraction = saturatedAdd(attraction, object->consumedCargoQty[slot]);
                        }
                    }
                    if (supply == 0 && attraction == 0)
                    {
                        continue;
                    }
                    const auto townId = TownManager::getClosestTown(World::toWorldSpace(tilePos));
                    auto* endpoint = townId.has_value() ? getTownEndpoint(*townId) : nullptr;
                    if (endpoint != nullptr)
                    {
                        endpoint->endpoint.supply = saturatedAdd(endpoint->endpoint.supply, supply);
                        endpoint->endpoint.attraction = saturatedAdd(endpoint->endpoint.attraction, attraction);
                    }
                }
            }
        }

        for (auto& industry : IndustryManager::industries())
        {
            if (industry.empty() || industry.hasFlags(IndustryFlags::isGhost) || industry.under_construction != kIndustryConstructionComplete)
            {
                continue;
            }
            const auto* object = industry.getObject();
            if (object == nullptr)
            {
                continue;
            }
            uint64_t supply = 0;
            uint64_t attraction = 0;
            for (uint8_t slot = 0; slot < 2; ++slot)
            {
                if (object->producedCargoType[slot] == cargo)
                {
                    supply = saturatedAdd(supply, static_cast<uint64_t>(industry.dailyProduction[slot]) * horizon);
                }
            }
            for (uint8_t slot = 0; slot < 3; ++slot)
            {
                if (object->requiredCargoType[slot] == cargo)
                {
                    const auto currentDemand = static_cast<uint64_t>(industry.receivedCargoQuantityPreviousMonth[slot]);
                    const auto targetDemand = static_cast<uint64_t>(industry.dailyProductionTarget[0]) * industry.productionRate / 256 * 30;
                    attraction = saturatedAdd(attraction, std::max<uint64_t>(1, std::max(currentDemand, targetDemand)));
                }
            }
            if (supply != 0 || attraction != 0)
            {
                auto& endpoint = getIndustryEndpoint(industry).endpoint;
                endpoint.supply = supply;
                endpoint.attraction = attraction;
            }
        }

        const auto& cargoState = getStateConst();
        for (auto& station : StationManager::stations())
        {
            if (station.empty())
            {
                continue;
            }
            const auto stationId = station.id();
            const auto& stats = station.cargoStats[cargo];
            const auto fromProducer = (stats.flags & StationCargoStatsFlags::acceptedFromProducer) != StationCargoStatsFlags::none;
            for (auto& [key, data] : endpoints)
            {
                bool inCatchment = false;
                if (key.kind == EndpointKind::town)
                {
                    inCatchment = station.town == static_cast<TownId>(key.id);
                }
                else if (key.kind == EndpointKind::industry)
                {
                    const auto* industry = IndustryManager::get(static_cast<IndustryId>(key.id));
                    inCatchment = industry != nullptr && industry->stationsInRange.get(static_cast<uint16_t>(stationId));
                }
                if (inCatchment && fromProducer)
                {
                    data.sourceStations.push_back(stationId);
                }
                if (inCatchment && stats.isAccepted())
                {
                    data.destinationStations.push_back(stationId);
                }
            }

            if (!townCargo && stats.isAccepted())
            {
                const auto represented = std::ranges::any_of(endpoints, [&](const auto& item) {
                    return std::ranges::find(item.second.destinationStations, stationId) != item.second.destinationStations.end();
                });
                if (!represented)
                {
                    const EndpointKey key{ EndpointKind::station, static_cast<uint16_t>(stationId) };
                    auto& data = endpoints[key];
                    data.endpoint.key = key;
                    data.endpoint.position = { station.x, station.y };
                    data.endpoint.name = station.name;
                    data.endpoint.town = station.town;
                    data.endpoint.z = station.z;
                    const auto attraction = cargoState.stationAttraction.find({ stationId, cargo });
                    data.endpoint.attraction = attraction == cargoState.stationAttraction.end() ? 8 : std::max<uint32_t>(1, attraction->second);
                    data.destinationStations.push_back(stationId);
                }
            }
        }

        struct NetworkEdge
        {
            uint16_t to{};
            uint64_t capacity{};
        };
        std::map<std::pair<StationId, StationId>, uint64_t> pairCapacitiesQ16;
        for (const auto& [key, stats] : cargoState.serviceEdges)
        {
            if (key.cargo == cargo)
            {
                auto& capacity = pairCapacitiesQ16[{ key.from, key.to }];
                capacity = saturatedAdd(capacity, getDailyCapacityQ16(stats));
            }
        }
        std::array<std::vector<NetworkEdge>, Limits::kMaxStations> graph;
        for (const auto& [key, dailyCapacity] : pairCapacitiesQ16)
        {
            const auto from = static_cast<uint16_t>(key.first);
            const auto to = static_cast<uint16_t>(key.second);
            if (from < graph.size() && to < graph.size())
            {
                graph[from].push_back({ to, roundCapacity(saturatedMultiply(dailyCapacity, horizon)) });
            }
        }
        const auto getNetworkCapacities = [&](const EndpointData& origin) {
            std::array<uint64_t, Limits::kMaxStations> capacities{};
            std::priority_queue<std::pair<uint64_t, uint16_t>> queue;
            for (const auto station : origin.sourceStations)
            {
                const auto index = static_cast<uint16_t>(station);
                capacities[index] = std::numeric_limits<uint64_t>::max();
                queue.emplace(capacities[index], index);
            }
            while (!queue.empty())
            {
                const auto [capacity, station] = queue.top();
                queue.pop();
                if (capacity != capacities[station])
                {
                    continue;
                }
                for (const auto& edge : graph[station])
                {
                    const auto candidate = std::min(capacity, edge.capacity);
                    if (candidate > capacities[edge.to])
                    {
                        capacities[edge.to] = candidate;
                        queue.emplace(candidate, edge.to);
                    }
                }
            }
            return capacities;
        };

        DestinationModel model;
        for (const auto& [_, data] : endpoints)
        {
            if (data.endpoint.supply == 0 && data.endpoint.attraction == 0)
            {
                continue;
            }
            model.endpoints.push_back(data.endpoint);
        }

        model.flows = allocateLatentDemand(model.endpoints, cargoState.settings.routing.distanceEffect);
        std::map<EndpointKey, std::array<uint64_t, Limits::kMaxStations>> routeCapacities;
        for (auto& flow : model.flows)
        {
            const auto& origin = endpoints.at(flow.origin);
            const auto& destination = endpoints.at(flow.destination);
            const auto hasStationAccess = !origin.sourceStations.empty() && !destination.destinationStations.empty();
            if (hasStationAccess)
            {
                auto [capacities, inserted] = routeCapacities.try_emplace(flow.origin);
                if (inserted)
                {
                    capacities->second = getNetworkCapacities(origin);
                }
                for (const auto station : destination.destinationStations)
                {
                    flow.capacity = std::max(flow.capacity, capacities->second[static_cast<uint16_t>(station)]);
                }
            }
            if (flow.capacity == std::numeric_limits<uint64_t>::max())
            {
                flow.capacity = flow.demand;
            }
            flow.gap = !hasStationAccess      ? GapReason::noStation
                : flow.capacity == 0          ? GapReason::noRoute
                : flow.capacity < flow.demand ? GapReason::capacityShortfall
                                              : GapReason::served;
        }
        return model;
    }

    State captureState()
    {
        return makeState();
    }

    bool validateState(const State& state)
    {
        if (state.days.size() > kMaximumHorizonDays + 1 || !std::ranges::is_sorted(state.days, {}, &DailyRecord::day))
        {
            return false;
        }
        uint32_t previousDay = 0;
        bool hasPreviousDay = false;
        for (const auto& day : state.days)
        {
            if ((hasPreviousDay && day.day == previousDay) || day.services.size() > kMaxDailyRecords
                || !std::ranges::is_sorted(day.services, {}, &ServiceMetric::key))
            {
                return false;
            }
            previousDay = day.day;
            hasPreviousDay = true;
            ServiceKey previousKey{};
            bool hasPreviousKey = false;
            for (const auto& metric : day.services)
            {
                const auto key = metric.key;
                if (key.cargo >= 32 || key.from == StationId::null || key.to == StationId::null || key.from == key.to
                    || metric.observedThroughput > metric.throughput || metric.observedThroughput > metric.offeredCapacity
                    || (hasPreviousKey && key == previousKey))
                {
                    return false;
                }
                previousKey = key;
                hasPreviousKey = true;
            }
        }
        return true;
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state))
        {
            return false;
        }
        _days.clear();
        for (const auto& day : state.days)
        {
            auto& metrics = _days[day.day];
            for (const auto& metric : day.services)
            {
                metrics.emplace(metric.key, metric);
            }
        }
        _revision++;
        return true;
    }

    bool isDefault(const State& state)
    {
        return state.days.empty();
    }
}
