// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Routing.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace OpenLoco::CargoDist
{
    namespace
    {
        constexpr size_t kNoIndex = std::numeric_limits<size_t>::max();
        constexpr uint64_t kMaximumCost = std::numeric_limits<uint64_t>::max() - 1;

        enum class PlannedEdgeKind : uint8_t
        {
            ride,
            board,
            alight,
        };

        struct CanonicalRide
        {
            size_t from;
            size_t to;
            uint32_t capacity;
            uint32_t travelTime;
            ServicePoint departure;
            ServicePoint arrival;
            uint32_t waitTime;
        };

        struct Occurrence
        {
            size_t station;
            ServicePoint point;
        };

        struct PlannedEdge
        {
            size_t from;
            size_t to;
            uint32_t capacity;
            uint32_t travelTime;
            PlannedEdgeKind kind;
            ServicePoint departure;
            ServicePoint arrival;
            uint64_t flow = 0;
        };

        struct PlannedGraph
        {
            std::vector<size_t> nodeStations;
            std::vector<PlannedEdge> edges;
            std::vector<Occurrence> occurrences;
            std::vector<Occurrence> arrivals;
        };

        struct SinkAllocation
        {
            size_t node;
            uint32_t amount;
            uint64_t remainder;
        };

        using ShortestPathQueueEntry = std::tuple<uint64_t, size_t, size_t>;

        class ShortestPathQueue : public std::priority_queue<ShortestPathQueueEntry, std::vector<ShortestPathQueueEntry>, std::greater<ShortestPathQueueEntry>>
        {
        public:
            void clear()
            {
                this->c.clear();
            }

            void reserve(size_t size)
            {
                this->c.reserve(size);
            }
        };

        struct ShortestPathScratch
        {
            void reset(size_t nodeCount)
            {
                const auto infinity = std::numeric_limits<uint64_t>::max();
                distance.assign(nodeCount, infinity);
                boardings.assign(nodeCount, kNoIndex);
                previous.assign(nodeCount, kNoIndex);
                settled.assign(nodeCount, false);
                queue.clear();
                queue.reserve(nodeCount);
                path.clear();
                path.reserve(nodeCount);
            }

            std::vector<uint64_t> distance;
            std::vector<size_t> boardings;
            std::vector<size_t> previous;
            std::vector<bool> settled;
            ShortestPathQueue queue;
            std::vector<size_t> path;
        };

        constexpr uint16_t stationValue(StationId station)
        {
            return static_cast<uint16_t>(station);
        }

        constexpr uint16_t serviceValue(ServiceId service)
        {
            return static_cast<uint16_t>(service);
        }

        uint64_t saturatedAdd(uint64_t lhs, uint64_t rhs)
        {
            if (lhs >= kMaximumCost || rhs >= kMaximumCost - lhs)
            {
                return kMaximumCost;
            }
            return lhs + rhs;
        }

        uint64_t saturatedMultiply(uint64_t lhs, uint64_t rhs)
        {
            if (lhs == 0 || rhs == 0)
            {
                return 0;
            }
            if (lhs > kMaximumCost / rhs)
            {
                return kMaximumCost;
            }
            return lhs * rhs;
        }

        uint64_t geometricDistance(const RoutingNode& lhs, const RoutingNode& rhs)
        {
            const int64_t dx = static_cast<int64_t>(lhs.x) - rhs.x;
            const int64_t dy = static_cast<int64_t>(lhs.y) - rhs.y;
            const uint64_t squared = static_cast<uint64_t>(dx * dx + dy * dy);
            uint64_t low = 0;
            uint64_t high = static_cast<uint64_t>(dx < 0 ? -dx : dx) + static_cast<uint64_t>(dy < 0 ? -dy : dy);
            while (low < high)
            {
                const auto middle = low + (high - low) / 2;
                if (middle * middle >= squared)
                {
                    high = middle;
                }
                else
                {
                    low = middle + 1;
                }
            }
            return low;
        }

        std::vector<RoutingNode> canonicalNodes(const RoutingGraph& graph)
        {
            auto nodes = graph.nodes;
            nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const auto& node) {
                            return node.station == StationId::null;
                        }),
                        nodes.end());
            std::sort(nodes.begin(), nodes.end(), [](const auto& lhs, const auto& rhs) {
                return std::make_tuple(stationValue(lhs.station), lhs.x, lhs.y, lhs.supply, lhs.accepts, lhs.attraction)
                    < std::make_tuple(stationValue(rhs.station), rhs.x, rhs.y, rhs.supply, rhs.accepts, rhs.attraction);
            });
            nodes.erase(std::unique(nodes.begin(), nodes.end(), [](const auto& lhs, const auto& rhs) {
                            return lhs.station == rhs.station;
                        }),
                        nodes.end());
            return nodes;
        }

        size_t findNode(const std::vector<RoutingNode>& nodes, StationId station)
        {
            const auto it = std::lower_bound(nodes.begin(), nodes.end(), station, [](const auto& node, StationId value) {
                return stationValue(node.station) < stationValue(value);
            });
            return it != nodes.end() && it->station == station ? static_cast<size_t>(it - nodes.begin()) : kNoIndex;
        }

        bool validServicePoint(const ServicePoint& point)
        {
            return point.service != ServiceId::null && point.occurrence != kNoServiceOccurrence;
        }

        bool occurrenceLess(const Occurrence& lhs, const Occurrence& rhs)
        {
            return std::make_tuple(lhs.station, serviceValue(lhs.point.service), lhs.point.occurrence)
                < std::make_tuple(rhs.station, serviceValue(rhs.point.service), rhs.point.occurrence);
        }

        std::vector<CanonicalRide> canonicalRides(const RoutingGraph& graph, const std::vector<RoutingNode>& nodes)
        {
            std::vector<CanonicalRide> rides;
            rides.reserve(graph.edges.size());
            for (const auto& edge : graph.edges)
            {
                const auto from = findNode(nodes, edge.from);
                const auto to = findNode(nodes, edge.to);
                if (edge.capacity == 0 || from == kNoIndex || to == kNoIndex || from == to)
                {
                    continue;
                }
                const auto legacy = edge.departure.empty() && edge.arrival.empty();
                if (!legacy
                    && (!validServicePoint(edge.departure)
                        || !validServicePoint(edge.arrival)
                        || edge.departure.service != edge.arrival.service))
                {
                    continue;
                }
                rides.push_back({ from, to, edge.capacity, edge.travelTime, edge.departure, edge.arrival, edge.waitTime });
            }
            std::sort(rides.begin(), rides.end(), [](const auto& lhs, const auto& rhs) {
                return std::make_tuple(lhs.from, lhs.to, serviceValue(lhs.departure.service), lhs.departure.occurrence, serviceValue(lhs.arrival.service), lhs.arrival.occurrence, lhs.travelTime, lhs.capacity, lhs.waitTime)
                    < std::make_tuple(rhs.from, rhs.to, serviceValue(rhs.departure.service), rhs.departure.occurrence, serviceValue(rhs.arrival.service), rhs.arrival.occurrence, rhs.travelTime, rhs.capacity, rhs.waitTime);
            });
            return rides;
        }

        size_t findOccurrence(const std::vector<Occurrence>& occurrences, const Occurrence& value)
        {
            const auto it = std::lower_bound(occurrences.begin(), occurrences.end(), value, occurrenceLess);
            return it != occurrences.end() && it->station == value.station && it->point == value.point
                ? static_cast<size_t>(it - occurrences.begin())
                : kNoIndex;
        }

        PlannedGraph makePlannedGraph(const RoutingGraph& graph, const std::vector<RoutingNode>& nodes)
        {
            const auto rides = canonicalRides(graph, nodes);
            std::vector<Occurrence> occurrences;
            std::vector<Occurrence> arrivals;
            occurrences.reserve(rides.size() * 2);
            arrivals.reserve(rides.size());
            for (const auto& ride : rides)
            {
                if (!ride.departure.empty())
                {
                    occurrences.push_back({ ride.from, ride.departure });
                    occurrences.push_back({ ride.to, ride.arrival });
                    arrivals.push_back({ ride.to, ride.arrival });
                }
            }
            std::sort(occurrences.begin(), occurrences.end(), occurrenceLess);
            occurrences.erase(std::unique(occurrences.begin(), occurrences.end(), [](const auto& lhs, const auto& rhs) {
                                  return lhs.station == rhs.station && lhs.point == rhs.point;
                              }),
                              occurrences.end());
            std::sort(arrivals.begin(), arrivals.end(), occurrenceLess);
            arrivals.erase(std::unique(arrivals.begin(), arrivals.end(), [](const auto& lhs, const auto& rhs) {
                               return lhs.station == rhs.station && lhs.point == rhs.point;
                           }),
                           arrivals.end());

            std::vector<size_t> nodeStations;
            nodeStations.reserve(nodes.size() + occurrences.size());
            for (size_t station = 0; station < nodes.size(); ++station)
            {
                nodeStations.push_back(station);
            }
            for (const auto& occurrence : occurrences)
            {
                nodeStations.push_back(occurrence.station);
            }

            std::vector<PlannedEdge> edges;
            edges.reserve(rides.size() * 3);
            for (const auto& ride : rides)
            {
                if (ride.departure.empty())
                {
                    edges.push_back({ ride.from, ride.to, ride.capacity, ride.travelTime, PlannedEdgeKind::ride, {}, {} });
                    continue;
                }

                const auto departure = nodes.size() + findOccurrence(occurrences, { ride.from, ride.departure });
                const auto arrival = nodes.size() + findOccurrence(occurrences, { ride.to, ride.arrival });
                edges.push_back({ ride.from, departure, 0, ride.waitTime, PlannedEdgeKind::board, {}, {} });
                edges.push_back({ departure, arrival, ride.capacity, ride.travelTime, PlannedEdgeKind::ride, ride.departure, ride.arrival });
                edges.push_back({ arrival, ride.to, 0, 0, PlannedEdgeKind::alight, {}, {} });
            }
            std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
                return std::make_tuple(lhs.from, lhs.to, lhs.kind, lhs.travelTime, lhs.capacity, serviceValue(lhs.departure.service), lhs.departure.occurrence, serviceValue(lhs.arrival.service), lhs.arrival.occurrence)
                    < std::make_tuple(rhs.from, rhs.to, rhs.kind, rhs.travelTime, rhs.capacity, serviceValue(rhs.departure.service), rhs.departure.occurrence, serviceValue(rhs.arrival.service), rhs.arrival.occurrence);
            });
            return { std::move(nodeStations), std::move(edges), std::move(occurrences), std::move(arrivals) };
        }

        std::vector<std::vector<size_t>> makeAdjacency(size_t nodeCount, const std::vector<PlannedEdge>& edges)
        {
            std::vector<std::vector<size_t>> adjacency(nodeCount);
            for (size_t edge = 0; edge < edges.size(); ++edge)
            {
                adjacency[edges[edge].from].push_back(edge);
            }
            return adjacency;
        }

        std::vector<bool> reachableNodes(size_t source, const std::vector<PlannedEdge>& edges, const std::vector<std::vector<size_t>>& adjacency)
        {
            std::vector<bool> reachable(adjacency.size());
            std::vector<size_t> pending{ source };
            reachable[source] = true;
            for (size_t current = 0; current < pending.size(); ++current)
            {
                for (const auto edge : adjacency[pending[current]])
                {
                    const auto next = edges[edge].to;
                    if (!reachable[next])
                    {
                        reachable[next] = true;
                        pending.push_back(next);
                    }
                }
            }
            return reachable;
        }

        uint64_t edgeCost(
            const PlannedEdge& edge,
            const std::vector<RoutingNode>& nodes,
            const std::vector<size_t>& nodeStations,
            bool timeSensitive,
            uint8_t saturation)
        {
            if (edge.kind == PlannedEdgeKind::board)
            {
                return timeSensitive ? edge.travelTime : 0;
            }
            if (edge.kind == PlannedEdgeKind::alight)
            {
                return 0;
            }
            const uint64_t base = timeSensitive && edge.travelTime != 0
                ? edge.travelTime
                : geometricDistance(nodes[nodeStations[edge.from]], nodes[nodeStations[edge.to]]);
            const uint64_t saturationPercent = std::min<uint64_t>(saturation, 100);
            const uint64_t threshold = std::max<uint64_t>(1, static_cast<uint64_t>(edge.capacity) * saturationPercent / 100);
            const uint64_t scale = base + 1;
            const uint64_t wholePenalty = saturatedMultiply(scale, edge.flow / threshold);
            const uint64_t fractionalPenalty = saturatedMultiply(scale, edge.flow % threshold) / threshold;
            return saturatedAdd(base, saturatedAdd(wholePenalty, fractionalPenalty));
        }

        const std::vector<size_t>& shortestPath(
            size_t source,
            size_t destination,
            const std::vector<RoutingNode>& nodes,
            const std::vector<size_t>& nodeStations,
            const std::vector<PlannedEdge>& edges,
            const std::vector<std::vector<size_t>>& adjacency,
            bool timeSensitive,
            uint8_t saturation,
            ShortestPathScratch& scratch)
        {
            scratch.reset(nodeStations.size());
            auto& distance = scratch.distance;
            auto& boardings = scratch.boardings;
            auto& previous = scratch.previous;
            auto& settled = scratch.settled;
            auto& queue = scratch.queue;
            auto& path = scratch.path;
            distance[source] = 0;
            boardings[source] = 0;
            queue.emplace(0, 0, source);

            while (!queue.empty())
            {
                const auto [currentDistance, currentBoardings, current] = queue.top();
                queue.pop();
                if (settled[current] || currentDistance != distance[current] || currentBoardings != boardings[current])
                {
                    continue;
                }
                settled[current] = true;
                if (current == destination)
                {
                    break;
                }

                for (const auto edgeIndex : adjacency[current])
                {
                    const auto& edge = edges[edgeIndex];
                    if (settled[edge.to])
                    {
                        continue;
                    }
                    const auto candidate = saturatedAdd(currentDistance, edgeCost(edge, nodes, nodeStations, timeSensitive, saturation));
                    const auto candidateBoardings = currentBoardings + (edge.kind == PlannedEdgeKind::board ? 1 : 0);
                    const bool preferredTie = candidate == distance[edge.to] && candidateBoardings == boardings[edge.to]
                        && (previous[edge.to] == kNoIndex
                            || std::tie(edge.from, edgeIndex) < std::tie(edges[previous[edge.to]].from, previous[edge.to]));
                    if (std::tie(candidate, candidateBoardings) < std::tie(distance[edge.to], boardings[edge.to]) || preferredTie)
                    {
                        distance[edge.to] = candidate;
                        boardings[edge.to] = candidateBoardings;
                        previous[edge.to] = edgeIndex;
                        queue.emplace(candidate, candidateBoardings, edge.to);
                    }
                }
            }

            if (previous[destination] == kNoIndex)
            {
                return path;
            }
            for (auto current = destination; current != source;)
            {
                const auto edge = previous[current];
                if (edge == kNoIndex || path.size() == nodeStations.size())
                {
                    path.clear();
                    return path;
                }
                path.push_back(edge);
                current = edges[edge].from;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<SinkAllocation> allocateSupply(
            const RoutingNode& source,
            uint32_t supply,
            const std::vector<size_t>& sinks,
            const std::vector<RoutingNode>& nodes,
            uint8_t distanceEffect)
        {
            constexpr uint64_t kAttractionScale = uint64_t{ 1 } << 32;
            std::vector<uint64_t> weights;
            weights.reserve(sinks.size());
            uint64_t totalWeight = 0;
            uint64_t maximumAttraction = 1;
            for (const auto sink : sinks)
            {
                maximumAttraction = std::max<uint64_t>(maximumAttraction, nodes[sink].attraction);
            }
            for (const auto sink : sinks)
            {
                uint64_t distanceWeight = kAttractionScale;
                if (distanceEffect != 0)
                {
                    const auto distance = std::max<uint64_t>(1, geometricDistance(source, nodes[sink]));
                    const auto denominator = uint64_t{ 100 } + static_cast<uint64_t>(distanceEffect) * distance;
                    distanceWeight = std::max<uint64_t>(1, kAttractionScale / denominator);
                }
                const auto attraction = std::max<uint64_t>(1, nodes[sink].attraction);
                const auto weight = std::max<uint64_t>(1, saturatedMultiply(distanceWeight, attraction) / maximumAttraction);
                weights.push_back(weight);
                totalWeight += weight;
            }

            std::vector<SinkAllocation> allocations;
            allocations.reserve(sinks.size());
            uint64_t allocated = 0;
            for (size_t i = 0; i < sinks.size(); ++i)
            {
                const uint64_t weightedSupply = static_cast<uint64_t>(supply) * weights[i];
                const auto amount = static_cast<uint32_t>(weightedSupply / totalWeight);
                allocations.push_back({ sinks[i], amount, weightedSupply % totalWeight });
                allocated += amount;
            }

            std::vector<size_t> remainderOrder(allocations.size());
            for (size_t i = 0; i < remainderOrder.size(); ++i)
            {
                remainderOrder[i] = i;
            }
            std::sort(remainderOrder.begin(), remainderOrder.end(), [&](size_t lhs, size_t rhs) {
                if (allocations[lhs].remainder != allocations[rhs].remainder)
                {
                    return allocations[lhs].remainder > allocations[rhs].remainder;
                }
                return stationValue(nodes[allocations[lhs].node].station) < stationValue(nodes[allocations[rhs].node].station);
            });
            const auto remaining = static_cast<size_t>(static_cast<uint64_t>(supply) - allocated);
            for (size_t i = 0; i < remaining; ++i)
            {
                ++allocations[remainderOrder[i]].amount;
            }
            return allocations;
        }
    }

    std::vector<FlowShare> calculateAsymmetricFlows(const RoutingGraph& graph, const RoutingSettings& settings)
    {
        const auto nodes = canonicalNodes(graph);
        auto planned = makePlannedGraph(graph, nodes);
        auto& edges = planned.edges;
        const auto adjacency = makeAdjacency(planned.nodeStations.size(), edges);
        ShortestPathScratch shortestPathScratch;
        using ShareKey = std::tuple<StationId, StationId, StationId, ServicePoint, ServicePoint, ServicePoint>;
        std::map<ShareKey, uint64_t> shares;
        std::map<std::tuple<size_t, StationId, ServicePoint>, uint64_t> demands;

        const auto addShare = [&shares](ShareKey key, uint32_t amount) {
            auto& total = shares[std::move(key)];
            total = saturatedAdd(total, amount);
        };

        for (size_t source = 0; source < nodes.size(); ++source)
        {
            if (nodes[source].supply != 0)
            {
                auto& amount = demands[{ source, nodes[source].station, {} }];
                amount = saturatedAdd(amount, nodes[source].supply);
            }
        }
        for (const auto& demand : graph.demands)
        {
            const auto source = findNode(nodes, demand.source);
            if (source != kNoIndex && demand.origin != StationId::null && demand.amount != 0)
            {
                auto& amount = demands[{ source, demand.origin, demand.incoming }];
                amount = saturatedAdd(amount, demand.amount);
            }
        }

        for (const auto& [demandKey, demandAmount] : demands)
        {
            const auto& [sourceIndex, origin, initialIncoming] = demandKey;
            const auto supply = static_cast<uint32_t>(std::min<uint64_t>(demandAmount, std::numeric_limits<uint32_t>::max()));
            auto sourceNode = sourceIndex;
            if (!initialIncoming.empty())
            {
                const Occurrence incoming{ sourceIndex, initialIncoming };
                if (findOccurrence(planned.arrivals, incoming) != kNoIndex)
                {
                    sourceNode = nodes.size() + findOccurrence(planned.occurrences, incoming);
                }
            }

            const auto reachable = reachableNodes(sourceNode, edges, adjacency);
            std::vector<size_t> sinks;
            for (size_t sink = 0; sink < nodes.size(); ++sink)
            {
                const auto isLocalConsumption = sink == sourceIndex && nodes[sourceIndex].station != origin;
                if ((sink != sourceIndex || isLocalConsumption) && reachable[sink] && nodes[sink].accepts)
                {
                    sinks.push_back(sink);
                }
            }
            if (sinks.empty())
            {
                continue;
            }

            const auto allocations = allocateSupply(nodes[sourceIndex], supply, sinks, nodes, settings.distanceEffect);
            const uint32_t accuracy = std::max<uint32_t>(1, settings.accuracy);
            const uint32_t chunkSize = supply / accuracy + (supply % accuracy != 0);
            for (const auto& allocation : allocations)
            {
                if (allocation.amount == 0)
                {
                    continue;
                }
                if (allocation.node == sourceNode)
                {
                    const auto destination = nodes[allocation.node].station;
                    addShare(ShareKey{ destination, origin, destination, initialIncoming, {}, {} }, allocation.amount);
                    continue;
                }
                for (uint32_t remaining = allocation.amount; remaining != 0;)
                {
                    const auto chunk = std::min(remaining, chunkSize);
                    const auto& path = shortestPath(sourceNode, allocation.node, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, shortestPathScratch);
                    if (path.empty())
                    {
                        break;
                    }
                    auto incoming = initialIncoming;
                    for (const auto edgeIndex : path)
                    {
                        auto& edge = edges[edgeIndex];
                        if (edge.kind != PlannedEdgeKind::ride)
                        {
                            continue;
                        }
                        edge.flow = saturatedAdd(edge.flow, chunk);
                        const auto station = nodes[planned.nodeStations[edge.from]].station;
                        const auto via = nodes[planned.nodeStations[edge.to]].station;
                        addShare(ShareKey{ station, origin, via, incoming, edge.departure, edge.arrival }, chunk);
                        incoming = edge.arrival;
                    }
                    const auto destination = nodes[allocation.node].station;
                    addShare(ShareKey{ destination, origin, destination, incoming, {}, {} }, chunk);
                    remaining -= chunk;
                }
            }
        }

        std::vector<FlowShare> result;
        result.reserve(shares.size());
        using FlowKey = std::tuple<StationId, StationId, ServicePoint>;
        const auto flowKey = [](const ShareKey& key) {
            return FlowKey{ std::get<0>(key), std::get<1>(key), std::get<3>(key) };
        };
        std::map<FlowKey, uint64_t> flowTotals;
        for (const auto& [key, amount] : shares)
        {
            auto& total = flowTotals[flowKey(key)];
            total = saturatedAdd(total, amount);
        }

        std::map<ShareKey, uint32_t> representableShares;
        std::map<FlowKey, uint64_t> representableTotals;
        constexpr auto kMaximumFlowWeight = std::numeric_limits<uint32_t>::max();
        for (const auto& [key, amount] : shares)
        {
            const auto group = flowKey(key);
            const auto total = flowTotals.at(group);
            const auto divisor = total > kMaximumFlowWeight ? total / kMaximumFlowWeight + (total % kMaximumFlowWeight != 0) : 1;
            const auto representableAmount = static_cast<uint32_t>(std::max<uint64_t>(1, amount / divisor));
            representableShares.emplace(key, representableAmount);
            representableTotals[group] += representableAmount;
        }
        for (auto& [key, amount] : representableShares)
        {
            auto& total = representableTotals.at(flowKey(key));
            if (total > kMaximumFlowWeight && amount > 1)
            {
                const auto reduction = std::min<uint64_t>(amount - 1, total - kMaximumFlowWeight);
                amount -= static_cast<uint32_t>(reduction);
                total -= reduction;
            }
            result.push_back({ std::get<0>(key), std::get<1>(key), std::get<2>(key), amount, std::get<3>(key), std::get<4>(key), std::get<5>(key) });
        }
        return result;
    }
}
