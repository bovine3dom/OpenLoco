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

        struct PlannedEdge
        {
            size_t from;
            size_t to;
            uint32_t capacity;
            uint32_t travelTime;
            uint64_t flow = 0;
        };

        struct SinkAllocation
        {
            size_t node;
            uint32_t amount;
            uint64_t remainder;
        };

        using ShortestPathQueueEntry = std::pair<uint64_t, size_t>;

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
                previous.assign(nodeCount, kNoIndex);
                settled.assign(nodeCount, false);
                queue.clear();
                queue.reserve(nodeCount);
                path.clear();
                path.reserve(nodeCount);
            }

            std::vector<uint64_t> distance;
            std::vector<size_t> previous;
            std::vector<bool> settled;
            ShortestPathQueue queue;
            std::vector<size_t> path;
        };

        constexpr uint16_t stationValue(StationId station)
        {
            return static_cast<uint16_t>(station);
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

        std::vector<PlannedEdge> canonicalEdges(const RoutingGraph& graph, const std::vector<RoutingNode>& nodes)
        {
            std::vector<PlannedEdge> edges;
            edges.reserve(graph.edges.size());
            for (const auto& edge : graph.edges)
            {
                const auto from = findNode(nodes, edge.from);
                const auto to = findNode(nodes, edge.to);
                if (edge.capacity == 0 || from == kNoIndex || to == kNoIndex || from == to)
                {
                    continue;
                }
                edges.push_back({ from, to, edge.capacity, edge.travelTime });
            }
            std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
                return std::tie(lhs.from, lhs.to, lhs.travelTime, lhs.capacity)
                    < std::tie(rhs.from, rhs.to, rhs.travelTime, rhs.capacity);
            });
            return edges;
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

        uint64_t edgeCost(const PlannedEdge& edge, const std::vector<RoutingNode>& nodes, bool timeSensitive, uint8_t saturation)
        {
            const uint64_t base = timeSensitive && edge.travelTime != 0
                ? edge.travelTime
                : geometricDistance(nodes[edge.from], nodes[edge.to]);
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
            const std::vector<PlannedEdge>& edges,
            const std::vector<std::vector<size_t>>& adjacency,
            bool timeSensitive,
            uint8_t saturation,
            ShortestPathScratch& scratch)
        {
            scratch.reset(nodes.size());
            auto& distance = scratch.distance;
            auto& previous = scratch.previous;
            auto& settled = scratch.settled;
            auto& queue = scratch.queue;
            auto& path = scratch.path;
            distance[source] = 0;
            queue.emplace(0, source);

            while (!queue.empty())
            {
                const auto [currentDistance, current] = queue.top();
                queue.pop();
                if (settled[current] || currentDistance != distance[current])
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
                    const auto candidate = saturatedAdd(currentDistance, edgeCost(edge, nodes, timeSensitive, saturation));
                    const bool preferredTie = candidate == distance[edge.to]
                        && (previous[edge.to] == kNoIndex
                            || std::tie(edge.from, edgeIndex) < std::tie(edges[previous[edge.to]].from, previous[edge.to]));
                    if (candidate < distance[edge.to] || preferredTie)
                    {
                        distance[edge.to] = candidate;
                        previous[edge.to] = edgeIndex;
                        queue.emplace(candidate, edge.to);
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
                if (edge == kNoIndex || path.size() == nodes.size())
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
        auto edges = canonicalEdges(graph, nodes);
        const auto adjacency = makeAdjacency(nodes.size(), edges);
        ShortestPathScratch shortestPathScratch;
        std::map<std::tuple<StationId, StationId, StationId>, uint64_t> shares;
        std::map<std::pair<size_t, StationId>, uint64_t> demands;

        for (size_t source = 0; source < nodes.size(); ++source)
        {
            if (nodes[source].supply != 0)
            {
                demands[{ source, nodes[source].station }] += nodes[source].supply;
            }
        }
        for (const auto& demand : graph.demands)
        {
            const auto source = findNode(nodes, demand.source);
            if (source != kNoIndex && demand.origin != StationId::null && demand.amount != 0)
            {
                demands[{ source, demand.origin }] += demand.amount;
            }
        }

        for (const auto& [demandKey, demandAmount] : demands)
        {
            const auto [sourceIndex, origin] = demandKey;
            const auto supply = static_cast<uint32_t>(std::min<uint64_t>(demandAmount, std::numeric_limits<uint32_t>::max()));
            const auto reachable = reachableNodes(sourceIndex, edges, adjacency);
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
                if (allocation.node == sourceIndex)
                {
                    shares[{ nodes[sourceIndex].station, origin, nodes[sourceIndex].station }] += allocation.amount;
                    continue;
                }
                for (uint32_t remaining = allocation.amount; remaining != 0;)
                {
                    const auto chunk = std::min(remaining, chunkSize);
                    const auto& path = shortestPath(sourceIndex, allocation.node, nodes, edges, adjacency, graph.timeSensitive, settings.saturation, shortestPathScratch);
                    if (path.empty())
                    {
                        break;
                    }
                    for (const auto edgeIndex : path)
                    {
                        auto& edge = edges[edgeIndex];
                        edge.flow = saturatedAdd(edge.flow, chunk);
                        shares[{ nodes[edge.from].station, origin, nodes[edge.to].station }] += chunk;
                    }
                    const auto destination = nodes[allocation.node].station;
                    shares[{ destination, origin, destination }] += chunk;
                    remaining -= chunk;
                }
            }
        }

        std::vector<FlowShare> result;
        result.reserve(shares.size());
        for (const auto& [key, amount] : shares)
        {
            if (amount != 0)
            {
                result.push_back({ std::get<0>(key), std::get<1>(key), std::get<2>(key), static_cast<uint32_t>(amount) });
            }
        }
        return result;
    }
}
