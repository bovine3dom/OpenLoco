// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Routing.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>
#if defined(_MSC_VER) && defined(_M_X64)
#include <immintrin.h>
#include <intrin.h>
#endif

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
            uint32_t headway;
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
            uint32_t headway{};
            size_t pairedRide = kNoIndex;
        };

        struct PlannedGraph
        {
            std::vector<size_t> nodeStations;
            std::vector<PlannedEdge> edges;
            std::vector<Occurrence> occurrences;
            std::vector<Occurrence> arrivals;
            std::vector<std::vector<size_t>> ridePredecessors;
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

        struct PreparedDemand
        {
            uint32_t amount;
            std::vector<size_t> sinks;
            std::vector<size_t> passengerSinks;
            std::vector<uint64_t> costs;
            size_t sourceNode;
            size_t visitedNode;
            StationId origin;
            ServicePoint incoming;
        };

        struct PassengerPair
        {
            size_t first;
            size_t second;
            int64_t credit{};
            uint64_t weight{};
            bool reverseFirst{};
        };

        struct PassengerSink
        {
            size_t demand;
            size_t destination;
            int64_t credit{};
            uint64_t weight{};
        };

        class CapacityNetwork
        {
        public:
            struct Edge
            {
                size_t to;
                size_t reverse;
                uint64_t capacity;
            };

            explicit CapacityNetwork(size_t size)
                : _edges(size)
            {
            }

            void addEdge(size_t from, size_t to, uint64_t capacity)
            {
                const auto forward = _edges[from].size();
                const auto reverse = _edges[to].size();
                _edges[from].push_back({ to, reverse, capacity });
                _edges[to].push_back({ from, forward, 0 });
            }

            uint64_t maximumFlow(size_t source, size_t destination)
            {
                uint64_t total = 0;
                while (buildLevels(source, destination))
                {
                    _next.assign(_edges.size(), 0);
                    while (const auto amount = send(source, destination, std::numeric_limits<uint64_t>::max()))
                    {
                        total += amount;
                    }
                }
                return total;
            }

            std::vector<bool> residualReachable(size_t source) const
            {
                std::vector<bool> reachable(_edges.size());
                std::vector<size_t> pending{ source };
                reachable[source] = true;
                for (size_t i = 0; i < pending.size(); ++i)
                {
                    for (const auto& edge : _edges[pending[i]])
                    {
                        if (edge.capacity != 0 && !reachable[edge.to])
                        {
                            reachable[edge.to] = true;
                            pending.push_back(edge.to);
                        }
                    }
                }
                return reachable;
            }

        private:
            bool buildLevels(size_t source, size_t destination)
            {
                _level.assign(_edges.size(), -1);
                std::queue<size_t> pending;
                pending.push(source);
                _level[source] = 0;
                while (!pending.empty())
                {
                    const auto current = pending.front();
                    pending.pop();
                    for (const auto& edge : _edges[current])
                    {
                        if (edge.capacity != 0 && _level[edge.to] == -1)
                        {
                            _level[edge.to] = _level[current] + 1;
                            pending.push(edge.to);
                        }
                    }
                }
                return _level[destination] != -1;
            }

            uint64_t send(size_t current, size_t destination, uint64_t available)
            {
                if (current == destination)
                {
                    return available;
                }
                for (auto& next = _next[current]; next < _edges[current].size(); ++next)
                {
                    auto& edge = _edges[current][next];
                    if (edge.capacity == 0 || _level[edge.to] != _level[current] + 1)
                    {
                        continue;
                    }
                    const auto amount = send(edge.to, destination, std::min(available, edge.capacity));
                    if (amount != 0)
                    {
                        edge.capacity -= amount;
                        _edges[edge.to][edge.reverse].capacity += amount;
                        return amount;
                    }
                }
                return 0;
            }

            std::vector<std::vector<Edge>> _edges;
            std::vector<int32_t> _level;
            std::vector<size_t> _next;
        };

        class CostNetwork
        {
        public:
            struct Edge
            {
                size_t to;
                size_t reverse;
                uint64_t capacity;
                int64_t cost;
            };

            struct EdgeReference
            {
                size_t from;
                size_t edge;
            };

            explicit CostNetwork(size_t size)
                : _edges(size)
            {
            }

            EdgeReference addEdge(size_t from, size_t to, uint64_t capacity, int64_t cost)
            {
                const auto forward = _edges[from].size();
                const auto reverse = _edges[to].size();
                _edges[from].push_back({ to, reverse, capacity, cost });
                _edges[to].push_back({ from, forward, 0, -cost });
                return { from, forward };
            }

            uint64_t minimumCostFlow(size_t source, size_t destination, uint64_t required)
            {
                constexpr auto kInfinity = std::numeric_limits<int64_t>::max();
                uint64_t total = 0;
                std::vector<int64_t> distance(_edges.size());
                std::vector<size_t> previousNode(_edges.size());
                std::vector<size_t> previousEdge(_edges.size());
                std::vector<bool> queued(_edges.size());
                while (total != required)
                {
                    std::fill(distance.begin(), distance.end(), kInfinity);
                    std::fill(previousNode.begin(), previousNode.end(), kNoIndex);
                    std::fill(previousEdge.begin(), previousEdge.end(), kNoIndex);
                    std::fill(queued.begin(), queued.end(), false);
                    std::queue<size_t> pending;
                    pending.push(source);
                    queued[source] = true;
                    distance[source] = 0;
                    while (!pending.empty())
                    {
                        const auto current = pending.front();
                        pending.pop();
                        queued[current] = false;
                        for (size_t edgeIndex = 0; edgeIndex < _edges[current].size(); ++edgeIndex)
                        {
                            const auto& edge = _edges[current][edgeIndex];
                            if (edge.capacity == 0)
                            {
                                continue;
                            }
                            const auto candidate = addCost(distance[current], edge.cost);
                            if (candidate < distance[edge.to])
                            {
                                distance[edge.to] = candidate;
                                previousNode[edge.to] = current;
                                previousEdge[edge.to] = edgeIndex;
                                if (!queued[edge.to])
                                {
                                    pending.push(edge.to);
                                    queued[edge.to] = true;
                                }
                            }
                        }
                    }
                    if (previousNode[destination] == kNoIndex)
                    {
                        break;
                    }

                    auto amount = required - total;
                    for (auto current = destination; current != source; current = previousNode[current])
                    {
                        amount = std::min(amount, _edges[previousNode[current]][previousEdge[current]].capacity);
                    }
                    for (auto current = destination; current != source; current = previousNode[current])
                    {
                        auto& edge = _edges[previousNode[current]][previousEdge[current]];
                        edge.capacity -= amount;
                        _edges[edge.to][edge.reverse].capacity += amount;
                    }
                    total += amount;
                }
                return total;
            }

            uint64_t flow(EdgeReference reference) const
            {
                const auto& edge = _edges[reference.from][reference.edge];
                return _edges[edge.to][edge.reverse].capacity;
            }

        private:
            static int64_t addCost(int64_t lhs, int64_t rhs)
            {
                if (rhs > 0 && lhs > std::numeric_limits<int64_t>::max() - rhs)
                {
                    return std::numeric_limits<int64_t>::max();
                }
                if (rhs < 0 && lhs < std::numeric_limits<int64_t>::min() - rhs)
                {
                    return std::numeric_limits<int64_t>::min();
                }
                return lhs + rhs;
            }

            std::vector<std::vector<Edge>> _edges;
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

        uint64_t scalePercent(uint64_t value, uint8_t percent)
        {
            return saturatedAdd(saturatedMultiply(value / 100, percent), (value % 100) * percent / 100);
        }

        uint64_t ratioQ16(uint64_t numerator, uint64_t denominator)
        {
            constexpr uint64_t kScale = uint64_t{ 1 } << 16;
            if (numerator >= denominator)
            {
                return kScale;
            }

            uint64_t result = 0;
            auto remainder = numerator;
            for (uint8_t bit = 0; bit < 16; ++bit)
            {
                result *= 2;
                if (remainder >= denominator - remainder)
                {
                    remainder -= denominator - remainder;
                    ++result;
                }
                else
                {
                    remainder *= 2;
                }
            }
            return result;
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
                rides.push_back({ from, to, edge.capacity, edge.travelTime, edge.departure, edge.arrival, edge.waitTime, edge.headway });
            }
            std::sort(rides.begin(), rides.end(), [](const auto& lhs, const auto& rhs) {
                return std::make_tuple(lhs.from, lhs.to, serviceValue(lhs.departure.service), lhs.departure.occurrence, serviceValue(lhs.arrival.service), lhs.arrival.occurrence, lhs.travelTime, lhs.capacity, lhs.waitTime, lhs.headway)
                    < std::make_tuple(rhs.from, rhs.to, serviceValue(rhs.departure.service), rhs.departure.occurrence, serviceValue(rhs.arrival.service), rhs.arrival.occurrence, rhs.travelTime, rhs.capacity, rhs.waitTime, rhs.headway);
            });
            std::map<std::pair<size_t, ServicePoint>, size_t> departures;
            std::erase_if(rides, [&](const auto& ride) {
                return !ride.departure.empty() && !departures.emplace(std::pair{ ride.from, ride.departure }, ride.to).second;
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
                edges.push_back({ ride.from, departure, ride.capacity, ride.waitTime, PlannedEdgeKind::board, {}, {}, 0, ride.headway });
                edges.push_back({ departure, arrival, ride.capacity, ride.travelTime, PlannedEdgeKind::ride, ride.departure, ride.arrival });
                edges.push_back({ arrival, ride.to, 0, 0, PlannedEdgeKind::alight, {}, {} });
            }
            std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
                return std::make_tuple(lhs.from, lhs.to, lhs.kind, lhs.travelTime, lhs.capacity, serviceValue(lhs.departure.service), lhs.departure.occurrence, serviceValue(lhs.arrival.service), lhs.arrival.occurrence)
                    < std::make_tuple(rhs.from, rhs.to, rhs.kind, rhs.travelTime, rhs.capacity, serviceValue(rhs.departure.service), rhs.departure.occurrence, serviceValue(rhs.arrival.service), rhs.arrival.occurrence);
            });
            std::map<size_t, size_t> ridesByDeparture;
            std::vector<std::vector<size_t>> ridePredecessors(nodeStations.size());
            for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
            {
                const auto& edge = edges[edgeIndex];
                if (edge.kind == PlannedEdgeKind::ride && !edge.departure.empty())
                {
                    ridesByDeparture.emplace(edge.from, edgeIndex);
                    ridePredecessors[edge.to].push_back(edge.from);
                }
            }
            for (auto& edge : edges)
            {
                if (edge.kind != PlannedEdgeKind::board)
                {
                    continue;
                }
                edge.pairedRide = ridesByDeparture.at(edge.to);
            }
            return { std::move(nodeStations), std::move(edges), std::move(occurrences), std::move(arrivals), std::move(ridePredecessors) };
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

        std::vector<bool> reachableNodes(size_t source, const std::vector<PlannedEdge>& edges, const std::vector<std::vector<size_t>>& adjacency, size_t visitedNode)
        {
            std::vector<bool> reachable(adjacency.size());
            std::vector<size_t> pending{ source };
            reachable[source] = true;
            for (size_t current = 0; current < pending.size(); ++current)
            {
                for (const auto edge : adjacency[pending[current]])
                {
                    const auto next = edges[edge].to;
                    if (!reachable[next] && next != visitedNode)
                    {
                        reachable[next] = true;
                        pending.push_back(next);
                    }
                }
            }
            return reachable;
        }

        uint64_t edgeCost(
            size_t edgeIndex,
            const std::vector<PlannedEdge>& edges,
            const std::vector<RoutingNode>& nodes,
            const std::vector<size_t>& nodeStations,
            bool timeSensitive,
            uint8_t saturation)
        {
            const auto& edge = edges[edgeIndex];
            if (edge.kind == PlannedEdgeKind::board)
            {
                if (!timeSensitive)
                {
                    return 0;
                }
                const auto& ride = edges[edge.pairedRide];
                const auto queuedDepartures = ride.flow / std::max<uint32_t>(1, edge.capacity);
                return saturatedAdd(edge.travelTime, saturatedMultiply(edge.headway, queuedDepartures));
            }
            if (edge.kind == PlannedEdgeKind::alight)
            {
                return 0;
            }
            const uint64_t base = timeSensitive && edge.travelTime != 0
                ? edge.travelTime
                : geometricDistance(nodes[nodeStations[edge.from]], nodes[nodeStations[edge.to]]);
            if (timeSensitive && !edge.departure.empty())
            {
                return base;
            }
            const uint64_t saturationPercent = std::min<uint64_t>(saturation, 100);
            const uint64_t threshold = std::max<uint64_t>(1, static_cast<uint64_t>(edge.capacity) * saturationPercent / 100);
            const uint64_t scale = base + 1;
            const uint64_t wholePenalty = saturatedMultiply(scale, edge.flow / threshold);
            const uint64_t fractionalPenalty = saturatedMultiply(scale, edge.flow % threshold) / threshold;
            return saturatedAdd(base, saturatedAdd(wholePenalty, fractionalPenalty));
        }

        const std::vector<size_t>& reconstructPath(
            size_t source,
            size_t destination,
            const std::vector<size_t>& nodeStations,
            const std::vector<PlannedEdge>& edges,
            ShortestPathScratch& scratch)
        {
            auto& path = scratch.path;
            path.clear();
            if (destination == kNoIndex || destination == source || scratch.previous[destination] == kNoIndex)
            {
                return path;
            }
            for (auto current = destination; current != source;)
            {
                const auto edge = scratch.previous[current];
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

        const std::vector<size_t>& shortestPath(
            size_t source,
            size_t destination,
            const std::vector<RoutingNode>& nodes,
            const std::vector<size_t>& nodeStations,
            const std::vector<PlannedEdge>& edges,
            const std::vector<std::vector<size_t>>& adjacency,
            bool timeSensitive,
            uint8_t saturation,
            size_t visitedNode,
            size_t forbiddenReturnStation,
            ShortestPathScratch& scratch)
        {
            scratch.reset(nodeStations.size());
            auto& distance = scratch.distance;
            auto& boardings = scratch.boardings;
            auto& previous = scratch.previous;
            auto& settled = scratch.settled;
            auto& queue = scratch.queue;
            if (visitedNode != kNoIndex)
            {
                settled[visitedNode] = true;
            }
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
                    if (settled[edge.to]
                        || (forbiddenReturnStation != kNoIndex
                            && nodeStations[current] != forbiddenReturnStation
                            && nodeStations[edge.to] == forbiddenReturnStation))
                    {
                        continue;
                    }
                    const auto candidate = saturatedAdd(currentDistance, edgeCost(edgeIndex, edges, nodes, nodeStations, timeSensitive, saturation));
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

            return reconstructPath(source, destination, nodeStations, edges, scratch);
        }

        std::vector<uint64_t> destinationWeights(
            const std::vector<size_t>& sinks,
            const std::vector<RoutingNode>& nodes,
            const std::vector<uint64_t>& journeyCosts,
            uint8_t distanceEffect)
        {
            constexpr uint64_t kMaximumWeight = uint64_t{ 1 } << 32;
            uint64_t minimumCost = kMaximumCost;
            for (const auto sink : sinks)
            {
                minimumCost = std::min(minimumCost, std::max<uint64_t>(1, journeyCosts[sink]));
            }

            std::vector<uint64_t> weights;
            weights.reserve(sinks.size());
            uint64_t maximumScore = 1;
            for (const auto sink : sinks)
            {
                const auto cost = std::max<uint64_t>(1, journeyCosts[sink]);
                const auto effectiveCost = saturatedAdd(minimumCost, scalePercent(cost - minimumCost, distanceEffect));
                const auto proximity = ratioQ16(minimumCost, effectiveCost);
                const auto score = saturatedMultiply(proximity * proximity, std::max<uint64_t>(1, nodes[sink].attraction));
                weights.push_back(score);
                maximumScore = std::max(maximumScore, score);
            }
            const auto divisor = maximumScore / kMaximumWeight + (maximumScore % kMaximumWeight != 0);
            for (auto& weight : weights)
            {
                weight = std::max<uint64_t>(1, weight / divisor);
            }
            return weights;
        }

        struct MultiplyDivideResult
        {
            uint64_t quotient;
            uint64_t remainder;
        };

        MultiplyDivideResult proportionalShare(uint64_t lhs, uint64_t rhs, uint64_t divisor)
        {
            assert(lhs < divisor && rhs <= divisor);
            if (rhs == 0)
            {
                return {};
            }
            uint64_t quotient = 0;
            uint64_t remainder = 0;
            int8_t bit = 63;
            while ((rhs & (uint64_t{ 1 } << bit)) == 0)
            {
                --bit;
            }
            for (; bit >= 0; --bit)
            {
                uint8_t digit = 0;
                if (remainder >= divisor - remainder)
                {
                    remainder -= divisor - remainder;
                    ++digit;
                }
                else
                {
                    remainder *= 2;
                }
                if ((rhs & (uint64_t{ 1 } << bit)) != 0)
                {
                    if (remainder >= divisor - lhs)
                    {
                        remainder -= divisor - lhs;
                        ++digit;
                    }
                    else
                    {
                        remainder += lhs;
                    }
                }
                quotient = quotient * 2 + digit;
            }
            return { quotient, remainder };
        }

        uint64_t scaleRatio(uint64_t value, uint64_t numerator, uint64_t denominator)
        {
            assert(denominator != 0 && numerator <= denominator);
            if (numerator == denominator)
            {
                return value;
            }
#if defined(_MSC_VER) && defined(_M_X64)
            uint64_t high;
            const auto low = _umul128(value, numerator, &high);
            uint64_t remainder;
            return _udiv128(high, low, denominator, &remainder);
#elif defined(__SIZEOF_INT128__)
            return static_cast<uint64_t>(static_cast<__uint128_t>(value) * numerator / denominator);
#else
            if (value == 0 || numerator == 0)
            {
                return 0;
            }
            if (value <= denominator)
            {
                return proportionalShare(numerator, value, denominator).quotient;
            }
            const auto whole = value / denominator * numerator;
            return whole + proportionalShare(value % denominator, numerator, denominator).quotient;
#endif
        }

        void updatePassengerPairWeights(
            std::vector<PassengerPair>& pairs,
            const std::vector<PreparedDemand>& demands,
            const std::vector<RoutingNode>& nodes,
            const std::vector<std::vector<uint64_t>>& journeyCosts,
            const std::vector<uint32_t>& budgets,
            const std::vector<size_t>& componentOf,
            uint8_t distanceEffect)
        {
            constexpr uint64_t kMaximumWeight = uint64_t{ 1 } << 32;
            std::vector<uint64_t> costs(pairs.size(), kUnreachableJourneyCost);
            std::vector<uint64_t> attractions(pairs.size());
            std::vector<uint64_t> minimumCosts(demands.size(), kMaximumCost);
            std::vector<uint64_t> maximumAttractions(demands.size(), 1);
            for (size_t i = 0; i < pairs.size(); ++i)
            {
                auto& pair = pairs[i];
                pair.weight = 0;
                if (budgets[pair.first] == 0 || budgets[pair.second] == 0)
                {
                    continue;
                }
                const auto firstCost = journeyCosts[pair.first][demands[pair.second].sourceNode];
                const auto secondCost = journeyCosts[pair.second][demands[pair.first].sourceNode];
                if (firstCost == kUnreachableJourneyCost || secondCost == kUnreachableJourneyCost)
                {
                    continue;
                }
                costs[i] = firstCost / 2 + secondCost / 2 + (firstCost % 2 + secondCost % 2 + 1) / 2;
                costs[i] = std::max<uint64_t>(1, costs[i]);
                attractions[i] = std::max<uint64_t>(1, nodes[demands[pair.first].sourceNode].attraction)
                    * std::max<uint64_t>(1, nodes[demands[pair.second].sourceNode].attraction);
                const auto component = componentOf[pair.first];
                minimumCosts[component] = std::min(minimumCosts[component], costs[i]);
                maximumAttractions[component] = std::max(maximumAttractions[component], attractions[i]);
            }

            for (size_t i = 0; i < pairs.size(); ++i)
            {
                auto& pair = pairs[i];
                if (costs[i] == kUnreachableJourneyCost)
                {
                    continue;
                }
                const auto component = componentOf[pair.first];
                const auto minimumCost = minimumCosts[component];
                const auto effectiveCost = saturatedAdd(minimumCost, scalePercent(costs[i] - minimumCost, distanceEffect));
                auto weight = scaleRatio(kMaximumWeight, attractions[i], maximumAttractions[component]);
                weight = scaleRatio(weight, minimumCost, effectiveCost);
                weight = scaleRatio(weight, minimumCost, effectiveCost);
                pair.weight = std::max<uint64_t>(1, weight);
            }
        }

        void updatePassengerSinkWeights(
            std::vector<PassengerSink>& sinks,
            const std::vector<PreparedDemand>& demands,
            const std::vector<RoutingNode>& nodes,
            const std::vector<std::vector<uint64_t>>& journeyCosts,
            const std::vector<uint32_t>& budgets,
            const std::vector<std::vector<size_t>>& demandSinks,
            uint8_t distanceEffect)
        {
            for (size_t demand = 0; demand < demands.size(); ++demand)
            {
                if (budgets[demand] == 0 || demandSinks[demand].empty())
                {
                    continue;
                }
                const auto weights = destinationWeights(demands[demand].passengerSinks, nodes, journeyCosts[demand], distanceEffect);
                for (size_t i = 0; i < demandSinks[demand].size(); ++i)
                {
                    sinks[demandSinks[demand][i]].weight = weights[i];
                }
            }
        }

        std::vector<uint64_t> weightedTargets(uint64_t total, const std::vector<size_t>& sinks, const std::vector<RoutingNode>& nodes)
        {
            uint64_t weightTotal = 0;
            for (const auto sink : sinks)
            {
                weightTotal += std::max<uint64_t>(1, nodes[sink].attraction);
            }

            struct Remainder
            {
                size_t index;
                uint64_t value;
            };
            std::vector<uint64_t> targets(sinks.size());
            std::vector<Remainder> remainders;
            remainders.reserve(sinks.size());
            uint64_t allocated = 0;
            const auto whole = total / weightTotal;
            const auto partial = total % weightTotal;
            for (size_t i = 0; i < sinks.size(); ++i)
            {
                const auto weight = std::max<uint64_t>(1, nodes[sinks[i]].attraction);
                const auto product = proportionalShare(partial, weight, weightTotal);
                targets[i] = whole * weight + product.quotient;
                allocated += targets[i];
                remainders.push_back({ i, product.remainder });
            }
            std::sort(remainders.begin(), remainders.end(), [&](const auto& lhs, const auto& rhs) {
                return lhs.value != rhs.value
                    ? lhs.value > rhs.value
                    : stationValue(nodes[sinks[lhs.index]].station) < stationValue(nodes[sinks[rhs.index]].station);
            });
            for (uint64_t remaining = total - allocated; remaining != 0; --remaining)
            {
                ++targets[remainders[total - allocated - remaining].index];
            }
            return targets;
        }

        std::vector<uint64_t> calculateFairSinkTargets(const std::vector<PreparedDemand>& demands, const std::vector<RoutingNode>& nodes)
        {
            struct Subproblem
            {
                std::vector<size_t> demands;
                std::vector<size_t> sinks;
            };

            std::vector<uint64_t> result(nodes.size());
            if (demands.empty())
            {
                return result;
            }

            Subproblem initial;
            initial.demands.resize(demands.size());
            for (size_t i = 0; i < demands.size(); ++i)
            {
                initial.demands[i] = i;
                initial.sinks.insert(initial.sinks.end(), demands[i].sinks.begin(), demands[i].sinks.end());
            }
            std::sort(initial.sinks.begin(), initial.sinks.end());
            initial.sinks.erase(std::unique(initial.sinks.begin(), initial.sinks.end()), initial.sinks.end());

            std::vector<Subproblem> pending;
            pending.push_back(std::move(initial));
            while (!pending.empty())
            {
                auto problem = std::move(pending.back());
                pending.pop_back();
                uint64_t total = 0;
                for (const auto demand : problem.demands)
                {
                    total += demands[demand].amount;
                }
                const auto targets = weightedTargets(total, problem.sinks, nodes);
                const auto source = size_t{ 0 };
                const auto demandBegin = size_t{ 1 };
                const auto sinkBegin = demandBegin + problem.demands.size();
                const auto destination = sinkBegin + problem.sinks.size();
                CapacityNetwork network(destination + 1);
                std::vector<size_t> sinkPositions(nodes.size(), kNoIndex);
                for (size_t i = 0; i < problem.sinks.size(); ++i)
                {
                    sinkPositions[problem.sinks[i]] = i;
                    network.addEdge(sinkBegin + i, destination, targets[i]);
                }
                for (size_t i = 0; i < problem.demands.size(); ++i)
                {
                    const auto demand = problem.demands[i];
                    network.addEdge(source, demandBegin + i, demands[demand].amount);
                    for (const auto sink : demands[demand].sinks)
                    {
                        if (sinkPositions[sink] != kNoIndex)
                        {
                            network.addEdge(demandBegin + i, sinkBegin + sinkPositions[sink], demands[demand].amount);
                        }
                    }
                }

                if (network.maximumFlow(source, destination) == total)
                {
                    for (size_t i = 0; i < problem.sinks.size(); ++i)
                    {
                        result[problem.sinks[i]] = targets[i];
                    }
                    continue;
                }

                const auto reachable = network.residualReachable(source);
                Subproblem constrained;
                Subproblem remainder;
                for (size_t i = 0; i < problem.demands.size(); ++i)
                {
                    (reachable[demandBegin + i] ? constrained.demands : remainder.demands).push_back(problem.demands[i]);
                }
                for (size_t i = 0; i < problem.sinks.size(); ++i)
                {
                    (reachable[sinkBegin + i] ? constrained.sinks : remainder.sinks).push_back(problem.sinks[i]);
                }

                const bool validPartition = !constrained.demands.empty() && !constrained.sinks.empty()
                    && constrained.demands.size() != problem.demands.size() && constrained.sinks.size() != problem.sinks.size();
                assert(validPartition);
                if (!validPartition)
                {
                    return {};
                }
                pending.push_back(std::move(remainder));
                pending.push_back(std::move(constrained));
            }
            return result;
        }

        std::vector<std::vector<std::pair<size_t, uint32_t>>> assignDemandTargets(
            const std::vector<PreparedDemand>& demands,
            const std::vector<RoutingNode>& nodes,
            const std::vector<uint64_t>& targets,
            uint8_t distanceEffect)
        {
            std::vector<std::vector<std::pair<size_t, uint32_t>>> result(demands.size());
            std::vector<size_t> sinks;
            for (size_t sink = 0; sink < targets.size(); ++sink)
            {
                if (targets[sink] != 0)
                {
                    sinks.push_back(sink);
                }
            }
            if (demands.empty() || sinks.empty())
            {
                return result;
            }

            const auto source = size_t{ 0 };
            const auto demandBegin = size_t{ 1 };
            const auto sinkBegin = demandBegin + demands.size();
            const auto destination = sinkBegin + sinks.size();
            CostNetwork network(destination + 1);
            std::vector<size_t> sinkPositions(nodes.size(), kNoIndex);
            for (size_t i = 0; i < sinks.size(); ++i)
            {
                sinkPositions[sinks[i]] = i;
                network.addEdge(sinkBegin + i, destination, targets[sinks[i]], 0);
            }

            struct AssignmentEdge
            {
                size_t demand;
                size_t sink;
                CostNetwork::EdgeReference edge;
            };
            std::vector<AssignmentEdge> assignments;
            uint64_t total = 0;
            const auto costLimit = std::numeric_limits<int64_t>::max() / std::max<uint64_t>(4, static_cast<uint64_t>(destination + 1) * 4);
            for (size_t demand = 0; demand < demands.size(); ++demand)
            {
                total += demands[demand].amount;
                network.addEdge(source, demandBegin + demand, demands[demand].amount, 0);
                for (size_t i = 0; i < demands[demand].sinks.size(); ++i)
                {
                    const auto sink = demands[demand].sinks[i];
                    if (sinkPositions[sink] == kNoIndex)
                    {
                        continue;
                    }
                    const auto scaledCost = scalePercent(demands[demand].costs[i], distanceEffect);
                    const auto cost = static_cast<int64_t>(std::min<uint64_t>(scaledCost, costLimit));
                    assignments.push_back({ demand, sink, network.addEdge(demandBegin + demand, sinkBegin + sinkPositions[sink], demands[demand].amount, cost) });
                }
            }
            const auto assigned = network.minimumCostFlow(source, destination, total);
            assert(assigned == total);
            if (assigned != total)
            {
                return {};
            }
            for (const auto& assignment : assignments)
            {
                const auto amount = network.flow(assignment.edge);
                if (amount != 0)
                {
                    result[assignment.demand].emplace_back(assignment.sink, static_cast<uint32_t>(amount));
                }
            }
            return result;
        }
    }

    std::vector<FlowShare> calculateAsymmetricFlows(const RoutingGraph& graph, const RoutingSettings& settings)
    {
        const auto nodes = canonicalNodes(graph);
        auto planned = makePlannedGraph(graph, nodes);
        auto& edges = planned.edges;
        const auto adjacency = makeAdjacency(planned.nodeStations.size(), edges);
        ShortestPathScratch shortestPathScratch;
        using ShareKey = std::tuple<StationId, StationId, StationId, ServicePoint, ServicePoint, ServicePoint, StationId>;
        std::map<ShareKey, uint64_t> shares;
        std::map<std::tuple<bool, size_t, StationId, ServicePoint, StationId>, uint64_t> demands;

        const auto addShare = [&shares](ShareKey key, uint32_t amount) {
            auto& total = shares[std::move(key)];
            total = saturatedAdd(total, amount);
        };

        for (size_t source = 0; source < nodes.size(); ++source)
        {
            if (nodes[source].supply != 0)
            {
                auto& amount = demands[{ true, source, nodes[source].station, {}, StationId::null }];
                amount = saturatedAdd(amount, nodes[source].supply);
            }
        }
        for (const auto& demand : graph.demands)
        {
            const auto source = findNode(nodes, demand.source);
            if (source != kNoIndex && demand.origin != StationId::null && demand.amount != 0)
            {
                auto& amount = demands[{ demand.incoming.empty(), source, demand.origin, demand.incoming, demand.destination }];
                amount = saturatedAdd(amount, demand.amount);
            }
        }

        std::vector<PreparedDemand> fixedDemands;
        std::vector<PreparedDemand> flexibleDemands;
        std::vector<PreparedDemand> passengerDemands;
        for (const auto& [demandKey, demandAmount] : demands)
        {
            const auto& [platformDemand, sourceIndex, origin, initialIncoming, fixedDestination] = demandKey;
            const auto amount = static_cast<uint32_t>(std::min<uint64_t>(demandAmount, std::numeric_limits<uint32_t>::max()));
            auto sourceNode = sourceIndex;
            if (!initialIncoming.empty())
            {
                const Occurrence incoming{ sourceIndex, initialIncoming };
                if (findOccurrence(planned.arrivals, incoming) != kNoIndex)
                {
                    sourceNode = nodes.size() + findOccurrence(planned.occurrences, incoming);
                }
            }

            auto visitedNode = kNoIndex;
            if (sourceNode != sourceIndex)
            {
                const auto& predecessors = planned.ridePredecessors[sourceNode];
                if (predecessors.size() != 1)
                {
                    sourceNode = sourceIndex;
                }
                else if (fixedDestination == StationId::null || nodes[planned.nodeStations[predecessors.front()]].station != fixedDestination)
                {
                    // The packet has already traversed this occurrence; revisiting it would join two acyclic plans into a cycle.
                    visitedNode = predecessors.front();
                }
            }
            auto reachable = graph.passengerRouting
                ? std::vector<bool>(planned.nodeStations.size())
                : reachableNodes(sourceNode, edges, adjacency, visitedNode);
            if (graph.passengerRouting)
            {
                shortestPath(sourceNode, kNoIndex, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, visitedNode, planned.nodeStations[sourceNode], shortestPathScratch);
                for (size_t node = 0; node < reachable.size(); ++node)
                {
                    reachable[node] = shortestPathScratch.distance[node] != kUnreachableJourneyCost;
                }
            }
            if (fixedDestination != StationId::null)
            {
                const auto sink = findNode(nodes, fixedDestination);
                if (sink != kNoIndex && reachable[sink] && nodes[sink].accepts)
                {
                    fixedDemands.push_back({ amount, { sink }, {}, {}, sourceNode, visitedNode, origin, initialIncoming });
                }
                continue;
            }

            const auto pairablePassenger = graph.passengerRouting && platformDemand && origin == nodes[sourceIndex].station;
            PreparedDemand prepared{ amount, {}, {}, {}, sourceNode, visitedNode, origin, initialIncoming };
            for (size_t sink = 0; sink < nodes.size(); ++sink)
            {
                if ((sink != sourceIndex || !platformDemand) && reachable[sink] && nodes[sink].accepts)
                {
                    prepared.sinks.push_back(sink);
                    if (nodes[sink].passengerSink)
                    {
                        prepared.passengerSinks.push_back(sink);
                    }
                }
            }
            if (pairablePassenger)
            {
                passengerDemands.push_back(std::move(prepared));
            }
            else if (!prepared.sinks.empty())
            {
                flexibleDemands.push_back(std::move(prepared));
            }
        }

        const uint32_t accuracy = std::max<uint32_t>(1, settings.accuracy);
        const auto routeAmount = [&](const PreparedDemand& demand, size_t destinationNode, uint32_t amount) {
            const auto destination = nodes[destinationNode].station;
            if (destinationNode == demand.sourceNode)
            {
                addShare(ShareKey{ destination, demand.origin, destination, demand.incoming, {}, {}, destination }, amount);
                return;
            }

            const auto chunkSize = demand.amount / accuracy + (demand.amount % accuracy != 0);
            const auto forbiddenReturnStation = graph.passengerRouting ? planned.nodeStations[demand.sourceNode] : kNoIndex;
            uint32_t iterations = 0;
            for (uint32_t remaining = amount; remaining != 0;)
            {
                const auto& path = shortestPath(demand.sourceNode, destinationNode, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, demand.visitedNode, forbiddenReturnStation, shortestPathScratch);
                if (path.empty())
                {
                    break;
                }
                auto chunk = std::min(remaining, chunkSize);
                if (iterations < accuracy)
                {
                    for (const auto edgeIndex : path)
                    {
                        const auto& edge = edges[edgeIndex];
                        if (edge.kind == PlannedEdgeKind::ride && !edge.departure.empty())
                        {
                            const auto capacity = std::max<uint32_t>(1, edge.capacity);
                            chunk = static_cast<uint32_t>(std::min<uint64_t>(chunk, capacity - edge.flow % capacity));
                        }
                    }
                }
                auto incoming = demand.incoming;
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
                    addShare(ShareKey{ station, demand.origin, via, incoming, edge.departure, edge.arrival, destination }, chunk);
                    incoming = edge.arrival;
                }
                addShare(ShareKey{ destination, demand.origin, destination, incoming, {}, {}, destination }, chunk);
                remaining -= chunk;
                ++iterations;
            }
        };

        for (const auto& demand : fixedDemands)
        {
            routeAmount(demand, demand.sinks.front(), demand.amount);
        }

        if (graph.passengerRouting)
        {
            for (const auto& demand : flexibleDemands)
            {
                std::vector<int64_t> credits(demand.sinks.size());
                const auto chunkSize = demand.amount / accuracy + (demand.amount % accuracy != 0);
                for (uint32_t remaining = demand.amount; remaining != 0;)
                {
                    shortestPath(demand.sourceNode, kNoIndex, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, demand.visitedNode, planned.nodeStations[demand.sourceNode], shortestPathScratch);
                    const auto weights = destinationWeights(demand.sinks, nodes, shortestPathScratch.distance, settings.distanceEffect);
                    uint64_t totalWeight = 0;
                    size_t chosen = 0;
                    for (size_t i = 0; i < demand.sinks.size(); ++i)
                    {
                        totalWeight += weights[i];
                        credits[i] += static_cast<int64_t>(weights[i]);
                        if (credits[i] > credits[chosen]
                            || (credits[i] == credits[chosen] && stationValue(nodes[demand.sinks[i]].station) < stationValue(nodes[demand.sinks[chosen]].station)))
                        {
                            chosen = i;
                        }
                    }
                    credits[chosen] -= static_cast<int64_t>(totalWeight);
                    const auto amount = std::min(remaining, chunkSize);
                    routeAmount(demand, demand.sinks[chosen], amount);
                    remaining -= amount;
                }
            }

            std::vector<size_t> pairableDemands(passengerDemands.size());
            std::vector<uint32_t> remaining(passengerDemands.size());
            for (size_t demand = 0; demand < passengerDemands.size(); ++demand)
            {
                pairableDemands[demand] = demand;
                remaining[demand] = passengerDemands[demand].amount;
            }

            std::vector<PassengerPair> pairs;
            for (size_t first = 0; first < pairableDemands.size(); ++first)
            {
                const auto firstDemand = pairableDemands[first];
                for (size_t second = first + 1; second < pairableDemands.size(); ++second)
                {
                    const auto secondDemand = pairableDemands[second];
                    const auto& firstSinks = passengerDemands[firstDemand].sinks;
                    const auto& secondSinks = passengerDemands[secondDemand].sinks;
                    if (std::binary_search(firstSinks.begin(), firstSinks.end(), passengerDemands[secondDemand].sourceNode)
                        && std::binary_search(secondSinks.begin(), secondSinks.end(), passengerDemands[firstDemand].sourceNode))
                    {
                        pairs.push_back({ firstDemand, secondDemand });
                    }
                }
            }

            std::vector<std::vector<size_t>> demandPairs(passengerDemands.size());
            for (size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex)
            {
                const auto& pair = pairs[pairIndex];
                demandPairs[pair.first].push_back(pairIndex);
                demandPairs[pair.second].push_back(pairIndex);
            }
            std::vector<size_t> componentOf(passengerDemands.size(), kNoIndex);
            std::vector<std::vector<size_t>> components;
            for (const auto start : pairableDemands)
            {
                if (componentOf[start] != kNoIndex)
                {
                    continue;
                }
                const auto component = components.size();
                components.push_back({ start });
                componentOf[start] = component;
                for (size_t i = 0; i < components.back().size(); ++i)
                {
                    const auto current = components.back()[i];
                    for (const auto pairIndex : demandPairs[current])
                    {
                        const auto& pair = pairs[pairIndex];
                        const auto next = pair.first == current ? pair.second : pair.first;
                        if (componentOf[next] == kNoIndex)
                        {
                            componentOf[next] = component;
                            components.back().push_back(next);
                        }
                    }
                }
            }

            const auto demandPrecedes = [&](size_t lhs, size_t rhs) {
                return remaining[lhs] != remaining[rhs]
                    ? remaining[lhs] > remaining[rhs]
                    : stationValue(passengerDemands[lhs].origin) < stationValue(passengerDemands[rhs].origin);
            };
            // Mutual reachability partitions platform origins into cliques. Removing the unavoidable
            // largest-origin excess leaves each component with enough other demand to pair every unit.
            for (const auto& component : components)
            {
                uint64_t total = 0;
                auto largest = component.front();
                for (const auto demand : component)
                {
                    total += remaining[demand];
                    if (demandPrecedes(demand, largest))
                    {
                        largest = demand;
                    }
                }
                const auto others = total - remaining[largest];
                const auto excess = remaining[largest] > others ? remaining[largest] - others : total % 2;
                remaining[largest] -= static_cast<uint32_t>(excess);
                if (excess != 0)
                {
                    routeAmount(passengerDemands[largest], passengerDemands[largest].sourceNode, static_cast<uint32_t>(excess));
                }
            }

            struct ComponentState
            {
                uint64_t total{};
                std::array<size_t, 3> largest{ kNoIndex, kNoIndex, kNoIndex };
            };
            std::vector<ComponentState> componentStates(components.size());
            const auto updateComponent = [&](size_t component) {
                auto& state = componentStates[component];
                state = {};
                for (const auto demand : components[component])
                {
                    state.total += remaining[demand];
                    auto candidate = demand;
                    for (auto& largest : state.largest)
                    {
                        if (largest == kNoIndex || demandPrecedes(candidate, largest))
                        {
                            std::swap(largest, candidate);
                        }
                    }
                }
            };
            for (size_t component = 0; component < components.size(); ++component)
            {
                updateComponent(component);
            }

            std::vector<PassengerSink> passengerSinks;
            std::vector<std::vector<size_t>> demandSinks(passengerDemands.size());
            for (size_t demand = 0; demand < passengerDemands.size(); ++demand)
            {
                for (const auto destination : passengerDemands[demand].passengerSinks)
                {
                    demandSinks[demand].push_back(passengerSinks.size());
                    passengerSinks.push_back({ demand, destination });
                }
            }
            std::vector<std::vector<uint64_t>> journeyCosts(passengerDemands.size());
            std::vector<uint64_t> cachedEdgeCosts(edges.size());
            std::vector<bool> cachedActiveDemands(passengerDemands.size());
            bool pairWeightsCurrent = false;
            const auto updatePairWeights = [&](const std::vector<uint32_t>& budgets) {
                auto unchanged = pairWeightsCurrent;
                for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
                {
                    const auto cost = edgeCost(edgeIndex, edges, nodes, planned.nodeStations, graph.timeSensitive, settings.saturation);
                    unchanged = unchanged && cost == cachedEdgeCosts[edgeIndex];
                    cachedEdgeCosts[edgeIndex] = cost;
                }
                for (const auto demand : pairableDemands)
                {
                    const auto active = budgets[demand] != 0;
                    unchanged = unchanged && active == cachedActiveDemands[demand];
                    cachedActiveDemands[demand] = active;
                }
                if (unchanged)
                {
                    return;
                }
                pairWeightsCurrent = true;

                for (const auto demand : pairableDemands)
                {
                    if (!cachedActiveDemands[demand] || (demandPairs[demand].empty() && demandSinks[demand].empty()))
                    {
                        continue;
                    }
                    shortestPath(passengerDemands[demand].sourceNode, kNoIndex, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, passengerDemands[demand].visitedNode, planned.nodeStations[passengerDemands[demand].sourceNode], shortestPathScratch);
                    journeyCosts[demand].assign(shortestPathScratch.distance.begin(), shortestPathScratch.distance.begin() + nodes.size());
                }
                updatePassengerPairWeights(pairs, passengerDemands, nodes, journeyCosts, budgets, componentOf, settings.distanceEffect);
                updatePassengerSinkWeights(passengerSinks, passengerDemands, nodes, journeyCosts, budgets, demandSinks, settings.distanceEffect);
            };

            const auto maximumPairAmount = [&](const PassengerPair& pair) {
                const auto component = componentOf[pair.first];
                const auto& state = componentStates[component];
                uint64_t amount = std::min(remaining[pair.first], remaining[pair.second]);
                auto largestOther = kNoIndex;
                for (const auto candidate : state.largest)
                {
                    if (candidate != kNoIndex && candidate != pair.first && candidate != pair.second)
                    {
                        largestOther = candidate;
                        break;
                    }
                }
                if (largestOther != kNoIndex)
                {
                    const auto doubled = static_cast<uint64_t>(remaining[largestOther]) * 2;
                    amount = doubled > state.total ? 0 : std::min(amount, (state.total - doubled) / 2);
                }
                return static_cast<uint32_t>(amount);
            };

            const auto allocatePairs = [&](std::vector<uint32_t>& budgets) {
                updatePairWeights(budgets);
                auto demandOrder = pairableDemands;
                std::sort(demandOrder.begin(), demandOrder.end(), demandPrecedes);

                bool progress = false;
                for (const auto demand : demandOrder)
                {
                    while (budgets[demand] != 0)
                    {
                        auto chosen = kNoIndex;
                        bool chosenSink = false;
                        int64_t chosenCredit = std::numeric_limits<int64_t>::min();
                        uint64_t totalWeight = 0;
                        const auto isBetter = [&](int64_t credit, bool sink, size_t index) {
                            if (chosen == kNoIndex || credit != chosenCredit)
                            {
                                return chosen == kNoIndex || credit > chosenCredit;
                            }
                            if (sink != chosenSink)
                            {
                                return sink;
                            }
                            if (sink)
                            {
                                return stationValue(nodes[passengerSinks[index].destination].station)
                                    < stationValue(nodes[passengerSinks[chosen].destination].station);
                            }
                            const auto& lhs = pairs[index];
                            const auto& rhs = pairs[chosen];
                            return std::tie(passengerDemands[lhs.first].origin, passengerDemands[lhs.second].origin)
                                < std::tie(passengerDemands[rhs.first].origin, passengerDemands[rhs.second].origin);
                        };
                        for (const auto pairIndex : demandPairs[demand])
                        {
                            auto& pair = pairs[pairIndex];
                            const auto partner = pair.first == demand ? pair.second : pair.first;
                            if (pair.weight == 0 || budgets[partner] == 0 || maximumPairAmount(pair) == 0)
                            {
                                continue;
                            }
                            pair.credit += static_cast<int64_t>(pair.weight);
                            totalWeight += pair.weight;
                            if (isBetter(pair.credit, false, pairIndex))
                            {
                                chosen = pairIndex;
                                chosenSink = false;
                                chosenCredit = pair.credit;
                            }
                        }
                        for (const auto sinkIndex : demandSinks[demand])
                        {
                            auto& sink = passengerSinks[sinkIndex];
                            if (sink.weight == 0)
                            {
                                continue;
                            }
                            sink.credit += static_cast<int64_t>(sink.weight);
                            totalWeight += sink.weight;
                            if (isBetter(sink.credit, true, sinkIndex))
                            {
                                chosen = sinkIndex;
                                chosenSink = true;
                                chosenCredit = sink.credit;
                            }
                        }
                        if (chosen == kNoIndex)
                        {
                            break;
                        }

                        if (chosenSink)
                        {
                            auto& sink = passengerSinks[chosen];
                            const auto chunk = passengerDemands[demand].amount / accuracy + (passengerDemands[demand].amount % accuracy != 0);
                            const auto amount = std::min(budgets[demand], std::max<uint32_t>(1, chunk));
                            sink.credit -= static_cast<int64_t>(totalWeight);
                            routeAmount(passengerDemands[demand], sink.destination, amount);
                            remaining[demand] -= amount;
                            budgets[demand] -= amount;
                            updateComponent(componentOf[demand]);
                            progress = true;
                            continue;
                        }

                        auto& pair = pairs[chosen];
                        const auto amount = std::min({ budgets[pair.first], budgets[pair.second], maximumPairAmount(pair) });
                        pair.credit -= static_cast<int64_t>(totalWeight);
                        const auto routeFirst = [&](size_t from, size_t to) {
                            routeAmount(passengerDemands[from], passengerDemands[to].sourceNode, amount);
                        };
                        if (pair.reverseFirst)
                        {
                            routeFirst(pair.second, pair.first);
                            routeFirst(pair.first, pair.second);
                        }
                        else
                        {
                            routeFirst(pair.first, pair.second);
                            routeFirst(pair.second, pair.first);
                        }
                        pair.reverseFirst = !pair.reverseFirst;
                        remaining[pair.first] -= amount;
                        remaining[pair.second] -= amount;
                        budgets[pair.first] -= amount;
                        budgets[pair.second] -= amount;
                        updateComponent(componentOf[pair.first]);
                        progress = true;
                    }
                }
                return progress;
            };

            std::vector<uint32_t> budgets(passengerDemands.size());
            for (uint32_t round = 0; round < accuracy; ++round)
            {
                for (const auto demand : pairableDemands)
                {
                    const auto chunk = passengerDemands[demand].amount / accuracy + (passengerDemands[demand].amount % accuracy != 0);
                    budgets[demand] = std::min(remaining[demand], chunk);
                }
                if (!allocatePairs(budgets))
                {
                    break;
                }
            }
            for (const auto demand : pairableDemands)
            {
                budgets[demand] = remaining[demand];
            }
            allocatePairs(budgets);
            for (const auto demand : pairableDemands)
            {
                if (remaining[demand] != 0)
                {
                    routeAmount(passengerDemands[demand], passengerDemands[demand].sourceNode, remaining[demand]);
                }
            }
        }
        else
        {
            for (auto& demand : flexibleDemands)
            {
                shortestPath(demand.sourceNode, kNoIndex, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, demand.visitedNode, kNoIndex, shortestPathScratch);
                demand.costs.reserve(demand.sinks.size());
                for (const auto sink : demand.sinks)
                {
                    demand.costs.push_back(shortestPathScratch.distance[sink]);
                }
            }
            const auto targets = calculateFairSinkTargets(flexibleDemands, nodes);
            const auto assignments = assignDemandTargets(flexibleDemands, nodes, targets, settings.distanceEffect);
            for (size_t demand = 0; demand < flexibleDemands.size(); ++demand)
            {
                for (const auto [sink, amount] : assignments[demand])
                {
                    routeAmount(flexibleDemands[demand], sink, amount);
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
            result.push_back({ std::get<0>(key), std::get<1>(key), std::get<2>(key), amount, std::get<3>(key), std::get<4>(key), std::get<5>(key), std::get<6>(key) });
        }
        return result;
    }

    std::vector<StationJourneyCost> calculateJourneyCosts(const RoutingGraph& graph, StationId source, ServicePoint departure)
    {
        const auto nodes = canonicalNodes(graph);
        const auto sourceStation = findNode(nodes, source);
        if (sourceStation == kNoIndex)
        {
            return {};
        }

        auto planned = makePlannedGraph(graph, nodes);
        auto sourceNode = sourceStation;
        uint64_t initialCost = 0;
        auto visitedNode = kNoIndex;
        if (!departure.empty())
        {
            const auto occurrence = findOccurrence(planned.occurrences, { sourceStation, departure });
            if (occurrence == kNoIndex)
            {
                return {};
            }
            const auto departureNode = nodes.size() + occurrence;
            const auto ride = std::find_if(planned.edges.begin(), planned.edges.end(), [&](const auto& edge) {
                return edge.kind == PlannedEdgeKind::ride && edge.from == departureNode && edge.departure == departure;
            });
            if (ride == planned.edges.end())
            {
                return {};
            }
            initialCost = edgeCost(static_cast<size_t>(ride - planned.edges.begin()), planned.edges, nodes, planned.nodeStations, graph.timeSensitive, 100);
            visitedNode = departureNode;
            sourceNode = ride->to;
        }

        const auto adjacency = makeAdjacency(planned.nodeStations.size(), planned.edges);
        ShortestPathScratch scratch;
        const auto forbiddenReturnStation = graph.passengerRouting ? sourceStation : kNoIndex;
        shortestPath(sourceNode, kNoIndex, nodes, planned.nodeStations, planned.edges, adjacency, graph.timeSensitive, 100, visitedNode, forbiddenReturnStation, scratch);
        std::vector<StationJourneyCost> result;
        result.reserve(nodes.size());
        for (size_t node = 0; node < nodes.size(); ++node)
        {
            const auto cost = scratch.distance[node] == kUnreachableJourneyCost
                ? kUnreachableJourneyCost
                : saturatedAdd(initialCost, scratch.distance[node]);
            result.push_back({ nodes[node].station, cost });
        }
        return result;
    }

    uint64_t calculateJourneyCost(const RoutingGraph& graph, StationId source, StationId destination, ServicePoint departure)
    {
        const auto costs = calculateJourneyCosts(graph, source, departure);
        const auto found = std::lower_bound(costs.begin(), costs.end(), destination, [](const auto& item, StationId value) {
            return stationValue(item.destination) < stationValue(value);
        });
        if (found == costs.end() || found->destination != destination)
        {
            return kUnreachableJourneyCost;
        }
        return found->cost;
    }
}
