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
            // Generate the fractional bits without overflowing numerator * kScale.
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
                    if (settled[edge.to])
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
            std::vector<uint64_t> weights;
            weights.reserve(sinks.size());
            uint64_t minimumCost = kMaximumCost;
            for (const auto sink : sinks)
            {
                minimumCost = std::min(minimumCost, std::max<uint64_t>(1, journeyCosts[sink]));
            }
            uint64_t maximumScore = 1;
            for (const auto sink : sinks)
            {
                const auto cost = std::max<uint64_t>(1, journeyCosts[sink]);
                const auto effectiveCost = saturatedAdd(minimumCost, scalePercent(cost - minimumCost, distanceEffect));
                const auto proximity = ratioQ16(minimumCost, effectiveCost);
                const auto attraction = std::max<uint64_t>(1, nodes[sink].attraction);
                const auto score = saturatedMultiply(proximity * proximity, attraction);
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

        for (const auto& [demandKey, demandAmount] : demands)
        {
            const auto& [platformDemand, sourceIndex, origin, initialIncoming, fixedDestination] = demandKey;
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
            const auto reachable = reachableNodes(sourceNode, edges, adjacency, visitedNode);
            const uint32_t accuracy = std::max<uint32_t>(1, settings.accuracy);
            const uint32_t chunkSize = supply / accuracy + (supply % accuracy != 0);
            const auto routeAmount = [&](size_t destinationNode, uint32_t amount, uint32_t boundaryIterations, bool reuseCurrentTree) {
                const auto destination = nodes[destinationNode].station;
                if (destinationNode == sourceNode)
                {
                    addShare(ShareKey{ destination, origin, destination, initialIncoming, {}, {}, destination }, amount);
                    return;
                }

                uint32_t iterations = 0;
                for (uint32_t remaining = amount; remaining != 0; ++iterations)
                {
                    const auto& path = reuseCurrentTree
                        ? reconstructPath(sourceNode, destinationNode, planned.nodeStations, edges, shortestPathScratch)
                        : shortestPath(sourceNode, destinationNode, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, visitedNode, shortestPathScratch);
                    reuseCurrentTree = false;
                    if (path.empty())
                    {
                        break;
                    }
                    auto chunk = std::min(remaining, chunkSize);
                    if (iterations < boundaryIterations)
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
                        addShare(ShareKey{ station, origin, via, incoming, edge.departure, edge.arrival, destination }, chunk);
                        incoming = edge.arrival;
                    }
                    addShare(ShareKey{ destination, origin, destination, incoming, {}, {}, destination }, chunk);
                    remaining -= chunk;
                }
            };

            if (fixedDestination != StationId::null)
            {
                const auto sink = findNode(nodes, fixedDestination);
                if (sink != kNoIndex && reachable[sink] && nodes[sink].accepts)
                {
                    routeAmount(sink, supply, accuracy, false);
                }
                continue;
            }

            std::vector<size_t> sinks;
            for (size_t sink = 0; sink < nodes.size(); ++sink)
            {
                if ((sink != sourceIndex || !platformDemand) && reachable[sink] && nodes[sink].accepts)
                {
                    sinks.push_back(sink);
                }
            }
            if (sinks.empty())
            {
                continue;
            }

            std::vector<int64_t> credits(sinks.size());
            for (uint32_t remaining = supply; remaining != 0;)
            {
                shortestPath(sourceNode, kNoIndex, nodes, planned.nodeStations, edges, adjacency, graph.timeSensitive, settings.saturation, visitedNode, shortestPathScratch);
                const auto weights = destinationWeights(sinks, nodes, shortestPathScratch.distance, settings.distanceEffect);
                uint64_t totalWeight = 0;
                size_t chosen = 0;
                for (size_t i = 0; i < sinks.size(); ++i)
                {
                    totalWeight += weights[i];
                    credits[i] += static_cast<int64_t>(weights[i]);
                    if (credits[i] > credits[chosen]
                        || (credits[i] == credits[chosen] && stationValue(nodes[sinks[i]].station) < stationValue(nodes[sinks[chosen]].station)))
                    {
                        chosen = i;
                    }
                }
                credits[chosen] -= static_cast<int64_t>(totalWeight);
                const auto batch = std::min(remaining, chunkSize);
                routeAmount(sinks[chosen], batch, 1, true);
                remaining -= batch;
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
        shortestPath(sourceNode, kNoIndex, nodes, planned.nodeStations, planned.edges, adjacency, graph.timeSensitive, 100, visitedNode, scratch);
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
