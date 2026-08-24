// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Routing.h>

#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;

namespace
{
    constexpr StationId station(uint16_t value)
    {
        return static_cast<StationId>(value);
    }

    constexpr ServicePoint servicePoint(uint16_t service, uint16_t occurrence)
    {
        return { static_cast<ServiceId>(service), occurrence };
    }

    RoutingNode node(
        uint16_t id,
        int16_t x,
        int16_t y,
        uint32_t supply = 0,
        bool accepts = false,
        uint32_t attraction = 1,
        bool passengerSink = false,
        TownId town = TownId::null)
    {
        return { station(id), x, y, supply, accepts, attraction, passengerSink, town };
    }

    RoutingEdge edge(uint16_t from, uint16_t to, uint32_t capacity = 100, uint32_t travelTime = 1)
    {
        return { station(from), station(to), capacity, travelTime };
    }

    RoutingEdge serviceEdge(uint16_t from, uint16_t to, uint16_t service, uint16_t departure, uint16_t arrival, uint32_t travelTime, uint32_t waitTime, uint32_t capacity = 1000)
    {
        return { station(from), station(to), capacity, travelTime, servicePoint(service, departure), servicePoint(service, arrival), waitTime, waitTime * 2 };
    }

    uint32_t amountAt(
        const std::vector<FlowShare>& flows,
        uint16_t current,
        uint16_t origin,
        uint16_t via,
        ServicePoint incoming = {},
        ServicePoint departure = {},
        ServicePoint arrival = {})
    {
        uint32_t amount = 0;
        for (const auto& flow : flows)
        {
            if (flow.station == station(current)
                && flow.origin == station(origin)
                && flow.via == station(via)
                && flow.incoming == incoming
                && flow.departure == departure
                && flow.arrival == arrival)
            {
                amount += flow.amount;
            }
        }
        return amount;
    }

    uint32_t amountToDestination(const std::vector<FlowShare>& flows, uint16_t origin, uint16_t destination)
    {
        uint32_t amount = 0;
        for (const auto& flow : flows)
        {
            if (flow.station == station(destination)
                && flow.origin == station(origin)
                && flow.via == station(destination)
                && flow.destination == station(destination))
            {
                amount += flow.amount;
            }
        }
        return amount;
    }

    uint32_t totalToDestination(const std::vector<FlowShare>& flows, uint16_t destination)
    {
        uint32_t amount = 0;
        for (const auto& flow : flows)
        {
            if (flow.station == station(destination)
                && flow.via == station(destination)
                && flow.destination == station(destination))
            {
                amount += flow.amount;
            }
        }
        return amount;
    }

    void expectSameFlows(const std::vector<FlowShare>& lhs, const std::vector<FlowShare>& rhs)
    {
        ASSERT_EQ(lhs.size(), rhs.size());
        for (size_t i = 0; i < lhs.size(); ++i)
        {
            EXPECT_EQ(lhs[i].station, rhs[i].station);
            EXPECT_EQ(lhs[i].origin, rhs[i].origin);
            EXPECT_EQ(lhs[i].via, rhs[i].via);
            EXPECT_EQ(lhs[i].amount, rhs[i].amount);
            EXPECT_EQ(lhs[i].incoming, rhs[i].incoming);
            EXPECT_EQ(lhs[i].departure, rhs[i].departure);
            EXPECT_EQ(lhs[i].arrival, rhs[i].arrival);
            EXPECT_EQ(lhs[i].destination, rhs[i].destination);
        }
    }

    uint32_t accessibilityAt(const std::vector<StationAccessibility>& accessibility, uint16_t id)
    {
        const auto found = std::find_if(accessibility.begin(), accessibility.end(), [id](const auto& item) {
            return item.station == station(id);
        });
        return found == accessibility.end() ? 0 : found->score;
    }

    RoutingGraph parallelGraph()
    {
        return {
            { node(1, 0, 0, 80), node(2, 10, -10), node(3, 10, 10), node(4, 20, 0, 0, true) },
            { edge(1, 2, 10, 10), edge(2, 4, 10, 10), edge(1, 3, 10, 10), edge(3, 4, 10, 10) },
            true,
            {},
        };
    }

    RoutingGraph continuationGraph()
    {
        return {
            { node(1, 0, 0, 1), node(2, 10, 0), node(3, 20, 0, 0, true) },
            {
                serviceEdge(1, 2, 1, 0, 1, 5, 1),
                serviceEdge(2, 3, 1, 1, 2, 5, 20),
                serviceEdge(1, 2, 2, 0, 1, 1, 1),
                serviceEdge(2, 3, 3, 0, 1, 1, 13),
            },
            true,
            {},
        };
    }

    RoutingGraph accessibilityGraph()
    {
        return {
            {
                node(1, 0, 0, 0, true, 100, false, TownId(1)),
                node(2, 10, 0, 0, true, 100, false, TownId(2)),
            },
            {
                serviceEdge(1, 2, 1, 0, 1, 10, 2, 20),
                serviceEdge(2, 1, 2, 0, 1, 10, 2, 20),
            },
            true,
            {},
            true,
        };
    }
}

TEST(CargoDistRouting, DirectRouteAndLocalConsumption)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 80), node(2, 10, 0, 0, true) },
        { edge(1, 2) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    ASSERT_EQ(flows.size(), 2U);
    EXPECT_EQ(amountAt(flows, 1, 1, 2), 80U);
    EXPECT_EQ(amountAt(flows, 2, 1, 2), 80U);
}

TEST(CargoDistRouting, TwoHopTransferBalancesAtIntermediateStation)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 60), node(2, 10, 0), node(3, 20, 0, 0, true) },
        { edge(1, 2), edge(2, 3) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    ASSERT_EQ(flows.size(), 3U);
    EXPECT_EQ(amountAt(flows, 1, 1, 2), 60U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3), 60U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 60U);
}

TEST(CargoDistRouting, RoutesDemandFromCurrentStationWithOriginalOrigin)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0), node(3, 20, 0, 0, true) },
        { edge(2, 3) },
        false,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 40 });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 3), 40U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 40U);
}

TEST(CargoDistRouting, WaitingDemandCannotBeConsumedAtCurrentStation)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0, 0, true) },
        {},
        false,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 40 });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_TRUE(flows.empty());
}

TEST(CargoDistRouting, IncomingDemandCanBeConsumedAtCurrentStation)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0, 0, true) },
        { serviceEdge(1, 2, 1, 0, 1, 1, 1) },
        true,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 40, servicePoint(1, 1) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 2, servicePoint(1, 1)), 40U);
}

TEST(CargoDistRouting, DisconnectedSinkReceivesNoFlow)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 50), node(2, 10, 0, 0, true) },
        {},
        false,
        {},
    };

    EXPECT_TRUE(calculateAsymmetricFlows(graph).empty());
}

TEST(CargoDistRouting, AcceptingOriginDoesNotConsumeItsOwnSupply)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 50, true) },
        {},
        false,
        {},
    };

    EXPECT_TRUE(calculateAsymmetricFlows(graph).empty());
}

TEST(CargoDistRouting, ZeroDistanceEffectAllocatesEquallyWithStationIdRemainder)
{
    const RoutingGraph graph{
        { node(10, 0, 0, 5), node(3, 20, 0, 0, true), node(2, 10, 0, 0, true) },
        { edge(10, 3), edge(10, 2) },
        false,
        {},
    };
    RoutingSettings settings{};
    settings.distanceEffect = 0;

    const auto flows = calculateAsymmetricFlows(graph, settings);

    EXPECT_EQ(amountAt(flows, 10, 10, 2), 3U);
    EXPECT_EQ(amountAt(flows, 10, 10, 3), 2U);
    EXPECT_EQ(amountAt(flows, 2, 10, 2), 3U);
    EXPECT_EQ(amountAt(flows, 3, 10, 3), 2U);
}

TEST(CargoDistRouting, EqualSinkTargetsIgnoreDistance)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, 10, 0, 0, true), node(3, 100, 0, 0, true) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);
    EXPECT_EQ(amountAt(flows, 2, 1, 2), 50U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 50U);
}

TEST(CargoDistRouting, AttractionWeightsEquidistantSinks)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, -10, 0, 0, true, 10), node(3, 10, 0, 0, true, 90) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };

    RoutingSettings settings{};
    settings.accuracy = 100;

    const auto flows = calculateAsymmetricFlows(graph, settings);

    EXPECT_EQ(amountAt(flows, 2, 1, 2), 10U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 90U);
}

TEST(CargoDistRouting, JourneyCostDoesNotChangeDestinationTargets)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 1000), node(2, 10, 0, 0, true, 10), node(3, 100, 0, 0, true, 100) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };
    RoutingSettings noDistance{};
    noDistance.distanceEffect = 0;

    const auto distanceFlows = calculateAsymmetricFlows(graph);
    const auto noDistanceFlows = calculateAsymmetricFlows(graph, noDistance);
    EXPECT_EQ(amountAt(distanceFlows, 2, 1, 2), amountAt(noDistanceFlows, 2, 1, 2));
    EXPECT_EQ(amountAt(distanceFlows, 3, 1, 3), amountAt(noDistanceFlows, 3, 1, 3));
    EXPECT_EQ(amountAt(distanceFlows, 2, 1, 2), 91U);
    EXPECT_EQ(amountAt(distanceFlows, 3, 1, 3), 909U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandCapsLargerOrigin)
{
    RoutingGraph graph{
        { node(1, 0, 0, 100, true, 100), node(2, 10, 0, 30, true, 30) },
        { edge(1, 2, 1000), edge(2, 1, 1000) },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 2), 30U);
    EXPECT_EQ(amountToDestination(flows, 2, 1), 30U);
    EXPECT_EQ(amountToDestination(flows, 1, 1), 70U);
    EXPECT_EQ(amountToDestination(flows, 2, 2), 0U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandPairsProductionDemands)
{
    RoutingGraph graph{
        { node(1, 0, 0, 0, true), node(2, 10, 0, 0, true), node(3, 20, 0, 0, true) },
        { edge(1, 2, 1000), edge(2, 1, 1000), edge(1, 3, 1000) },
        false,
        {
            { station(1), station(1), 10 },
            { station(2), station(2), 10 },
            { station(1), station(1), 4, {}, station(3) },
        },
    };
    graph.passengerRouting = true;

    const auto expected = calculateAsymmetricFlows(graph);
    EXPECT_EQ(amountToDestination(expected, 1, 2), 10U);
    EXPECT_EQ(amountToDestination(expected, 2, 1), 10U);
    EXPECT_EQ(amountToDestination(expected, 1, 3), 4U);

    std::reverse(graph.nodes.begin(), graph.nodes.end());
    std::reverse(graph.edges.begin(), graph.edges.end());
    std::reverse(graph.demands.begin(), graph.demands.end());
    expectSameFlows(expected, calculateAsymmetricFlows(graph));
}

TEST(CargoDistRouting, PassengerIndustrySinkReceivesWithoutProducing)
{
    RoutingGraph graph{
        { node(1, 0, 0, 40, true), node(2, 20, 0, 40, true), node(3, 10, 20, 0, true, 100, true) },
        {
            edge(1, 2),
            edge(2, 1),
            edge(1, 3),
            edge(3, 1),
            edge(2, 3),
            edge(3, 2),
        },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_GT(amountToDestination(flows, 1, 3), 0U);
    EXPECT_GT(amountToDestination(flows, 2, 3), 0U);
    EXPECT_EQ(amountToDestination(flows, 3, 1), 0U);
    EXPECT_EQ(amountToDestination(flows, 3, 2), 0U);
}

TEST(CargoDistRouting, PassengerIndustrySinkSupplyJoinsSymmetricPairing)
{
    RoutingGraph graph{
        { node(1, 0, 0, 40, true), node(2, 20, 0, 40, true), node(3, 10, 20, 40, true, 100, true) },
        {
            edge(1, 2),
            edge(2, 1),
            edge(1, 3),
            edge(3, 1),
            edge(2, 3),
            edge(3, 2),
        },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_GT(amountToDestination(flows, 1, 3), 0U);
    EXPECT_GT(amountToDestination(flows, 2, 3), 0U);
    EXPECT_GT(amountToDestination(flows, 3, 1) + amountToDestination(flows, 3, 2), 0U);
}

TEST(CargoDistRouting, SymmetricGravityUsesBothEndpointAttractions)
{
    constexpr std::array<uint32_t, 4> supplies = { 100, 100, 20, 20 };
    constexpr std::array<uint32_t, 4> attractions = { 100, 100, 10, 10 };
    RoutingGraph graph;
    graph.timeSensitive = true;
    graph.passengerRouting = true;
    for (uint16_t from = 1; from <= supplies.size(); ++from)
    {
        graph.nodes.push_back(node(from, from * 10, 0, supplies[from - 1], true, attractions[from - 1]));
        for (uint16_t to = 1; to < from; ++to)
        {
            graph.edges.push_back(edge(from, to, 1000, 100000));
            graph.edges.push_back(edge(to, from, 1000, 100000));
        }
    }
    RoutingSettings settings{};
    settings.accuracy = 100;

    const auto flows = calculateAsymmetricFlows(graph, settings);

    EXPECT_EQ(amountToDestination(flows, 1, 2), amountToDestination(flows, 2, 1));
    EXPECT_EQ(amountToDestination(flows, 1, 3), amountToDestination(flows, 3, 1));
    EXPECT_GT(amountToDestination(flows, 1, 2), amountToDestination(flows, 1, 3));

    auto uniformAttraction = graph;
    for (auto& candidate : uniformAttraction.nodes)
    {
        candidate.attraction = 1;
    }
    const auto uniformFlows = calculateAsymmetricFlows(uniformAttraction, settings);
    EXPECT_GT(amountToDestination(flows, 1, 2), amountToDestination(uniformFlows, 1, 2));

    auto relabelled = graph;
    for (auto& candidate : relabelled.nodes)
    {
        candidate.station = station(5 - static_cast<uint16_t>(candidate.station));
    }
    for (auto& candidate : relabelled.edges)
    {
        candidate.from = station(5 - static_cast<uint16_t>(candidate.from));
        candidate.to = station(5 - static_cast<uint16_t>(candidate.to));
    }
    const auto relabelledFlows = calculateAsymmetricFlows(relabelled, settings);
    for (uint16_t from = 1; from <= 4; ++from)
    {
        for (uint16_t to = 1; to <= 4; ++to)
        {
            EXPECT_NEAR(amountToDestination(flows, from, to), amountToDestination(relabelledFlows, 5 - from, 5 - to), 2);
        }
    }

    graph.nodes.push_back(node(5, 50, 0, 10, true, std::numeric_limits<uint32_t>::max()));
    graph.nodes.push_back(node(6, 60, 0, 10, true, std::numeric_limits<uint32_t>::max()));
    graph.edges.push_back(edge(5, 6, 1000, 1));
    graph.edges.push_back(edge(6, 5, 1000, 1));
    const auto extendedFlows = calculateAsymmetricFlows(graph, settings);
    for (uint16_t from = 1; from <= 4; ++from)
    {
        for (uint16_t to = 1; to <= 4; ++to)
        {
            EXPECT_EQ(amountToDestination(flows, from, to), amountToDestination(extendedFlows, from, to));
        }
    }
}

TEST(CargoDistRouting, SymmetricGravityUsesBothDirectionalCosts)
{
    const auto calculate = [](uint32_t outwardCost, uint32_t returnCost, uint8_t distanceEffect = 100) {
        RoutingGraph graph;
        graph.timeSensitive = true;
        for (uint16_t from = 1; from <= 4; ++from)
        {
            graph.nodes.push_back(node(from, from * 10, 0, 160, true));
            for (uint16_t to = 1; to <= 4; ++to)
            {
                if (from != to)
                {
                    const auto cost = from == 1 && to == 2 ? outwardCost : from == 2 && to == 1 ? returnCost
                                                                                                : 10U;
                    graph.edges.push_back(edge(from, to, 1000, cost));
                }
            }
        }
        graph.passengerRouting = true;
        RoutingSettings settings{};
        settings.distanceEffect = distanceEffect;
        return calculateAsymmetricFlows(graph, settings);
    };

    const auto cheap = calculate(1, 1);
    const auto expensiveReturn = calculate(1, 100);
    const auto expensiveOutward = calculate(100, 1);
    const auto noDistanceCheap = calculate(1, 1, 0);
    const auto noDistanceExpensive = calculate(1, 100, 0);
    EXPECT_GT(amountToDestination(cheap, 1, 2), amountToDestination(expensiveReturn, 1, 2));
    for (uint16_t from = 1; from <= 4; ++from)
    {
        for (uint16_t to = 1; to <= 4; ++to)
        {
            EXPECT_EQ(amountToDestination(expensiveReturn, from, to), amountToDestination(expensiveOutward, from, to));
            EXPECT_EQ(amountToDestination(noDistanceCheap, from, to), amountToDestination(noDistanceExpensive, from, to));
        }
    }
}

TEST(CargoDistRouting, SymmetricPassengerDemandUsesAllMatchableSupply)
{
    RoutingGraph graph{
        {
            node(1, 0, 0, 10, true),
            node(2, 10, 0, 10, true),
            node(3, 20, 0, 20, true),
        },
        {
            edge(1, 2, 1000),
            edge(2, 1, 1000),
            edge(1, 3, 1000),
            edge(3, 1, 1000),
            edge(2, 3, 1000),
            edge(3, 2, 1000),
        },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 3), 10U);
    EXPECT_EQ(amountToDestination(flows, 3, 1), 10U);
    EXPECT_EQ(amountToDestination(flows, 2, 3), 10U);
    EXPECT_EQ(amountToDestination(flows, 3, 2), 10U);
    EXPECT_EQ(amountToDestination(flows, 1, 1), 0U);
    EXPECT_EQ(amountToDestination(flows, 2, 2), 0U);
    EXPECT_EQ(amountToDestination(flows, 3, 3), 0U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandKeepsDistantPairsMatchable)
{
    RoutingGraph graph{
        { node(1, 0, 0, 1, true), node(2, 10, 0, 1, true), node(3, 20, 0, 2, true) },
        {
            edge(1, 2, 1000, 1),
            edge(2, 1, 1000, 1),
            edge(2, 3, 1000, 100000),
            edge(3, 2, 1000, 100000),
        },
        true,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 3), 1U);
    EXPECT_EQ(amountToDestination(flows, 2, 3), 1U);
    EXPECT_EQ(amountToDestination(flows, 3, 1), 1U);
    EXPECT_EQ(amountToDestination(flows, 3, 2), 1U);
    EXPECT_EQ(amountToDestination(flows, 1, 1), 0U);
    EXPECT_EQ(amountToDestination(flows, 2, 2), 0U);
    EXPECT_EQ(amountToDestination(flows, 3, 3), 0U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandAssignsOddRemainderDeterministically)
{
    RoutingGraph graph{
        { node(1, 0, 0, 1, true), node(2, 10, 0, 1, true), node(3, 20, 0, 1, true) },
        {
            edge(1, 2),
            edge(2, 1),
            edge(1, 3),
            edge(3, 1),
            edge(2, 3),
            edge(3, 2),
        },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 1), 1U);
    EXPECT_EQ(amountToDestination(flows, 2, 3), 1U);
    EXPECT_EQ(amountToDestination(flows, 3, 2), 1U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandRequiresAReachableReturn)
{
    RoutingGraph graph{
        { node(1, 0, 0, 10, true), node(2, 10, 0, 10, true) },
        { edge(1, 2, 1000) },
        false,
        {},
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 2), 0U);
    EXPECT_EQ(amountToDestination(flows, 2, 1), 0U);
    EXPECT_EQ(amountToDestination(flows, 1, 1), 10U);
    EXPECT_EQ(amountToDestination(flows, 2, 2), 10U);
}

TEST(CargoDistRouting, SymmetricPassengerDemandConservesEveryOrigin)
{
    constexpr std::array<uint32_t, 4> supplies = { 100, 40, 30, 20 };
    RoutingGraph graph;
    graph.timeSensitive = false;
    for (uint16_t from = 1; from <= supplies.size(); ++from)
    {
        graph.nodes.push_back(node(from, from * 10, 0, supplies[from - 1], true));
        for (uint16_t to = 1; to < from; ++to)
        {
            graph.edges.push_back(edge(from, to, 1000));
            graph.edges.push_back(edge(to, from, 1000));
        }
    }
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    uint32_t unassigned = 0;
    for (uint16_t from = 1; from <= supplies.size(); ++from)
    {
        uint32_t total = 0;
        for (uint16_t to = 1; to <= supplies.size(); ++to)
        {
            const auto amount = amountToDestination(flows, from, to);
            total += amount;
            if (from == to)
            {
                unassigned += amount;
            }
            else
            {
                EXPECT_EQ(amount, amountToDestination(flows, to, from));
            }
        }
        EXPECT_EQ(total, supplies[from - 1]);
    }
    EXPECT_EQ(unassigned, 10U);
}

TEST(CargoDistRouting, BalancesSinkTargetsAcrossMultipleSources)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0, 100),
            node(2, 10, 0, 100),
            node(3, 20, 0, 0, true),
            node(4, 30, 0, 0, true),
        },
        { edge(1, 3, 1000), edge(1, 4, 1000), edge(2, 3, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 4), 100U);
    EXPECT_EQ(amountToDestination(flows, 2, 3), 100U);
    EXPECT_EQ(totalToDestination(flows, 3), 100U);
    EXPECT_EQ(totalToDestination(flows, 4), 100U);
}

TEST(CargoDistRouting, BalancesOverlappingReachability)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0, 100),
            node(2, 10, 0, 100),
            node(3, 20, 0, 0, true),
            node(4, 30, 0, 0, true),
            node(5, 40, 0, 0, true),
        },
        { edge(1, 3, 1000), edge(1, 4, 1000), edge(2, 4, 1000), edge(2, 5, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(totalToDestination(flows, 3), 67U);
    EXPECT_EQ(totalToDestination(flows, 4), 67U);
    EXPECT_EQ(totalToDestination(flows, 5), 66U);
}

TEST(CargoDistRouting, RedistributesTargetsConstrainedByReachability)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0, 100),
            node(2, 10, 0, 100),
            node(3, 20, 0, 100),
            node(4, 30, 0, 0, true),
            node(5, 40, 0, 0, true),
            node(6, 50, 0, 0, true),
        },
        { edge(1, 4, 1000), edge(2, 4, 1000), edge(3, 4, 1000), edge(3, 5, 1000), edge(3, 6, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(totalToDestination(flows, 4), 200U);
    EXPECT_EQ(totalToDestination(flows, 5), 50U);
    EXPECT_EQ(totalToDestination(flows, 6), 50U);
}

TEST(CargoDistRouting, AssignsSourcesByJourneyCostAfterBalancingTargets)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0, 100),
            node(2, 100, 0, 100),
            node(3, 10, 0, 0, true),
            node(4, 90, 0, 0, true),
        },
        { edge(1, 3, 1000), edge(1, 4, 1000), edge(2, 3, 1000), edge(2, 4, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 1, 3), 100U);
    EXPECT_EQ(amountToDestination(flows, 2, 4), 100U);
}

TEST(CargoDistRouting, DestinationCountIsNotLimitedByRoutingAccuracy)
{
    RoutingGraph graph;
    graph.nodes.push_back(node(1, 0, 0, 34));
    for (uint16_t destination = 2; destination <= 18; ++destination)
    {
        graph.nodes.push_back(node(destination, destination * 10, 0, 0, true));
        graph.edges.push_back(edge(1, destination, 1000));
    }

    const auto flows = calculateAsymmetricFlows(graph);

    for (uint16_t destination = 2; destination <= 18; ++destination)
    {
        EXPECT_EQ(totalToDestination(flows, destination), 2U);
    }
}

TEST(CargoDistRouting, FixedDestinationsDoNotConsumeFlexibleSinkTargets)
{
    RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, 10, 0, 0, true), node(3, 20, 0, 0, true) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };
    graph.demands.push_back({ station(1), station(9), 100, {}, station(2) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountToDestination(flows, 9, 2), 100U);
    EXPECT_EQ(amountToDestination(flows, 1, 2), 50U);
    EXPECT_EQ(amountToDestination(flows, 1, 3), 50U);
}

TEST(CargoDistRouting, HigherAttractionDestinationKeepsFlowThroughIntermediateStop)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 110), node(2, 10, 0, 0, true, 10), node(3, 20, 0, 0, true, 100) },
        { edge(1, 2, 1000), edge(2, 3, 1000) },
        false,
        {},
    };
    RoutingSettings settings{};
    settings.distanceEffect = 0;
    settings.accuracy = 110;

    const auto flows = calculateAsymmetricFlows(graph, settings);

    EXPECT_EQ(amountAt(flows, 1, 1, 2), 110U);
    EXPECT_EQ(amountAt(flows, 2, 1, 2), 10U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3), 100U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 100U);
}

TEST(CargoDistRouting, CongestionSplitsFlowAcrossParallelRoutes)
{
    const auto flows = calculateAsymmetricFlows(parallelGraph());
    const auto repeatedFlows = calculateAsymmetricFlows(parallelGraph());
    const auto upperRoute = amountAt(flows, 1, 1, 2);
    const auto lowerRoute = amountAt(flows, 1, 1, 3);

    EXPECT_GT(upperRoute, 0U);
    EXPECT_GT(lowerRoute, 0U);
    EXPECT_EQ(upperRoute + lowerRoute, 80U);
    EXPECT_EQ(amountAt(flows, 2, 1, 4), upperRoute);
    EXPECT_EQ(amountAt(flows, 3, 1, 4), lowerRoute);
    EXPECT_EQ(amountAt(flows, 4, 1, 4), 80U);
    expectSameFlows(flows, repeatedFlows);
}

TEST(CargoDistRouting, InputOrderDoesNotAffectResult)
{
    auto ordered = parallelGraph();
    auto reversed = ordered;
    std::reverse(reversed.nodes.begin(), reversed.nodes.end());
    std::reverse(reversed.edges.begin(), reversed.edges.end());

    expectSameFlows(calculateAsymmetricFlows(ordered), calculateAsymmetricFlows(reversed));
}

TEST(CargoDistRouting, ZeroCapacityEdgeIsIgnored)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 50), node(2, 10, 0, 0, true) },
        { edge(1, 2, 0) },
        false,
        {},
    };

    EXPECT_TRUE(calculateAsymmetricFlows(graph).empty());
}

TEST(CargoDistRouting, EqualTargetsIgnoreCompleteJourneyCost)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, 10, 0, 0, true), node(3, 100, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 100, 20),
            serviceEdge(1, 3, 2, 0, 1, 10, 1),
        },
        true,
        {},
    };
    RoutingSettings settings{};
    settings.accuracy = 100;

    const auto flows = calculateAsymmetricFlows(graph, settings);
    EXPECT_EQ(amountToDestination(flows, 1, 2), 50U);
    EXPECT_EQ(amountToDestination(flows, 1, 3), 50U);
}

TEST(CargoDistRouting, TargetWeightsSurviveExtremeRatios)
{
    constexpr auto maximum = std::numeric_limits<uint32_t>::max();
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, -10, 0, 0, true, 1), node(3, 10, 0, 0, true, maximum) },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 0, 100'000),
            serviceEdge(1, 3, 2, 0, 1, maximum, 0, 100'000),
        },
        true,
        {},
    };
    RoutingSettings settings{};
    settings.accuracy = 100;

    const auto flows = calculateAsymmetricFlows(graph, settings);
    EXPECT_EQ(amountToDestination(flows, 1, 2), 0U);
    EXPECT_EQ(amountToDestination(flows, 1, 3), 100U);
}

TEST(CargoDistRouting, CongestionDoesNotChangeDestinationTargets)
{
    const auto calculate = [](uint32_t cheapCapacity) {
        const RoutingGraph graph{
            { node(1, 0, 0, 160), node(2, -10, 0, 0, true), node(3, 10, 0, 0, true) },
            {
                serviceEdge(1, 2, 1, 0, 1, 1, 1, cheapCapacity),
                serviceEdge(1, 3, 2, 0, 1, 5, 1, 1000),
            },
            true,
            {},
        };
        return calculateAsymmetricFlows(graph);
    };

    const auto uncongested = calculate(1000);
    const auto congested = calculate(10);

    EXPECT_EQ(amountToDestination(uncongested, 1, 2), 80U);
    EXPECT_EQ(amountToDestination(uncongested, 1, 3), 80U);
    EXPECT_EQ(amountToDestination(congested, 1, 2), 80U);
    EXPECT_EQ(amountToDestination(congested, 1, 3), 80U);
}

TEST(CargoDistRouting, SymmetricGravityRespondsToPairCongestion)
{
    const auto calculate = [](uint32_t cheapCapacity) {
        RoutingGraph graph;
        graph.timeSensitive = true;
        graph.passengerRouting = true;
        for (uint16_t from = 1; from <= 4; ++from)
        {
            graph.nodes.push_back(node(from, from * 10, 0, 160, true));
            for (uint16_t to = 1; to <= 4; ++to)
            {
                if (from == to)
                {
                    continue;
                }
                const auto firstPair = (from == 1 && to == 2) || (from == 2 && to == 1);
                const auto secondPair = (from == 3 && to == 4) || (from == 4 && to == 3);
                graph.edges.push_back(edge(from, to, firstPair ? cheapCapacity : 1000, firstPair || secondPair ? 1 : 5));
            }
        }
        return calculateAsymmetricFlows(graph);
    };

    const auto uncongested = calculate(1000);
    const auto congested = calculate(10);

    EXPECT_LT(amountToDestination(congested, 1, 2), amountToDestination(uncongested, 1, 2));
    EXPECT_EQ(amountToDestination(congested, 1, 2), amountToDestination(congested, 2, 1));
    EXPECT_EQ(amountToDestination(uncongested, 1, 2), amountToDestination(uncongested, 2, 1));
    EXPECT_GT(
        amountToDestination(congested, 1, 3) + amountToDestination(congested, 1, 4),
        amountToDestination(uncongested, 1, 3) + amountToDestination(uncongested, 1, 4));
    for (uint16_t origin = 1; origin <= 4; ++origin)
    {
        uint32_t total = 0;
        for (uint16_t destination = 1; destination <= 4; ++destination)
        {
            total += amountToDestination(congested, origin, destination);
            if (origin != destination)
            {
                EXPECT_EQ(amountToDestination(congested, origin, destination), amountToDestination(congested, destination, origin));
            }
        }
        EXPECT_EQ(total, 160U);
        EXPECT_EQ(amountToDestination(congested, origin, origin), 0U);
    }
}

TEST(CargoDistRouting, LocalRoutingDoesNotLoopBackThroughDepartureStation)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, -10, 0), node(3, -20, 0), node(4, 10, 0, 0, true) },
        {
            { station(1), station(4), 1, 1, servicePoint(2, 0), servicePoint(2, 1), 1, 100 },
            serviceEdge(1, 2, 1, 0, 1, 1, 1),
            serviceEdge(2, 3, 1, 1, 2, 1, 1),
            serviceEdge(3, 1, 1, 2, 3, 1, 1),
            serviceEdge(1, 4, 1, 3, 0, 1, 100),
        },
        true,
        { { station(1), station(1), 20, {}, station(4) } },
    };
    graph.passengerRouting = true;

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1)), 0U);
    EXPECT_EQ(
        amountAt(flows, 1, 1, 4, {}, servicePoint(1, 3), servicePoint(1, 0))
            + amountAt(flows, 1, 1, 4, {}, servicePoint(2, 0), servicePoint(2, 1)),
        20U);
    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(4), servicePoint(1, 0)), kUnreachableJourneyCost);
    EXPECT_NE(calculateJourneyCost(graph, station(1), station(4), servicePoint(2, 0)), kUnreachableJourneyCost);
}

TEST(CargoDistRouting, WaitMakesFastInfrequentServiceLoseToSlowerFrequentService)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 1), node(2, 10, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 20),
            serviceEdge(1, 2, 2, 0, 1, 10, 1),
        },
        true,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1)), 0U);
    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(2, 0), servicePoint(2, 1)), 1U);
    EXPECT_EQ(amountAt(flows, 2, 1, 2, servicePoint(2, 1)), 1U);
}

TEST(CargoDistRouting, FullDeparturesAddQueueHeadways)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 300), node(2, 10, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 5, 5, 100),
            serviceEdge(1, 2, 2, 0, 1, 20, 2, 100),
        },
        true,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);
    const auto fast = amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1));
    const auto stopping = amountAt(flows, 1, 1, 2, {}, servicePoint(2, 0), servicePoint(2, 1));

    EXPECT_GT(fast, 0U);
    EXPECT_GT(stopping, 0U);
    EXPECT_EQ(fast + stopping, 300U);
}

TEST(CargoDistRouting, QueueBoundaryMovesTheNextPassenger)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 101), node(2, 10, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 5, 5, 100),
            serviceEdge(1, 2, 2, 0, 1, 12, 1, 100),
        },
        true,
        {},
    };
    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1)), 100U);
    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(2, 0), servicePoint(2, 1)), 1U);
}

TEST(CargoDistRouting, QueueSplittingKeepsIdenticalServicesWithinChunkSize)
{
    constexpr auto supply = std::numeric_limits<uint32_t>::max();
    const RoutingGraph graph{
        { node(1, 0, 0, supply), node(2, 10, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 1, 1),
            serviceEdge(1, 2, 2, 0, 1, 1, 1, 1),
        },
        true,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    const auto first = amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1));
    const auto second = amountAt(flows, 1, 1, 2, {}, servicePoint(2, 0), servicePoint(2, 1));
    EXPECT_EQ(static_cast<uint64_t>(first) + second, supply);
    const auto chunkSize = supply / RoutingSettings{}.accuracy + (supply % RoutingSettings{}.accuracy != 0);
    EXPECT_LE(std::max(first, second) - std::min(first, second), chunkSize);
}

TEST(CargoDistRouting, FixedDestinationIsNotReallocated)
{
    RoutingGraph graph{
        {
            node(1, 0, 0),
            node(2, 10, 0, 0, true),
            node(3, 20, 0, 0, true),
        },
        { edge(1, 2), edge(1, 3) },
        false,
        {},
    };
    graph.demands.push_back({ station(1), station(1), 10, {}, station(3) });

    const auto flows = calculateAsymmetricFlows(graph);

    ASSERT_FALSE(flows.empty());
    EXPECT_TRUE(std::all_of(flows.begin(), flows.end(), [](const auto& flow) {
        return flow.destination == station(3);
    }));
    EXPECT_EQ(amountAt(flows, 1, 1, 2), 0U);
    EXPECT_EQ(amountAt(flows, 1, 1, 3), 10U);
}

TEST(CargoDistRouting, PresentServiceUsesCompleteJourneyCost)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0),
            node(2, 10, 0),
            node(3, 10, 10),
            node(4, 20, 0, 0, true),
        },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 1),
            serviceEdge(2, 4, 1, 1, 2, 100, 1),
            serviceEdge(1, 3, 2, 0, 1, 10, 100),
            serviceEdge(3, 4, 2, 1, 2, 10, 100),
        },
        true,
        {},
    };

    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(4)), 102U);
    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(4), servicePoint(2, 0)), 20U);
}

TEST(CargoDistRouting, PresentServiceJourneyTakesItsFirstLeg)
{
    const RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0), node(3, 20, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 1, 0, 1, 1),
            serviceEdge(2, 1, 1, 0, 1, 1, 1),
            serviceEdge(1, 3, 2, 0, 1, 1, 1),
        },
        true,
        {},
    };

    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(3)), 2U);
    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(3), servicePoint(1, 1)), kUnreachableJourneyCost);
}

TEST(CargoDistRouting, ThroughPassengersReserveOnwardCapacityFirst)
{
    RoutingGraph graph{
        {
            node(1, 0, 0),
            node(2, 10, 0, 10),
            node(3, 20, 0, 0, true),
        },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 10, 10),
            serviceEdge(2, 3, 1, 1, 2, 1, 10, 10),
            serviceEdge(2, 3, 2, 0, 1, 5, 1, 10),
        },
        true,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 10, servicePoint(1, 1), station(3) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(1, 1), servicePoint(1, 2)), 10U);
    EXPECT_EQ(amountAt(flows, 2, 2, 3, {}, servicePoint(1, 1), servicePoint(1, 2)), 0U);
    EXPECT_EQ(amountAt(flows, 2, 2, 3, {}, servicePoint(2, 0), servicePoint(2, 1)), 10U);
}

TEST(CargoDistRouting, ContinuingOnSameServicePaysOnlyOneWait)
{
    const auto flows = calculateAsymmetricFlows(continuationGraph());

    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(1, 0), servicePoint(1, 1)), 1U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(1, 1), servicePoint(1, 2)), 1U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3, servicePoint(1, 2)), 1U);
    EXPECT_EQ(amountAt(flows, 1, 1, 2, {}, servicePoint(2, 0), servicePoint(2, 1)), 0U);
}

TEST(CargoDistRouting, IncomingServiceDemandContinuesWithoutAnotherWait)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0), node(3, 20, 0, 0, true) },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 100),
            serviceEdge(2, 3, 1, 1, 2, 1, 100),
            serviceEdge(2, 3, 2, 0, 1, 10, 1),
        },
        true,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 1, servicePoint(1, 1) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(1, 1), servicePoint(1, 2)), 1U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(2, 0), servicePoint(2, 1)), 0U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3, servicePoint(1, 2)), 1U);
}

TEST(CargoDistRouting, IncomingServiceDemandReturnsOnlyToItsDestination)
{
    RoutingGraph graph{
        { node(1, 0, 0, 0, true), node(2, 10, 0), node(3, 20, 0, 0, true), node(4, -10, 0) },
        {
            serviceEdge(1, 2, 1, 1, 0, 1, 1),
            serviceEdge(2, 1, 1, 0, 1, 1, 1),
            serviceEdge(1, 3, 2, 0, 1, 1, 1),
            serviceEdge(2, 3, 3, 0, 1, 5, 1),
        },
        true,
        {},
    };
    graph.demands.push_back({ station(2), station(4), 10, servicePoint(1, 0), station(3) });
    graph.demands.push_back({ station(2), station(4), 10, servicePoint(1, 0), station(1) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 4, 3, servicePoint(1, 0), servicePoint(3, 0), servicePoint(3, 1)), 10U);
    EXPECT_EQ(amountAt(flows, 2, 4, 1, servicePoint(1, 0), servicePoint(1, 0), servicePoint(1, 1)), 10U);
}

TEST(CargoDistRouting, DepartureOnlyIncomingServiceStillPaysBoardingWait)
{
    RoutingGraph graph{
        { node(2, 0, 0), node(3, 10, 0, 0, true) },
        {
            serviceEdge(2, 3, 1, 1, 2, 1, 100),
            serviceEdge(2, 3, 2, 0, 1, 10, 1),
        },
        true,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 1, servicePoint(1, 1) });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(1, 1), servicePoint(1, 2)), 0U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3, servicePoint(1, 1), servicePoint(2, 0), servicePoint(2, 1)), 1U);
}

TEST(CargoDistRouting, FlowWeightScalingPreservesSmallOptions)
{
    constexpr auto maximum = std::numeric_limits<uint32_t>::max();
    RoutingGraph graph{
        {
            node(1, 0, 0),
            node(2, 0, 0),
            node(3, 10, 0),
            node(4, 20, 0, 0, true, 1),
            node(5, 20, 0, 0, true, maximum),
        },
        {
            edge(1, 3, maximum),
            edge(2, 3, maximum),
            edge(3, 4, maximum),
            edge(3, 5, maximum),
        },
        false,
        {
            { station(1), station(9), maximum, {}, station(5) },
            { station(2), station(9), 2, {}, station(4) },
        },
    };

    const auto flows = calculateAsymmetricFlows(graph);
    const auto small = amountAt(flows, 3, 9, 4);
    const auto large = amountAt(flows, 3, 9, 5);

    EXPECT_EQ(small, 1U);
    EXPECT_GT(large, 0U);
    EXPECT_LE(static_cast<uint64_t>(small) + large, maximum);
}

TEST(CargoDistRouting, ServiceRoutingIsIndependentOfInputOrder)
{
    auto ordered = continuationGraph();
    auto reversed = ordered;
    std::reverse(reversed.nodes.begin(), reversed.nodes.end());
    std::reverse(reversed.edges.begin(), reversed.edges.end());
    std::reverse(reversed.demands.begin(), reversed.demands.end());

    expectSameFlows(calculateAsymmetricFlows(ordered), calculateAsymmetricFlows(reversed));
}

TEST(CargoDistRouting, AccessibilityUsesReciprocalJourneyQualityAndBoundedCapacity)
{
    const auto graph = accessibilityGraph();

    const auto journeys = calculateJourneyCosts(graph, station(1));
    ASSERT_EQ(journeys.size(), 2U);
    EXPECT_EQ(journeys[1].cost, 12U);
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(graph), 1), 62U);

    auto slower = graph;
    for (auto& route : slower.edges)
    {
        route.travelTime = 40;
    }
    EXPECT_LT(accessibilityAt(calculateStationAccessibility(slower), 1), 62U);

    auto moreAttractive = graph;
    moreAttractive.nodes[1].attraction = 200;
    EXPECT_GT(accessibilityAt(calculateStationAccessibility(moreAttractive), 1), 62U);

    auto fasterLowCapacity = graph;
    fasterLowCapacity.edges.push_back(serviceEdge(1, 2, 3, 0, 1, 1, 0, 1));
    fasterLowCapacity.edges.push_back(serviceEdge(2, 1, 4, 0, 1, 1, 0, 1));
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(fasterLowCapacity), 1), 95U);

    auto oneWay = graph;
    oneWay.edges.pop_back();
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(oneWay), 1), 0U);

    auto bounded = graph;
    for (auto& route : bounded.edges)
    {
        route.capacity = 256;
    }
    const auto boundedScore = accessibilityAt(calculateStationAccessibility(bounded), 1);
    for (auto& route : bounded.edges)
    {
        route.capacity = 1000;
    }
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(bounded), 1), boundedScore);
}

TEST(CargoDistRouting, AccessibilityChargesWaitOnlyWhenBoardingAThroughService)
{
    const RoutingGraph graph{
        {
            node(1, 0, 0, 0, false, 0, false, TownId(1)),
            node(2, 10, 0, 0, false, 0, false, TownId(2)),
            node(3, 20, 0, 0, true, 100, false, TownId(3)),
        },
        {
            serviceEdge(1, 2, 1, 0, 1, 1, 10, 20),
            serviceEdge(2, 3, 1, 1, 2, 1, 10, 20),
            serviceEdge(3, 2, 2, 0, 1, 1, 10, 20),
            serviceEdge(2, 1, 2, 1, 2, 1, 10, 20),
        },
        true,
        {},
        true,
    };

    EXPECT_EQ(calculateJourneyCost(graph, station(1), station(3)), 12U);
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(graph), 1), 62U);
}

TEST(CargoDistRouting, AccessibilityCountsEachReachableTownOnceInStableOrder)
{
    auto graph = accessibilityGraph();
    const auto oneStationScore = accessibilityAt(calculateStationAccessibility(graph), 1);
    ASSERT_GT(oneStationScore, 0U);

    graph.nodes[1].town = TownId(1);
    EXPECT_EQ(accessibilityAt(calculateStationAccessibility(graph), 1), oneStationScore);

    graph.nodes.push_back(node(3, 20, 0, 0, true, 100, false, TownId(1)));
    graph.edges.push_back(serviceEdge(1, 3, 3, 0, 1, 10, 2, 20));
    graph.edges.push_back(serviceEdge(3, 1, 4, 0, 1, 10, 2, 20));
    const auto sameTown = calculateStationAccessibility(graph);
    EXPECT_EQ(accessibilityAt(sameTown, 1), oneStationScore);

    graph.nodes.back().town = TownId(3);
    const auto separateTowns = calculateStationAccessibility(graph);
    EXPECT_GT(accessibilityAt(separateTowns, 1), oneStationScore);

    std::reverse(graph.nodes.begin(), graph.nodes.end());
    std::reverse(graph.edges.begin(), graph.edges.end());
    EXPECT_EQ(calculateStationAccessibility(graph), separateTowns);
}

TEST(CargoDistRouting, AccessibilityHandlesManyServiceOccurrencesDeterministically)
{
    constexpr uint16_t kStationCount = 24;
    constexpr uint16_t kParallelServices = 8;
    RoutingGraph graph{};
    graph.timeSensitive = true;
    graph.passengerRouting = true;
    for (uint16_t id = 1; id <= kStationCount; ++id)
    {
        graph.nodes.push_back(node(id, id * 10, 0, 0, true, 100, false, TownId(id)));
    }
    uint16_t service = 1;
    for (uint16_t id = 1; id < kStationCount; ++id)
    {
        for (uint16_t parallel = 0; parallel < kParallelServices; ++parallel)
        {
            graph.edges.push_back(serviceEdge(id, id + 1, service++, 0, 1, 1, 1, 20));
            graph.edges.push_back(serviceEdge(id + 1, id, service++, 0, 1, 1, 1, 20));
        }
    }

    const auto accessibility = calculateStationAccessibility(graph);
    ASSERT_EQ(accessibility.size(), kStationCount);

    std::reverse(graph.nodes.begin(), graph.nodes.end());
    std::reverse(graph.edges.begin(), graph.edges.end());
    EXPECT_EQ(calculateStationAccessibility(graph), accessibility);
}
