// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Routing.h>

#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;

namespace
{
    constexpr StationId station(uint16_t value)
    {
        return static_cast<StationId>(value);
    }

    RoutingNode node(uint16_t id, int16_t x, int16_t y, uint32_t supply = 0, bool accepts = false, uint32_t attraction = 1)
    {
        return { station(id), x, y, supply, accepts, attraction };
    }

    RoutingEdge edge(uint16_t from, uint16_t to, uint32_t capacity = 100, uint32_t travelTime = 1)
    {
        return { station(from), station(to), capacity, travelTime };
    }

    uint32_t amountAt(const std::vector<FlowShare>& flows, uint16_t current, uint16_t origin, uint16_t via)
    {
        const auto found = std::find_if(flows.begin(), flows.end(), [&](const auto& flow) {
            return flow.station == station(current) && flow.origin == station(origin) && flow.via == station(via);
        });
        return found == flows.end() ? 0 : found->amount;
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
        }
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

TEST(CargoDistRouting, OutstandingDemandCanBeConsumedAtCurrentStation)
{
    RoutingGraph graph{
        { node(1, 0, 0), node(2, 10, 0, 0, true) },
        {},
        false,
        {},
    };
    graph.demands.push_back({ station(2), station(1), 40 });

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 2), 40U);
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

TEST(CargoDistRouting, DistanceEffectPrefersCloserSink)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, 10, 0, 0, true), node(3, 100, 0, 0, true) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);
    const auto nearAmount = amountAt(flows, 2, 1, 2);
    const auto farAmount = amountAt(flows, 3, 1, 3);

    EXPECT_GT(nearAmount, farAmount);
    EXPECT_EQ(nearAmount + farAmount, 100U);
}

TEST(CargoDistRouting, AttractionWeightsEquidistantSinks)
{
    const RoutingGraph graph{
        { node(1, 0, 0, 100), node(2, -10, 0, 0, true, 10), node(3, 10, 0, 0, true, 90) },
        { edge(1, 2, 1000), edge(1, 3, 1000) },
        false,
        {},
    };

    const auto flows = calculateAsymmetricFlows(graph);

    EXPECT_EQ(amountAt(flows, 2, 1, 2), 10U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 90U);
}

TEST(CargoDistRouting, DistanceModeratesDestinationAttraction)
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
    const auto nearAmount = amountAt(distanceFlows, 2, 1, 2);
    const auto farAmount = amountAt(distanceFlows, 3, 1, 3);

    EXPECT_GT(farAmount, nearAmount);
    EXPECT_LT(farAmount, amountAt(noDistanceFlows, 3, 1, 3));
    EXPECT_EQ(nearAmount + farAmount, 1000U);
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

    const auto flows = calculateAsymmetricFlows(graph, settings);

    EXPECT_EQ(amountAt(flows, 1, 1, 2), 110U);
    EXPECT_EQ(amountAt(flows, 2, 1, 2), 10U);
    EXPECT_EQ(amountAt(flows, 2, 1, 3), 100U);
    EXPECT_EQ(amountAt(flows, 3, 1, 3), 100U);
}

TEST(CargoDistRouting, CongestionSplitsFlowAcrossParallelRoutes)
{
    const auto flows = calculateAsymmetricFlows(parallelGraph());
    const auto upperRoute = amountAt(flows, 1, 1, 2);
    const auto lowerRoute = amountAt(flows, 1, 1, 3);

    EXPECT_GT(upperRoute, 0U);
    EXPECT_GT(lowerRoute, 0U);
    EXPECT_EQ(upperRoute + lowerRoute, 80U);
    EXPECT_EQ(amountAt(flows, 2, 1, 4), upperRoute);
    EXPECT_EQ(amountAt(flows, 3, 1, 4), lowerRoute);
    EXPECT_EQ(amountAt(flows, 4, 1, 4), 80U);
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
