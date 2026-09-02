// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/CargoDist.h>
#include <OpenLoco/CargoDist/FlowAnalytics.h>
#include <OpenLoco/Date.h>
#include <OpenLoco/Graphics/RenderTarget.h>
#include <OpenLoco/Graphics/SoftwareDrawingContext.h>
#include <OpenLoco/Ui/Windows/CargoFlowOverlay.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;
using namespace OpenLoco::Ui::Windows;

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

    const PlannedServiceEdge* findEdge(const std::vector<PlannedServiceEdge>& edges, StationId from, StationId to)
    {
        const auto edge = std::find_if(edges.begin(), edges.end(), [=](const auto& item) {
            return item.from == from && item.to == to;
        });
        return edge == edges.end() ? nullptr : &*edge;
    }
}

TEST(CargoFlowOverlayTest, AggregatesDirectedPlannedDemandAndCapacity)
{
    reset();
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(1, 0), servicePoint(1, 1) }] = { 20, 10, 2, 0, 20 };
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(2, 0), servicePoint(2, 1) }] = { 30, 12, 3, 0, 30 };
    state.serviceEdges[{ 0, station(2), station(1) }] = { 75, 10, 0, 0, 75 };
    state.serviceEdges[{ 0, station(4), station(5) }] = { 30, 10, 0, 0, 30 };
    state.serviceEdges[{ 1, station(1), station(2) }] = { 200, 10, 0, 0, 200 };
    state.flows[{ 0, station(1), station(6) }] = {
        { station(1), 30, 0 },
        { station(2), 12, 0, servicePoint(1, 0), servicePoint(1, 1) },
        { station(3), 10, 0 },
    };
    state.flows[{ 0, station(1), station(6), servicePoint(3, 1) }] = {
        { station(2), 8, 0, servicePoint(2, 0), servicePoint(2, 1) },
    };
    state.flows[{ 0, station(1), station(7) }] = {
        { station(2), 40, 0 },
        { StationId::null, 10, 0 },
    };
    state.flows[{ 0, station(2), station(6) }] = { { station(1), 7, 0 } };
    state.flows[{ 0, station(8), station(6) }] = { { station(9), std::numeric_limits<uint32_t>::max(), 0 } };
    state.flows[{ 0, station(8), station(7) }] = { { station(9), std::numeric_limits<uint32_t>::max(), 0 } };
    state.flows[{ 1, station(1), station(6) }] = { { station(2), 100, 0 } };

    const auto edges = getPlannedServiceEdges(0);

    ASSERT_EQ(edges.size(), 5);
    const auto* outbound = findEdge(edges, station(1), station(2));
    ASSERT_NE(outbound, nullptr);
    EXPECT_EQ(outbound->plannedDemand, 60);
    ASSERT_TRUE(outbound->capacity.has_value());
    EXPECT_EQ(*outbound->capacity, 50);

    const auto* unserved = findEdge(edges, station(1), station(3));
    ASSERT_NE(unserved, nullptr);
    EXPECT_EQ(unserved->plannedDemand, 10);
    EXPECT_FALSE(unserved->capacity.has_value());

    const auto* inbound = findEdge(edges, station(2), station(1));
    ASSERT_NE(inbound, nullptr);
    EXPECT_EQ(inbound->plannedDemand, 7);
    ASSERT_TRUE(inbound->capacity.has_value());
    EXPECT_EQ(*inbound->capacity, 75);

    const auto* unused = findEdge(edges, station(4), station(5));
    ASSERT_NE(unused, nullptr);
    EXPECT_EQ(unused->plannedDemand, 0);
    ASSERT_TRUE(unused->capacity.has_value());
    EXPECT_EQ(*unused->capacity, 30);
    ASSERT_TRUE(unused->serviceCapacity.has_value());
    EXPECT_EQ(*unused->serviceCapacity, 30);

    const auto* large = findEdge(edges, station(8), station(9));
    ASSERT_NE(large, nullptr);
    EXPECT_EQ(large->plannedDemand, static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) * 2);
}

TEST(CargoFlowOverlayTest, MapsDemandToJgrStyleSaturationBuckets)
{
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(0, std::nullopt), 0);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(1, std::nullopt), 11);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(1, 0), 11);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(1, 1), 5);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(2, 1), 11);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(50, 100), 2);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(100, 100), 5);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(101, 100), 6);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(200, 100), 11);
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(std::numeric_limits<uint64_t>::max(), 100), 11);
}

TEST(CargoFlowOverlayTest, ScalesAbsoluteValuesToMapMaximum)
{
    const std::array<uint64_t, 4> values = { 0, 10, 50, 100 };

    const auto buckets = CargoFlowOverlay::calculateScaleBuckets(values, CargoFlowOverlay::ScaleMode::absolute);

    EXPECT_EQ(buckets, (std::vector<uint8_t>{ 0, 2, 6, 11 }));
}

TEST(CargoFlowOverlayTest, ScalesActualLoadAgainstOneHundredPercent)
{
    const std::array<uint64_t, 4> values = { 0, 2500, 5000, 10'000 };

    const auto buckets = CargoFlowOverlay::calculateScaleBuckets(values, CargoFlowOverlay::ScaleMode::absolute, 10'000);

    EXPECT_EQ(buckets, (std::vector<uint8_t>{ 0, 3, 6, 11 }));
}

TEST(CargoFlowOverlayTest, ScalesValuesByPercentileRank)
{
    const std::array<uint64_t, 5> values = { 0, 10, 20, 30, 40 };

    const auto buckets = CargoFlowOverlay::calculateScaleBuckets(values, CargoFlowOverlay::ScaleMode::percentiles);

    EXPECT_EQ(buckets, (std::vector<uint8_t>{ 0, 3, 6, 9, 11 }));
}

TEST(CargoFlowOverlayTest, CalculatesOctileTileDistance)
{
    constexpr auto kTile = World::kTileSize;

    EXPECT_DOUBLE_EQ(CargoFlowOverlay::calculateOctileTileDistance({ 0, 0 }, { 0, 0 }), 0.0);
    EXPECT_DOUBLE_EQ(CargoFlowOverlay::calculateOctileTileDistance({ 0, 0 }, { kTile * 3, 0 }), 3.0);
    EXPECT_NEAR(CargoFlowOverlay::calculateOctileTileDistance({ 0, 0 }, { kTile, kTile }), std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(CargoFlowOverlay::calculateOctileTileDistance({ 0, 0 }, { kTile * 3, kTile * 2 }), 1.0 + 2.0 * std::sqrt(2.0), 1e-9);
}

TEST(CargoFlowOverlayTest, GivesEqualPercentileValuesTheSameBucket)
{
    const std::array<uint64_t, 3> values = { 10, 10, 10 };

    const auto buckets = CargoFlowOverlay::calculateScaleBuckets(values, CargoFlowOverlay::ScaleMode::percentiles);

    EXPECT_EQ(buckets, (std::vector<uint8_t>{ 11, 11, 11 }));
}

TEST(CargoFlowOverlayTest, SummarisesCompletedDaysWithinSelectedHorizon)
{
    FlowAnalytics::State state;
    state.days = {
        { 69, { { { 0, station(1), station(2) }, 100, 100, 200, 300, uint64_t{ 400 } << 16 } } },
        { 70, { { { 0, station(1), station(2) }, 10, 10, 20, 20, uint64_t{ 30 } << 16 } } },
        { 99, {
                  { { 0, station(1), station(2) }, 1, 1, 2, 2, uint64_t{ 3 } << 16 },
                  { { 0, station(2), station(1) }, 4, 4, 8, 5, uint64_t{ 6 } << 16 },
                  { { 1, station(1), station(2) }, 7, 7, 9, 8, uint64_t{ 9 } << 16 },
              } },
        { 100, { { { 0, station(1), station(2) }, 1000, 1000, 2000, 2000, uint64_t{ 3000 } << 16 } } },
    };

    const auto summaries = FlowAnalytics::summarise(state, 0, 30, 100);

    ASSERT_EQ(summaries.size(), 2);
    EXPECT_EQ(summaries[0], (FlowAnalytics::ServiceSummary{ station(1), station(2), 11, 11, 22, 22, uint64_t{ 33 } << 16 }));
    EXPECT_EQ(summaries[1], (FlowAnalytics::ServiceSummary{ station(2), station(1), 4, 4, 8, 5, uint64_t{ 6 } << 16 }));
}

TEST(CargoFlowOverlayTest, AllocatesLatentDemandByAttraction)
{
    std::vector<FlowAnalytics::Endpoint> endpoints{
        { { FlowAnalytics::EndpointKind::town, 0 }, { 0, 0 }, {}, 100, 0, 0 },
        { { FlowAnalytics::EndpointKind::town, 1 }, { 0, 0 }, {}, 0, 1, 0 },
        { { FlowAnalytics::EndpointKind::town, 2 }, { 0, 0 }, {}, 0, 3, 0 },
    };

    const auto flows = FlowAnalytics::allocateLatentDemand(endpoints, 0);

    ASSERT_EQ(flows.size(), 2);
    EXPECT_EQ(flows[0].origin, endpoints[0].key);
    EXPECT_EQ(flows[0].destination, endpoints[1].key);
    EXPECT_EQ(flows[0].demand, 25);
    EXPECT_EQ(flows[1].destination, endpoints[2].key);
    EXPECT_EQ(flows[1].demand, 75);
}

TEST(CargoFlowOverlayTest, AllocatesLatentDemandByDistance)
{
    std::vector<FlowAnalytics::Endpoint> endpoints{
        { { FlowAnalytics::EndpointKind::industry, 0 }, { 0, 0 }, {}, 100, 0, 0 },
        { { FlowAnalytics::EndpointKind::industry, 1 }, { 1024, 0 }, {}, 0, 1, 0 },
        { { FlowAnalytics::EndpointKind::industry, 2 }, { 2048, 0 }, {}, 0, 1, 0 },
    };

    const auto flows = FlowAnalytics::allocateLatentDemand(endpoints, 100);

    ASSERT_EQ(flows.size(), 2);
    EXPECT_EQ(flows[0].demand, 80);
    EXPECT_EQ(flows[1].demand, 20);
}

TEST(CargoFlowOverlayTest, RecordsSameEndpointDemandLocally)
{
    std::vector<FlowAnalytics::Endpoint> endpoints{
        { { FlowAnalytics::EndpointKind::town, 0 }, { 0, 0 }, {}, 5, 1, 0 },
        { { FlowAnalytics::EndpointKind::town, 1 }, { 0, 0 }, {}, 0, 1, 0 },
    };

    const auto flows = FlowAnalytics::allocateLatentDemand(endpoints, 0);

    ASSERT_EQ(flows.size(), 1);
    EXPECT_EQ(endpoints[0].localDemand, 3);
    EXPECT_EQ(flows[0].demand, 2);
}

TEST(CargoFlowOverlayTest, LatentDemandConservesMaximumSupply)
{
    constexpr auto kMaximum = std::numeric_limits<uint64_t>::max();
    std::vector<FlowAnalytics::Endpoint> endpoints{
        { { FlowAnalytics::EndpointKind::industry, 0 }, { 0, 0 }, {}, kMaximum, 0, 0 },
        { { FlowAnalytics::EndpointKind::industry, 1 }, { 0, 0 }, {}, 0, kMaximum, 0 },
        { { FlowAnalytics::EndpointKind::industry, 2 }, { 0, 0 }, {}, 0, kMaximum, 0 },
    };

    const auto flows = FlowAnalytics::allocateLatentDemand(endpoints, 0);

    ASSERT_EQ(flows.size(), 2);
    EXPECT_EQ(flows[0].demand + flows[1].demand, kMaximum);
}

TEST(CargoFlowOverlayTest, RestoringEmptyStateClearsHistory)
{
    const auto originalDay = getCurrentDay();
    setCurrentDay(100);
    FlowAnalytics::reset();
    FlowAnalytics::recordDeparture(0, station(1), station(2), 10, 20);
    ASSERT_FALSE(FlowAnalytics::isDefault(FlowAnalytics::captureState()));

    EXPECT_TRUE(FlowAnalytics::restoreState({}));
    EXPECT_TRUE(FlowAnalytics::isDefault(FlowAnalytics::captureState()));
    setCurrentDay(originalDay);
}

TEST(CargoFlowOverlayTest, SamplesThroughputPlanAndDailyServiceCapacity)
{
    const auto originalDay = getCurrentDay();
    reset();
    auto& state = getState();
    state.settings.modes[0] = DistributionMode::asymmetric;
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(1, 0), servicePoint(1, 1) }] = { 40, 10, 48, 96, 40 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 30, 0, servicePoint(1, 0), servicePoint(1, 1) },
    };
    setCurrentDay(99);
    FlowAnalytics::recordDeparture(0, station(1), station(2), 10, 20);
    setCurrentDay(100);
    FlowAnalytics::updateDaily();

    const auto summaries = FlowAnalytics::getServiceSummaries(0, 30);

    ASSERT_EQ(summaries.size(), 1);
    EXPECT_EQ(summaries[0].throughput, 10);
    EXPECT_EQ(summaries[0].observedThroughput, 10);
    EXPECT_EQ(summaries[0].offeredCapacity, 20);
    EXPECT_EQ(summaries[0].plannedDemand, 30);
    EXPECT_EQ(FlowAnalytics::roundCapacity(summaries[0].capacityQ16), 40);
    setCurrentDay(originalDay);
    reset();
}

TEST(CargoFlowOverlayTest, WeightsActualLoadByDepartureCapacity)
{
    const auto originalDay = getCurrentDay();
    reset();
    setCurrentDay(99);
    FlowAnalytics::recordDeparture(0, station(1), station(2), 10, 10);
    FlowAnalytics::recordDeparture(0, station(1), station(2), 5, 10);
    FlowAnalytics::recordDeparture(0, station(1), station(2), 0, 10);
    setCurrentDay(100);
    FlowAnalytics::updateDaily();

    const auto summaries = FlowAnalytics::getServiceSummaries(0, 30);

    ASSERT_EQ(summaries.size(), 1);
    EXPECT_EQ(summaries[0].throughput, 15);
    EXPECT_EQ(summaries[0].observedThroughput, 15);
    EXPECT_EQ(summaries[0].offeredCapacity, 30);
    setCurrentDay(originalDay);
    reset();
}

TEST(CargoFlowOverlayTest, KeepsNewObservationsSeparateFromLegacySameDayThroughput)
{
    const auto originalDay = getCurrentDay();
    FlowAnalytics::State state;
    state.days = {
        { 99, { { { 0, station(1), station(2) }, 100, 0, 0, 0, 0 } } },
    };
    ASSERT_TRUE(FlowAnalytics::restoreState(state));
    setCurrentDay(99);
    FlowAnalytics::recordDeparture(0, station(1), station(2), 5, 10);
    setCurrentDay(100);
    FlowAnalytics::updateDaily();

    const auto summaries = FlowAnalytics::getServiceSummaries(0, 30);

    ASSERT_EQ(summaries.size(), 1);
    EXPECT_EQ(summaries[0].throughput, 105);
    EXPECT_EQ(summaries[0].observedThroughput, 5);
    EXPECT_EQ(summaries[0].offeredCapacity, 10);
    setCurrentDay(originalDay);
    reset();
}

TEST(CargoFlowOverlayTest, RejectsObservedThroughputAboveCapacity)
{
    FlowAnalytics::State state;
    state.days = {
        { 99, { { { 0, station(1), station(2) }, 11, 11, 10, 0, 0 } } },
    };

    EXPECT_FALSE(FlowAnalytics::validateState(state));
}

TEST(CargoFlowOverlayTest, SeparatesStoppingAndLimitedStopLinks)
{
    reset();
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(1, 0), servicePoint(1, 1) }] = { 40, 10, 2, 4, 40 };
    state.serviceEdges[{ 0, station(2), station(3), servicePoint(1, 1), servicePoint(1, 2) }] = { 40, 10, 2, 4, 40 };
    state.serviceEdges[{ 0, station(1), station(3), servicePoint(2, 0), servicePoint(2, 1) }] = { 20, 12, 5, 10, 20 };
    state.flows[{ 0, station(1), station(1), {}, station(3) }] = {
        { station(2), 120, 0, servicePoint(1, 0), servicePoint(1, 1) },
        { station(3), 80, 0, servicePoint(2, 0), servicePoint(2, 1) },
    };
    state.flows[{ 0, station(2), station(1), servicePoint(1, 1), station(3) }] = {
        { station(3), 120, 0, servicePoint(1, 1), servicePoint(1, 2) },
    };

    const auto edges = getPlannedServiceEdges(0);

    const auto* firstStoppingLeg = findEdge(edges, station(1), station(2));
    const auto* secondStoppingLeg = findEdge(edges, station(2), station(3));
    const auto* limitedStopLeg = findEdge(edges, station(1), station(3));
    ASSERT_NE(firstStoppingLeg, nullptr);
    ASSERT_NE(secondStoppingLeg, nullptr);
    ASSERT_NE(limitedStopLeg, nullptr);
    EXPECT_EQ(firstStoppingLeg->plannedDemand, 120);
    EXPECT_EQ(secondStoppingLeg->plannedDemand, 120);
    EXPECT_EQ(limitedStopLeg->plannedDemand, 80);
    EXPECT_EQ(*firstStoppingLeg->capacity, 40);
    EXPECT_EQ(*limitedStopLeg->capacity, 20);
}

TEST(CargoFlowOverlayTest, AggregateLinkUsesBusiestServiceSaturation)
{
    reset();
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(1, 0), servicePoint(1, 1) }] = { 10, 5, 2, 4, 40 };
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(2, 0), servicePoint(2, 1) }] = { 100, 10, 2, 4, 100 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 20, 0, servicePoint(1, 0), servicePoint(1, 1) },
    };
    state.stationCargo[{ station(1), 0 }].append({ 20, station(1), station(2), 0, servicePoint(1, 0), servicePoint(1, 1), station(2) });

    const auto links = getPlannedServiceEdges(0);
    const auto* link = findEdge(links, station(1), station(2));

    ASSERT_NE(link, nullptr);
    EXPECT_EQ(link->plannedDemand, 20);
    EXPECT_EQ(*link->capacity, 140);
    EXPECT_EQ(link->servicePlannedDemand, 20);
    EXPECT_EQ(link->committedDemand, 20);
    EXPECT_EQ(link->waitingDemand, 20);
    EXPECT_EQ(link->incomingDemand, 0);
    EXPECT_EQ(*link->serviceCapacity, 40);
    EXPECT_EQ(link->serviceDeparture, servicePoint(1, 0));
    EXPECT_EQ(link->serviceArrival, servicePoint(1, 1));
    EXPECT_EQ(CargoFlowOverlay::getSaturationBucket(link->committedDemand, link->serviceCapacity), 2);
}

TEST(CargoFlowOverlayTest, SeparatesFutureTransferPlanFromWaitingCargo)
{
    reset();
    constexpr auto departure = servicePoint(2369, 0);
    constexpr auto arrival = servicePoint(2369, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(78), station(79), departure, arrival }] = { 200, 10, 2, 4, 200 };
    state.flows[{ 0, station(78), station(46), servicePoint(2362, 5), station(80) }] = {
        { station(79), 597, 0, departure, arrival },
    };
    state.stationCargo[{ station(78), 0 }].append({ 1477, station(46), station(77), 0, servicePoint(2362, 5), servicePoint(2362, 6) });

    const auto links = getPlannedServiceEdges(0);
    const auto* link = findEdge(links, station(78), station(79));

    ASSERT_NE(link, nullptr);
    EXPECT_EQ(link->servicePlannedDemand, 597);
    EXPECT_EQ(link->committedDemand, 0);
    EXPECT_EQ(link->waitingDemand, 0);
    EXPECT_EQ(link->incomingDemand, 0);
    EXPECT_EQ(link->serviceDeparture, departure);
    EXPECT_EQ(link->serviceArrival, arrival);
}

TEST(CargoFlowOverlayTest, DrawsCompleteLinesInEitherDirection)
{
    std::array<uint8_t, 25> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 5, 5, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);

    drawingCtx.drawLine({ 4, 0 }, { 0, 4 }, 1);
    for (auto i = 0; i < 5; ++i)
    {
        EXPECT_EQ(pixels[i * 5 + (4 - i)], 1);
    }

    pixels.fill(0);
    drawingCtx.drawLine({ 4, 2 }, { 0, 2 }, 2);
    for (auto x = 0; x < 5; ++x)
    {
        EXPECT_EQ(pixels[2 * 5 + x], 2);
    }

    pixels.fill(0);
    drawingCtx.drawLine({ 2, 4 }, { 2, 0 }, 3);
    for (auto y = 0; y < 5; ++y)
    {
        EXPECT_EQ(pixels[y * 5 + 2], 3);
    }
}

TEST(CargoFlowOverlayTest, DrawsDestinationEndpointMarkers)
{
    std::array<uint8_t, 25> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 5, 5, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);

    const std::array markers = {
        CargoFlowOverlay::ProjectedMarker{ { 2, 2 }, PaletteIndex::yellowB },
    };
    CargoFlowOverlay::drawMarkers(drawingCtx, markers);

    EXPECT_EQ(pixels[2], PaletteIndex::black3);
    EXPECT_EQ(pixels[10], PaletteIndex::black3);
    EXPECT_EQ(pixels[12], PaletteIndex::yellowB);
    EXPECT_EQ(pixels[14], PaletteIndex::black3);
    EXPECT_EQ(pixels[22], PaletteIndex::black3);
}

TEST(CargoFlowOverlayTest, ClipsLinksThatCrossTheViewport)
{
    std::array<uint8_t, 25> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 5, 5, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);
    const std::array links = {
        CargoFlowOverlay::ProjectedLink{ { -5, 2 }, { 9, 2 }, PaletteIndex::red6 },
    };

    CargoFlowOverlay::drawLinks(drawingCtx, links);

    for (auto x = 0; x < 5; ++x)
    {
        EXPECT_EQ(pixels[2 * 5 + x], PaletteIndex::black3);
    }
}
