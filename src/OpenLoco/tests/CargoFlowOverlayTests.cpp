// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/CargoDist.h>
#include <OpenLoco/Graphics/RenderTarget.h>
#include <OpenLoco/Graphics/SoftwareDrawingContext.h>
#include <OpenLoco/Ui/Windows/CargoFlowOverlay.h>

#include <algorithm>
#include <array>
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
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(1, 0), servicePoint(1, 1) }] = { 20, 10, 2 };
    state.serviceEdges[{ 0, station(1), station(2), servicePoint(2, 0), servicePoint(2, 1) }] = { 30, 12, 3 };
    state.serviceEdges[{ 0, station(2), station(1) }] = { 75, 10 };
    state.serviceEdges[{ 0, station(4), station(5) }] = { 30, 10 };
    state.serviceEdges[{ 1, station(1), station(2) }] = { 200, 10 };
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
