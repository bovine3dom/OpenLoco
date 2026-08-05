// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/CargoDist.h>

#include <algorithm>
#include <gtest/gtest.h>

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
}

TEST(CargoDistPackets, AppendsAndCoalescesMatchingPackets)
{
    PacketList packets;
    packets.append({ 20, station(1), station(2), 3 });
    packets.append({ 30, station(1), station(2), 3 });

    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.quantity(), 50U);
}

TEST(CargoDistPackets, TakesOnlyRequestedNextHop)
{
    PacketList packets;
    packets.append({ 20, station(1), station(2), 3 });
    packets.append({ 30, station(1), station(3), 4 });

    auto taken = packets.takeFor(station(3), 12);

    EXPECT_EQ(taken.quantity(), 12U);
    EXPECT_EQ(taken.quantityFor(station(3)), 12U);
    EXPECT_EQ(packets.quantity(), 38U);
    EXPECT_EQ(packets.quantityFor(station(3)), 18U);
}

TEST(CargoDistPackets, KeepsServicePlansDistinct)
{
    const auto departure1 = servicePoint(1, 2);
    const auto departure2 = servicePoint(2, 2);
    PacketList packets;
    packets.append({ 20, station(1), station(2), 3, departure1, servicePoint(1, 3) });
    packets.append({ 30, station(1), station(2), 3, departure2, servicePoint(2, 3) });

    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(packets.quantityFor(station(2)), 50U);
    EXPECT_EQ(packets.quantityFor(station(2), departure1), 20U);

    const auto taken = packets.takeFor(station(2), departure2, 12);
    EXPECT_EQ(taken.quantityFor(station(2), departure2), 12U);
    EXPECT_EQ(packets.quantityFor(station(2), departure2), 18U);
}

TEST(CargoDistPackets, KeepsDestinationsDistinct)
{
    PacketList packets;
    packets.append({ 20, station(1), station(2), 3, {}, {}, station(4) });
    packets.append({ 30, station(1), station(2), 3, {}, {}, station(5) });

    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(packets.takeForJourney(station(4), station(2), {}, 50).quantity(), 20U);
    EXPECT_EQ(packets.quantity(), 30U);
}

TEST(CargoDistPackets, RepresentativeOriginUsesLargestQuantityThenLowestId)
{
    PacketList packets;
    packets.append({ 20, station(3), station(5), 0 });
    packets.append({ 20, station(2), station(5), 1 });
    packets.append({ 10, station(4), station(5), 0 });

    EXPECT_EQ(packets.representativeOrigin(), station(2));
}

TEST(CargoDistPackets, AgesOnlyTransferredCargoAtStation)
{
    PacketList packets;
    packets.append({ 10, station(1), station(2), 0 });
    packets.append({ 10, station(3), station(2), 254 });
    packets.ageAtStation(station(1));

    EXPECT_EQ(packets.packets()[0].age, 0);
    EXPECT_EQ(packets.packets()[1].age, 255);
}

TEST(CargoDistPackets, SmoothFlowSelectionIsDeterministicAndWeighted)
{
    reset();
    const std::array flows = {
        FlowShare{ station(1), station(1), station(2), 3, {}, {}, {}, station(2) },
        FlowShare{ station(1), station(1), station(3), 1, {}, {}, {}, station(3) },
    };
    setFlows(0, flows);

    uint32_t via2 = 0;
    uint32_t via3 = 0;
    for (auto i = 0; i < 40; ++i)
    {
        const auto shares = allocateVia(0, station(1), station(1), 1);
        ASSERT_EQ(shares.size(), 1U);
        const auto via = shares.front().via;
        via2 += via == station(2);
        via3 += via == station(3);
    }

    EXPECT_EQ(via2, 30U);
    EXPECT_EQ(via3, 10U);
}

TEST(CargoDistPackets, BatchFlowAllocationIsWeightedByQuantity)
{
    reset();
    const std::array flows = {
        FlowShare{ station(1), station(1), station(2), 3, {}, {}, {}, station(2) },
        FlowShare{ station(1), station(1), station(3), 1, {}, {}, {}, station(3) },
    };
    setFlows(0, flows);

    const auto shares = allocateVia(0, station(1), station(1), 40);

    ASSERT_EQ(shares.size(), 2U);
    EXPECT_EQ(shares[0].via, station(2));
    EXPECT_EQ(shares[0].amount, 30U);
    EXPECT_EQ(shares[1].via, station(3));
    EXPECT_EQ(shares[1].amount, 10U);
}

TEST(CargoDistPackets, DestinationSelectionExcludesUnusableLocalFlow)
{
    reset();
    const std::array flows = {
        FlowShare{ station(2), station(1), station(2), 10, {}, {}, {}, station(2) },
        FlowShare{ station(2), station(1), station(3), 10, {}, {}, {}, station(3) },
    };
    setFlows(0, flows);

    const auto shares = allocateVia(0, station(2), station(1), StationId::null, 20, {}, station(2));

    ASSERT_EQ(shares.size(), 1U);
    EXPECT_EQ(shares.front().amount, 20U);
    EXPECT_EQ(shares.front().via, station(3));
    EXPECT_EQ(shares.front().destination, station(3));
}

TEST(CargoDistPackets, FilteredAllocationHandlesFullScaleCursors)
{
    reset();
    auto& options = getState().flows[{ 0, station(1), station(1), {}, station(6) }];
    options = {
        { station(2), 1, 500'000 },
        { station(3), 999'999, -500'000 },
    };

    const auto shares = allocateVia(0, station(1), station(1), station(6), 1, ServicePoint{}, station(3));

    ASSERT_EQ(shares.size(), 1U);
    EXPECT_EQ(shares.front().via, station(2));
    EXPECT_EQ(shares.front().amount, 1U);
    EXPECT_EQ(options[0].current, 500'000);
    EXPECT_EQ(options[1].current, -500'000);
}

TEST(CargoDistPackets, FilteredAllocationDoesNotStarveEligibleRoutes)
{
    reset();
    auto& options = getState().flows[{ 0, station(1), station(1), {}, station(6) }];
    options = {
        { station(2), 1, 0 },
        { station(3), 1, 3 },
        { station(4), 1, -3 },
    };

    uint32_t via2 = 0;
    uint32_t via3 = 0;
    for (size_t i = 0; i < 20; ++i)
    {
        const auto shares = allocateVia(0, station(1), station(1), station(6), 1, ServicePoint{}, station(4));
        ASSERT_EQ(shares.size(), 1U);
        via2 += shares.front().via == station(2);
        via3 += shares.front().via == station(3);
    }

    EXPECT_LE(std::max(via2, via3) - std::min(via2, via3), 2U);
}

TEST(CargoDistPackets, FilteredAllocationRescalesFullWeightCredit)
{
    reset();
    auto& options = getState().flows[{ 0, station(1), station(1), {}, station(6) }];
    options = {
        { station(2), 1999, 0 },
        { station(3), 1, 0 },
        { station(4), 1, 0 },
    };
    for (size_t i = 0; i < 668; ++i)
    {
        allocateVia(0, station(1), station(1), station(6), 1);
    }

    uint32_t via3 = 0;
    uint32_t via4 = 0;
    for (size_t i = 0; i < 1000; ++i)
    {
        const auto shares = allocateVia(0, station(1), station(1), station(6), 1, ServicePoint{}, station(2));
        ASSERT_EQ(shares.size(), 1U);
        via3 += shares.front().via == station(3);
        via4 += shares.front().via == station(4);
    }

    EXPECT_LE(std::max(via3, via4) - std::min(via3, via4), 2U);
}

TEST(CargoDistPackets, FilteredAllocationBalancesChangingEligibility)
{
    reset();
    auto& options = getState().flows[{ 0, station(1), station(1), {}, station(6) }];
    options = {
        { station(2), 1, 0 },
        { station(3), 1, 0 },
        { station(4), 1, 0 },
    };
    std::array<uint32_t, 3> counts{};

    for (size_t i = 0; i < 600; ++i)
    {
        const auto excluded = i % 2 == 0 ? station(4) : StationId::null;
        const auto shares = allocateVia(0, station(1), station(1), station(6), 1, ServicePoint{}, excluded);
        ASSERT_EQ(shares.size(), 1U);
        ++counts[static_cast<uint16_t>(shares.front().via) - 2];
    }

    EXPECT_NEAR(counts[0], 250, 2);
    EXPECT_NEAR(counts[1], 250, 2);
    EXPECT_NEAR(counts[2], 100, 2);
}

TEST(CargoDistPackets, RemovingStationDropsOriginsButOnlyClearsNextHops)
{
    PacketList packets;
    packets.append({ 10, station(1), station(3), 0, {}, {}, station(4) });
    packets.append({ 20, station(2), station(1), 0, servicePoint(4, 2), servicePoint(4, 3), station(4) });

    packets.removeStationReferences(station(1));

    ASSERT_EQ(packets.packets().size(), 1);
    EXPECT_EQ(packets.quantity(), 20U);
    EXPECT_EQ(packets.packets()[0].origin, station(2));
    EXPECT_EQ(packets.packets()[0].nextHop, StationId::null);
    EXPECT_TRUE(packets.packets()[0].departure.empty());
    EXPECT_TRUE(packets.packets()[0].arrival.empty());
    EXPECT_EQ(packets.packets()[0].destination, station(4));
}

TEST(CargoDistPackets, RemovingServiceClearsPlansButKeepsOrigins)
{
    PacketList packets;
    packets.append({ 20, station(2), station(3), 0, servicePoint(4, 2), servicePoint(4, 3) });
    packets.append({ 10, station(1), station(3), 0, servicePoint(5, 2), servicePoint(5, 3) });

    packets.removeServiceReferences(static_cast<ServiceId>(4));

    ASSERT_EQ(packets.size(), 2U);
    const auto cleared = std::ranges::find_if(packets.packets(), [](const auto& packet) { return packet.origin == station(2); });
    ASSERT_NE(cleared, packets.packets().end());
    EXPECT_EQ(cleared->nextHop, StationId::null);
    EXPECT_TRUE(cleared->departure.empty());
    EXPECT_TRUE(cleared->arrival.empty());
}

TEST(CargoDistPackets, RemovingDestinationKeepsCargoForReassignment)
{
    PacketList packets;
    packets.append({ 20, station(2), station(3), 0, servicePoint(4, 2), servicePoint(4, 3), station(1) });

    packets.removeStationReferences(station(1));

    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.quantity(), 20U);
    EXPECT_EQ(packets.packets().front().destination, StationId::null);
    EXPECT_EQ(packets.packets().front().nextHop, StationId::null);
}

TEST(CargoDistPackets, ClearingOnboardServiceKeepsNextHop)
{
    PacketList packets;
    packets.append({ 20, station(2), station(3), 0, servicePoint(4, 2), servicePoint(4, 3), station(5) });

    packets.removeServiceReferences(static_cast<ServiceId>(4), true);

    EXPECT_EQ(packets.packets().front().nextHop, station(3));
    EXPECT_TRUE(packets.packets().front().departure.empty());
    EXPECT_TRUE(packets.packets().front().arrival.empty());
}

TEST(CargoDistPackets, FlowAllocationDistinguishesIncomingAndOutgoingServices)
{
    reset();
    const auto incoming1 = servicePoint(1, 4);
    const auto incoming2 = servicePoint(2, 4);
    const auto departure1 = servicePoint(3, 5);
    const auto departure2 = servicePoint(4, 5);
    const std::array flows = {
        FlowShare{ station(1), station(9), station(2), 1, incoming1, departure2, servicePoint(4, 6), station(8) },
        FlowShare{ station(1), station(9), station(2), 1, incoming1, departure1, servicePoint(3, 6), station(8) },
        FlowShare{ station(1), station(9), station(3), 1, incoming2, servicePoint(5, 5), servicePoint(5, 6), station(8) },
    };
    setFlows(0, flows);

    const auto first = allocateVia(0, station(1), station(9), 1, incoming1);
    const auto second = allocateVia(0, station(1), station(9), 1, incoming1);
    const auto otherIncoming = allocateVia(0, station(1), station(9), 1, incoming2);

    ASSERT_EQ(first.size(), 1U);
    EXPECT_EQ(first.front().via, station(2));
    EXPECT_EQ(first.front().departure, departure1);
    EXPECT_EQ(first.front().arrival, servicePoint(3, 6));
    ASSERT_EQ(second.size(), 1U);
    EXPECT_EQ(second.front().departure, departure2);
    ASSERT_EQ(otherIncoming.size(), 1U);
    EXPECT_EQ(otherIncoming.front().via, station(3));
}
