// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/CargoDist.h>

#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;

namespace
{
    constexpr StationId station(uint16_t value)
    {
        return static_cast<StationId>(value);
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
        FlowShare{ station(1), station(1), station(2), 3 },
        FlowShare{ station(1), station(1), station(3), 1 },
    };
    setFlows(0, flows);

    uint32_t via2 = 0;
    uint32_t via3 = 0;
    for (auto i = 0; i < 40; ++i)
    {
        const auto via = chooseVia(0, station(1), station(1));
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
        FlowShare{ station(1), station(1), station(2), 3 },
        FlowShare{ station(1), station(1), station(3), 1 },
    };
    setFlows(0, flows);

    const auto shares = allocateVia(0, station(1), station(1), 40);

    ASSERT_EQ(shares.size(), 2U);
    EXPECT_EQ(shares[0].via, station(2));
    EXPECT_EQ(shares[0].amount, 30U);
    EXPECT_EQ(shares[1].via, station(3));
    EXPECT_EQ(shares[1].amount, 10U);
}

TEST(CargoDistPackets, RemovingStationDropsOriginsButOnlyClearsNextHops)
{
    PacketList packets;
    packets.append({ 10, station(1), station(3), 0 });
    packets.append({ 20, station(2), station(1), 0 });

    packets.removeStationReferences(station(1));

    ASSERT_EQ(packets.packets().size(), 1);
    EXPECT_EQ(packets.quantity(), 20U);
    EXPECT_EQ(packets.packets()[0].origin, station(2));
    EXPECT_EQ(packets.packets()[0].nextHop, StationId::null);
}
