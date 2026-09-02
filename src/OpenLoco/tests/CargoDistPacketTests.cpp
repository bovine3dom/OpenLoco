// SPDX-License-Identifier: MIT
#include "Ui/CargoRouteTree.h"
#include <OpenLoco/CargoDist/CargoDist.h>

#include <algorithm>
#include <gtest/gtest.h>
#include <limits>
#include <set>

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

TEST(CargoDistPackets, SplitsTransferCreditProportionally)
{
    PacketList packets;
    packets.append({ 3, station(1), station(2), 3, {}, {}, station(4), 10 });

    const auto taken = packets.take(1);

    ASSERT_EQ(taken.size(), 1U);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(taken.packets().front().transferCredit, 3);
    EXPECT_EQ(packets.packets().front().transferCredit, 7);
}

TEST(CargoDistPackets, CoalescesTransferCredits)
{
    PacketList packets;
    packets.append({ 2, station(1), station(2), 3, {}, {}, station(4), 5 });
    packets.append({ 3, station(1), station(2), 3, {}, {}, station(4), 7 });

    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.quantity(), 5U);
    EXPECT_EQ(packets.packets().front().transferCredit, 12);
}

TEST(CargoDistPackets, CoalescesMatchingHolidayPacketsAcrossOtherCohorts)
{
    CargoPacket outbound{ 2, station(1), station(2), 3, {}, {}, station(4), 5 };
    outbound.tripKind = PassengerTripKind::holidayOutbound;
    outbound.holidayIndustry = IndustryId(3);
    outbound.homeTown = TownId(4);
    auto returning = outbound;
    returning.tripKind = PassengerTripKind::holidayReturn;
    returning.transferCredit = 6;
    PacketList packets;
    packets.append(outbound);
    packets.append(returning);
    outbound.quantity = 3;
    outbound.transferCredit = 7;
    packets.append(outbound);

    ASSERT_EQ(packets.size(), 2U);
    const auto merged = std::ranges::find_if(packets.packets(), [](const auto& packet) { return packet.tripKind == PassengerTripKind::holidayOutbound; });
    ASSERT_NE(merged, packets.packets().end());
    EXPECT_EQ(merged->quantity, 5);
    EXPECT_EQ(merged->transferCredit, 12);
}

TEST(CargoDistPackets, RatingLossRemovesTripKindsInPacketOrder)
{
    CargoPacket outbound{ 5, station(1), station(2), 0, {}, {}, station(2) };
    outbound.tripKind = PassengerTripKind::holidayOutbound;
    outbound.holidayIndustry = IndustryId(3);
    outbound.homeTown = TownId(4);
    auto returning = outbound;
    returning.quantity = 7;
    returning.tripKind = PassengerTripKind::holidayReturn;
    PacketList packets;
    packets.append(outbound);
    packets.append(returning);
    packets.append({ 11, station(1), station(2), 0, {}, {}, station(2) });

    EXPECT_EQ(packets.size(), 3);
    EXPECT_EQ(packets.removeForRating(20), 20);
    ASSERT_EQ(packets.size(), 1);
    EXPECT_EQ(packets.packets().front().tripKind, PassengerTripKind::holidayReturn);
    EXPECT_EQ(packets.quantity(), 3);
}

TEST(CargoDistPackets, RemovesExpiredJourneys)
{
    PacketList packets;
    packets.append({ 5, station(1), station(2), std::numeric_limits<uint8_t>::max() - 1 });
    packets.append({ 7, station(1), station(2), std::numeric_limits<uint8_t>::max() });
    auto returning = CargoPacket{ 9, station(2), station(1), std::numeric_limits<uint8_t>::max() };
    returning.tripKind = PassengerTripKind::holidayReturn;
    packets.append(returning);

    EXPECT_EQ(packets.removeExpired(), 16);
    ASSERT_EQ(packets.size(), 1);
    EXPECT_EQ(packets.quantity(), 5);
}

TEST(CargoDistPackets, RepeatedSplitsConserveMaximumTransferCredit)
{
    constexpr auto kQuantity = std::numeric_limits<uint16_t>::max();
    constexpr auto kCredit = static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * kQuantity;
    PacketList packets;
    packets.append({ kQuantity, station(1), station(2), 3, {}, {}, station(4), kCredit });

    const auto first = packets.take(1);
    const auto second = packets.take(kQuantity / 2);

    ASSERT_EQ(first.size(), 1U);
    ASSERT_EQ(second.size(), 1U);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(first.packets().front().transferCredit + second.packets().front().transferCredit + packets.packets().front().transferCredit, kCredit);
}

TEST(CargoDistPackets, SummarisesRoutesAcrossPacketCohorts)
{
    PacketList packets;
    packets.append({ 2, station(1), station(2), 3, {}, {}, station(4) });
    packets.append({ 3, station(1), station(2), 4, {}, {}, station(4) });
    packets.append({ 4, station(1), station(3), 3, {}, {}, station(4) });
    packets.append({ 5, station(2), station(3), 3, {}, {}, station(4) });
    packets.append({ 1, station(1), StationId::null, 3 });

    const auto summaries = getRouteSummaries(packets);

    ASSERT_EQ(summaries.size(), 4U);
    EXPECT_EQ(summaries[0], (CargoRouteSummary{ station(1), station(4), station(2), 5 }));
    EXPECT_EQ(summaries[1], (CargoRouteSummary{ station(1), station(4), station(3), 4 }));
    EXPECT_EQ(summaries[2], (CargoRouteSummary{ station(1), StationId::null, StationId::null, 1 }));
    EXPECT_EQ(summaries[3], (CargoRouteSummary{ station(2), station(4), station(3), 5 }));
}

TEST(CargoDistPackets, RouteSummaryQuantityDoesNotWrap)
{
    constexpr size_t kPacketCount = 65538;
    constexpr auto kPacketQuantity = std::numeric_limits<uint16_t>::max();
    PacketList::Container cohorts(kPacketCount, CargoPacket{ kPacketQuantity, station(1), station(2), 3, {}, {}, station(4) });

    const auto summaries = getRouteSummaries(PacketList::fromPackets(std::move(cohorts)));

    ASSERT_EQ(summaries.size(), 1U);
    EXPECT_EQ(summaries.front().quantity, static_cast<uint64_t>(kPacketCount) * kPacketQuantity);
}

TEST(CargoDistPackets, GroupsRouteSummariesInRequestedOrder)
{
    const std::vector summaries = {
        CargoRouteSummary{ station(1), station(4), station(2), 5 },
        CargoRouteSummary{ station(1), station(4), station(3), 4 },
        CargoRouteSummary{ station(1), StationId::null, StationId::null, 1 },
        CargoRouteSummary{ station(2), station(4), station(3), 5 },
    };
    constexpr std::array order = {
        CargoRouteField::origin,
        CargoRouteField::destination,
        CargoRouteField::nextHop,
    };

    const auto tree = getRouteTree(summaries, order);

    EXPECT_EQ(tree, (std::vector<CargoRouteNode>{
                        { station(1), 10, {
                                              { station(4), 9, { { station(2), 5, {} }, { station(3), 4, {} } } },
                                              { StationId::null, 1, { { StationId::null, 1, {} } } },
                                          } },
                        { station(2), 5, { { station(4), 5, { { station(3), 5, {} } } } } },
                    }));

    constexpr std::array destinationFirst = {
        CargoRouteField::destination,
        CargoRouteField::origin,
        CargoRouteField::nextHop,
    };
    const auto destinationTree = getRouteTree(summaries, destinationFirst);
    EXPECT_EQ(destinationTree, (std::vector<CargoRouteNode>{
                                   { station(4), 14, {
                                                         { station(1), 9, { { station(2), 5, {} }, { station(3), 4, {} } } },
                                                         { station(2), 5, { { station(3), 5, {} } } },
                                                     } },
                                   { StationId::null, 1, { { station(1), 1, { { StationId::null, 1, {} } } } } },
                               }));

    constexpr std::array invalidOrder = {
        CargoRouteField::origin,
        CargoRouteField::origin,
        CargoRouteField::nextHop,
    };
    EXPECT_TRUE(getRouteTree(summaries, invalidOrder).empty());
}

TEST(CargoDistPackets, RouteTreeQuantityDoesNotWrap)
{
    const std::vector summaries = {
        CargoRouteSummary{ station(1), station(2), station(2), std::numeric_limits<uint64_t>::max() },
        CargoRouteSummary{ station(1), station(2), station(2), 1 },
    };
    constexpr std::array order = {
        CargoRouteField::origin,
        CargoRouteField::destination,
        CargoRouteField::nextHop,
    };

    const auto tree = getRouteTree(summaries, order);

    constexpr auto kMax = std::numeric_limits<uint64_t>::max();
    EXPECT_EQ(tree, (std::vector<CargoRouteNode>{ { station(1), kMax, { { station(2), kMax, { { station(2), kMax, {} } } } } } }));
}

TEST(CargoRouteTree, FlattensExpandedGroupsWithinRowLimit)
{
    const std::vector<CargoRouteNode> tree = {
        { station(1), 10, { { station(2), 10, { { station(3), 10, {} } } } } },
        { station(4), 5, {} },
    };
    Ui::CargoRouteTree::GroupKey sourceKey{};
    sourceKey.depth = 1;
    sourceKey.stations[0] = station(1);
    auto destinationKey = sourceKey;
    destinationKey.depth = 2;
    destinationKey.stations[1] = station(2);
    const std::set expandedGroups = { sourceKey, destinationKey };

    std::vector<Ui::CargoRouteTree::Row> rows;
    size_t omittedRows = 0;
    Ui::CargoRouteTree::appendRows(rows, tree, Ui::CargoRouteTree::GroupOrder::sourceDestinationVia, expandedGroups, 2, omittedRows);

    ASSERT_EQ(rows.size(), 2U);
    EXPECT_EQ(rows[0].station, station(1));
    EXPECT_EQ(rows[0].field, CargoRouteField::origin);
    EXPECT_TRUE(rows[0].expanded);
    EXPECT_EQ(rows[1].station, station(2));
    EXPECT_EQ(rows[1].field, CargoRouteField::destination);
    EXPECT_TRUE(rows[1].expanded);
    EXPECT_EQ(omittedRows, 2U);
}

TEST(CargoRouteTree, ExpandsAllNestedGroups)
{
    const std::vector<CargoRouteNode> tree = {
        { station(1), 10, { { station(2), 10, { { station(3), 10, {} } } } } },
        { station(4), 5, {} },
    };

    std::set<Ui::CargoRouteTree::GroupKey> expandedGroups;
    Ui::CargoRouteTree::expandAllGroups(expandedGroups, tree);

    Ui::CargoRouteTree::GroupKey sourceKey{};
    sourceKey.depth = 1;
    sourceKey.stations[0] = station(1);
    auto destinationKey = sourceKey;
    destinationKey.depth = 2;
    destinationKey.stations[1] = station(2);

    EXPECT_TRUE(expandedGroups.contains(sourceKey));
    EXPECT_TRUE(expandedGroups.contains(destinationKey));
    EXPECT_EQ(expandedGroups.size(), 2U);
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

TEST(CargoDistPackets, AgesAndExpiresOnlyTransferredCargoAtStation)
{
    PacketList packets;
    packets.append({ 10, station(1), station(2), 0 });
    packets.append({ 10, station(3), station(2), 254 });
    packets.ageAtStation(station(1));

    EXPECT_EQ(packets.packets()[0].age, 0);
    EXPECT_EQ(packets.packets()[1].age, 255);
    EXPECT_EQ(packets.removeExpired(), 10);
    ASSERT_EQ(packets.size(), 1);
    EXPECT_EQ(packets.packets().front().origin, station(1));
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

TEST(CargoDistPackets, RemovingHomeStationPreservesHolidayPassengerMetadata)
{
    CargoPacket packet{ 10, station(1), station(3), 0, {}, {}, station(4) };
    packet.tripKind = PassengerTripKind::holidayOutbound;
    packet.holidayIndustry = IndustryId(2);
    packet.homeTown = TownId(5);
    PacketList packets;
    packets.append(packet);

    packets.removeStationReferences(station(1));

    ASSERT_EQ(packets.size(), 1);
    EXPECT_EQ(packets.packets().front().origin, station(3));
    EXPECT_EQ(packets.packets().front().tripKind, PassengerTripKind::holidayOutbound);
    EXPECT_EQ(packets.packets().front().homeTown, TownId(5));
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
