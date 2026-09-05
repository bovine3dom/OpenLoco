// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Entities/EntityManager.h"
#include "GameState.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleTail.h"
#include "World/Station.h"
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;

namespace
{
    constexpr StationId station(uint16_t id)
    {
        return static_cast<StationId>(id);
    }

    constexpr ServicePoint point(uint16_t occurrence, uint16_t service = 7)
    {
        return { static_cast<ServiceId>(service), occurrence };
    }

    constexpr VehicleServiceLeg kFirst{ 0, station(1), station(2), point(0), point(1) };
    constexpr VehicleServiceLeg kSecond{ 0, station(2), station(3), point(1), point(2) };
    constexpr VehicleServiceLeg kThird{ 0, station(3), station(4), point(2), point(3) };
    constexpr VehicleServiceLeg kReturn{ 0, station(4), station(1), point(3), point(0) };

    CargoPacket packet(uint16_t quantity = 8, StationId destination = station(4))
    {
        return { quantity, station(1), station(2), 0, point(0), point(1), destination };
    }

    class CargoDistVehicleRoute : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            reset();
            EntityManager::reset();
            Vehicles::OrderManager::reset();
            for (size_t i = 0; i < _stations.size(); ++i)
            {
                auto& value = getGameState().stations[i + 1];
                _stations[i] = value;
                value = {};
                value.name = StringId(1);
                value.stationTileSize = 1;
                value.cargoStats[0].isAccepted(true);
            }
            head = createComponent<Vehicles::VehicleHead>();
            auto* veh1 = createComponent<Vehicles::Vehicle1>();
            auto* veh2 = createComponent<Vehicles::Vehicle2>();
            front = createComponent<Vehicles::VehicleBogie>();
            auto* back = createComponent<Vehicles::VehicleBogie>();
            body = createComponent<Vehicles::VehicleBody>();
            auto* tail = createComponent<Vehicles::VehicleTail>();
            body->setSubType(Vehicles::VehicleEntityType::body_start);
            const std::array<Vehicles::VehicleBase*, 7> components = { head, veh1, veh2, front, back, body, tail };
            for (size_t i = 0; i < components.size(); ++i)
            {
                components[i]->head = head->id;
                components[i]->setNextCar(i + 1 < components.size() ? components[i + 1]->id : EntityId::null);
            }
            body->primaryCargo = { 1, 0, 255, StationId::null, 0, 0 };
            front->secondaryCargo = body->primaryCargo;
            primary = { body->id, VehicleCargoSlot::primary };
            secondary = { front->id, VehicleCargoSlot::secondary };
            Vehicles::OrderManager::allocateOrders(*head);
            getState().vehicleServiceLegs[head->id] = { kFirst, kSecond, kThird, kReturn };
        }

        void TearDown() override
        {
            reset();
            Vehicles::OrderManager::reset();
            EntityManager::reset();
            for (size_t i = 0; i < _stations.size(); ++i)
            {
                getGameState().stations[i + 1] = _stations[i];
            }
        }

        template<typename T>
        static T* createComponent()
        {
            auto* entity = EntityManager::createEntityVehicle();
            entity->baseType = EntityBaseType::vehicle;
            auto* component = reinterpret_cast<T*>(entity);
            component->setSubType(T::kVehicleThingType);
            return component;
        }

        static std::vector<FlowOption>& flow(const VehicleServiceLeg& incoming, StationId destination = station(4))
        {
            return getState().flows[{ 0, incoming.to, station(1), incoming.arrival, destination }];
        }

        void setPackets(const PacketList& packets)
        {
            getState().vehicleCargo[primary] = packets;
            body->primaryCargo.qty = static_cast<uint8_t>(packets.quantity());
            ++getState().cargoRevision;
        }

        std::vector<CargoRouteSummary> preview(CargoPacket cargo = packet())
        {
            setPackets(PacketList::fromPackets({ cargo }));
            VehicleRoutePreview cache;
            cache.update(*head);
            return cache.summaries.at(primary);
        }

        void forceUnloadAt(uint16_t stop, uint8_t cargo = 0, bool afterRouteThrough = false)
        {
            auto& legs = getState().vehicleServiceLegs.at(head->id);
            for (uint16_t i = 1; i <= 4; ++i)
            {
                const Vehicles::OrderStopAt order{ station(i) };
                Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - 1, &order);
                legs[i - 1].currentOrder = head->sizeOfOrderTable - 1;
                if (i == stop)
                {
                    if (afterRouteThrough)
                    {
                        const Vehicles::OrderRouteThrough through{ station(5) };
                        Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - 1, &through);
                    }
                    const Vehicles::OrderUnloadAll unload{ cargo };
                    Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - 1, &unload);
                }
            }
            legs.back().currentOrder = 0;
            getState().servicesDirty = false;
        }

        Vehicles::VehicleHead* head{};
        Vehicles::VehicleBody* body{};
        Vehicles::VehicleBogie* front{};
        VehicleCargoKey primary{};
        VehicleCargoKey secondary{};

    private:
        std::array<Station, 5> _stations;
    };
}

TEST_F(CargoDistVehicleRoute, DirectAtDestinationOrAfterSameServiceStops)
{
    EXPECT_EQ(preview(packet(8, station(2))), (std::vector<CargoRouteSummary>{ { station(1), station(2), StationId::null, 8, true } }));
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 8, true } }));
}

TEST_F(CargoDistVehicleRoute, FirstTransferIsAfterRemainingSameServiceStops)
{
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(0, 8), point(1, 8) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(3), 8 } }));
}

TEST_F(CargoDistVehicleRoute, SplitsQuantitiesBetweenDirectAndFirstTransferStations)
{
    flow(kFirst) = {
        { station(3), 1, 0, point(1), point(2) },
        { station(4), 1, 0, point(0, 8), point(1, 8) },
    };
    flow(kSecond) = {
        { station(4), 1, 0, point(2), point(3) },
        { station(4), 1, 0, point(0, 9), point(1, 9) },
    };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{
                             { station(1), station(4), station(2), 4 },
                             { station(1), station(4), station(3), 2 },
                             { station(1), station(4), StationId::null, 2, true },
                         }));
    flow(kSecond).clear();
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{
                             { station(1), station(4), station(2), 4 },
                             { station(1), station(4), StationId::null, 4 },
                         }));
}

TEST_F(CargoDistVehicleRoute, UsesPrivateCursorsAcrossCohortsWithoutChangingState)
{
    flow(kFirst) = {
        { station(3), 1, 0, point(1), point(2) },
        { station(4), 1, 0, point(0, 8), point(1, 8) },
    };
    flow(kSecond) = { { station(4), 1, 0, point(2), point(3) } };
    auto older = packet(1);
    older.age = 1;
    const auto packets = PacketList::fromPackets({ packet(1), older });
    setPackets(packets);
    const auto routingRevision = getStateConst().routingRevision;
    const auto cargoRevision = getStateConst().cargoRevision;
    VehicleRoutePreview cache;
    ASSERT_TRUE(cache.update(*head));
    const auto summaries = cache.summaries.at(primary);
    EXPECT_EQ(summaries, (std::vector<CargoRouteSummary>{
                             { station(1), station(4), station(2), 1 },
                             { station(1), station(4), StationId::null, 1, true },
                         }));
    EXPECT_FALSE(cache.update(*head));
    VehicleRoutePreview another;
    EXPECT_TRUE(another.update(*head));
    EXPECT_EQ(another.summaries.at(primary), summaries);
    EXPECT_EQ(flow(kFirst)[0].current, 0);
    EXPECT_EQ(flow(kFirst)[1].current, 0);
    EXPECT_EQ(getStateConst().routingRevision, routingRevision);
    EXPECT_EQ(getStateConst().cargoRevision, cargoRevision);
    EXPECT_EQ(packets.packets()[0], packet(1));
    EXPECT_EQ(packets.packets()[1], older);
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST_F(CargoDistVehicleRoute, SharesCursorsAcrossCompartmentsInUnloadingOrder)
{
    flow(kFirst) = {
        { station(3), 1, 0, point(1), point(2) },
        { station(4), 1, 0, point(0, 8), point(1, 8) },
    };
    flow(kSecond) = { { station(4), 1, 0, point(2), point(3) } };
    setPackets(PacketList::fromPackets({ packet(1) }));
    getState().vehicleCargo[secondary] = getState().vehicleCargo[primary];
    front->secondaryCargo.qty = 1;

    VehicleRoutePreview cache;
    ASSERT_TRUE(cache.update(*head));
    // Showing only the primary compartment must still account for the collapsed secondary.
    EXPECT_EQ(cache.summaries.at(primary), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(2), 1 } }));
    EXPECT_EQ(cache.summaries.at(secondary), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 1, true } }));
    EXPECT_EQ(flow(kFirst)[0].current, 0);
    EXPECT_EQ(flow(kFirst)[1].current, 0);
    EXPECT_FALSE(cache.update(*head));

    auto& stationCargo = getGameState().stations[2].cargoStats[0];
    const auto first = unloadVehicleCargo(secondary, front->secondaryCargo, station(2), stationCargo, {}, false, kSecond, head->id);
    const auto second = unloadVehicleCargo(primary, body->primaryCargo, station(2), stationCargo, {}, false, kSecond, head->id);
    EXPECT_EQ(first.quantity(), 0U);
    EXPECT_EQ(second.transferred, 1U);
    EXPECT_EQ(front->secondaryCargo.qty, 1);
    EXPECT_EQ(body->primaryCargo.qty, 0);
}

TEST_F(CargoDistVehicleRoute, ForcedUnloadAtAcceptingStopIsDirectDeliveryThere)
{
    forceUnloadAt(2);
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(2), StationId::null, 8, true } }));
    EXPECT_EQ(getVehicleCargoConst(primary)->packets().front().destination, station(4));

    auto& stationCargo = getGameState().stations[2].cargoStats[0];
    const auto unloaded = unloadVehicleCargo(primary, body->primaryCargo, station(2), stationCargo, {}, true, kSecond, head->id);
    EXPECT_EQ(unloaded.delivered.quantity(), 8U);
    EXPECT_EQ(unloaded.transferred, 0U);
}

TEST_F(CargoDistVehicleRoute, ForcedUnloadCannotStayAboardAtNonAcceptingStop)
{
    forceUnloadAt(2);
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(2), 8 } }));

    auto& stationCargo = getGameState().stations[2].cargoStats[0];
    const auto unloaded = unloadVehicleCargo(primary, body->primaryCargo, station(2), stationCargo, {}, true, kSecond, head->id);
    EXPECT_EQ(unloaded.delivered.quantity(), 0U);
    EXPECT_EQ(unloaded.transferred, 8U);
    EXPECT_EQ(body->primaryCargo.qty, 0);
}

TEST_F(CargoDistVehicleRoute, UsesCargoOrdersAtLaterServiceStop)
{
    forceUnloadAt(3);
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(3), StationId::null, 8, true } }));
}

TEST_F(CargoDistVehicleRoute, IgnoresUnloadOrdersForOtherCargoOrBeyondRoutableOrders)
{
    forceUnloadAt(2, 1);
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    const std::vector<CargoRouteSummary> direct = { { station(1), station(4), StationId::null, 8, true } };
    EXPECT_EQ(preview(), direct);
    Vehicles::OrderManager::freeOrders(head);
    Vehicles::OrderManager::allocateOrders(*head);
    forceUnloadAt(2, 0, true);
    EXPECT_EQ(preview(), direct);
}

TEST_F(CargoDistVehicleRoute, GraphDirtyRefreshesCachedTreeAfterDestinationLosesAcceptance)
{
    setPackets(PacketList::fromPackets({ packet(8, station(2)) }));
    VehicleRoutePreview cache;
    std::vector<CargoRouteNode> tree;
    const auto refreshTree = [&] {
        const auto changed = cache.update(*head);
        if (changed)
        {
            tree = getRouteTree(cache.summaries.at(primary), { CargoRouteField::nextHop, CargoRouteField::origin, CargoRouteField::destination });
        }
        return changed;
    };
    ASSERT_TRUE(refreshTree());
    ASSERT_EQ(tree.size(), 1U);
    EXPECT_TRUE(tree.front().direct);
    EXPECT_FALSE(refreshTree());
    const auto routingRevision = getStateConst().routingRevision;
    const auto cargoRevision = getStateConst().cargoRevision;
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    markGraphDirty();
    EXPECT_EQ(getStateConst().routingRevision, routingRevision);
    EXPECT_EQ(getStateConst().cargoRevision, cargoRevision);
    EXPECT_TRUE(refreshTree());
    EXPECT_FALSE(tree.front().direct);
    EXPECT_EQ(tree.front().station, StationId::null);
    // Acceptance can change again while the same graph recalculation is pending.
    getGameState().stations[2].cargoStats[0].isAccepted(true);
    EXPECT_TRUE(refreshTree());
    EXPECT_TRUE(tree.front().direct);
}

TEST_F(CargoDistVehicleRoute, RequiresExactOccurrenceAndLegForContinuation)
{
    flow(kFirst) = { { station(3), 8, 0, point(5), point(6) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(2), 8 } }));
    flow(kFirst) = { { station(4), 8, 0, point(1), point(3) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(2), 8 } }));
}

TEST_F(CargoDistVehicleRoute, RepeatedStationAtDifferentOccurrenceCanStillBeDirect)
{
    const VehicleServiceLeg repeated{ 0, station(3), station(2), point(2), point(3) };
    const VehicleServiceLeg final{ 0, station(2), station(4), point(3), point(4) };
    getState().vehicleServiceLegs[head->id] = { kFirst, kSecond, repeated, final };
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(2), 8, 0, point(2), point(3) } };
    flow(repeated) = { { station(4), 8, 0, point(3), point(4) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 8, true } }));
}

TEST_F(CargoDistVehicleRoute, MissingRoutesAndStaleServicesRemainAwaitingRoute)
{
    const std::vector<CargoRouteSummary> awaiting = { { station(1), station(4), StationId::null, 8 } };
    EXPECT_EQ(preview(), awaiting);
    auto unrouted = packet();
    unrouted.nextHop = StationId::null;
    EXPECT_EQ(preview(unrouted), awaiting);
    unrouted = packet();
    unrouted.arrival = {};
    EXPECT_EQ(preview(unrouted), awaiting);
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    getState().servicesDirty = true;
    EXPECT_EQ(preview(), awaiting);
    getState().servicesDirty = false;
    getState().vehicleServiceLegs.clear();
    EXPECT_EQ(preview(), awaiting);
}

TEST_F(CargoDistVehicleRoute, CyclicContinuationTerminatesAsAwaitingRoute)
{
    const VehicleServiceLeg cycle{ 0, station(4), station(2), point(3), point(1) };
    getState().vehicleServiceLegs[head->id] = { kFirst, kSecond, kThird, cycle };
    flow(kFirst, station(5)) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond, station(5)) = { { station(4), 8, 0, point(2), point(3) } };
    flow(kThird, station(5)) = { { station(2), 8, 0, point(3), point(1) } };
    EXPECT_EQ(preview(packet(8, station(5))), (std::vector<CargoRouteSummary>{ { station(1), station(5), StationId::null, 8 } }));
}

TEST_F(CargoDistVehicleRoute, PlatformFallbackAndReversalMatchUnloading)
{
    getState().flows[{ 0, station(2), station(1), {}, station(4) }] = { { station(4), 8, 0, point(0, 8), point(1, 8) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), station(2), 8 } }));

    const VehicleServiceLeg reverse{ 0, station(2), station(1), point(1), point(0) };
    getState().vehicleServiceLegs[head->id] = { kFirst, reverse };
    flow(kFirst) = { { station(1), 8, 0, point(1), point(0) } };
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 8 } }));
    flow(kFirst, station(1)) = { { station(1), 8, 0, point(1), point(0) } };
    EXPECT_EQ(preview(packet(8, station(1))), (std::vector<CargoRouteSummary>{ { station(1), station(1), StationId::null, 8, true } }));
}

TEST_F(CargoDistVehicleRoute, HolidayPassengersDoNotUseOrdinaryFlows)
{
    flow(kFirst) = { { station(3), 8, 0, point(1), point(2) } };
    flow(kSecond) = { { station(4), 8, 0, point(2), point(3) } };
    for (const auto kind : { PassengerTripKind::holidayOutbound, PassengerTripKind::holidayReturn })
    {
        auto holiday = packet();
        holiday.tripKind = kind;
        EXPECT_EQ(preview(holiday), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 8 } }));
        holiday.destination = station(2);
        EXPECT_EQ(preview(holiday), (std::vector<CargoRouteSummary>{ { station(1), station(2), StationId::null, 8, true } }));
    }
}

TEST_F(CargoDistVehicleRoute, FasterUnplannedContinuationStaysDirect)
{
    for (const auto& leg : { kSecond, kThird })
    {
        getState().serviceEdges[{ 0, leg.from, leg.to, leg.departure, leg.arrival }] = { 100, 1, 10, 20 };
    }
    EXPECT_EQ(preview(), (std::vector<CargoRouteSummary>{ { station(1), station(4), StationId::null, 8, true } }));
}
