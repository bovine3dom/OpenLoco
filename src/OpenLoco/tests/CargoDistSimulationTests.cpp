// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Entities/EntityManager.h"
#include "GameState.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleTail.h"
#include "World/Station.h"
#include <OpenLoco/CargoDist/Save.h>
#include <array>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::CargoDist;

namespace
{
    constexpr StationId station(uint16_t value)
    {
        return static_cast<StationId>(value);
    }

    constexpr EntityId entity(uint16_t value)
    {
        return static_cast<EntityId>(value);
    }

    constexpr ServicePoint servicePoint(uint16_t service, uint16_t occurrence)
    {
        return { static_cast<ServiceId>(service), occurrence };
    }

    constexpr VehicleServiceLeg serviceLeg(uint16_t from, uint16_t to, uint16_t service, uint16_t departure, uint16_t arrival)
    {
        return { 0, station(from), station(to), servicePoint(service, departure), servicePoint(service, arrival) };
    }

    uint32_t serviceCapacity(uint8_t cargo, uint16_t from, uint16_t to)
    {
        uint32_t result = 0;
        for (const auto& [key, edge] : getStateConst().serviceEdges)
        {
            if (key.cargo == cargo && key.from == station(from) && key.to == station(to))
            {
                result += edge.capacity;
            }
        }
        return result;
    }

    class CargoDistServiceSimulationTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            _station1 = getGameState().stations[1];
            _station2 = getGameState().stations[2];
            EntityManager::reset();
            Vehicles::OrderManager::reset();
            reset();

            auto& source = getGameState().stations[1];
            source = {};
            source.name = StringId(1);
            source.x = 0;
            source.y = 0;
            auto& destination = getGameState().stations[2];
            destination = {};
            destination.name = StringId(1);
            destination.x = 6;
            destination.y = 0;
            destination.cargoStats[0].isAccepted(true);
            getState().settings.modes[0] = DistributionMode::asymmetric;
        }

        void TearDown() override
        {
            reset();
            Vehicles::OrderManager::reset();
            EntityManager::reset();
            getGameState().stations[1] = _station1;
            getGameState().stations[2] = _station2;
        }

        template<typename T>
        static T* createComponent()
        {
            auto* entity = EntityManager::createEntityVehicle();
            entity->baseType = EntityBaseType::vehicle;
            auto* component = reinterpret_cast<Vehicles::VehicleBase*>(entity);
            component->setSubType(T::kVehicleThingType);
            return static_cast<T*>(component);
        }

        static Vehicles::VehicleHead* createVehicle(bool reverseOrder = false, uint32_t acceptedTypes = 1, uint8_t cargoType = 0, std::optional<uint8_t> waitForCargo = std::nullopt)
        {
            auto* head = createComponent<Vehicles::VehicleHead>();
            auto* veh1 = createComponent<Vehicles::Vehicle1>();
            auto* veh2 = createComponent<Vehicles::Vehicle2>();
            auto* front = createComponent<Vehicles::VehicleBogie>();
            auto* back = createComponent<Vehicles::VehicleBogie>();
            auto* body = createComponent<Vehicles::VehicleBody>();
            auto* tail = createComponent<Vehicles::VehicleTail>();
            body->setSubType(Vehicles::VehicleEntityType::body_start);

            const std::array<Vehicles::VehicleBase*, 7> components = { head, veh1, veh2, front, back, body, tail };
            for (size_t i = 0; i < components.size(); ++i)
            {
                components[i]->head = head->id;
                components[i]->owner = CompanyId(0);
                components[i]->mode = TransportMode::road;
                components[i]->tileX = 0;
                components[i]->setNextCar(i + 1 < components.size() ? components[i + 1]->id : EntityId::null);
            }
            head->vehicleType = VehicleType::bus;
            head->status = Vehicles::Status::travelling;
            head->stationId = StationId::null;
            head->vehicleFlags = Vehicles::VehicleFlags::none;
            veh1->var_48 = Vehicles::Flags48::none;
            veh2->maxSpeed = Speed16(21);
            body->primaryCargo = { acceptedTypes, cargoType, 10, StationId::null, 0, 0 };
            EntityManager::moveEntityToList(head, EntityManager::EntityListType::vehicleHead);

            Vehicles::OrderManager::allocateOrders(*head);
            const Vehicles::OrderStopAt first{ reverseOrder ? station(2) : station(1) };
            Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd), &first);
            if (waitForCargo.has_value())
            {
                const Vehicles::OrderWaitFor waitFor{ *waitForCargo };
                Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd), &waitFor);
            }
            const Vehicles::OrderStopAt second{ reverseOrder ? station(1) : station(2) };
            Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd), &second);
            return head;
        }

        Station _station1{};
        Station _station2{};
    };
}

TEST_F(CargoDistServiceSimulationTest, AdditionalVehicleReducesExpectedWait)
{
    createVehicle(true);
    recalculateNow();
    ASSERT_EQ(getStateConst().serviceEdges.size(), 2);
    const auto firstWait = getStateConst().serviceEdges.begin()->second.waitTime;

    createVehicle();
    recalculateNow();

    ASSERT_EQ(getStateConst().serviceEdges.size(), 2);
    const auto& firstEdge = getStateConst().serviceEdges.begin()->second;
    EXPECT_EQ(firstWait, 6);
    EXPECT_EQ(firstEdge.waitTime, 3);
    EXPECT_EQ(firstEdge.capacity, 20);
}

TEST_F(CargoDistServiceSimulationTest, FullLoadOrdersRestrictOnlyFlexibleCompartments)
{
    getState().settings.modes[1] = DistributionMode::asymmetric;
    createVehicle(false, 0b11, 0, 1);
    createVehicle(false, 0b01, 0, 1);

    recalculateNow();

    EXPECT_EQ(serviceCapacity(0, 1, 2), 10);
    EXPECT_EQ(serviceCapacity(1, 1, 2), 10);
}

TEST_F(CargoDistServiceSimulationTest, FullLoadCapacityAppliesAtItsDeparture)
{
    getState().settings.modes[1] = DistributionMode::asymmetric;
    auto* head = createVehicle(false, 0b11, 0, 0);
    const Vehicles::OrderWaitFor waitForCargo1{ 1 };
    Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd), &waitForCargo1);

    recalculateNow();

    EXPECT_EQ(serviceCapacity(0, 1, 2), 10);
    EXPECT_EQ(serviceCapacity(1, 1, 2), 0);
    EXPECT_EQ(serviceCapacity(0, 2, 1), 0);
    EXPECT_EQ(serviceCapacity(1, 2, 1), 10);
}

TEST_F(CargoDistServiceSimulationTest, MultipleFullLoadOrdersUseRuntimeCargoPriority)
{
    getState().settings.modes[1] = DistributionMode::asymmetric;
    auto* head = createVehicle(false, 0b11, 0, 0);
    const Vehicles::OrderWaitFor waitForCargo1{ 1 };
    const auto secondStopOffset = head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd) - sizeof(Vehicles::OrderStopAt);
    Vehicles::OrderManager::insertOrder(head, secondStopOffset, &waitForCargo1);

    recalculateNow();

    EXPECT_EQ(serviceCapacity(0, 1, 2), 0);
    EXPECT_EQ(serviceCapacity(1, 1, 2), 10);
}

TEST_F(CargoDistServiceSimulationTest, DirtyServicesRebuildBeforeResolvingLeg)
{
    auto* head = createVehicle();
    recalculateNow();
    const auto initial = getStateConst().vehicleServiceLegs.at(head->id).front();
    head->currentOrder = initial.currentOrder;
    head->stationId = initial.from;
    head->status = Vehicles::Status::loading;
    getState().nextRecalculationDay = 1234;

    markServicesDirty();

    const auto current = getCurrentServiceLeg(*head);
    ASSERT_TRUE(current.has_value());
    EXPECT_EQ(current->from, initial.from);
    EXPECT_EQ(current->to, initial.to);
    EXPECT_FALSE(getStateConst().servicesDirty);
    EXPECT_FALSE(getStateConst().graphDirty);
    EXPECT_EQ(getStateConst().nextRecalculationDay, 1234);
}

TEST(CargoDistSimulation, SynchronisesNativeCargoMirrors)
{
    reset();
    auto& packets = getOrCreateStationCargo(station(1), 0);
    packets.append({ 10, station(2), station(4), 2 });
    packets.append({ 30, station(3), station(4), 4 });
    StationCargoStats cargo{};

    synchroniseStationCargo(station(1), 0, cargo);

    EXPECT_EQ(cargo.quantity, 40);
    EXPECT_EQ(cargo.origin, station(3));
    EXPECT_EQ(cargo.enrouteAge, 3);
}

TEST(CargoDistSimulation, AddsProducedCargoUsingCurrentFlow)
{
    reset();
    const std::array flows = { FlowShare{ station(1), station(1), station(2), 20 } };
    setFlows(0, flows);
    StationCargoStats cargo{};

    addProducedCargo(station(1), 0, cargo, 20);

    ASSERT_NE(getStationCargoConst(station(1), 0), nullptr);
    EXPECT_EQ(getStationCargoConst(station(1), 0)->quantityFor(station(2)), 20);
    EXPECT_EQ(cargo.quantity, 20);
    EXPECT_EQ(cargo.origin, station(1));
    EXPECT_EQ(getStateConst().supply.at({ 0, station(1) }), 20);
}

TEST(CargoDistSimulation, SplitsProducedCargoByFlowQuantity)
{
    reset();
    const std::array flows = {
        FlowShare{ station(1), station(1), station(2), 3 },
        FlowShare{ station(1), station(1), station(3), 1 },
    };
    setFlows(0, flows);
    StationCargoStats cargo{};

    addProducedCargo(station(1), 0, cargo, 40);

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->quantityFor(station(2)), 30);
    EXPECT_EQ(packets->quantityFor(station(3)), 10);
}

TEST(CargoDistSimulation, RecordsSupplyWhenStationIsFull)
{
    reset();
    getOrCreateStationCargo(station(1), 0).append({ std::numeric_limits<uint16_t>::max(), station(1), StationId::null, 0 });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(1), 0, cargo);

    addProducedCargo(station(1), 0, cargo, 10);

    EXPECT_EQ(cargo.quantity, std::numeric_limits<uint16_t>::max());
    EXPECT_EQ(getStateConst().supply.at({ 0, station(1) }), 10);
}

TEST(CargoDistSimulation, AppliesNativeStationLossAndPacketAgeing)
{
    reset();
    getOrCreateStationCargo(station(2), 0).append({ 10, station(1), station(3), 4 });
    StationCargoStats cargo{};
    cargo.quantity = 7;
    cargo.origin = station(1);

    updateStationCargoDaily(station(2), 0, cargo, 10);

    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->quantity(), 7);
    EXPECT_EQ(packets->packets()[0].age, 5);
    EXPECT_EQ(cargo.enrouteAge, 5);
}

TEST(CargoDistSimulation, LoadsOnlyCargoForVehiclesNextStop)
{
    reset();
    constexpr auto leg = serviceLeg(1, 2, 4, 0, 1);
    auto& waiting = getOrCreateStationCargo(station(1), 0);
    waiting.append({ 20, station(1), station(2), 3, leg.departure, leg.arrival });
    waiting.append({ 15, station(1), station(3), 2, leg.departure, leg.arrival });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    const auto loaded = loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, leg);

    EXPECT_EQ(loaded, 20);
    EXPECT_EQ(vehicleCargo.qty, 20);
    EXPECT_EQ(stationCargo.quantity, 15);
    EXPECT_EQ(getVehicleCargoConst(key)->quantityFor(station(2), leg.departure), 20);
}

TEST(CargoDistSimulation, LoadsOnlyCargoForExactService)
{
    reset();
    constexpr auto leg = serviceLeg(1, 2, 4, 0, 1);
    constexpr auto otherLeg = serviceLeg(1, 2, 5, 0, 1);
    auto& waiting = getOrCreateStationCargo(station(1), 0);
    waiting.append({ 11, station(1), station(2), 1, leg.departure, leg.arrival });
    waiting.append({ 13, station(1), station(2), 1, otherLeg.departure, otherLeg.arrival });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    EXPECT_EQ(getLoadableQuantity(station(1), 0, leg), 11);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, leg), 11);

    EXPECT_EQ(vehicleCargo.qty, 11);
    EXPECT_EQ(getStationCargoConst(station(1), 0)->quantityFor(station(2), otherLeg.departure), 13);
}

TEST(CargoDistSimulation, DeliversCargoAtItsFlowSink)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(2), 20 } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.delivered.quantity(), 20);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(stationCargo.quantity, 0);
}

TEST(CargoDistSimulation, TransfersCargoAlongItsNextFlowLeg)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20 } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.delivered.quantity(), 0);
    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(stationCargo.quantity, 20);
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantityFor(station(3)), 20);
}

TEST(CargoDistSimulation, ReroutesCargoWhoseNextHopIsNoLongerServed)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20 } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(9), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    const std::array remainingStops = { station(4) };

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, remainingStops, false, std::nullopt);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantityFor(station(3)), 20);
}

TEST(CargoDistSimulation, ForcedUnloadDeliversAllAtAcceptingStation)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20 } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(4), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);
    const std::array remainingStops = { station(4) };

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, remainingStops, true, std::nullopt);

    EXPECT_EQ(result.delivered.quantity(), 20);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_EQ(vehicleCargo.qty, 0);
}

TEST(CargoDistSimulation, CapsTransferredCargoAtNativeStationLimit)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20 } };
    setFlows(0, flows);
    auto& waiting = getOrCreateStationCargo(station(2), 0);
    waiting.append({ 65530, station(4), station(3), 0 });
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(2), 0, stationCargo);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(stationCargo.quantity, std::numeric_limits<uint16_t>::max());
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantity(), std::numeric_limits<uint16_t>::max());
}

TEST(CargoDistSimulation, ContinuesOnSameServiceWithoutTransfer)
{
    reset();
    constexpr auto onward = serviceLeg(2, 3, 7, 1, 2);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, onward.departure, onward.arrival },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, onward);

    EXPECT_EQ(result.quantity(), 0);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_EQ(vehicleCargo.qty, 20);
    EXPECT_EQ(stationCargo.quantity, 0);
    const auto* packets = getVehicleCargoConst(key);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets()[0].nextHop, station(3));
    EXPECT_EQ(packets->packets()[0].departure, onward.departure);
    EXPECT_EQ(packets->packets()[0].arrival, onward.arrival);
}

TEST(CargoDistSimulation, TransfersWhenSelectedServiceDiffers)
{
    reset();
    constexpr auto onward = serviceLeg(2, 3, 7, 1, 2);
    constexpr auto transfer = serviceLeg(2, 3, 8, 0, 1);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, transfer.departure, transfer.arrival },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, onward);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(vehicleCargo.qty, 0);
    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets()[0].departure, transfer.departure);
    EXPECT_EQ(packets->packets()[0].arrival, transfer.arrival);
}

TEST(CargoDistSimulation, RetriesPlatformFlowForStaleIncomingService)
{
    reset();
    constexpr auto transfer = serviceLeg(2, 3, 8, 0, 1);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, {}, transfer.departure, transfer.arrival },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(99, 0), servicePoint(99, 1) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.transferred, 20);
    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->quantityFor(station(3), transfer.departure), 20);
}

TEST(CargoDistSimulation, ForcedUnloadDoesNotContinueOnSameService)
{
    reset();
    constexpr auto onward = serviceLeg(2, 3, 7, 1, 2);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, onward.departure, onward.arrival },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, true, onward);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantityFor(station(3), onward.departure), 20);
}

TEST(CargoDistSimulation, RemovingStationKeepsFlowCursorsSerializable)
{
    reset();
    EntityManager::reset();
    getState().flows[{ 0, station(1), station(2) }] = {
        { station(3), 75, -25 },
        { station(4), 25, 25 },
    };

    removeStation(station(3));

    const auto& options = getStateConst().flows.at({ 0, station(1), station(2) });
    ASSERT_EQ(options.size(), 1);
    EXPECT_EQ(options.front().current, 0);
    EXPECT_NO_THROW(encodeState(getStateConst()));
}

TEST(CargoDistSimulation, FlowDirtinessKeepsCachedServiceLegs)
{
    reset();
    EntityManager::reset();
    auto leg = serviceLeg(1, 2, 7, 0, 1);
    leg.currentOrder = 4;
    auto* headEntity = EntityManager::createEntityVehicle();
    headEntity->baseType = EntityBaseType::vehicle;
    auto* headBase = reinterpret_cast<Vehicles::VehicleBase*>(headEntity);
    headBase->setSubType(Vehicles::VehicleEntityType::head);
    auto* head = headBase->asVehicleHead();
    head->currentOrder = 4;
    head->stationId = station(1);
    head->status = Vehicles::Status::loading;
    getState().vehicleServiceLegs[head->id] = { leg };

    EXPECT_TRUE(getCurrentServiceLeg(*head).has_value());

    markGraphDirty();

    EXPECT_TRUE(getCurrentServiceLeg(*head).has_value());
}

TEST(CargoDistSimulation, RemovingServiceLeaderClearsStalePlans)
{
    reset();
    constexpr auto removed = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto retained = serviceLeg(1, 3, 8, 0, 1);
    auto& state = getState();
    state.vehicleServiceLegs[entity(7)] = { removed };
    state.stationCargo[{ station(1), 0 }].append({ 10, station(1), station(2), 0, removed.departure, removed.arrival });
    state.serviceEdges[{ 0, station(1), station(2), removed.departure, removed.arrival }] = { 10, 5, 3 };
    state.flows[{ 0, station(1), station(1) }] = {
        { station(2), 10, 0, removed.departure, removed.arrival },
        { station(3), 10, 0, retained.departure, retained.arrival },
    };

    removeVehicleService(entity(7));

    const auto& packet = state.stationCargo.at({ station(1), 0 }).packets().front();
    EXPECT_EQ(packet.nextHop, StationId::null);
    EXPECT_TRUE(packet.departure.empty());
    EXPECT_TRUE(state.serviceEdges.empty());
    const auto& options = state.flows.at({ 0, station(1), station(1) });
    ASSERT_EQ(options.size(), 1);
    EXPECT_EQ(options.front().departure, retained.departure);
    EXPECT_EQ(options.front().current, 0);
    EXPECT_TRUE(state.graphDirty);
}

TEST(CargoDistSimulation, StationAttractionMarksEnabledGraphDirty)
{
    reset();
    getState().settings.modes[0] = DistributionMode::asymmetric;

    setStationAttraction(station(1), 0, 100);

    EXPECT_EQ(getStateConst().stationAttraction.at({ station(1), 0 }), 100U);
    EXPECT_TRUE(getStateConst().graphDirty);

    getState().graphDirty = false;
    setStationAttraction(station(1), 0, 100);
    EXPECT_FALSE(getStateConst().graphDirty);

    setStationAttraction(station(1), 0, 0);
    EXPECT_FALSE(getStateConst().stationAttraction.contains({ station(1), 0 }));
    EXPECT_TRUE(getStateConst().graphDirty);
}
