// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Entities/EntityManager.h"
#include "GameState.h"
#include "Objects/CargoObject.h"
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
#include <algorithm>
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
            _station3 = getGameState().stations[3];
            _station4 = getGameState().stations[4];
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
            auto& intermediate = getGameState().stations[3];
            intermediate = {};
            intermediate.name = StringId(1);
            intermediate.x = 12;
            intermediate.y = 0;
            auto& fartherDestination = getGameState().stations[4];
            fartherDestination = {};
            fartherDestination.name = StringId(1);
            fartherDestination.x = 18;
            fartherDestination.y = 0;
            getState().settings.modes[0] = DistributionMode::asymmetric;
        }

        void TearDown() override
        {
            reset();
            Vehicles::OrderManager::reset();
            EntityManager::reset();
            getGameState().stations[1] = _station1;
            getGameState().stations[2] = _station2;
            getGameState().stations[3] = _station3;
            getGameState().stations[4] = _station4;
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

        static Vehicles::VehicleHead* createVehicle(bool reverseOrder = false, uint32_t acceptedTypes = 1, uint8_t cargoType = 0, std::optional<uint8_t> waitForCargo = std::nullopt, uint8_t capacity = 10)
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
            body->primaryCargo = { acceptedTypes, cargoType, capacity, StationId::null, 0, 0 };
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

        static Vehicles::VehicleHead* createVehicleWithStops(const std::vector<StationId>& stops)
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
            body->primaryCargo = { 1, 0, 10, StationId::null, 0, 0 };
            EntityManager::moveEntityToList(head, EntityManager::EntityListType::vehicleHead);

            Vehicles::OrderManager::allocateOrders(*head);
            for (const auto& stop : stops)
            {
                const Vehicles::OrderStopAt order{ stop };
                Vehicles::OrderManager::insertOrder(head, head->sizeOfOrderTable - sizeof(Vehicles::OrderEnd), &order);
            }
            return head;
        }

        Station _station1{};
        Station _station2{};
        Station _station3{};
        Station _station4{};
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
    EXPECT_EQ(firstEdge.capacity, 10);
    EXPECT_EQ(firstEdge.headway, 6);
    EXPECT_EQ(firstEdge.fleetCapacity, 20);
}

TEST_F(CargoDistServiceSimulationTest, MixedVehicleCapacityUsesAverageDeparture)
{
    createVehicle(false, 1, 0, std::nullopt, 10);
    createVehicle(false, 1, 0, std::nullopt, 20);

    recalculateNow();

    ASSERT_EQ(getStateConst().serviceEdges.size(), 2);
    EXPECT_EQ(getStateConst().serviceEdges.begin()->second.capacity, 15);
    EXPECT_EQ(getStateConst().serviceEdges.begin()->second.fleetCapacity, 30);
}

TEST_F(CargoDistServiceSimulationTest, CrushLoadingPolicyBelongsToDepartureStop)
{
    auto* head = createVehicle();
    auto* firstStop = Vehicles::OrderRingView(head->orderTableOffset).begin()->as<Vehicles::OrderStopAt>();
    ASSERT_NE(firstStop, nullptr);
    firstStop->setCrushLoading(true);

    recalculateNow();

    const auto& legs = getStateConst().vehicleServiceLegs.at(head->id);
    const auto firstLeg = std::ranges::find(legs, station(1), &VehicleServiceLeg::from);
    const auto secondLeg = std::ranges::find(legs, station(2), &VehicleServiceLeg::from);
    ASSERT_NE(firstLeg, legs.end());
    ASSERT_NE(secondLeg, legs.end());
    EXPECT_TRUE(firstLeg->crushLoading);
    EXPECT_FALSE(secondLeg->crushLoading);

    head->stationId = station(1);
    head->currentOrder = firstLeg->currentOrder;
    EXPECT_TRUE(head->isCrushLoadingAtCurrentStop());
    head->stationId = station(2);
    head->currentOrder = secondLeg->currentOrder;
    EXPECT_FALSE(head->isCrushLoadingAtCurrentStop());
}

TEST_F(CargoDistServiceSimulationTest, RepeatedStopDoesNotCreateSelfEdge)
{
    createVehicleWithStops({ station(1), station(1), station(2) });

    recalculateNow();

    ASSERT_EQ(getStateConst().serviceEdges.size(), 2);
    EXPECT_TRUE(std::ranges::none_of(getStateConst().serviceEdges, [](const auto& entry) { return entry.first.from == entry.first.to; }));
}

TEST_F(CargoDistServiceSimulationTest, IndustryAndBuildingSinksUseSemanticTargetWeights)
{
    auto& industrySink = getGameState().stations[2].cargoStats[0];
    industrySink.industryId = IndustryId(0);
    auto& buildingSink = getGameState().stations[4].cargoStats[0];
    buildingSink.isAccepted(true);
    buildingSink.industryId = IndustryId::null;
    setStationAttraction(station(2), 0, 100);
    setStationAttraction(station(4), 0, 24);
    getState().supply[{ 0, station(1) }] = 32;
    createVehicleWithStops({ station(1), station(2), station(4) });

    recalculateNow();

    const auto& destinations = getStateConst().destinationFlows.at({ 0, station(1), station(1) });
    const auto industry = std::find_if(destinations.begin(), destinations.end(), [](const auto& option) { return option.destination == station(2); });
    const auto building = std::find_if(destinations.begin(), destinations.end(), [](const auto& option) { return option.destination == station(4); });
    ASSERT_NE(industry, destinations.end());
    ASSERT_NE(building, destinations.end());
    EXPECT_EQ(industry->weight, 8U);
    EXPECT_EQ(building->weight, 24U);
}

TEST(CargoDistSimulation, PassengerIndustryUsesRecordedAttraction)
{
    EXPECT_EQ(getRoutingAttraction(true, true, 96), 96U);
    EXPECT_EQ(getRoutingAttraction(false, true, 96), 8U);
    EXPECT_EQ(getRoutingAttraction(true, true, 0), 8U);
    EXPECT_EQ(getRoutingAttraction(false, false, 24), 24U);
}

TEST(CargoDistSimulation, ClassifiesPassengerAndMailAsTownCargo)
{
    EXPECT_TRUE(isTownCargoCategory(CargoCategory::passengers));
    EXPECT_TRUE(isTownCargoCategory(CargoCategory::mail));
    EXPECT_FALSE(isTownCargoCategory(CargoCategory::goods));
    EXPECT_FALSE(isTownCargoCategory(CargoCategory::null));
    EXPECT_FALSE(isTownCargoCategory(static_cast<CargoCategory>(0x1234)));
}

TEST(CargoDistSimulation, PassengerIndustryAttractionUsesBoundedPatronageBonus)
{
    EXPECT_EQ(getPassengerIndustryBonus(0), 8U);
    EXPECT_EQ(getPassengerIndustryBonus(64), 24U);
    EXPECT_EQ(getPassengerIndustryBonus(160), 48U);
    EXPECT_EQ(getPassengerIndustryBonus(std::numeric_limits<uint16_t>::max()), 48U);

    EXPECT_EQ(getPassengerIndustryAttraction(8, 8), 8U);
    EXPECT_EQ(getPassengerIndustryAttraction(21, 48), 61U);
}

TEST(CargoDistSimulation, PassengerIndustryBonusIsSharedAcrossStations)
{
    uint32_t total = 0;
    for (uint32_t i = 0; i < 5; ++i)
    {
        total += getSharedPassengerIndustryBonus(48, 5, i);
    }
    EXPECT_EQ(total, 48U);
    EXPECT_EQ(getSharedPassengerIndustryBonus(48, 5, 0), 10U);
    EXPECT_EQ(getSharedPassengerIndustryBonus(48, 5, 4), 9U);
}

TEST(CargoDistSimulation, PassengerIndustryBonusRemainsBoundedWithManyStations)
{
    uint32_t total = 0;
    for (uint32_t stationIndex = 0; stationIndex < 9; ++stationIndex)
    {
        total += getPassengerIndustryAttraction(8, getSharedPassengerIndustryBonus(8, 9, stationIndex));
    }

    EXPECT_EQ(total, 8U);
    EXPECT_EQ(getPassengerIndustryAttraction(8, getSharedPassengerIndustryBonus(8, 9, 8)), 0U);
}

TEST(CargoDistSimulation, PassengerIndustrySinkRequiresSupplyOnlyFromProducers)
{
    EXPECT_TRUE(isPassengerIndustrySink(false, false));
    EXPECT_TRUE(isPassengerIndustrySink(false, true));
    EXPECT_TRUE(isPassengerIndustrySink(true, false));
    EXPECT_FALSE(isPassengerIndustrySink(true, true));
}

TEST_F(CargoDistServiceSimulationTest, ZeroProductionIsANoOp)
{
    ASSERT_FALSE(getStateConst().graphDirty);

    StationCargoStats cargo{};
    addProducedCargo(station(1), 0, cargo, 0);

    EXPECT_FALSE(getStateConst().graphDirty);
    EXPECT_FALSE(getStateConst().stationCargo.contains({ station(1), 0 }));
    EXPECT_FALSE(getStateConst().supply.contains({ 0, station(1) }));
}

TEST_F(CargoDistServiceSimulationTest, RoutingMetadataRefreshPreservesCargoAcceptance)
{
    auto& destination = getGameState().stations[2];
    destination.cargoStats[0].industryId = IndustryId(0);
    setStationAttraction(station(2), 0, 100);
    getState().graphDirty = false;

    destination.refreshCargoRoutingMetadata();

    EXPECT_TRUE(destination.cargoStats[0].isAccepted());
    EXPECT_EQ(destination.cargoStats[0].industryId, IndustryId::null);
    EXPECT_FALSE(getStateConst().stationAttraction.contains({ station(2), 0 }));
    EXPECT_TRUE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, CommittedDemandIncludesOnlyWaitingAndNextTransfer)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    auto* body = (*train.cars.begin()).body;
    constexpr auto feederArrival = servicePoint(7, 1);
    constexpr auto departure = servicePoint(8, 0);
    constexpr auto arrival = servicePoint(8, 1);
    auto& state = getState();
    state.flows[{ 0, station(2), station(1), feederArrival, station(4) }] = {
        { station(3), 3, 0, departure, arrival },
        { station(4), 1, 0, servicePoint(9, 0), servicePoint(9, 1) },
    };
    state.vehicleCargo[{ body->id, VehicleCargoSlot::primary }].append({ 20, station(1), station(2), 0, servicePoint(7, 0), feederArrival, station(4) });
    state.stationCargo[{ station(2), 0 }].append({ 7, station(1), station(3), 0, departure, arrival, station(4) });

    const auto demand = getCommittedServiceDemands(0);

    const auto selected = demand.at({ 0, station(2), station(3), departure, arrival });
    EXPECT_EQ(selected.waiting, 7);
    EXPECT_EQ(selected.incoming, 15);
    const auto alternative = demand.at({ 0, station(2), station(4), servicePoint(9, 0), servicePoint(9, 1) });
    EXPECT_EQ(alternative.waiting, 0);
    EXPECT_EQ(alternative.incoming, 5);
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

TEST_F(CargoDistServiceSimulationTest, RecalculationBuildsFlowsForAllEnabledCargoes)
{
    getGameState().stations[2].cargoStats[1].isAccepted(true);
    getState().settings.modes[1] = DistributionMode::asymmetric;
    createVehicle(false, 0b11);
    getState().stationCargo[{ station(1), 0 }].append({ 10, station(1), StationId::null, 0, {}, {}, station(2) });
    getState().stationCargo[{ station(1), 1 }].append({ 10, station(1), StationId::null, 0, {}, {}, station(2) });

    recalculateNow();

    EXPECT_TRUE(getStateConst().flows.contains({ 0, station(1), station(1), {}, station(2) }));
    EXPECT_TRUE(getStateConst().flows.contains({ 1, station(1), station(1), {}, station(2) }));
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

TEST_F(CargoDistServiceSimulationTest, DirtyServicesDoNotExposeStaleLeg)
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
    EXPECT_FALSE(current.has_value());
    EXPECT_TRUE(getStateConst().servicesDirty);
    EXPECT_TRUE(getStateConst().graphDirty);
    EXPECT_EQ(getStateConst().nextRecalculationDay, 1234);
}

TEST_F(CargoDistServiceSimulationTest, ServiceRecalculationCommitsAtItsDeadline)
{
    auto* head = createVehicle();
    recalculateNow();
    const auto initial = getStateConst().vehicleServiceLegs.at(head->id).front();
    head->currentOrder = initial.currentOrder;
    head->stationId = initial.from;
    head->status = Vehicles::Status::loading;

    markServicesDirty();
    update();

    EXPECT_TRUE(isServiceRecalculationPending());
    EXPECT_FALSE(getCurrentServiceLeg(*head).has_value());

    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_TRUE(getCurrentServiceLeg(*head).has_value());
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, ServiceRecalculationDiscardsSupersededGeneration)
{
    createVehicle();
    recalculateNow();

    markServicesDirty();
    update();
    markServicesDirty();
    getGameState().scenarioTicks += 48;
    update();

    EXPECT_TRUE(isServiceRecalculationPending());
    EXPECT_TRUE(getStateConst().graphDirty);

    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, RemovingVehicleDiscardsPendingServiceSnapshot)
{
    const auto vehicle = createVehicle()->id;
    recalculateNow();
    ASSERT_TRUE(getStateConst().vehicleServiceLegs.contains(vehicle));

    markServicesDirty();
    update();
    ASSERT_TRUE(isServiceRecalculationPending());

    removeVehicleService(vehicle);
    EntityManager::reset();
    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(getStateConst().vehicleServiceLegs.contains(vehicle));
    EXPECT_TRUE(isServiceRecalculationPending());

    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_FALSE(getStateConst().vehicleServiceLegs.contains(vehicle));
}

TEST_F(CargoDistServiceSimulationTest, ServiceRecalculationDiscardsStaleGraphSnapshot)
{
    auto* head = createVehicle();
    recalculateNow();
    const auto initial = getStateConst().vehicleServiceLegs.at(head->id).front();
    head->currentOrder = initial.currentOrder;
    head->stationId = initial.from;
    head->status = Vehicles::Status::loading;

    markServicesDirty();
    update();
    markGraphDirty();
    update();

    EXPECT_TRUE(isServiceRecalculationPending());
    EXPECT_FALSE(getCurrentServiceLeg(*head).has_value());
    EXPECT_TRUE(getStateConst().graphDirty);

    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, RestorePreservesCommittedAccessibilityWhileQueuingRefresh)
{
    State state;
    state.stationAttraction[{ station(2), 0 }] = 96;
    state.stationAccessibility[station(2)] = 123;
    state.hasStationAccessibilitySnapshot = true;
    state.graphDirty = true;
    state.requiresStationMetadataRefresh = true;

    restoreState(std::move(state));

    EXPECT_FALSE(getStateConst().requiresStationMetadataRefresh);
    EXPECT_FALSE(getStateConst().stationAttraction.contains({ station(2), 0 }));
    EXPECT_EQ(getStationAccessibility(station(2)), 123U);
    EXPECT_TRUE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, RestorePreservesEmptyCommittedAccessibilitySnapshot)
{
    State state;
    state.graphDirty = true;
    state.hasStationAccessibilitySnapshot = true;

    restoreState(std::move(state));

    EXPECT_TRUE(getStateConst().hasStationAccessibilitySnapshot);
    EXPECT_TRUE(getStateConst().stationAccessibility.empty());
    EXPECT_TRUE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, PeriodicRecalculationCommitsOnTheNextDayBoundary)
{
    createVehicle();
    recalculateNow();
    getGameState().currentDay = getStateConst().nextRecalculationDay - 1;
    getGameState().dayCounter = 0;
    getGameState().scenarioTicks = 0;

    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_EQ(getStateConst().nextRecalculationDay, getGameState().currentDay + 1);

    getGameState().currentDay++;
    getGameState().scenarioTicks += 97;
    update();

    EXPECT_EQ(getStateConst().nextRecalculationDay, getGameState().currentDay + getStateConst().settings.recalculationInterval);
}

TEST_F(CargoDistServiceSimulationTest, ResetDiscardsPendingServiceRecalculation)
{
    createVehicle();
    recalculateNow();
    markServicesDirty();
    update();

    ASSERT_TRUE(isServiceRecalculationPending());
    reset();

    EXPECT_FALSE(isServiceRecalculationPending());
    EXPECT_TRUE(getStateConst().serviceEdges.empty());
    EXPECT_TRUE(getStateConst().flows.empty());
}

TEST_F(CargoDistServiceSimulationTest, PendingRecalculationKeepsCommittedAccessibilitySnapshot)
{
    getState().stationAccessibility[station(1)] = 123;
    createVehicle();
    markServicesDirty();

    update();

    ASSERT_TRUE(isServiceRecalculationPending());
    EXPECT_EQ(getStationAccessibility(station(1)), 123U);

    reset();
    EXPECT_EQ(getStationAccessibility(station(1)), 0U);
}

TEST_F(CargoDistServiceSimulationTest, RecalculationReleasesRejectedDestination)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    getState().stationCargo[{ station(1), 0 }].append({ 10, station(1), station(2), 0, {}, {}, station(2) });

    recalculateNow();

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets().front().destination, StationId::null);
    EXPECT_EQ(packets->packets().front().nextHop, StationId::null);
}

TEST_F(CargoDistServiceSimulationTest, ReleasingRejectedDestinationAdvancesRoutingRevisionBeforeAsyncCommit)
{
    createVehicle();
    recalculateNow();
    getState().stationCargo[{ station(1), 0 }].append({ 10, station(1), station(2), 0, {}, {}, station(2) });
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    markServicesDirty();
    const auto routingRevision = getStateConst().routingRevision;

    update();

    ASSERT_TRUE(isServiceRecalculationPending());
    EXPECT_EQ(getStateConst().routingRevision, routingRevision + 1);
    EXPECT_EQ(getStateConst().stationCargo.at({ station(1), 0 }).packets().front().destination, StationId::null);
}

TEST_F(CargoDistServiceSimulationTest, RecalculationRoutesUnroutedWaitingCargo)
{
    createVehicle();
    auto& waiting = getState().stationCargo[{ station(1), 0 }];
    waiting.append({ 10, station(1), StationId::null, 0 });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(1), 0, cargo);

    recalculateNow();

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_FALSE(packets->packets().empty());
    EXPECT_EQ(packets->packets().front().destination, station(2));
    EXPECT_EQ(packets->packets().front().nextHop, station(2));
}

TEST_F(CargoDistServiceSimulationTest, RecalculationKeepsWaitingCargoRoute)
{
    createVehicle();
    constexpr auto leg = serviceLeg(1, 2, 7, 0, 1);
    auto& waiting = getState().stationCargo[{ station(1), 0 }];
    waiting.append({ 10, station(1), station(2), 0, leg.departure, leg.arrival, station(2) });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(1), 0, cargo);

    recalculateNow();

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets().front().destination, station(2));
    EXPECT_EQ(packets->packets().front().nextHop, station(2));
}

TEST_F(CargoDistServiceSimulationTest, RecalculationKeepsThroughCargoOnwardDestination)
{
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    createVehicle();
    constexpr auto leg = serviceLeg(1, 2, 7, 0, 1);
    auto& waiting = getState().stationCargo[{ station(2), 0 }];
    waiting.append({ 10, station(1), station(2), 0, leg.departure, leg.arrival, station(4) });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(2), 0, cargo);

    recalculateNow();

    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets().front().destination, station(4));
}

TEST_F(CargoDistServiceSimulationTest, MultiHopThroughCargoSurvivesRecalculation)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[3].cargoStats[0].isAccepted(true);
    auto* head = createVehicleWithStops({ station(1), station(2), station(3) });
    auto& state = getState();
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    StationCargoStats sourceCargo{};

    addProducedCargo(station(1), 0, sourceCargo, 20);
    EXPECT_EQ(getStationCargoConst(station(1), 0)->packets().front().nextHop, StationId::null);

    recalculateNow();

    const auto* routed = getStationCargoConst(station(1), 0);
    ASSERT_NE(routed, nullptr);
    ASSERT_EQ(routed->packets().size(), 1);
    EXPECT_EQ(routed->packets().front().destination, station(3));
    EXPECT_EQ(routed->packets().front().nextHop, station(2));

    const auto& legs = state.vehicleServiceLegs.at(head->id);
    ASSERT_GE(legs.size(), 2U);
    const auto firstLeg = legs.front();
    ASSERT_EQ(firstLeg.from, station(1));
    ASSERT_EQ(firstLeg.to, station(2));
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, StationId::null, 0, 0 };
    ASSERT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), sourceCargo, firstLeg), 20);

    StationCargoStats intermediateCargo{};
    const auto unloaded = unloadVehicleCargo(key, vehicleCargo, station(2), intermediateCargo, {}, false, std::nullopt);
    ASSERT_EQ(unloaded.transferred, 20);
    const auto* waiting = getStationCargoConst(station(2), 0);
    ASSERT_NE(waiting, nullptr);
    ASSERT_EQ(waiting->packets().size(), 1);
    EXPECT_EQ(waiting->packets().front().destination, station(3));
    EXPECT_EQ(waiting->packets().front().nextHop, station(3));

    recalculateNow();

    const auto* after = getStationCargoConst(station(2), 0);
    ASSERT_NE(after, nullptr);
    ASSERT_EQ(after->packets().size(), 1);
    EXPECT_EQ(after->packets().front().destination, station(3));
    EXPECT_EQ(after->packets().front().nextHop, station(3));
}

TEST_F(CargoDistServiceSimulationTest, AsyncRecalculationRoutesProducedCargo)
{
    createVehicle();
    StationCargoStats cargo{};
    addProducedCargo(station(1), 0, cargo, 20);
    getGameState().scenarioTicks = 0;

    update();

    EXPECT_TRUE(isServiceRecalculationPending());
    EXPECT_EQ(getStationCargoConst(station(1), 0)->packets().front().nextHop, StationId::null);

    getGameState().scenarioTicks += 48;
    update();

    EXPECT_FALSE(isServiceRecalculationPending());
    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_FALSE(packets->packets().empty());
    EXPECT_EQ(packets->packets().front().destination, station(2));
    EXPECT_EQ(packets->packets().front().nextHop, station(2));
}

TEST_F(CargoDistServiceSimulationTest, NewProductionDirtiesCleanGraphAndBecomesRoutable)
{
    createVehicle();
    recalculateNow();
    ASSERT_FALSE(getStateConst().graphDirty);
    ASSERT_FALSE(getStateConst().supply.contains({ 0, station(1) }));

    StationCargoStats cargo{};
    addProducedCargo(station(1), 0, cargo, 20);

    EXPECT_TRUE(getStateConst().graphDirty);
    recalculateNow();
    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_FALSE(packets->packets().empty());
    EXPECT_EQ(packets->packets().front().destination, station(2));
    EXPECT_EQ(packets->packets().front().nextHop, station(2));

    addProducedCargo(station(1), 0, cargo, 20);
    EXPECT_FALSE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, AsyncCommitKeepsCargoProducedAfterCaptureRoutable)
{
    createVehicle();
    getState().supply[{ 0, station(1) }] = 20;
    recalculateNow();
    getState().supply.clear();

    markServicesDirty();
    getGameState().scenarioTicks = 0;
    update();
    ASSERT_TRUE(isServiceRecalculationPending());

    StationCargoStats cargo{};
    addProducedCargo(station(1), 0, cargo, 20);
    ASSERT_EQ(getStationCargoConst(station(1), 0)->packets().front().destination, station(2));

    getGameState().scenarioTicks += 48;
    update();

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets().front().destination, station(2));
    EXPECT_EQ(packets->packets().front().nextHop, station(2));
}

TEST_F(CargoDistServiceSimulationTest, AsyncCommitKeepsCargoTransferredAfterCaptureRoutable)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[3].cargoStats[0].isAccepted(true);
    auto* head = createVehicleWithStops({ station(1), station(2), station(3) });
    StationCargoStats sourceCargo{};
    addProducedCargo(station(1), 0, sourceCargo, 20);
    recalculateNow();

    const auto& legs = getStateConst().vehicleServiceLegs.at(head->id);
    ASSERT_GE(legs.size(), 2U);
    const auto firstLeg = legs.front();
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, StationId::null, 0, 0 };
    ASSERT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), sourceCargo, firstLeg), 20);

    markServicesDirty();
    getGameState().scenarioTicks = 0;
    update();
    ASSERT_TRUE(isServiceRecalculationPending());

    StationCargoStats intermediateCargo{};
    const auto unloaded = unloadVehicleCargo(key, vehicleCargo, station(2), intermediateCargo, {}, false, std::nullopt);
    ASSERT_EQ(unloaded.transferred, 20);
    ASSERT_EQ(getStationCargoConst(station(2), 0)->packets().front().nextHop, station(3));

    getGameState().scenarioTicks += 48;
    update();

    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    EXPECT_EQ(packets->packets().front().destination, station(3));
    EXPECT_EQ(packets->packets().front().nextHop, station(3));
    EXPECT_EQ(packets->packets().front().departure, legs[1].departure);
    EXPECT_EQ(packets->packets().front().arrival, legs[1].arrival);
    EXPECT_EQ(getLoadableQuantity(station(2), 0, legs[1]), 20U);
}

TEST_F(CargoDistServiceSimulationTest, PresentServiceTakesCargoWhenCompleteJourneyIsBetter)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 10, 1, 2 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 10, 1, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 20, 0, planned.departure, planned.arrival },
    };
    auto& waiting = state.stationCargo[{ station(1), 0 }];
    waiting.append({ 20, station(1), station(2), 0, planned.departure, planned.arrival, station(2) });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 20);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, present), 10);

    EXPECT_EQ(stationCargo.quantity, 10);
    const auto* loaded = getVehicleCargoConst(key);
    ASSERT_NE(loaded, nullptr);
    ASSERT_EQ(loaded->packets().size(), 1);
    EXPECT_EQ(loaded->packets().front().destination, station(2));
    EXPECT_EQ(loaded->packets().front().departure, present.departure);
    EXPECT_EQ(loaded->packets().front().arrival, present.arrival);
}

TEST_F(CargoDistServiceSimulationTest, PresentServiceDoesNotTakeCargoWhenItsFirstLegCannotReachDestination)
{
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    constexpr auto planned = serviceLeg(1, 4, 8, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 7, 1, 0);
    constexpr auto returnLeg = serviceLeg(2, 1, 7, 0, 1);
    constexpr auto cheapPlatform = serviceLeg(1, 4, 9, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(4), planned.departure, planned.arrival }] = { 10, 10, 100, 200 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 10, 1, 1, 2 };
    state.serviceEdges[{ 0, station(2), station(1), returnLeg.departure, returnLeg.arrival }] = { 10, 1, 1, 2 };
    state.serviceEdges[{ 0, station(1), station(4), cheapPlatform.departure, cheapPlatform.arrival }] = { 10, 1, 1, 2 };
    state.flows[{ 0, station(1), station(1), {}, station(4) }] = {
        { station(4), 10, 0, planned.departure, planned.arrival },
    };
    auto& waiting = state.stationCargo[{ station(1), 0 }];
    waiting.append({ 10, station(1), station(4), 0, planned.departure, planned.arrival, station(4) });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 0);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, present), 0);
    EXPECT_EQ(stationCargo.quantity, 10);
    EXPECT_EQ(vehicleCargo.qty, 0);
}

TEST_F(CargoDistServiceSimulationTest, ExactCapacityDoesNotAddAnotherRuntimeHeadway)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 10, 5, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 10, 0, planned.departure, planned.arrival },
    };
    state.stationCargo[{ station(1), 0 }].append({ 10, station(1), station(2), 0, planned.departure, planned.arrival, station(2) });

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 0);
}

TEST_F(CargoDistServiceSimulationTest, FutureTransferPlanDoesNotCreateImmediateRuntimeQueue)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 10, 5, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 20, 0, planned.departure, planned.arrival },
    };
    state.stationCargo[{ station(1), 0 }].append({ 1, station(1), station(2), 0, planned.departure, planned.arrival, station(2) });

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 0);
}

TEST_F(CargoDistServiceSimulationTest, RuntimeQueueDivertsOnlyOverflowCohort)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 20, 5, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 20, 0, planned.departure, planned.arrival },
    };
    state.stationCargo[{ station(1), 0 }].append({ 20, station(1), station(2), 0, planned.departure, planned.arrival, station(2) });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 20, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 10);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, present), 10);
    EXPECT_EQ(stationCargo.quantity, 10);
    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 0);
}

TEST_F(CargoDistServiceSimulationTest, RuntimeQueueIncludesCargoProducedAfterPlanning)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 20, 5, 100, 200 };
    const std::array flows = {
        FlowShare{ station(1), station(1), station(2), 10, {}, planned.departure, planned.arrival, station(2) },
    };
    setFlows(0, flows);
    StationCargoStats stationCargo{};

    addProducedCargo(station(1), 0, stationCargo, 20);

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 10);
}

TEST_F(CargoDistServiceSimulationTest, RuntimeQueueRetiresLostCargo)
{
    constexpr auto planned = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto present = serviceLeg(1, 2, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(1), station(2), present.departure, present.arrival }] = { 20, 5, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(2) }] = {
        { station(2), 20, 0, planned.departure, planned.arrival },
    };
    state.stationCargo[{ station(1), 0 }].append({ 20, station(1), station(2), 0, planned.departure, planned.arrival, station(2) });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    stationCargo.quantity = 10;

    updateStationCargoDaily(station(1), 0, stationCargo, 20);

    EXPECT_EQ(getLoadableQuantity(station(1), 0, present), 0);
}

TEST_F(CargoDistServiceSimulationTest, ThroughCargoConsumesOnwardQueueCohort)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    constexpr auto planned = serviceLeg(2, 4, 7, 1, 2);
    constexpr auto present = serviceLeg(2, 4, 8, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(2), station(4), planned.departure, planned.arrival }] = { 10, 1, 1, 10 };
    state.serviceEdges[{ 0, station(2), station(4), present.departure, present.arrival }] = { 20, 5, 100, 200 };
    const std::array flows = {
        FlowShare{ station(2), station(1), station(4), 10, planned.departure, planned.departure, planned.arrival, station(4) },
        FlowShare{ station(2), station(2), station(4), 10, {}, planned.departure, planned.arrival, station(4) },
    };
    setFlows(0, flows);
    state.stationCargo[{ station(2), 0 }].append({ 10, station(2), station(4), 0, planned.departure, planned.arrival, station(4) });
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    state.vehicleCargo[key].append({ 10, station(1), station(2), 0, servicePoint(7, 0), planned.departure, station(4) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, station(1), 0, 10 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, planned);

    EXPECT_EQ(result.quantity(), 0);
    EXPECT_EQ(getLoadableQuantity(station(2), 0, present), 0);
}

TEST_F(CargoDistServiceSimulationTest, DynamicJourneyContinuesAcrossDifferentNextStop)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    constexpr auto plannedFirst = serviceLeg(1, 2, 7, 0, 1);
    constexpr auto plannedSecond = serviceLeg(2, 4, 7, 1, 2);
    constexpr auto presentFirst = serviceLeg(1, 3, 8, 0, 1);
    constexpr auto presentSecond = serviceLeg(3, 4, 8, 1, 2);
    auto& state = getState();
    state.serviceEdges[{ 0, station(1), station(2), plannedFirst.departure, plannedFirst.arrival }] = { 10, 10, 1, 2 };
    state.serviceEdges[{ 0, station(2), station(4), plannedSecond.departure, plannedSecond.arrival }] = { 10, 100, 1, 2 };
    state.serviceEdges[{ 0, station(1), station(3), presentFirst.departure, presentFirst.arrival }] = { 10, 1, 100, 200 };
    state.serviceEdges[{ 0, station(3), station(4), presentSecond.departure, presentSecond.arrival }] = { 10, 1, 100, 200 };
    state.flows[{ 0, station(1), station(1), {}, station(4) }] = {
        { station(2), 10, 0, plannedFirst.departure, plannedFirst.arrival },
    };
    state.flows[{ 0, station(2), station(1), plannedFirst.arrival, station(4) }] = {
        { station(4), 10, 0, plannedSecond.departure, plannedSecond.arrival },
    };
    auto& waiting = state.stationCargo[{ station(1), 0 }];
    waiting.append({ 10, station(1), station(2), 0, plannedFirst.departure, plannedFirst.arrival, station(4) });
    StationCargoStats sourceCargo{};
    synchroniseStationCargo(station(1), 0, sourceCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    ASSERT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), sourceCargo, presentFirst), 10);
    StationCargoStats intermediateCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(3), intermediateCargo, {}, false, presentSecond);

    EXPECT_EQ(result.quantity(), 0);
    EXPECT_EQ(vehicleCargo.qty, 10);
    const auto* loaded = getVehicleCargoConst(key);
    ASSERT_NE(loaded, nullptr);
    EXPECT_EQ(loaded->packets().front().nextHop, station(4));
    EXPECT_EQ(loaded->packets().front().departure, presentSecond.departure);
    EXPECT_EQ(loaded->packets().front().arrival, presentSecond.arrival);
}

TEST_F(CargoDistServiceSimulationTest, DynamicJourneyReturnsOnlyToItsDestination)
{
    getGameState().stations[1].cargoStats[0].isAccepted(true);
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    constexpr auto incoming = serviceLeg(1, 2, 7, 1, 0);
    constexpr auto reverse = serviceLeg(2, 1, 7, 0, 1);
    constexpr auto alternative = serviceLeg(2, 3, 8, 0, 1);
    constexpr auto fromPrevious = serviceLeg(1, 4, 9, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(2), station(1), reverse.departure, reverse.arrival }] = { 10, 1, 1, 2 };
    state.serviceEdges[{ 0, station(1), station(4), fromPrevious.departure, fromPrevious.arrival }] = { 10, 1, 1, 2 };
    state.flows[{ 0, station(2), station(1), incoming.arrival, station(4) }] = {
        { station(3), 10, 0, alternative.departure, alternative.arrival },
    };
    state.flows[{ 0, station(2), station(4), incoming.arrival, station(1) }] = {
        { station(1), 10, 0, reverse.departure, reverse.arrival },
    };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    state.vehicleCargo[key].append({ 10, station(1), station(2), 0, incoming.departure, incoming.arrival, station(4) });
    state.vehicleCargo[key].append({ 10, station(4), station(2), 0, incoming.departure, incoming.arrival, station(1) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 20, station(1), 0, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, reverse);

    EXPECT_EQ(result.transferred, 10);
    EXPECT_EQ(vehicleCargo.qty, 10);
    const auto* transferred = getStationCargoConst(station(2), 0);
    ASSERT_NE(transferred, nullptr);
    EXPECT_EQ(transferred->quantityFor(station(3), alternative.departure), 10);
    const auto* onboard = getVehicleCargoConst(key);
    ASSERT_NE(onboard, nullptr);
    EXPECT_EQ(onboard->quantityFor(station(1), reverse.departure), 10);
}

TEST_F(CargoDistServiceSimulationTest, ReverseFallbackDoesNotReloadAtSameStop)
{
    getGameState().stations[2].cargoStats[0].isAccepted(false);
    getGameState().stations[4].cargoStats[0].isAccepted(true);
    constexpr auto incoming = serviceLeg(1, 2, 7, 1, 0);
    constexpr auto reverse = serviceLeg(2, 1, 7, 0, 1);
    auto& state = getState();
    state.serviceEdges[{ 0, station(2), station(1), reverse.departure, reverse.arrival }] = { 10, 1, 1, 2 };
    state.flows[{ 0, station(2), station(1), {}, station(4) }] = {
        { station(1), 10, 0, reverse.departure, reverse.arrival },
    };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    state.vehicleCargo[key].append({ 10, station(1), station(2), 0, incoming.departure, incoming.arrival, station(4) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, station(1), 0, 10 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, reverse);

    EXPECT_EQ(result.transferred, 10);
    EXPECT_EQ(vehicleCargo.qty, 0);
    const auto* transferred = getStationCargoConst(station(2), 0);
    ASSERT_NE(transferred, nullptr);
    ASSERT_EQ(transferred->packets().size(), 1);
    EXPECT_EQ(transferred->packets().front().nextHop, StationId::null);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(2), stationCargo, reverse), 0);
    EXPECT_TRUE(getStateConst().graphDirty);
}

TEST_F(CargoDistServiceSimulationTest, SettlesPendingTransferRevenue)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    head->aiThoughtId = 0xFF;
    train.veh2->curMonthRevenue = 100;
    addVehicleRevenueAdjustment(head->id, 75);
    addVehicleRevenueAdjustment(head->id, -75);
    EXPECT_FALSE(consumeVehicleRevenueAdjustment(head->id).has_value());
    addVehicleRevenueAdjustment(head->id, 75);

    head->settleCargoIncome();

    EXPECT_EQ(train.veh2->curMonthRevenue, 175);
    EXPECT_FALSE(consumeVehicleRevenueAdjustment(head->id).has_value());
    EXPECT_NE(train.veh1->var_48 & Vehicles::Flags48::flag2, Vehicles::Flags48::none);

    addVehicleRevenueAdjustment(head->id, std::numeric_limits<int64_t>::max());
    head->settleCargoIncome();
    EXPECT_EQ(train.veh2->curMonthRevenue, std::numeric_limits<currency32_t>::max());
    addVehicleRevenueAdjustment(head->id, std::numeric_limits<int64_t>::min());
    head->settleCargoIncome();
    EXPECT_EQ(train.veh2->curMonthRevenue, std::numeric_limits<currency32_t>::min());
}

TEST_F(CargoDistServiceSimulationTest, KeepsRoutingEnabledForCreditedOnboardCargo)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    const VehicleCargoKey key{ train.cars.firstCar.body->id, VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 10, station(1), station(2), 0, {}, {}, station(2), 50 });

    setMode(0, DistributionMode::manual);

    EXPECT_EQ(getMode(0), DistributionMode::asymmetric);
}

TEST_F(CargoDistServiceSimulationTest, KeepsRoutingEnabledForOnboardHolidayPassengers)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    const VehicleCargoKey key{ train.cars.firstCar.body->id, VehicleCargoSlot::primary };
    CargoPacket packet{ 10, station(1), station(2), 0, {}, {}, station(2) };
    packet.tripKind = PassengerTripKind::holidayOutbound;
    packet.holidayIndustry = IndustryId(2);
    packet.homeTown = TownId(3);
    getOrCreateVehicleCargo(key).append(packet);

    setMode(0, DistributionMode::manual);

    EXPECT_EQ(getMode(0), DistributionMode::asymmetric);
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
    const std::array flows = { FlowShare{ station(1), station(1), station(2), 20, {}, {}, {}, station(2) } };
    setFlows(0, flows);
    StationCargoStats cargo{};
    const auto cargoRevision = getStateConst().cargoRevision;

    addProducedCargo(station(1), 0, cargo, 20);

    ASSERT_NE(getStationCargoConst(station(1), 0), nullptr);
    EXPECT_EQ(getStationCargoConst(station(1), 0)->quantityFor(station(2)), 20);
    EXPECT_EQ(cargo.quantity, 20);
    EXPECT_EQ(cargo.origin, station(1));
    EXPECT_EQ(getStationCargoConst(station(1), 0)->packets().front().destination, station(2));
    EXPECT_EQ(getStateConst().supply.at({ 0, station(1) }), 20);
    EXPECT_EQ(getStateConst().cargoRevision, cargoRevision + 1);
}

TEST(CargoDistSimulation, PreviewUsesFlowCursorWithoutAdvancingIt)
{
    reset();
    auto& options = getState().flows[{ 0, station(1), station(1), {}, station(4) }];
    options = {
        { station(2), 1, -kFlowCursorScale },
        { station(3), 1, kFlowCursorScale },
    };

    const auto preview = previewVia(0, station(1), station(1), station(4), 1);

    ASSERT_EQ(preview.size(), 1);
    EXPECT_EQ(preview.front().via, station(3));
    EXPECT_EQ(options[0].current, -kFlowCursorScale);
    EXPECT_EQ(options[1].current, kFlowCursorScale);
    const auto allocated = allocateVia(0, station(1), station(1), station(4), 1);
    ASSERT_EQ(allocated.size(), 1);
    EXPECT_EQ(allocated.front().via, preview.front().via);
}

TEST(CargoDistSimulation, SplitsProducedCargoByFlowQuantity)
{
    reset();
    const std::array flows = {
        FlowShare{ station(1), station(1), station(2), 3, {}, {}, {}, station(2) },
        FlowShare{ station(1), station(1), station(3), 1, {}, {}, {}, station(3) },
    };
    setFlows(0, flows);
    StationCargoStats cargo{};

    addProducedCargo(station(1), 0, cargo, 40);

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->quantityFor(station(2)), 30);
    EXPECT_EQ(packets->quantityFor(station(3)), 10);
}

TEST(CargoDistSimulation, KeepsUnmatchedProducedPassengersUnrouted)
{
    reset();
    RoutingGraph graph{
        {
            { station(1), 0, 0, 40, true },
            { station(2), 10, 0, 10, true },
        },
        {
            { station(1), station(2), 100, 1 },
            { station(2), station(1), 100, 1 },
        },
        false,
        {},
    };
    graph.passengerRouting = true;
    setFlows(0, calculateAsymmetricFlows(graph));
    StationCargoStats cargo{};

    addProducedCargo(station(1), 0, cargo, 40);

    const auto* packets = getStationCargoConst(station(1), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->quantityFor(StationId::null), 30U);
    EXPECT_EQ(packets->quantityFor(station(2)), 10U);
}

TEST(CargoDistSimulation, StoresProducedCargoAboveNativeStationLimit)
{
    reset();
    getOrCreateStationCargo(station(1), 0).append({ std::numeric_limits<uint16_t>::max(), station(1), StationId::null, 0 });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(1), 0, cargo);

    addProducedCargo(station(1), 0, cargo, 10);

    EXPECT_EQ(cargo.quantity, std::numeric_limits<uint16_t>::max());
    EXPECT_EQ(getStationCargoConst(station(1), 0)->quantity(), std::numeric_limits<uint16_t>::max() + 10U);
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

TEST_F(CargoDistServiceSimulationTest, LostCargoReversesTransferRevenueWithoutCashPayment)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    head->aiThoughtId = 0xFF;
    train.veh2->curMonthRevenue = 100;
    getOrCreateStationCargo(station(1), 0).append({ 10, station(2), station(2), 4, {}, {}, station(2), 60 });
    StationCargoStats cargo{};
    synchroniseStationCargo(station(1), 0, cargo);
    cargo.quantity = 0;
    const auto cashBefore = getGameState().companies[0].cash.asInt64();

    updateStationCargoDaily(station(1), 0, cargo, 10);

    EXPECT_EQ(getStationCargoConst(station(1), 0)->quantity(), 0U);
    EXPECT_EQ(cargo.quantity, 0);
    EXPECT_EQ(train.veh2->curMonthRevenue, 40);
    EXPECT_EQ(getGameState().companies[0].cash.asInt64(), cashBefore);
    EXPECT_FALSE(consumeVehicleRevenueAdjustment(head->id).has_value());
}

TEST_F(CargoDistServiceSimulationTest, ExpiredReleasedReturnReversesTransferRevenue)
{
    auto* head = createVehicle();
    Vehicles::Vehicle train(*head);
    head->aiThoughtId = 0xFF;
    train.veh2->curMonthRevenue = 100;
    PendingHolidayReturn pending{};
    pending.quantity = 10;
    pending.resortStation = station(1);
    pending.homeStation = station(2);
    pending.age = std::numeric_limits<uint8_t>::max() - 1;
    pending.released = true;
    pending.transferCredit = 60;
    getState().pendingHolidayReturns.push_back(pending);
    const auto cashBefore = getGameState().companies[0].cash.asInt64();

    updateDaily();

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    EXPECT_EQ(train.veh2->curMonthRevenue, 40);
    EXPECT_EQ(getGameState().companies[0].cash.asInt64(), cashBefore);
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

TEST(CargoDistSimulation, ExplicitCrushCapacityLoadsAboveNominalCapacity)
{
    reset();
    constexpr auto leg = serviceLeg(1, 2, 4, 0, 1);
    auto& waiting = getOrCreateStationCargo(station(1), 0);
    waiting.append({ 20, station(1), station(2), 3, leg.departure, leg.arrival });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 10, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, leg, 15), 15);
    EXPECT_EQ(vehicleCargo.maxQty, 10);
    EXPECT_EQ(vehicleCargo.qty, 15);
    EXPECT_EQ(stationCargo.quantity, 5);
    EXPECT_EQ(getVehicleCargoConst(key)->quantity(), 15U);
    EXPECT_EQ(loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, leg, 15), 0);
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
    const std::array flows = { FlowShare{ station(2), station(1), station(2), 20, {}, {}, {}, station(2) } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, {}, {}, station(2) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.delivered.quantity(), 20);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(stationCargo.quantity, 0);
}

TEST(CargoDistSimulation, TransferCreditsTrackMarginalProjectedValue)
{
    CargoPacket packet{ 20, station(1), station(2), 3 };

    const auto firstCredit = accrueTransferCredit(packet, 10);
    const auto secondCredit = accrueTransferCredit(packet, 100);
    const auto finalIncome = calculateFinalDeliveryIncome(packet.transferCredit, 110);

    EXPECT_EQ(firstCredit, 10);
    EXPECT_EQ(secondCredit, 90);
    EXPECT_EQ(packet.transferCredit, 100);
    EXPECT_EQ(finalIncome, 10);
    EXPECT_EQ(firstCredit + secondCredit + finalIncome, 110);
}

TEST(CargoDistSimulation, ReassignsCargoWhenItsDestinationStopsAccepting)
{
    reset();
    const std::array flows = {
        FlowShare{ station(2), station(1), station(2), 20, {}, {}, {}, station(2) },
        FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, {}, {}, station(2) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.transferred, 20);
    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->packets().front().destination, station(3));
    EXPECT_EQ(packets->packets().front().nextHop, station(3));
    EXPECT_TRUE(getStateConst().graphDirty);
}

TEST(CargoDistSimulation, TransfersCargoAlongItsNextFlowLeg)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, {}, {}, station(3) });
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

TEST(CargoDistSimulation, TransferCreditFollowsCargoToItsDestination)
{
    reset();
    constexpr auto transferLeg = serviceLeg(2, 3, 8, 0, 1);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, {}, transferLeg.departure, transferLeg.arrival, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey firstKey{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(firstKey).append({ 20, station(1), station(2), 3, {}, {}, station(3) });
    Vehicles::VehicleCargo firstCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats transferCargo{};

    const auto transfer = unloadVehicleCargo(firstKey, firstCargo, station(2), transferCargo, {}, false, std::nullopt, [](const auto&) { return 100; });

    ASSERT_EQ(transfer.transferCredits.size(), 1U);
    EXPECT_EQ(transfer.transferCredits.front().amount, 100);
    const auto* waiting = getStationCargoConst(station(2), 0);
    ASSERT_NE(waiting, nullptr);
    ASSERT_EQ(waiting->size(), 1U);
    EXPECT_EQ(waiting->packets().front().transferCredit, 100);

    const VehicleCargoKey secondKey{ entity(11), VehicleCargoSlot::primary };
    Vehicles::VehicleCargo secondCargo{ 1, 0, 30, StationId::null, 0, 0 };
    EXPECT_EQ(loadVehicleCargo(secondKey, secondCargo, station(2), transferCargo, transferLeg), 20);
    StationCargoStats destinationCargo{};
    destinationCargo.isAccepted(true);

    const auto delivery = unloadVehicleCargo(secondKey, secondCargo, station(3), destinationCargo, {}, false, std::nullopt);

    ASSERT_EQ(delivery.delivered.size(), 1U);
    EXPECT_EQ(delivery.delivered.packets().front().transferCredit, 100);
    EXPECT_EQ(calculateFinalDeliveryIncome(delivery.delivered.packets().front().transferCredit, 160), 60);
}

TEST(CargoDistSimulation, UnresolvedVehicleCargoCommitsSelectedDestination)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt);

    EXPECT_EQ(result.transferred, 20);
    const auto* packets = getStationCargoConst(station(2), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->packets().front().destination, station(3));
}

TEST(CargoDistSimulation, ReroutesCargoWhoseNextHopIsNoLongerServed)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(9), 3, {}, {}, station(3) });
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
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) } };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(4), 3, {}, {}, station(3) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);
    const std::array remainingStops = { station(4) };

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, remainingStops, true, std::nullopt);

    EXPECT_EQ(result.delivered.quantity(), 20);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_EQ(vehicleCargo.qty, 0);
}

TEST(CargoDistSimulation, ForcedUnloadKeepsHolidayPassengersForTheirExactDestination)
{
    reset();
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    CargoPacket packet{ 20, station(1), station(4), 3, {}, {}, station(3) };
    packet.tripKind = PassengerTripKind::holidayReturn;
    packet.holidayIndustry = IndustryId(3);
    packet.homeTown = TownId(4);
    getOrCreateVehicleCargo(key).append(packet);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    stationCargo.isAccepted(true);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, true, std::nullopt);

    EXPECT_TRUE(result.delivered.empty());
    EXPECT_EQ(result.transferred, 20);
    const auto& waiting = getStationCargoConst(station(2), 0)->packets().front();
    EXPECT_EQ(waiting.destination, station(3));
    EXPECT_EQ(waiting.tripKind, PassengerTripKind::holidayReturn);
}

TEST(CargoDistSimulation, StoresTransferredCargoAboveNativeStationLimit)
{
    reset();
    const std::array flows = { FlowShare{ station(2), station(1), station(3), 20, {}, {}, {}, station(3) } };
    setFlows(0, flows);
    auto& waiting = getOrCreateStationCargo(station(2), 0);
    waiting.append({ 65530, station(4), station(3), 0 });
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, {}, {}, station(3), 20 });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(2), 0, stationCargo);

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, std::nullopt, [](const auto& packet) { return packet.quantity * 10; });

    EXPECT_EQ(result.transferred, 20);
    ASSERT_EQ(result.transferCredits.size(), 1U);
    EXPECT_EQ(result.transferCredits.front().packet.quantity, 20);
    EXPECT_EQ(result.transferCredits.front().amount, 180);
    EXPECT_EQ(result.transferCredits.front().packet.transferCredit, 200);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(stationCargo.quantity, std::numeric_limits<uint16_t>::max());
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantity(), 65550U);
    ASSERT_NE(getVehicleCargoConst(key), nullptr);
    EXPECT_TRUE(getVehicleCargoConst(key)->empty());
}

TEST(CargoDistSimulation, ContinuesOnSameServiceWithoutTransfer)
{
    reset();
    constexpr auto onward = serviceLeg(2, 3, 7, 1, 2);
    const std::array flows = {
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, onward.departure, onward.arrival, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure, station(3) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false, onward, [](const auto&) { return 100; });

    EXPECT_EQ(result.quantity(), 0);
    EXPECT_EQ(result.transferred, 0);
    EXPECT_TRUE(result.transferCredits.empty());
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
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, transfer.departure, transfer.arrival, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure, station(3) });
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
        FlowShare{ station(2), station(1), station(3), 20, {}, transfer.departure, transfer.arrival, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(99, 0), servicePoint(99, 1), station(3) });
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
        FlowShare{ station(2), station(1), station(3), 20, onward.departure, onward.departure, onward.arrival, station(3) },
    };
    setFlows(0, flows);
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };
    getOrCreateVehicleCargo(key).append({ 20, station(1), station(2), 3, servicePoint(7, 0), onward.departure, station(3) });
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, station(1), 3, 20 };
    StationCargoStats stationCargo{};

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, true, onward);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(vehicleCargo.qty, 0);
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantityFor(station(3), onward.departure), 20);
}

TEST(CargoDistSimulation, RemovingStationClearsAccessibilityAndKeepsFlowCursorsSerializable)
{
    reset();
    EntityManager::reset();
    getState().stationAccessibility = { { station(3), 123 }, { station(4), 456 } };
    getState().hasStationAccessibilitySnapshot = true;
    getState().flows[{ 0, station(1), station(2), {}, station(4) }] = {
        { station(3), 75, -25 },
        { station(4), 25, 25 },
    };

    removeStation(station(3));

    EXPECT_EQ(getStationAccessibility(station(3)), 0U);
    EXPECT_EQ(getStationAccessibility(station(4)), 456U);
    const auto& options = getStateConst().flows.at({ 0, station(1), station(2), {}, station(4) });
    ASSERT_EQ(options.size(), 1);
    EXPECT_EQ(options.front().current, 0);
    const auto& destinations = getStateConst().destinationFlows.at({ 0, station(1), station(2) });
    ASSERT_EQ(destinations.size(), 1);
    EXPECT_EQ(destinations.front().weight, 25);
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
    state.stationCargo[{ station(1), 0 }].append({ 10, station(1), station(2), 0, removed.departure, removed.arrival, station(3) });
    state.serviceEdges[{ 0, station(1), station(2), removed.departure, removed.arrival }] = { 10, 5, 3 };
    state.flows[{ 0, station(1), station(1), {}, station(3) }] = {
        { station(2), 10, 0, removed.departure, removed.arrival },
        { station(3), 10, 0, retained.departure, retained.arrival },
    };

    removeVehicleService(entity(7));

    const auto& packet = state.stationCargo.at({ station(1), 0 }).packets().front();
    EXPECT_EQ(packet.nextHop, StationId::null);
    EXPECT_TRUE(packet.departure.empty());
    EXPECT_TRUE(state.serviceEdges.empty());
    const auto& options = state.flows.at({ 0, station(1), station(1), {}, station(3) });
    ASSERT_EQ(options.size(), 1);
    EXPECT_EQ(options.front().departure, retained.departure);
    EXPECT_EQ(options.front().current, 0);
    const auto& destinations = state.destinationFlows.at({ 0, station(1), station(1) });
    ASSERT_EQ(destinations.size(), 1);
    EXPECT_EQ(destinations.front().weight, 10);
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

TEST(CargoDistSimulation, HolidayPolicyUsesTownTiersAndBoundedPopularityCapacity)
{
    EXPECT_EQ(getHolidayPercentage(TownSize::hamlet), 1);
    EXPECT_EQ(getHolidayPercentage(TownSize::village), 1);
    EXPECT_EQ(getHolidayPercentage(TownSize::town), 3);
    EXPECT_EQ(getHolidayPercentage(TownSize::city), 6);
    EXPECT_EQ(getHolidayPercentage(TownSize::metropolis), 8);
    EXPECT_EQ(getResortCapacity(10, 0), 40);
    EXPECT_EQ(getResortCapacity(10, 100), 60);
    EXPECT_EQ(updateResortPopularity(80, 60, 30), 60);
    EXPECT_EQ(updateResortPopularity(80, 0, 0), 40);
}

class CargoDistHolidayReturnTest : public ::testing::Test
{
protected:
    static constexpr auto kHomeTown = TownId(10);
    static constexpr auto kResort = IndustryId(2);

    void SetUp() override
    {
        _home = getGameState().stations[10];
        _fallbackHome = getGameState().stations[11];
        _resort = getGameState().stations[12];
        _town = getGameState().towns[enumValue(kHomeTown)];
        _currentDay = getGameState().currentDay;
        EntityManager::reset();
        Vehicles::OrderManager::reset();
        reset();

        auto& town = getGameState().towns[enumValue(kHomeTown)];
        town = {};
        town.name = StringId(1);

        initialiseStation(10, kHomeTown);
        initialiseStation(11, kHomeTown);
        initialiseStation(12, TownId(11));
        getGameState().stations[10].cargoStats[0].isAccepted(true);
        getGameState().stations[12].cargoStats[0].industryId = kResort;

        getGameState().currentDay = 100;
        auto& state = getState();
        state.settings.modes[0] = DistributionMode::asymmetric;
        state.nextRecalculationDay = 200;
    }

    void TearDown() override
    {
        reset();
        Vehicles::OrderManager::reset();
        EntityManager::reset();
        getGameState().stations[10] = _home;
        getGameState().stations[11] = _fallbackHome;
        getGameState().stations[12] = _resort;
        getGameState().towns[enumValue(kHomeTown)] = _town;
        getGameState().currentDay = _currentDay;
    }

    static void initialiseStation(const uint16_t id, const TownId town)
    {
        auto& value = getGameState().stations[id];
        value = {};
        value.name = StringId(1);
        value.town = town;
    }

    static CargoPacket holidayPacket(const PassengerTripKind kind, const uint16_t quantity, const StationId origin, const StationId destination, const int64_t transferCredit = 0)
    {
        CargoPacket packet{ quantity, origin, destination, 0, {}, {}, destination, transferCredit };
        packet.tripKind = kind;
        packet.holidayIndustry = kResort;
        packet.homeTown = kHomeTown;
        return packet;
    }

private:
    Station _home{};
    Station _fallbackHome{};
    Station _resort{};
    Town _town{};
    uint32_t _currentDay{};
};

TEST_F(CargoDistHolidayReturnTest, ReleasesDuePassengersAsExactHomeboundCargo)
{
    getState().pendingHolidayReturns.push_back({ 100, 12, station(12), station(10), kHomeTown, kResort, 0 });

    updateDaily();

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    const auto* packets = getStationCargoConst(station(12), 0);
    ASSERT_NE(packets, nullptr);
    ASSERT_EQ(packets->packets().size(), 1);
    const auto& packet = packets->packets().front();
    EXPECT_EQ(packet.quantity, 12);
    EXPECT_EQ(packet.origin, station(12));
    EXPECT_EQ(packet.destination, station(10));
    EXPECT_EQ(packet.tripKind, PassengerTripKind::holidayReturn);
    EXPECT_EQ(packet.holidayIndustry, kResort);
    EXPECT_EQ(packet.homeTown, kHomeTown);
    EXPECT_EQ(getGameState().stations[12].cargoStats[0].quantity, 12);
}

TEST_F(CargoDistHolidayReturnTest, DefersReturnUntilSameTownFallbackAcceptsPassengers)
{
    getGameState().stations[10].cargoStats[0].isAccepted(false);
    getState().pendingHolidayReturns.push_back({ 100, 9, station(12), station(10), kHomeTown, kResort, 0 });

    updateDaily();

    ASSERT_EQ(getStateConst().pendingHolidayReturns.size(), 1);
    EXPECT_EQ(getStationCargoConst(station(12), 0), nullptr);

    getGameState().stations[11].cargoStats[0].isAccepted(true);
    updateDaily();

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    const auto* packets = getStationCargoConst(station(12), 0);
    ASSERT_NE(packets, nullptr);
    EXPECT_EQ(packets->packets().front().destination, station(11));
}

TEST_F(CargoDistHolidayReturnTest, AbandonsReturnWhenHomeRemainsUnavailable)
{
    getGameState().currentDay = 354;
    getGameState().stations[10].cargoStats[0].isAccepted(false);
    PendingHolidayReturn pending{};
    pending.releaseDay = 100;
    pending.quantity = 9;
    pending.resortStation = station(12);
    pending.homeStation = station(10);
    pending.homeTown = kHomeTown;
    pending.resort = kResort;
    getState().pendingHolidayReturns.push_back(pending);

    updateDaily();
    ASSERT_EQ(getStateConst().pendingHolidayReturns.size(), 1);

    getGameState().currentDay = 355;
    updateDaily();

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    EXPECT_EQ(getStationCargoConst(station(12), 0), nullptr);
}

TEST_F(CargoDistHolidayReturnTest, CompletesReturnAlreadyAtAHomeTownStation)
{
    getGameState().stations[12].town = kHomeTown;
    getGameState().stations[12].cargoStats[0].isAccepted(true);
    getState().pendingHolidayReturns.push_back({ 100, 9, station(12), station(12), kHomeTown, kResort, 0 });

    updateDaily();

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    EXPECT_EQ(getStationCargoConst(station(12), 0), nullptr);
}

TEST_F(CargoDistHolidayReturnTest, RepairsRejectedHomeDestinationWithinOriginalTown)
{
    getGameState().stations[10].cargoStats[0].isAccepted(false);
    getGameState().stations[11].cargoStats[0].isAccepted(true);
    getOrCreateStationCargo(station(12), 0).append(holidayPacket(PassengerTripKind::holidayReturn, 7, station(12), station(10)));

    updateDaily();

    const auto& repaired = getStationCargoConst(station(12), 0)->packets().front();
    EXPECT_EQ(repaired.destination, station(11));
    EXPECT_EQ(repaired.nextHop, StationId::null);
}

TEST_F(CargoDistHolidayReturnTest, RemovingDepartureStationPreservesReleasedReturn)
{
    auto packet = holidayPacket(PassengerTripKind::holidayReturn, 6, station(12), station(10), 42);
    packet.nextHop = StationId::null;
    packet.age = 9;
    getOrCreateStationCargo(station(12), 0).append(packet);

    removeStation(station(12));

    ASSERT_EQ(getStateConst().pendingHolidayReturns.size(), 1);
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().quantity, 6);
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().homeStation, station(10));
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().age, 9);
    EXPECT_TRUE(getStateConst().pendingHolidayReturns.front().released);
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().transferCredit, 42);
    EXPECT_EQ(getStationCargoConst(station(12), 0), nullptr);

    getGameState().stations[10].cargoStats[0].isAccepted(false);
    updateDaily();
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().age, 10);
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().transferCredit, 42);
}

TEST_F(CargoDistHolidayReturnTest, ClosingResortCancelsOutboundTripsAndReleasesGuests)
{
    getState().pendingHolidayReturns.push_back({ 120, 5, station(12), station(10), kHomeTown, kResort, 0 });
    getState().pendingHolidayReturns.push_back({ 120, 3, station(12), station(10), kHomeTown, IndustryId(3), 0 });
    getState().resorts[IndustryId(3)].guestDays = 17;
    getOrCreateStationCargo(station(10), 0).append(holidayPacket(PassengerTripKind::holidayOutbound, 7, station(10), station(12), 35));

    removeIndustry(kResort);

    ASSERT_EQ(getStateConst().pendingHolidayReturns.size(), 1);
    EXPECT_EQ(getStateConst().pendingHolidayReturns.front().resort, IndustryId(3));
    EXPECT_EQ(getStateConst().resorts.at(IndustryId(3)).guestDays, 17);
    EXPECT_EQ(getStationCargoConst(station(10), 0), nullptr);
    const auto* returning = getStationCargoConst(station(12), 0);
    ASSERT_NE(returning, nullptr);
    EXPECT_EQ(returning->quantity(), 5);
    EXPECT_EQ(returning->packets().front().tripKind, PassengerTripKind::holidayReturn);
}

TEST_F(CargoDistHolidayReturnTest, ClosingResortTurnsInTransitPassengersBackTowardHome)
{
    getOrCreateStationCargo(station(12), 0).append(holidayPacket(PassengerTripKind::holidayOutbound, 7, station(10), station(12), 35));

    removeIndustry(kResort);

    const auto* returning = getStationCargoConst(station(12), 0);
    ASSERT_NE(returning, nullptr);
    ASSERT_EQ(returning->packets().size(), 1);
    EXPECT_EQ(returning->packets().front().quantity, 7);
    EXPECT_EQ(returning->packets().front().tripKind, PassengerTripKind::holidayReturn);
    EXPECT_EQ(returning->packets().front().destination, station(10));
    EXPECT_EQ(returning->packets().front().nextHop, StationId::null);
    EXPECT_EQ(returning->packets().front().transferCredit, 35);
}

TEST_F(CargoDistHolidayReturnTest, ClosingResortDropsSourceLessGuestState)
{
    getGameState().stations[12].cargoStats[0].industryId = IndustryId::null;
    getState().pendingHolidayReturns.push_back({ 120, 5, StationId::null, station(10), kHomeTown, kResort, 0 });

    removeIndustry(kResort);

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
}

TEST_F(CargoDistHolidayReturnTest, RemovingHomeTownClearsImpossibleHolidayState)
{
    getOrCreateStationCargo(station(12), 0).append(holidayPacket(PassengerTripKind::holidayReturn, 6, station(12), station(10), 42));
    getState().pendingHolidayReturns.push_back({ 120, 5, station(12), station(10), kHomeTown, kResort, 0 });
    getState().holidaySources[{ station(10), 0 }] = {};

    removeTown(kHomeTown);

    EXPECT_TRUE(getStateConst().pendingHolidayReturns.empty());
    EXPECT_TRUE(getStateConst().holidaySources.empty());
    EXPECT_EQ(getStationCargoConst(station(12), 0), nullptr);
}
