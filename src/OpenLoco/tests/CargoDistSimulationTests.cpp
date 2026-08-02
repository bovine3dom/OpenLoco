// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Simulation.h>

#include "Vehicles/Vehicle.h"
#include "World/Station.h"
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
    auto& waiting = getOrCreateStationCargo(station(1), 0);
    waiting.append({ 20, station(1), station(2), 3 });
    waiting.append({ 15, station(1), station(3), 2 });
    StationCargoStats stationCargo{};
    synchroniseStationCargo(station(1), 0, stationCargo);
    Vehicles::VehicleCargo vehicleCargo{ 1, 0, 30, StationId::null, 0, 0 };
    const VehicleCargoKey key{ entity(10), VehicleCargoSlot::primary };

    const auto loaded = loadVehicleCargo(key, vehicleCargo, station(1), stationCargo, station(2));

    EXPECT_EQ(loaded, 20);
    EXPECT_EQ(vehicleCargo.qty, 20);
    EXPECT_EQ(stationCargo.quantity, 15);
    EXPECT_EQ(getVehicleCargoConst(key)->quantityFor(station(2)), 20);
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

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false);

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

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false);

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

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, remainingStops, false);

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

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, remainingStops, true);

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

    const auto result = unloadVehicleCargo(key, vehicleCargo, station(2), stationCargo, {}, false);

    EXPECT_EQ(result.transferred, 20);
    EXPECT_EQ(stationCargo.quantity, std::numeric_limits<uint16_t>::max());
    EXPECT_EQ(getStationCargoConst(station(2), 0)->quantity(), std::numeric_limits<uint16_t>::max());
}
