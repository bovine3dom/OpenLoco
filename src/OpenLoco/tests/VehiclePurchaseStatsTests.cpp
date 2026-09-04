#include "Date.h"
#include "Objects/VehicleObject.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehiclePurchaseStats.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles;

namespace
{
    struct CurrentYearGuard
    {
        CurrentYearGuard()
            : year(getCurrentYear())
        {
        }

        ~CurrentYearGuard()
        {
            setCurrentYear(year);
        }

        uint16_t year;
    };

    VehicleObject makeVehicleObject()
    {
        VehicleObject vehicle{};
        vehicle.mode = TransportMode::rail;
        vehicle.reliability = 80;
        vehicle.designed = 1900;
        vehicle.obsolete = 2100;
        vehicle.power = 1000;
        vehicle.weight = 100;
        vehicle.maxCargo[0] = 50;
        vehicle.compatibleCargoCategories[0] = 1U << 0;
        vehicle.numSimultaneousCargoTypes = 1;
        vehicle.carComponents[0].bodySpriteInd = 0;
        vehicle.bodySprites[0].halfLength = 64;
        vehicle.numCarComponents = 1;
        return vehicle;
    }
}

TEST(VehiclePurchaseStatsTest, CalculatesLandVehicleStatistics)
{
    CurrentYearGuard yearGuard;
    setCurrentYear(2000);
    const auto stats = calculateVehiclePurchaseStats(makeVehicleObject());

    EXPECT_EQ(stats.reliability, 80 * 256 + 255);
    EXPECT_EQ(stats.reliabilityLossPerDay, 4);
    EXPECT_EQ(stats.powerToWeightQ16, 10 * kPurchaseStatFractionalScale);
    EXPECT_EQ(stats.cargoCapacity, 50);
    EXPECT_EQ(stats.cargoType, 0);
    EXPECT_EQ(stats.capacityPerTileQ16, 50 * kPurchaseStatFractionalScale);
    EXPECT_EQ(purchaseStatToTenths(stats.powerToWeightQ16), 100);
    EXPECT_EQ(reliabilityLossPerYearTenths(stats.reliabilityLossPerDay), 57);
}

TEST(VehiclePurchaseStatsTest, UsesSelectedCargoCapacityAcrossCompartments)
{
    auto vehicle = makeVehicleObject();
    vehicle.maxCargo[1] = 20;
    vehicle.compatibleCargoCategories[0] = (1U << 1) | (1U << 2);
    vehicle.compatibleCargoCategories[1] = (1U << 2) | (1U << 3);
    vehicle.numSimultaneousCargoTypes = 2;

    EXPECT_EQ(calculateVehiclePurchaseStats(vehicle).cargoCapacity, 50);
    EXPECT_EQ(calculateVehiclePurchaseStats(vehicle, 1).cargoCapacity, 50);
    EXPECT_EQ(calculateVehiclePurchaseStats(vehicle, 2).cargoCapacity, 70);

    const auto secondaryCargoStats = calculateVehiclePurchaseStats(vehicle, 3);
    EXPECT_EQ(secondaryCargoStats.cargoCapacity, 20);
    EXPECT_EQ(secondaryCargoStats.cargoType, 3);

    const auto unsupportedCargoStats = calculateVehiclePurchaseStats(vehicle, 4);
    EXPECT_EQ(unsupportedCargoStats.cargoCapacity, 0);
    EXPECT_EQ(unsupportedCargoStats.cargoType, 0xFF);

    vehicle.compatibleCargoCategories[0] = 1U << 31;
    EXPECT_EQ(calculateVehiclePurchaseStats(vehicle, 31).cargoCapacity, 50);
}

TEST(VehiclePurchaseStatsTest, CalculatesFullLoadTransferTimeAcrossCompartments)
{
    auto vehicle = makeVehicleObject();
    vehicle.maxCargo[1] = 20;
    vehicle.compatibleCargoCategories[0] = (1U << 1) | (1U << 2);
    vehicle.compatibleCargoCategories[1] = (1U << 2) | (1U << 3);
    vehicle.numSimultaneousCargoTypes = 2;

    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 1, 256), 64);
    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 2, 256), 84);
    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 3, 256), 34);
    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 4, 256), 0);
    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 0xFF, 256), 0);
    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 2, 256, true), 154);
}

TEST(VehiclePurchaseStatsTest, AccumulatesLargeCargoTransferTimeouts)
{
    auto vehicle = makeVehicleObject();
    vehicle.maxCargo[0] = 255;
    vehicle.maxCargo[1] = 255;
    vehicle.compatibleCargoCategories[1] = 1U << 0;
    vehicle.numSimultaneousCargoTypes = 2;

    EXPECT_EQ(calculateFullLoadTimeTicks(vehicle, 0, std::numeric_limits<uint16_t>::max()), 130572);
    EXPECT_EQ(calculateCargoTransferTimeout(std::numeric_limits<uint16_t>::max(), 255, 12), std::numeric_limits<uint16_t>::max());
}

TEST(VehiclePurchaseStatsTest, HandlesObsoleteAndNonBreakingVehicles)
{
    CurrentYearGuard yearGuard;
    setCurrentYear(2000);
    auto vehicle = makeVehicleObject();
    vehicle.obsolete = 2000;
    auto stats = calculateVehiclePurchaseStats(vehicle);
    EXPECT_EQ(stats.reliabilityLossPerDay, 10);
    EXPECT_EQ(reliabilityLossPerYearTenths(stats.reliabilityLossPerDay), 143);

    vehicle.reliability = 0;
    stats = calculateVehiclePurchaseStats(vehicle);
    EXPECT_EQ(stats.reliability, 0);
    EXPECT_EQ(stats.reliabilityLossPerDay, 0);
}

TEST(VehiclePurchaseStatsTest, HandlesUnavailableRatios)
{
    CurrentYearGuard yearGuard;
    setCurrentYear(2000);
    auto vehicle = makeVehicleObject();
    vehicle.mode = TransportMode::air;
    vehicle.bodySprites[0].halfLength = 0;

    const auto stats = calculateVehiclePurchaseStats(vehicle);

    EXPECT_EQ(stats.powerToWeightQ16, 0);
    EXPECT_EQ(stats.capacityPerTileQ16, 0);
    EXPECT_EQ(calculatePowerToWeightQ16(1000, 0), 0);
    EXPECT_EQ(calculatePowerToWeightQ16(1200, 150), 8 * kPurchaseStatFractionalScale);
    EXPECT_EQ(purchaseStatToTenths(convertHpToKwQ16(10 * kPurchaseStatFractionalScale)), 75);
}
