#include "Vehicles/VehiclePurchaseStats.h"

#include "Date.h"
#include "Objects/VehicleObject.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleBogie.h"
#include <OpenLoco/Core/Numerics.hpp>
#include <algorithm>
#include <limits>

namespace OpenLoco::Vehicles
{
    namespace
    {
        uint32_t calculateRatioQ16(const uint64_t numerator, const uint32_t denominator)
        {
            if (denominator == 0)
            {
                return 0;
            }
            return static_cast<uint32_t>(std::min<uint64_t>((numerator << 16) / denominator, std::numeric_limits<uint32_t>::max()));
        }

        uint16_t getCargoCapacity(const VehicleObject& vehicleObject, const std::optional<uint8_t> cargoType)
        {
            if (!cargoType.has_value())
            {
                return vehicleObject.numSimultaneousCargoTypes != 0 && vehicleObject.compatibleCargoCategories[0] != 0
                    ? vehicleObject.maxCargo[0]
                    : 0;
            }
            if (*cargoType >= 32)
            {
                return 0;
            }

            uint16_t totalCapacity = 0;
            for (uint8_t compartment = 0; compartment < std::min<uint8_t>(vehicleObject.numSimultaneousCargoTypes, 2); ++compartment)
            {
                if ((vehicleObject.compatibleCargoCategories[compartment] & (1U << *cargoType)) != 0)
                {
                    totalCapacity += vehicleObject.maxCargo[compartment];
                }
            }
            return totalCapacity;
        }
    }

    uint32_t calculatePowerToWeightQ16(const uint32_t power, const uint32_t weight)
    {
        return calculateRatioQ16(power, weight);
    }

    uint32_t convertHpToKwQ16(const uint32_t valueQ16)
    {
        return static_cast<uint32_t>(static_cast<uint64_t>(valueQ16) * 764 / 1024);
    }

    uint32_t calculateFullLoadTimeTicks(const VehicleObject& vehicleObject, const uint8_t cargoType, const uint16_t cargoTransferTime, const bool crushLoading)
    {
        if (cargoType >= 32)
        {
            return 0;
        }

        uint32_t totalTicks = 0;
        bool carriesCargo = false;
        for (uint8_t compartment = 0; compartment < std::min<uint8_t>(vehicleObject.numSimultaneousCargoTypes, 2); ++compartment)
        {
            if ((vehicleObject.compatibleCargoCategories[compartment] & (1U << cargoType)) == 0)
            {
                continue;
            }
            carriesCargo = true;
            const auto nominalCapacity = vehicleObject.maxCargo[compartment];
            const auto capacity = crushLoading ? getCrushLoadCapacity(nominalCapacity) : nominalCapacity;
            totalTicks += calculateCargoTransferTimeout(cargoTransferTime, capacity, 1, capacity - nominalCapacity);
        }
        return carriesCargo ? totalTicks + kCargoTransferStartTimeout + vehicleObject.numCarComponents * 3 + 1 : 0;
    }

    VehiclePurchaseStats calculateVehiclePurchaseStats(const VehicleObject& vehicleObject, const std::optional<uint8_t> cargoType)
    {
        VehiclePurchaseStats stats;
        stats.reliability = calculateInitialReliability(vehicleObject);
        stats.reliabilityLossPerDay = calculateReliabilityLossPerDay(vehicleObject, getCurrentYear());
        if (vehicleObject.mode == TransportMode::rail || vehicleObject.mode == TransportMode::road)
        {
            stats.powerToWeightQ16 = calculatePowerToWeightQ16(vehicleObject.power, vehicleObject.weight);
        }
        stats.cargoCapacity = getCargoCapacity(vehicleObject, cargoType);
        if (stats.cargoCapacity != 0)
        {
            stats.cargoType = cargoType.value_or(static_cast<uint8_t>(Numerics::bitScanForward(vehicleObject.compatibleCargoCategories[0])));
        }
        stats.capacityPerTileQ16 = calculateRatioQ16(static_cast<uint64_t>(stats.cargoCapacity) * 128, vehicleObject.getLength());
        return stats;
    }
}
