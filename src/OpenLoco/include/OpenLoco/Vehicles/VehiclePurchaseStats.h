#pragma once

#include <cstdint>
#include <optional>

namespace OpenLoco
{
    struct VehicleObject;
}

namespace OpenLoco::Vehicles
{
    constexpr uint32_t kPurchaseStatFractionalScale = 1U << 16;

    struct VehiclePurchaseStats
    {
        uint16_t reliability{};
        uint16_t reliabilityLossPerDay{};
        uint32_t powerToWeightQ16{};
        uint16_t cargoCapacity{};
        uint8_t cargoType = 0xFF;
        uint32_t capacityPerTileQ16{};
    };

    uint32_t calculatePowerToWeightQ16(uint32_t power, uint32_t weight);
    uint32_t convertHpToKwQ16(uint32_t valueQ16);
    uint32_t calculateFullLoadTimeTicks(const VehicleObject& vehicleObject, uint8_t cargoType, uint16_t cargoTransferTime, bool crushLoading = false);
    VehiclePurchaseStats calculateVehiclePurchaseStats(const VehicleObject& vehicleObject, std::optional<uint8_t> cargoType = std::nullopt);

    constexpr uint32_t purchaseStatToTenths(const uint32_t valueQ16)
    {
        return static_cast<uint32_t>((static_cast<uint64_t>(valueQ16) * 10 + kPurchaseStatFractionalScale / 2) / kPurchaseStatFractionalScale);
    }

    constexpr uint32_t reliabilityLossPerYearTenths(const uint16_t lossPerDay)
    {
        return (static_cast<uint32_t>(lossPerDay) * 365 * 10 + 128) / 256;
    }
}
