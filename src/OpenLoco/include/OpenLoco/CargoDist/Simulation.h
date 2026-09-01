// SPDX-License-Identifier: MIT
#pragma once

#include "CargoDist.h"
#include "Economy/Currency.h"
#include <OpenLoco/World/Town.h>
#include <functional>

namespace OpenLoco
{
    struct GameState;
    struct StationCargoStats;

    namespace Vehicles
    {
        struct VehicleCargo;
        struct VehicleHead;
    }
}

namespace OpenLoco::CargoDist
{
    struct TransferCredit
    {
        CargoPacket packet;
        currency32_t amount{};
    };

    using TransferPaymentCalculator = std::function<currency32_t(const CargoPacket&)>;

    struct UnloadResult
    {
        PacketList delivered;
        std::vector<TransferCredit> transferCredits;
        uint16_t transferred{};

        uint32_t quantity() const { return delivered.quantity() + transferred; }
    };

    struct RecalculationMetrics
    {
        uint64_t preparationNanoseconds{};
        uint64_t graphNanoseconds{};
        uint64_t solveNanoseconds{};
        uint64_t waitNanoseconds{};
        uint64_t commitNanoseconds{};
        uint32_t calculations{};
    };

    void synchroniseStationCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo);
    void synchroniseVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo);
    constexpr uint32_t getRoutingAttraction(bool passengerRouting, bool industryDestination, uint32_t recordedAttraction)
    {
        return recordedAttraction != 0 && (passengerRouting || !industryDestination) ? recordedAttraction : 8;
    }
    constexpr uint32_t getPassengerIndustryBonus(const uint32_t previousMonthVisitors)
    {
        const auto patronageBonus = previousMonthVisitors / 4;
        return 8 + (patronageBonus < 40 ? patronageBonus : 40);
    }
    constexpr uint32_t getSharedPassengerIndustryBonus(const uint32_t bonus, const uint32_t stationCount, const uint32_t stationIndex)
    {
        return stationCount == 0 ? 0 : bonus / stationCount + (stationIndex < bonus % stationCount);
    }
    constexpr uint32_t getPassengerIndustryAttraction(const uint32_t recordedAttraction, const uint32_t resortBonus)
    {
        const auto townAttraction = recordedAttraction > 8 ? recordedAttraction - 8 : 0;
        return townAttraction + resortBonus;
    }
    constexpr bool isPassengerIndustrySink(const bool producesPassengers, const bool hasOutboundSupply)
    {
        return !producesPassengers || !hasOutboundSupply;
    }
    constexpr uint8_t getHolidayPercentage(const TownSize size)
    {
        constexpr std::array<uint8_t, 5> kPercentages = { 1, 1, 3, 6, 8 };
        const auto index = enumValue(size);
        return kPercentages[index < kPercentages.size() ? index : 0];
    }
    constexpr uint8_t updateResortPopularity(const uint8_t popularity, const uint8_t slopeScore, const uint8_t occupancyScore)
    {
        const auto target = (slopeScore + occupancyScore * 2) / 3;
        return static_cast<uint8_t>((popularity + target) / 2);
    }
    constexpr uint16_t getResortCapacity(const uint16_t liveSlopes, const uint8_t popularity)
    {
        const auto capacity = static_cast<uint32_t>(liveSlopes) * 4 * (200 + popularity) / 200;
        return static_cast<uint16_t>(std::min<uint32_t>(std::numeric_limits<uint16_t>::max(), capacity));
    }
    void setStationAttraction(StationId station, uint8_t cargo, uint32_t attraction);

    void addProducedCargo(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantity, bool generateHolidays = true);
    bool isHolidayResort(IndustryId industry, uint8_t cargo);
    void scheduleHolidayReturn(uint8_t cargo, StationId resortStation, const CargoPacket& packet);
    void updateResortsMonthly();
    void updateStationCargoDaily(StationId station, uint8_t cargo, StationCargoStats& nativeCargo, uint16_t quantityBeforeUpdate);
    void updateVehicleCargoDaily(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo);

    uint32_t getLoadableQuantity(StationId station, uint8_t cargo, const VehicleServiceLeg& serviceLeg);
    std::map<ServiceEdgeKey, CommittedServiceDemand> getCommittedServiceDemands(uint8_t cargo);
    uint16_t loadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, const VehicleServiceLeg& serviceLeg);
    UnloadResult unloadVehicleCargo(VehicleCargoKey key, Vehicles::VehicleCargo& nativeCargo, StationId station, StationCargoStats& nativeStationCargo, std::span<const StationId> remainingStops, bool forceUnload, std::optional<VehicleServiceLeg> onwardLeg, TransferPaymentCalculator transferPayment = {});
    currency32_t accrueTransferCredit(CargoPacket& packet, currency32_t projectedPayment);
    int64_t calculateFinalDeliveryIncome(int64_t transferCredit, currency32_t grossPayment);
    void addVehicleRevenueAdjustment(EntityId vehicle, int64_t adjustment);
    std::optional<int64_t> consumeVehicleRevenueAdjustment(EntityId vehicle);

    std::optional<VehicleServiceLeg> getCurrentServiceLeg(const Vehicles::VehicleHead& head);
    void recordVehicleDeparture(const Vehicles::VehicleHead& head, StationId from, StationId to);
    StationId getNextStop(const Vehicles::VehicleHead& head);
    void update();
    bool isServiceRecalculationPending();
    void notifyRecalculationDirty();
    void notifyGraphDirty();
    void cancelPendingRecalculation();
    RecalculationMetrics getRecalculationMetrics();
    void recalculateNow();
    void validateState(const State& state, const GameState& gameState, bool validatePassengerCargoTypes = true);
    void restoreState(State state);
    void updateDaily();

    void removeStation(StationId station);
    void removeIndustry(IndustryId industry);
    void removeTown(TownId town);
    void removeVehicleService(EntityId vehicle);
    void eraseVehicleCargoForComponent(EntityId component);
    void moveVehicleCargo(VehicleCargoKey source, VehicleCargoKey destination);
}
