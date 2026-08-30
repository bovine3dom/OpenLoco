// SPDX-License-Identifier: MIT
#pragma once

#include "Routing.h"
#include <OpenLoco/Types.hpp>
#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::CargoDist
{
    inline constexpr int64_t kFlowCursorScale = 1024;

    enum class DistributionMode : uint8_t
    {
        manual,
        asymmetric,
    };

    enum class PassengerTripKind : uint8_t
    {
        ordinary,
        holidayOutbound,
        holidayReturn,
    };

    struct CargoPacket
    {
        uint16_t quantity{};
        StationId origin = StationId::null;
        StationId nextHop = StationId::null;
        uint8_t age{};
        ServicePoint departure{};
        ServicePoint arrival{};
        StationId destination = StationId::null;
        int64_t transferCredit{};
        PassengerTripKind tripKind = PassengerTripKind::ordinary;
        IndustryId holidayIndustry = IndustryId::null;
        TownId homeTown = TownId::null;

        auto operator<=>(const CargoPacket&) const = default;

        CargoPacket extract(uint16_t amount);
    };

    class PacketList
    {
    public:
        using Container = std::vector<CargoPacket>;

        bool empty() const { return _packets.empty(); }
        size_t size() const { return _packets.size(); }
        uint32_t quantity() const;
        uint32_t quantityFor(StationId nextHop) const;
        uint32_t quantityFor(StationId nextHop, ServicePoint departure) const;
        StationId representativeOrigin() const;
        uint8_t averageAge() const;
        std::span<const CargoPacket> packets() const { return _packets; }
        static PacketList fromPackets(Container packets);

        template<typename TFunc>
        void transform(TFunc&& func)
        {
            for (auto& packet : _packets)
            {
                func(packet);
            }
            canonicalise();
        }

        void append(CargoPacket packet);
        void append(PacketList packets);
        PacketList take(uint32_t quantity);
        PacketList takeFor(StationId nextHop, uint32_t quantity);
        PacketList takeFor(StationId nextHop, ServicePoint departure, uint32_t quantity);
        PacketList takeForJourney(StationId destination, StationId nextHop, ServicePoint departure, uint32_t quantity);
        uint32_t remove(uint32_t quantity);
        uint32_t removeForRating(uint32_t quantity);
        uint32_t removeExpired();
        void removeStationReferences(StationId station);
        void removeServiceReferences(ServiceId service, bool preserveNextHop = false);
        void ageAtStation(StationId station);
        void ageInVehicle();

    private:
        void canonicalise();
        PacketList takeImpl(uint32_t quantity, std::optional<StationId> nextHop, std::optional<ServicePoint> departure, std::optional<StationId> destination = std::nullopt);

        Container _packets;
    };

    struct CargoRouteSummary
    {
        StationId origin = StationId::null;
        StationId destination = StationId::null;
        StationId nextHop = StationId::null;
        uint64_t quantity{};

        auto operator<=>(const CargoRouteSummary&) const = default;
    };

    enum class CargoRouteField : uint8_t
    {
        origin,
        destination,
        nextHop,
    };

    struct CargoRouteNode
    {
        StationId station = StationId::null;
        uint64_t quantity{};
        std::vector<CargoRouteNode> children;

        auto operator<=>(const CargoRouteNode&) const = default;
    };

    std::vector<CargoRouteSummary> getRouteSummaries(const PacketList& packets);
    std::vector<CargoRouteNode> getRouteTree(std::span<const CargoRouteSummary> summaries, const std::array<CargoRouteField, 3>& order);

    enum class VehicleCargoSlot : uint8_t
    {
        primary,
        secondary,
    };

    struct StationCargoKey
    {
        StationId station = StationId::null;
        uint8_t cargo{};

        auto operator<=>(const StationCargoKey&) const = default;
    };

    struct VehicleCargoKey
    {
        EntityId component = EntityId::null;
        VehicleCargoSlot slot = VehicleCargoSlot::primary;

        auto operator<=>(const VehicleCargoKey&) const = default;
    };

    struct ServiceEdgeKey
    {
        uint8_t cargo{};
        StationId from = StationId::null;
        StationId to = StationId::null;
        ServicePoint departure{};
        ServicePoint arrival{};

        auto operator<=>(const ServiceEdgeKey&) const = default;
    };

    struct ServiceEdgeStats
    {
        uint32_t capacity{};
        uint32_t travelTime{};
        uint32_t waitTime{};
        uint32_t headway{};
        uint32_t fleetCapacity{}; // Display capacity; routing uses capacity per departure above.
    };

    struct CommittedServiceDemand
    {
        uint64_t waiting{};
        uint64_t incoming{};

        uint64_t total() const { return waiting + incoming; }
    };

    struct VehicleServiceLeg
    {
        uint16_t currentOrder{};
        StationId from = StationId::null;
        StationId to = StationId::null;
        ServicePoint departure{};
        ServicePoint arrival{};

        auto operator<=>(const VehicleServiceLeg&) const = default;
    };

    struct PlannedServiceEdge
    {
        StationId from = StationId::null;
        StationId to = StationId::null;
        uint64_t plannedDemand{};
        std::optional<uint32_t> capacity;
        uint64_t servicePlannedDemand{};
        uint64_t committedDemand{};
        uint64_t waitingDemand{};
        uint64_t incomingDemand{};
        std::optional<uint32_t> serviceCapacity;
        ServicePoint serviceDeparture{};
        ServicePoint serviceArrival{};

        auto operator<=>(const PlannedServiceEdge&) const = default;
    };

    struct FlowKey
    {
        uint8_t cargo{};
        StationId station = StationId::null;
        StationId origin = StationId::null;
        ServicePoint incoming{};
        StationId destination = StationId::null;

        auto operator<=>(const FlowKey&) const = default;
    };

    struct FlowOption
    {
        StationId via = StationId::null;
        uint32_t weight{};
        int64_t current{};
        ServicePoint departure{};
        ServicePoint arrival{};
    };

    struct ViaShare
    {
        StationId via = StationId::null;
        uint32_t amount{};
        ServicePoint departure{};
        ServicePoint arrival{};
        StationId destination = StationId::null;
    };

    struct DestinationFlowKey
    {
        uint8_t cargo{};
        StationId station = StationId::null;
        StationId origin = StationId::null;
        ServicePoint incoming{};

        auto operator<=>(const DestinationFlowKey&) const = default;
    };

    struct DestinationOption
    {
        StationId destination = StationId::null;
        uint32_t weight{};
        int64_t current{};

        auto operator<=>(const DestinationOption&) const = default;
    };

    struct Settings
    {
        std::array<DistributionMode, 32> modes{};
        RoutingSettings routing{};
        uint16_t recalculationInterval = 8;
    };

    struct PendingHolidayReturn
    {
        uint32_t releaseDay{};
        uint16_t quantity{};
        StationId resortStation = StationId::null;
        StationId homeStation = StationId::null;
        TownId homeTown = TownId::null;
        IndustryId resort = IndustryId::null;
        uint8_t cargo{};
        uint8_t age{};
        bool released{};
        int64_t transferCredit{};

        auto operator<=>(const PendingHolidayReturn&) const = default;
    };

    struct ResortActivity
    {
        uint32_t guestDays{};
        uint16_t capacity{};
        uint16_t liveSlopes{};
        uint8_t popularity{};

        auto operator<=>(const ResortActivity&) const = default;
    };

    struct HolidaySourceState
    {
        uint8_t remainder{};
        uint32_t sequence{};

        auto operator<=>(const HolidaySourceState&) const = default;
    };

    struct State
    {
        Settings settings{};
        std::map<StationCargoKey, PacketList> stationCargo;
        std::map<VehicleCargoKey, PacketList> vehicleCargo;
        std::map<std::pair<uint8_t, StationId>, uint32_t> supply;
        std::map<StationCargoKey, uint32_t> stationAttraction;
        std::map<ServiceEdgeKey, ServiceEdgeStats> serviceEdges;
        std::map<EntityId, std::vector<VehicleServiceLeg>> vehicleServiceLegs;
        std::map<StationId, uint32_t> stationAccessibility;
        bool hasStationAccessibilitySnapshot{};
        std::map<IndustryId, ResortActivity> resorts;
        std::map<StationCargoKey, HolidaySourceState> holidaySources;
        std::vector<PendingHolidayReturn> pendingHolidayReturns;
        std::map<FlowKey, std::vector<FlowOption>> flows;
        std::map<DestinationFlowKey, std::vector<DestinationOption>> destinationFlows;
        std::map<EntityId, int64_t> pendingVehicleRevenueAdjustments;
        uint64_t routingRevision{};
        uint64_t cargoRevision{};
        uint32_t nextRecalculationDay{};
        bool graphDirty{};
        bool servicesDirty{};
        bool requiresStationMetadataRefresh{};
    };

    State& getState();
    const State& getStateConst();
    void reset();
    uint32_t getStationAccessibility(StationId station);

    DistributionMode getMode(uint8_t cargo);
    bool isEnabled(uint8_t cargo);
    bool hasOutstandingTransferCredits(uint8_t cargo);
    bool canDisableDistribution(uint8_t cargo);
    void setMode(uint8_t cargo, DistributionMode mode);
    void markGraphDirty();
    void markServicesDirty();
    std::vector<PlannedServiceEdge> getPlannedServiceEdges(uint8_t cargo);

    PacketList* getStationCargo(StationId station, uint8_t cargo);
    const PacketList* getStationCargoConst(StationId station, uint8_t cargo);
    PacketList& getOrCreateStationCargo(StationId station, uint8_t cargo);

    PacketList* getVehicleCargo(VehicleCargoKey key);
    const PacketList* getVehicleCargoConst(VehicleCargoKey key);
    PacketList& getOrCreateVehicleCargo(VehicleCargoKey key);
    void eraseVehicleCargo(VehicleCargoKey key);

    void setFlows(uint8_t cargo, std::span<const FlowShare> shares);
    void rebuildDestinationFlows(uint8_t cargo);
    void buildFlowMaps(std::map<FlowKey, std::vector<FlowOption>>& flows, std::map<DestinationFlowKey, std::vector<DestinationOption>>& destinationFlows, uint8_t cargo, std::span<const FlowShare> shares);
    std::vector<ViaShare> allocateFixedVia(std::map<FlowKey, std::vector<FlowOption>>& flows, uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming = {}, StationId excluded = StationId::null, StationId excluded2 = StationId::null);
    std::vector<ViaShare> previewFixedVia(const std::map<FlowKey, std::vector<FlowOption>>& flows, uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming = {}, StationId excluded = StationId::null, StationId excluded2 = StationId::null);
    std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming = {}, StationId excluded = StationId::null, StationId excluded2 = StationId::null);
    std::vector<ViaShare> previewVia(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, ServicePoint incoming = {}, StationId excluded = StationId::null, StationId excluded2 = StationId::null);

    inline std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, uint32_t quantity, ServicePoint incoming = {}, StationId excluded = StationId::null, StationId excluded2 = StationId::null)
    {
        return allocateVia(cargo, station, origin, StationId::null, quantity, incoming, excluded, excluded2);
    }

    inline std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, StationId destination, uint32_t quantity, StationId excluded, StationId excluded2 = StationId::null)
    {
        return allocateVia(cargo, station, origin, destination, quantity, {}, excluded, excluded2);
    }
}
