// SPDX-License-Identifier: MIT
#pragma once

#include "Routing.h"
#include <OpenLoco/Types.hpp>
#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

namespace OpenLoco::CargoDist
{
    enum class DistributionMode : uint8_t
    {
        manual,
        asymmetric,
    };

    struct CargoPacket
    {
        uint16_t quantity{};
        StationId origin = StationId::null;
        StationId nextHop = StationId::null;
        uint8_t age{};

        auto operator<=>(const CargoPacket&) const = default;
    };

    class PacketList
    {
    public:
        using Container = std::vector<CargoPacket>;

        bool empty() const { return _packets.empty(); }
        size_t size() const { return _packets.size(); }
        uint32_t quantity() const;
        uint32_t quantityFor(StationId nextHop) const;
        StationId representativeOrigin() const;
        uint8_t averageAge() const;
        std::span<const CargoPacket> packets() const { return _packets; }
        static PacketList fromPackets(Container packets);

        void append(CargoPacket packet);
        void append(PacketList packets);
        PacketList take(uint32_t quantity);
        PacketList takeFor(StationId nextHop, uint32_t quantity);
        uint32_t remove(uint32_t quantity);
        void removeStationReferences(StationId station);
        void ageAtStation(StationId station);
        void ageInVehicle();

    private:
        void canonicalise();
        PacketList takeImpl(uint32_t quantity, StationId nextHop, bool filterByNextHop);

        Container _packets;
    };

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

        auto operator<=>(const ServiceEdgeKey&) const = default;
    };

    struct ServiceEdgeStats
    {
        uint32_t capacity{};
        uint32_t travelTime{};
    };

    struct FlowKey
    {
        uint8_t cargo{};
        StationId station = StationId::null;
        StationId origin = StationId::null;

        auto operator<=>(const FlowKey&) const = default;
    };

    struct FlowOption
    {
        StationId via = StationId::null;
        uint32_t weight{};
        int64_t current{};
    };

    struct ViaShare
    {
        StationId via = StationId::null;
        uint32_t amount{};
    };

    struct Settings
    {
        std::array<DistributionMode, 32> modes{};
        RoutingSettings routing{};
        uint16_t recalculationInterval = 8;
    };

    struct State
    {
        Settings settings{};
        std::map<StationCargoKey, PacketList> stationCargo;
        std::map<VehicleCargoKey, PacketList> vehicleCargo;
        std::map<std::pair<uint8_t, StationId>, uint32_t> supply;
        std::map<ServiceEdgeKey, ServiceEdgeStats> serviceEdges;
        std::map<FlowKey, std::vector<FlowOption>> flows;
        uint32_t nextRecalculationDay{};
        bool graphDirty{};
    };

    State& getState();
    const State& getStateConst();
    void reset();

    DistributionMode getMode(uint8_t cargo);
    bool isEnabled(uint8_t cargo);
    void setMode(uint8_t cargo, DistributionMode mode);
    void markGraphDirty();

    PacketList* getStationCargo(StationId station, uint8_t cargo);
    const PacketList* getStationCargoConst(StationId station, uint8_t cargo);
    PacketList& getOrCreateStationCargo(StationId station, uint8_t cargo);

    PacketList* getVehicleCargo(VehicleCargoKey key);
    const PacketList* getVehicleCargoConst(VehicleCargoKey key);
    PacketList& getOrCreateVehicleCargo(VehicleCargoKey key);
    void eraseVehicleCargo(VehicleCargoKey key);

    void setFlows(uint8_t cargo, std::span<const FlowShare> shares);
    std::vector<ViaShare> allocateVia(uint8_t cargo, StationId station, StationId origin, uint32_t quantity, StationId excluded = StationId::null, StationId excluded2 = StationId::null);
    StationId chooseVia(uint8_t cargo, StationId station, StationId origin, StationId excluded = StationId::null, StationId excluded2 = StationId::null);
}
