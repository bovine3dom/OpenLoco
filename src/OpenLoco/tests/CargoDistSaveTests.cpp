// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Save.h>

#include "GameState.h"
#include "S5/S5.h"
#include "S5/S5File.h"
#include "S5/S5Options.h"
#include "S5/SawyerStream.h"
#include "Vehicles/Vehicle.h"
#include <OpenLoco/CargoDist/Simulation.h>
#include <OpenLoco/Core/MemoryStream.h>
#include <OpenLoco/GameRules.h>
#include <OpenLoco/S5/SaveExtension.h>
#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

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

    template<typename T>
    void appendValue(std::vector<std::byte>& data, T value)
    {
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            data.push_back(static_cast<std::byte>((static_cast<uint64_t>(value) >> (i * 8)) & 0xFF));
        }
    }

    std::vector<std::byte> legacyEncodedState(uint16_t version)
    {
        std::vector<std::byte> data;
        for (const auto value : std::array{ 'O', 'L', 'C', 'D', 'I', 'S', 'T', '\0' })
        {
            appendValue<uint8_t>(data, value);
        }
        appendValue<uint16_t>(data, version);
        appendValue<uint16_t>(data, 16);
        appendValue<uint32_t>(data, 0);

        for (uint8_t cargo = 0; cargo < 32; ++cargo)
        {
            appendValue<uint8_t>(data, cargo == 0 ? 1 : 0);
        }
        appendValue<uint8_t>(data, 100);
        appendValue<uint8_t>(data, 80);
        appendValue<uint8_t>(data, 16);
        appendValue<uint8_t>(data, 0);
        appendValue<uint16_t>(data, 8);
        appendValue<uint16_t>(data, 0);
        appendValue<uint32_t>(data, 42);
        appendValue<uint32_t>(data, 0);

        appendValue<uint32_t>(data, 1);
        appendValue<uint16_t>(data, 2);
        appendValue<uint8_t>(data, 0);
        appendValue<uint8_t>(data, 0);
        appendValue<uint32_t>(data, 1);
        appendValue<uint16_t>(data, 10);
        appendValue<uint16_t>(data, 1);
        appendValue<uint16_t>(data, 3);
        appendValue<uint8_t>(data, 4);
        appendValue<uint8_t>(data, 0);
        if (version >= 3)
        {
            appendValue<uint16_t>(data, 7);
            appendValue<uint16_t>(data, 1);
            appendValue<uint16_t>(data, 7);
            appendValue<uint16_t>(data, 2);
        }
        if (version >= 4)
        {
            appendValue<uint16_t>(data, 4);
        }
        if (version >= 5)
        {
            appendValue<int64_t>(data, 0);
        }

        appendValue<uint32_t>(data, 0);
        appendValue<uint32_t>(data, 0);
        appendValue<uint32_t>(data, 1);
        appendValue<uint8_t>(data, 0);
        appendValue<uint8_t>(data, 0);
        appendValue<uint16_t>(data, 2);
        appendValue<uint16_t>(data, 1);
        if (version >= 3)
        {
            appendValue<uint16_t>(data, static_cast<uint16_t>(ServiceId::null));
            appendValue<uint16_t>(data, kNoServiceOccurrence);
        }
        if (version >= 4)
        {
            appendValue<uint16_t>(data, 3);
        }
        appendValue<uint16_t>(data, 1);
        appendValue<uint16_t>(data, 3);
        appendValue<uint16_t>(data, 0);
        appendValue<uint32_t>(data, 10);
        appendValue<uint64_t>(data, 0);
        if (version >= 3)
        {
            appendValue<uint16_t>(data, 8);
            appendValue<uint16_t>(data, 1);
            appendValue<uint16_t>(data, 8);
            appendValue<uint16_t>(data, 2);
        }

        if (version >= 2)
        {
            appendValue<uint32_t>(data, 1);
            appendValue<uint16_t>(data, 2);
            appendValue<uint8_t>(data, 0);
            appendValue<uint8_t>(data, 0);
            appendValue<uint32_t>(data, 25);
        }
        if (version >= 4)
        {
            appendValue<uint32_t>(data, 0);
        }
        if (version >= 5)
        {
            appendValue<uint32_t>(data, 0);
        }

        const auto payloadSize = static_cast<uint32_t>(data.size() - 16);
        for (size_t i = 0; i < sizeof(payloadSize); ++i)
        {
            data[12 + i] = static_cast<std::byte>((payloadSize >> (i * 8)) & 0xFF);
        }
        return data;
    }

    void expectPacketListsEqual(const PacketList& lhs, const PacketList& rhs)
    {
        EXPECT_TRUE(std::ranges::equal(lhs.packets(), rhs.packets()));
    }

    void expectStatesEqual(const State& lhs, const State& rhs)
    {
        EXPECT_EQ(lhs.settings.modes, rhs.settings.modes);
        EXPECT_EQ(lhs.settings.routing.distanceEffect, rhs.settings.routing.distanceEffect);
        EXPECT_EQ(lhs.settings.routing.saturation, rhs.settings.routing.saturation);
        EXPECT_EQ(lhs.settings.routing.accuracy, rhs.settings.routing.accuracy);
        EXPECT_EQ(lhs.settings.recalculationInterval, rhs.settings.recalculationInterval);
        EXPECT_EQ(lhs.nextRecalculationDay, rhs.nextRecalculationDay);
        EXPECT_EQ(lhs.graphDirty, rhs.graphDirty);
        EXPECT_EQ(lhs.supply, rhs.supply);
        ASSERT_EQ(lhs.stationAttraction.size(), rhs.stationAttraction.size());
        for (const auto& [key, attraction] : lhs.stationAttraction)
        {
            EXPECT_EQ(attraction, rhs.stationAttraction.at(key));
        }
        ASSERT_EQ(lhs.stationCargo.size(), rhs.stationCargo.size());
        for (const auto& [key, packets] : lhs.stationCargo)
        {
            expectPacketListsEqual(packets, rhs.stationCargo.at(key));
        }
        ASSERT_EQ(lhs.vehicleCargo.size(), rhs.vehicleCargo.size());
        for (const auto& [key, packets] : lhs.vehicleCargo)
        {
            expectPacketListsEqual(packets, rhs.vehicleCargo.at(key));
        }
        ASSERT_EQ(lhs.flows.size(), rhs.flows.size());
        for (const auto& [key, options] : lhs.flows)
        {
            const auto& other = rhs.flows.at(key);
            ASSERT_EQ(options.size(), other.size());
            for (size_t i = 0; i < options.size(); ++i)
            {
                EXPECT_EQ(options[i].via, other[i].via);
                EXPECT_EQ(options[i].weight, other[i].weight);
                EXPECT_EQ(options[i].current, other[i].current);
                EXPECT_EQ(options[i].departure, other[i].departure);
                EXPECT_EQ(options[i].arrival, other[i].arrival);
            }
        }
        EXPECT_EQ(lhs.destinationFlows, rhs.destinationFlows);
        EXPECT_EQ(lhs.pendingVehicleRevenueAdjustments, rhs.pendingVehicleRevenueAdjustments);
    }

    State populatedState()
    {
        State state;
        state.settings.modes[0] = DistributionMode::asymmetric;
        state.settings.modes[3] = DistributionMode::asymmetric;
        state.settings.routing = { 75, 60, 12 };
        state.settings.recalculationInterval = 5;
        state.nextRecalculationDay = 1234;
        state.graphDirty = true;
        state.stationCargo[{ station(2), 0 }].append({ 30, station(1), station(3), 4, servicePoint(7, 1), servicePoint(7, 2), station(3), 120 });
        state.stationCargo[{ station(2), 0 }].append({ 10, station(4), StationId::null, 2, {}, {}, station(3) });
        state.vehicleCargo[{ entity(20), VehicleCargoSlot::primary }].append({ 25, station(1), station(2), 6, servicePoint(8, 3), servicePoint(8, 4), station(3), 80 });
        state.supply[{ 0, station(1) }] = 120;
        state.flows[{ 0, station(2), station(1), servicePoint(6, 4), station(3) }] = {
            { station(3), 75, -25, servicePoint(9, 1), servicePoint(9, 2) },
            { station(3), 25, 25, servicePoint(10, 1), servicePoint(10, 2) },
        };
        state.destinationFlows[{ 0, station(2), station(1), servicePoint(6, 4) }] = { { station(3), 100, 0 } };
        state.stationAttraction[{ station(2), 0 }] = 120;
        state.serviceEdges[{ 0, station(1), station(2), servicePoint(8, 3), servicePoint(8, 4) }] = { 40, 10, 2 };
        state.vehicleServiceLegs[entity(7)] = { { 3, station(1), station(2), servicePoint(8, 3), servicePoint(8, 4) } };
        state.pendingVehicleRevenueAdjustments[entity(7)] = -45;
        return state;
    }

    void writeMinimalSave(
        MemoryStream& stream,
        const State* cargoDistState,
        bool legacyExtension = false,
        const Vehicles::SharedOrderManager::State* sharedOrderState = nullptr,
        const Vehicles::RoutingManager::State* pathReservationState = nullptr,
        const Vehicles::VehicleAutoRenewal::State* vehicleAutoRenewalState = nullptr,
        bool legacyPathReservations = false,
        const Vehicles::RailTraffic::State* railTrafficState = nullptr,
        const GameRules::State* gameRulesState = nullptr,
        const S5::SaveExtension::VehicleObjectState* vehicleObjectState = nullptr)
    {
        SawyerStreamWriter writer(stream);
        S5::Header header{};
        header.type = S5::S5Type::savedGame;
        writer.writeChunk(SawyerEncoding::uncompressed, header);

        std::array<ObjectHeader, S5::Limits::kMaxObjectHeaders> requiredObjects{};
        writer.writeChunk(SawyerEncoding::uncompressed, requiredObjects.data(), sizeof(requiredObjects));

        auto gameState = std::make_unique<S5::GameState>();
        gameState->general.fixFlags = enumValue(S5::S5FixFlags::fixFlag1);
        writer.writeChunk(SawyerEncoding::uncompressed, *gameState);

        std::array<std::byte, sizeof(S5::TileElement)> tile{};
        writer.writeChunk(SawyerEncoding::uncompressed, tile.data(), tile.size());
        if (cargoDistState != nullptr || sharedOrderState != nullptr || pathReservationState != nullptr
            || vehicleAutoRenewalState != nullptr || railTrafficState != nullptr || gameRulesState != nullptr || vehicleObjectState != nullptr)
        {
            auto encoded = legacyExtension
                ? encodeState(*cargoDistState)
                : S5::SaveExtension::encode({
                      .cargoDistState = cargoDistState,
                      .sharedOrderState = sharedOrderState,
                      .pathReservationState = pathReservationState,
                      .vehicleAutoRenewalState = vehicleAutoRenewalState,
                      .railTrafficState = railTrafficState,
                      .gameRulesState = gameRulesState,
                      .vehicleObjectState = vehicleObjectState,
                  });
            if (legacyPathReservations)
            {
                constexpr std::array pathReservationsTag{ std::byte{ 'P' }, std::byte{ 'R' }, std::byte{ 'E' }, std::byte{ 'S' } };
                const auto section = std::search(encoded.begin(), encoded.end(), pathReservationsTag.begin(), pathReservationsTag.end());
                ASSERT_NE(section, encoded.end());
                section[4] = std::byte{ 1 };
                section[5] = std::byte{ 0 };
            }
            writer.writeChunk(SawyerEncoding::uncompressed, encoded.data(), encoded.size());
        }
        writer.writeChecksum();
        stream.setPosition(0);
    }

    void writeMinimalScenarioOrLandscape(
        MemoryStream& stream,
        S5::S5Type type,
        const GameRules::State* gameRulesState = nullptr,
        const S5::SaveExtension::VehicleObjectState* vehicleObjectState = nullptr)
    {
        SawyerStreamWriter writer(stream);
        S5::Header header{};
        header.type = type;
        if (type == S5::S5Type::landscape)
        {
            header.flags = S5::HeaderFlags::hasSaveDetails;
        }
        writer.writeChunk(SawyerEncoding::uncompressed, header);

        if (type == S5::S5Type::scenario)
        {
            S5::Options options{};
            writer.writeChunk(SawyerEncoding::uncompressed, options);
        }
        else
        {
            S5::SaveDetails details{};
            writer.writeChunk(SawyerEncoding::uncompressed, details);
        }
        std::array<ObjectHeader, S5::Limits::kMaxObjectHeaders> requiredObjects{};
        writer.writeChunk(SawyerEncoding::uncompressed, requiredObjects.data(), sizeof(requiredObjects));

        auto gameState = std::make_unique<S5::GameState>();
        gameState->general.fixFlags = enumValue(S5::S5FixFlags::fixFlag1);
        if (type == S5::S5Type::scenario)
        {
            writer.writeChunk(SawyerEncoding::uncompressed, &gameState->general, sizeof(S5::GeneralState));
            writer.writeChunk(SawyerEncoding::uncompressed, gameState->towns, 0x123480);
            writer.writeChunk(SawyerEncoding::uncompressed, gameState->animations, 0x79D80);
        }
        else
        {
            writer.writeChunk(SawyerEncoding::uncompressed, *gameState);
            std::array<std::byte, sizeof(S5::TileElement)> tile{};
            writer.writeChunk(SawyerEncoding::uncompressed, tile.data(), tile.size());
        }

        if (gameRulesState != nullptr || vehicleObjectState != nullptr)
        {
            const auto extension = S5::SaveExtension::encode({
                .gameRulesState = gameRulesState,
                .vehicleObjectState = vehicleObjectState,
            });
            writer.writeChunk(SawyerEncoding::uncompressed, extension.data(), extension.size());
        }
        writer.writeChecksum();
        stream.setPosition(0);
    }
}

TEST(CargoDistSave, RoundTripsCanonicalState)
{
    const auto original = populatedState();

    const auto encoded = encodeState(original);
    const auto decoded = decodeState(encoded);

    expectStatesEqual(original, decoded);
    EXPECT_TRUE(decoded.serviceEdges.empty());
    EXPECT_TRUE(decoded.vehicleServiceLegs.empty());
    EXPECT_EQ(std::to_integer<uint8_t>(encoded[8]), 6);
}

TEST(CargoDistSave, MigratesVersionOneWithoutStationAttraction)
{
    const auto decoded = decodeState(legacyEncodedState(1));

    EXPECT_TRUE(decoded.stationAttraction.empty());
    EXPECT_TRUE(decoded.flows.empty());
    EXPECT_TRUE(decoded.graphDirty);
    const auto& packet = decoded.stationCargo.at({ station(2), 0 }).packets().front();
    EXPECT_EQ(packet.nextHop, station(3));
    EXPECT_TRUE(packet.departure.empty());
    EXPECT_TRUE(packet.arrival.empty());
}

TEST(CargoDistSave, MigratesVersionTwoAndRetainsStationAttraction)
{
    const auto decoded = decodeState(legacyEncodedState(2));

    EXPECT_EQ(decoded.stationAttraction.at({ station(2), 0 }), 25U);
    EXPECT_TRUE(decoded.flows.empty());
    EXPECT_TRUE(decoded.graphDirty);
    const auto& packet = decoded.stationCargo.at({ station(2), 0 }).packets().front();
    EXPECT_TRUE(packet.departure.empty());
    EXPECT_TRUE(packet.arrival.empty());
}

TEST(CargoDistSave, MigratesVersionThreeAndReassignsDestinations)
{
    const auto decoded = decodeState(legacyEncodedState(3));

    EXPECT_TRUE(decoded.flows.empty());
    EXPECT_TRUE(decoded.destinationFlows.empty());
    EXPECT_TRUE(decoded.graphDirty);
    const auto& packet = decoded.stationCargo.at({ station(2), 0 }).packets().front();
    EXPECT_EQ(packet.destination, StationId::null);
    EXPECT_EQ(packet.departure, servicePoint(7, 1));
    EXPECT_EQ(packet.arrival, servicePoint(7, 2));
}

TEST(CargoDistSave, MigratesVersionFourWithoutTransferCredits)
{
    const auto decoded = decodeState(legacyEncodedState(4));

    const auto& packet = decoded.stationCargo.at({ station(2), 0 }).packets().front();
    EXPECT_EQ(packet.destination, station(4));
    EXPECT_EQ(packet.transferCredit, 0);
    EXPECT_TRUE(decoded.pendingVehicleRevenueAdjustments.empty());
}

TEST(CargoDistSave, MigratesVersionFiveAndForcesRecalculation)
{
    const auto decoded = decodeState(legacyEncodedState(5));

    EXPECT_TRUE(decoded.graphDirty);
    EXPECT_EQ(decoded.stationCargo.at({ station(2), 0 }).packets().front().destination, station(4));
}

TEST(CargoDistSave, RoundTripsStationCargoAboveNativeLimit)
{
    State state;
    auto& packets = state.stationCargo[{ station(2), 0 }];
    packets.append({ std::numeric_limits<uint16_t>::max(), station(1), station(3), 4 });
    packets.append({ 20, station(1), station(3), 5 });

    const auto decoded = decodeState(encodeState(state));

    EXPECT_EQ(decoded.stationCargo.at({ station(2), 0 }).quantity(), std::numeric_limits<uint16_t>::max() + 20U);
}

TEST(CargoDistSave, EncodingIsDeterministic)
{
    const auto state = populatedState();
    auto reordered = populatedState();
    auto& packets = reordered.stationCargo.at({ station(2), 0 });
    packets = {};
    packets.append({ 10, station(4), StationId::null, 2, {}, {}, station(3) });
    packets.append({ 30, station(1), station(3), 4, servicePoint(7, 1), servicePoint(7, 2), station(3), 120 });

    EXPECT_EQ(encodeState(state), encodeState(reordered));
}

TEST(CargoDistSave, RoundTripsEmptyManualState)
{
    const State state;

    const auto decoded = decodeState(encodeState(state));

    expectStatesEqual(state, decoded);
}

TEST(CargoDistSave, ValidatesNativeStateBeforeRestore)
{
    State state;
    state.settings.modes[0] = DistributionMode::asymmetric;
    state.stationCargo[{ station(1), 0 }].append({ 10, station(1), StationId::null, 2 });
    auto gameState = std::make_unique<GameState>();
    auto& stationCargo = gameState->stations[1].cargoStats[0];
    gameState->stations[1].name = StringId(1);
    stationCargo.quantity = 10;
    stationCargo.origin = station(1);
    stationCargo.enrouteAge = 2;

    EXPECT_NO_THROW(validateState(state, *gameState));

    stationCargo.quantity = 9;
    EXPECT_THROW(validateState(state, *gameState), std::runtime_error);
}

TEST(CargoDistSave, ValidatesExtendedStationCargoAgainstClampedNativeState)
{
    State state;
    state.settings.modes[0] = DistributionMode::asymmetric;
    auto& packets = state.stationCargo[{ station(1), 0 }];
    packets.append({ std::numeric_limits<uint16_t>::max(), station(1), StationId::null, 2 });
    packets.append({ 10, station(1), StationId::null, 3 });
    auto gameState = std::make_unique<GameState>();
    auto& stationCargo = gameState->stations[1].cargoStats[0];
    gameState->stations[1].name = StringId(1);
    stationCargo.quantity = std::numeric_limits<uint16_t>::max();
    stationCargo.origin = station(1);
    stationCargo.enrouteAge = 2;

    EXPECT_NO_THROW(validateState(state, *gameState));

    stationCargo.quantity--;
    EXPECT_THROW(validateState(state, *gameState), std::runtime_error);
}

TEST(CargoDistSave, ValidatesPendingRevenueVehicle)
{
    State state;
    state.pendingVehicleRevenueAdjustments[entity(7)] = 10;
    auto gameState = std::make_unique<GameState>();

    EXPECT_THROW(validateState(state, *gameState), std::runtime_error);

    auto* head = reinterpret_cast<Vehicles::VehicleBase*>(&gameState->entities[7]);
    head->baseType = EntityBaseType::vehicle;
    head->setSubType(Vehicles::VehicleEntityType::head);
    head->id = entity(7);
    EXPECT_NO_THROW(validateState(state, *gameState));
}

TEST(CargoDistSave, RejectsTruncatedData)
{
    auto encoded = encodeState(populatedState());
    encoded.pop_back();

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsPacketCountLargerThanPayload)
{
    auto encoded = encodeState(populatedState());
    std::fill(encoded.begin() + 72, encoded.begin() + 76, std::byte{ 0xFF });

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsUnknownVersion)
{
    auto encoded = encodeState(populatedState());
    encoded[8] = std::byte{ 7 };

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsInvalidMode)
{
    auto encoded = encodeState(populatedState());
    encoded[16] = std::byte{ 0xFF };

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsNegativeTransferCredit)
{
    State state;
    state.stationCargo[{ station(2), 0 }].append({ 10, station(1), station(3), 4, {}, {}, station(3), -1 });

    EXPECT_THROW(encodeState(state), std::runtime_error);
}

TEST(CargoDistSave, RejectsInvalidFlowCursor)
{
    State state;
    state.flows[{ 0, station(2), station(1), servicePoint(6, 4), station(3) }] = {
        { station(3), 1, 0, servicePoint(9, 1), servicePoint(9, 2) },
    };
    auto encoded = encodeState(state);
    std::fill(encoded.begin() + 100, encoded.begin() + 108, std::byte{ 0x7F });

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, FilteredAllocationKeepsCursorsSerializable)
{
    State state;
    state.settings.modes[0] = DistributionMode::asymmetric;
    auto& options = state.flows[{ 0, station(1), station(1), {}, station(6) }];
    options = {
        { station(2), 1, -4, servicePoint(2, 0), servicePoint(2, 1) },
        { station(3), 1, 4, servicePoint(3, 0), servicePoint(3, 1) },
        { station(4), 1, 4, servicePoint(4, 0), servicePoint(4, 1) },
        { station(5), 1, -4, servicePoint(5, 0), servicePoint(5, 1) },
    };
    state.destinationFlows[{ 0, station(1), station(1) }] = { { station(6), 4, 0 } };
    getState() = state;

    allocateVia(0, station(1), station(1), station(6), 1, ServicePoint{}, station(5));

    EXPECT_NO_THROW(encodeState(getStateConst()));
    EXPECT_TRUE(std::any_of(options.begin(), options.end(), [](const auto& option) { return option.current != 0; }));
    reset();
}

TEST(CargoDistSave, RejectsInvalidServicePointEncoding)
{
    State state;
    state.stationCargo[{ station(2), 0 }].append({ 10, station(1), station(3), 4, servicePoint(7, 1), servicePoint(7, 2), station(3) });
    auto encoded = encodeState(state);
    encoded[86] = std::byte{ 0xFF };
    encoded[87] = std::byte{ 0xFF };

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsMismatchedFlowServices)
{
    State state;
    state.flows[{ 0, station(2), station(1), {}, station(3) }] = {
        { station(3), 1, 0, servicePoint(7, 1), servicePoint(8, 2) },
    };

    EXPECT_THROW(encodeState(state), std::runtime_error);
}

TEST(CargoDistSave, RoundTripsMoreServiceOptionsThanStations)
{
    State state;
    state.settings.modes[0] = DistributionMode::asymmetric;
    auto& options = state.flows[{ 0, station(2), station(1), {}, station(3) }];
    for (uint16_t service = 0; service < 1025; ++service)
    {
        options.push_back({ station(3), 1, 0, servicePoint(service, 0), servicePoint(service, 1) });
    }

    const auto decoded = decodeState(encodeState(state));

    ASSERT_EQ(decoded.flows.size(), 1);
    EXPECT_EQ(decoded.flows.begin()->second.size(), 1025);
}

TEST(CargoDistSave, RejectsOutOfRangeServiceReference)
{
    State state;
    state.settings.modes[0] = DistributionMode::asymmetric;
    state.stationCargo[{ station(2), 0 }].append({ 10, station(1), station(3), 4, servicePoint(20000, 0), servicePoint(20000, 1), station(3) });

    EXPECT_THROW(encodeState(state), std::runtime_error);
}

TEST(CargoDistSave, S5TailRoundTripsExtension)
{
    const auto original = populatedState();
    Vehicles::SharedOrderManager::State sharedOrders;
    sharedOrders.groups = { { { entity(3), entity(8) } } };
    Vehicles::RoutingManager::State pathReservations;
    pathReservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    pathReservations.continuations[7] = { 0 };
    Vehicles::VehicleAutoRenewal::State vehicleAutoRenewal;
    vehicleAutoRenewal.companies[3] = { true, 40 };
    Vehicles::RailTraffic::State railTraffic;
    railTraffic.history.push_back({ { 320, 352, 32, 0, 0 }, 12 * Vehicles::RailTraffic::kOneTick, 0, 7 });
    const GameRules::State gameRules{ .vehiclesNeverExpire = true };
    MemoryStream stream;
    writeMinimalSave(stream, &original, false, &sharedOrders, &pathReservations, &vehicleAutoRenewal, false, &railTraffic, &gameRules);

    const auto save = S5::loadSave(stream);

    ASSERT_TRUE(save->cargoDistState.has_value());
    expectStatesEqual(original, *save->cargoDistState);
    ASSERT_TRUE(save->sharedOrderState.has_value());
    EXPECT_EQ(*save->sharedOrderState, sharedOrders);
    ASSERT_TRUE(save->pathReservationState.has_value());
    EXPECT_EQ(*save->pathReservationState, pathReservations);
    EXPECT_FALSE(save->discardPathReservationsOnLoad);
    ASSERT_TRUE(save->vehicleAutoRenewalState.has_value());
    EXPECT_EQ(*save->vehicleAutoRenewalState, vehicleAutoRenewal);
    ASSERT_TRUE(save->railTrafficState.has_value());
    EXPECT_EQ(*save->railTrafficState, railTraffic);
    ASSERT_TRUE(save->gameRulesState.has_value());
    EXPECT_EQ(*save->gameRulesState, gameRules);
}

TEST(CargoDistSave, ScenarioTailRoundTripsRulesAndExtendedVehicleObjects)
{
    const GameRules::State gameRules{ .vehiclesNeverExpire = true, .extendedVehicleObjects = true };
    S5::SaveExtension::VehicleObjectState vehicleObjects;
    ObjectHeader header{};
    header.flags = enumValue(ObjectType::vehicle);
    header.checksum = 999;
    vehicleObjects.objects.push_back({ 999, header });
    vehicleObjects.companyUnlocks[3].set(S5::SaveExtension::kExtendedVehicleObjectCount - 1, true);
    MemoryStream stream;
    writeMinimalScenarioOrLandscape(stream, S5::S5Type::scenario, &gameRules, &vehicleObjects);

    const auto save = S5::loadSave(stream);

    ASSERT_EQ(save->header.type, S5::S5Type::scenario);
    ASSERT_TRUE(save->gameRulesState.has_value());
    EXPECT_EQ(*save->gameRulesState, gameRules);
    ASSERT_TRUE(save->vehicleObjectState.has_value());
    EXPECT_EQ(*save->vehicleObjectState, vehicleObjects);
    EXPECT_TRUE(save->vehicleObjectState->companyUnlocks[3][S5::SaveExtension::kExtendedVehicleObjectCount - 1]);

    const auto runtimeHeaders = S5::importRequiredObjectHeaders(save->requiredObjects, &*save->vehicleObjectState);
    constexpr size_t kVehicleObjectOffset = 389;
    EXPECT_EQ(runtimeHeaders[kVehicleObjectOffset + 999].checksum, 999);
}

TEST(CargoDistSave, LegacyMaplessScenarioWithoutExtensionStillLoads)
{
    MemoryStream stream;
    writeMinimalScenarioOrLandscape(stream, S5::S5Type::scenario);

    const auto save = S5::loadSave(stream);

    ASSERT_EQ(save->header.type, S5::S5Type::scenario);
    ASSERT_TRUE(save->scenarioOptions != nullptr);
    EXPECT_TRUE(save->tileElements.empty());
    EXPECT_FALSE(save->gameRulesState.has_value());
    EXPECT_FALSE(save->vehicleObjectState.has_value());
}

TEST(CargoDistSave, RejectsExtendedVehicleObjectsWithoutRule)
{
    S5::SaveExtension::VehicleObjectState vehicleObjects;
    ObjectHeader header{};
    header.flags = enumValue(ObjectType::vehicle);
    vehicleObjects.objects.push_back({ 224, header });
    MemoryStream stream;
    writeMinimalSave(stream, nullptr, false, nullptr, nullptr, nullptr, false, nullptr, nullptr, &vehicleObjects);

    EXPECT_ANY_THROW(S5::loadSave(stream));
}

TEST(CargoDistSave, LandscapeTailRoundTripsRulesAndExtendedVehicleObjects)
{
    const GameRules::State gameRules{ .extendedVehicleObjects = true };
    S5::SaveExtension::VehicleObjectState vehicleObjects;
    ObjectHeader header{};
    header.flags = enumValue(ObjectType::vehicle);
    header.checksum = 224;
    vehicleObjects.objects.push_back({ 224, header });
    vehicleObjects.companyUnlocks[0].set(0, true);
    MemoryStream stream;
    writeMinimalScenarioOrLandscape(stream, S5::S5Type::landscape, &gameRules, &vehicleObjects);

    const auto save = S5::loadSave(stream);

    ASSERT_EQ(save->header.type, S5::S5Type::landscape);
    EXPECT_EQ(save->scenarioOptions, nullptr);
    ASSERT_TRUE(save->saveDetails != nullptr);
    ASSERT_TRUE(save->gameRulesState.has_value());
    EXPECT_EQ(*save->gameRulesState, gameRules);
    ASSERT_TRUE(save->vehicleObjectState.has_value());
    EXPECT_EQ(*save->vehicleObjectState, vehicleObjects);
}

TEST(CargoDistSave, LegacyLandscapeWithoutExtensionStillLoads)
{
    MemoryStream stream;
    writeMinimalScenarioOrLandscape(stream, S5::S5Type::landscape);

    const auto save = S5::loadSave(stream);

    ASSERT_EQ(save->header.type, S5::S5Type::landscape);
    EXPECT_EQ(save->scenarioOptions, nullptr);
    ASSERT_TRUE(save->saveDetails != nullptr);
    EXPECT_FALSE(save->gameRulesState.has_value());
    EXPECT_FALSE(save->vehicleObjectState.has_value());
}

TEST(CargoDistSave, MarksVersionOnePathReservationsForRecalculation)
{
    Vehicles::RoutingManager::State pathReservations;
    pathReservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    MemoryStream stream;
    writeMinimalSave(stream, nullptr, false, nullptr, &pathReservations, nullptr, true);

    const auto save = S5::loadSave(stream);

    ASSERT_TRUE(save->pathReservationState.has_value());
    EXPECT_EQ(*save->pathReservationState, pathReservations);
    EXPECT_TRUE(save->discardPathReservationsOnLoad);
}

TEST(CargoDistSave, LegacyS5WithoutExtensionStillLoads)
{
    MemoryStream stream;
    writeMinimalSave(stream, nullptr);

    const auto save = S5::loadSave(stream);

    EXPECT_FALSE(save->cargoDistState.has_value());
    EXPECT_FALSE(save->sharedOrderState.has_value());
    EXPECT_FALSE(save->vehicleAutoRenewalState.has_value());
    EXPECT_FALSE(save->gameRulesState.has_value());
}

TEST(CargoDistSave, LegacyDirectExtensionStillLoads)
{
    const auto original = populatedState();
    MemoryStream stream;
    writeMinimalSave(stream, &original, true);

    const auto save = S5::loadSave(stream);

    ASSERT_TRUE(save->cargoDistState.has_value());
    expectStatesEqual(original, *save->cargoDistState);
    EXPECT_FALSE(save->sharedOrderState.has_value());
    EXPECT_FALSE(save->vehicleAutoRenewalState.has_value());
    EXPECT_FALSE(save->gameRulesState.has_value());
}
