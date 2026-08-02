// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Save.h>

#include "GameState.h"
#include "S5/S5.h"
#include "S5/S5File.h"
#include "S5/S5Options.h"
#include "S5/SawyerStream.h"
#include <OpenLoco/CargoDist/Simulation.h>
#include <OpenLoco/Core/MemoryStream.h>
#include <algorithm>
#include <array>
#include <gtest/gtest.h>
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
            }
        }
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
        state.stationCargo[{ station(2), 0 }].append({ 30, station(1), station(3), 4 });
        state.stationCargo[{ station(2), 0 }].append({ 10, station(4), StationId::null, 2 });
        state.vehicleCargo[{ entity(20), VehicleCargoSlot::primary }].append({ 25, station(1), station(2), 6 });
        state.supply[{ 0, station(1) }] = 120;
        state.flows[{ 0, station(2), station(1) }] = {
            { station(3), 75, -25 },
            { station(4), 25, 25 },
        };
        state.stationAttraction[{ station(2), 0 }] = 120;
        state.serviceEdges[{ 0, station(1), station(2) }] = { 40, 10 };
        return state;
    }

    void writeMinimalSave(MemoryStream& stream, const State* cargoDistState)
    {
        SawyerStreamWriter writer(stream);
        S5::Header header{};
        header.type = S5::S5Type::savedGame;
        writer.writeChunk(SawyerEncoding::uncompressed, header);

        std::array<ObjectHeader, 859> requiredObjects{};
        writer.writeChunk(SawyerEncoding::uncompressed, requiredObjects.data(), sizeof(requiredObjects));

        auto gameState = std::make_unique<S5::GameState>();
        gameState->general.fixFlags = enumValue(S5::S5FixFlags::fixFlag1);
        writer.writeChunk(SawyerEncoding::uncompressed, *gameState);

        std::array<std::byte, sizeof(S5::TileElement)> tile{};
        writer.writeChunk(SawyerEncoding::uncompressed, tile.data(), tile.size());
        if (cargoDistState != nullptr)
        {
            const auto encoded = encodeState(*cargoDistState);
            writer.writeChunk(SawyerEncoding::uncompressed, encoded.data(), encoded.size());
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
}

TEST(CargoDistSave, DecodesVersionOneWithoutStationAttraction)
{
    auto encoded = encodeState(State{});
    encoded[8] = std::byte{ 1 };
    encoded.resize(encoded.size() - sizeof(uint32_t));
    const auto payloadSize = static_cast<uint32_t>(encoded.size() - 16);
    for (size_t i = 0; i < sizeof(payloadSize); ++i)
    {
        encoded[12 + i] = static_cast<std::byte>((payloadSize >> (i * 8)) & 0xFF);
    }

    const auto decoded = decodeState(encoded);

    EXPECT_TRUE(decoded.stationAttraction.empty());
}

TEST(CargoDistSave, EncodingIsDeterministic)
{
    const auto state = populatedState();
    auto reordered = populatedState();
    auto& packets = reordered.stationCargo.at({ station(2), 0 });
    packets = {};
    packets.append({ 10, station(4), StationId::null, 2 });
    packets.append({ 30, station(1), station(3), 4 });

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

TEST(CargoDistSave, RejectsTruncatedData)
{
    auto encoded = encodeState(populatedState());
    encoded.pop_back();

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsUnknownVersion)
{
    auto encoded = encodeState(populatedState());
    encoded[8] = std::byte{ 3 };

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsInvalidMode)
{
    auto encoded = encodeState(populatedState());
    encoded[16] = std::byte{ 0xFF };

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, RejectsInvalidFlowCursor)
{
    auto state = populatedState();
    state.stationAttraction.clear();
    auto encoded = encodeState(state);
    std::fill(encoded.end() - sizeof(uint32_t) - sizeof(int64_t), encoded.end() - sizeof(uint32_t), std::byte{ 0x7F });

    EXPECT_THROW(decodeState(encoded), std::runtime_error);
}

TEST(CargoDistSave, S5TailRoundTripsExtension)
{
    const auto original = populatedState();
    MemoryStream stream;
    writeMinimalSave(stream, &original);

    const auto save = S5::loadSave(stream);

    ASSERT_TRUE(save->cargoDistState.has_value());
    expectStatesEqual(original, *save->cargoDistState);
}

TEST(CargoDistSave, LegacyS5WithoutExtensionStillLoads)
{
    MemoryStream stream;
    writeMinimalSave(stream, nullptr);

    const auto save = S5::loadSave(stream);

    EXPECT_FALSE(save->cargoDistState.has_value());
}
