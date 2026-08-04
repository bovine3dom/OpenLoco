// SPDX-License-Identifier: MIT
#include <OpenLoco/S5/SaveExtension.h>

#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

using namespace OpenLoco;

namespace
{
    constexpr EntityId entity(uint16_t value)
    {
        return static_cast<EntityId>(value);
    }

    CargoDist::State cargoDistState()
    {
        CargoDist::State state;
        state.settings.modes[3] = CargoDist::DistributionMode::asymmetric;
        state.settings.recalculationInterval = 5;
        state.nextRecalculationDay = 1234;
        state.graphDirty = true;
        return state;
    }

    Vehicles::SharedOrderManager::State sharedOrderState(std::initializer_list<std::initializer_list<uint16_t>> groups)
    {
        Vehicles::SharedOrderManager::State state;
        for (const auto group : groups)
        {
            auto& members = state.groups.emplace_back().members;
            for (const auto id : group)
            {
                members.push_back(entity(id));
            }
        }
        return state;
    }

    Vehicles::VehicleAutoRenewal::State vehicleAutoRenewalState()
    {
        Vehicles::VehicleAutoRenewal::State state;
        state.companies[2] = { true, 35 };
        state.companies[8] = { false, 70 };
        return state;
    }

    uint16_t readU16(const std::span<const std::byte> data, size_t offset)
    {
        return std::to_integer<uint8_t>(data[offset])
            | (static_cast<uint16_t>(std::to_integer<uint8_t>(data[offset + 1])) << 8);
    }

    uint32_t readU32(const std::span<const std::byte> data, size_t offset)
    {
        uint32_t value = 0;
        for (size_t i = 0; i < sizeof(value); ++i)
        {
            value |= static_cast<uint32_t>(std::to_integer<uint8_t>(data[offset + i])) << (i * 8);
        }
        return value;
    }

    void writeU16(std::vector<std::byte>& data, size_t offset, uint16_t value)
    {
        for (size_t i = 0; i < sizeof(value); ++i)
        {
            data[offset + i] = static_cast<std::byte>((value >> (i * 8)) & 0xFF);
        }
    }

    void writeU32(std::vector<std::byte>& data, size_t offset, uint32_t value)
    {
        for (size_t i = 0; i < sizeof(value); ++i)
        {
            data[offset + i] = static_cast<std::byte>((value >> (i * 8)) & 0xFF);
        }
    }

    void expectTag(const std::span<const std::byte> data, size_t offset, const char* tag)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_EQ(data[offset + i], static_cast<std::byte>(tag[i]));
        }
    }
}

TEST(SaveExtension, RoundTripsBothSections)
{
    const auto cargo = cargoDistState();
    const auto shared = sharedOrderState({ { 8, 3 }, { 11, 15, 12 } });

    const auto encoded = S5::SaveExtension::encode({ &cargo, &shared });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.cargoDistState.has_value());
    EXPECT_EQ(CargoDist::encodeState(*decoded.cargoDistState), CargoDist::encodeState(cargo));
    ASSERT_TRUE(decoded.sharedOrderState.has_value());
    EXPECT_EQ(*decoded.sharedOrderState, sharedOrderState({ { 3, 8 }, { 11, 12, 15 } }));
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);

    const auto cargoBytes = CargoDist::encodeState(cargo);
    expectTag(encoded, 16, "CDST");
    EXPECT_EQ(readU16(encoded, 20), 1);
    ASSERT_EQ(readU32(encoded, 24), cargoBytes.size());
    EXPECT_TRUE(std::ranges::equal(cargoBytes, std::span(encoded).subspan(28, cargoBytes.size())));
    expectTag(encoded, 28 + cargoBytes.size(), "SHOR");
    EXPECT_EQ(readU16(encoded, 32 + cargoBytes.size()), 1);
    EXPECT_EQ(readU16(encoded, 34 + cargoBytes.size()), 1);
}

TEST(SaveExtension, EncodingIsDeterministic)
{
    const auto first = sharedOrderState({ { 9, 7 }, { 5, 3, 4 } });
    const auto second = sharedOrderState({ { 3, 4, 5 }, { 7, 9 } });

    EXPECT_EQ(S5::SaveExtension::encode({ nullptr, &first }), S5::SaveExtension::encode({ nullptr, &second }));
}

TEST(SaveExtension, RoundTripsSharedOrdersOnly)
{
    const auto shared = sharedOrderState({ { 1, 2 } });

    const auto decoded = S5::SaveExtension::decode(S5::SaveExtension::encode({ nullptr, &shared }));

    EXPECT_FALSE(decoded.cargoDistState.has_value());
    ASSERT_TRUE(decoded.sharedOrderState.has_value());
    EXPECT_EQ(*decoded.sharedOrderState, shared);
}

TEST(SaveExtension, RoundTripsPathReservations)
{
    Vehicles::RoutingManager::State reservations;
    reservations.pathReservedRoutings[7] = (uint64_t{ 1 } << 3) | (uint64_t{ 1 } << 63);
    reservations.pathReservedRoutings[42] = uint64_t{ 1 } << 9;

    const auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, &reservations });
    const auto decoded = S5::SaveExtension::decode(encoded);

    EXPECT_FALSE(decoded.cargoDistState.has_value());
    EXPECT_FALSE(decoded.sharedOrderState.has_value());
    ASSERT_TRUE(decoded.pathReservationState.has_value());
    EXPECT_EQ(*decoded.pathReservationState, reservations);
    EXPECT_FALSE(decoded.discardPathReservationsOnLoad);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "PRES");
    EXPECT_EQ(readU16(encoded, 20), 2);
    EXPECT_EQ(readU16(encoded, 22), 1);
}

TEST(SaveExtension, DecodesVersionOnePathReservationsForRecalculation)
{
    Vehicles::RoutingManager::State reservations;
    reservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, &reservations });
    writeU16(encoded, 20, 1);

    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.pathReservationState.has_value());
    EXPECT_EQ(*decoded.pathReservationState, reservations);
    EXPECT_TRUE(decoded.discardPathReservationsOnLoad);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
}

TEST(SaveExtension, RejectsUnknownPathReservationVersion)
{
    Vehicles::RoutingManager::State reservations;
    reservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, &reservations });
    writeU16(encoded, 20, 3);

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RoundTripsVehicleAutoRenewal)
{
    const auto renewal = vehicleAutoRenewalState();

    const auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &renewal });
    const auto decoded = S5::SaveExtension::decode(encoded);

    EXPECT_FALSE(decoded.cargoDistState.has_value());
    EXPECT_FALSE(decoded.sharedOrderState.has_value());
    EXPECT_FALSE(decoded.pathReservationState.has_value());
    ASSERT_TRUE(decoded.vehicleAutoRenewalState.has_value());
    EXPECT_EQ(*decoded.vehicleAutoRenewalState, renewal);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "VREN");
    EXPECT_EQ(readU16(encoded, 20), 1);
    EXPECT_EQ(readU16(encoded, 22), 1);
    EXPECT_EQ(readU32(encoded, 24), Limits::kMaxCompanies * 2);
}

TEST(SaveExtension, VehicleAutoRenewalEncodingIsDeterministic)
{
    auto first = vehicleAutoRenewalState();
    auto second = Vehicles::VehicleAutoRenewal::State{};
    second.companies[8] = { false, 70 };
    second.companies[2] = { true, 35 };

    EXPECT_EQ(
        S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &first }),
        S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &second }));
}

TEST(SaveExtension, DecodesLegacyCargoDist)
{
    const auto cargo = cargoDistState();
    const auto encoded = CargoDist::encodeState(cargo);

    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.cargoDistState.has_value());
    EXPECT_EQ(CargoDist::encodeState(*decoded.cargoDistState), encoded);
    EXPECT_FALSE(decoded.sharedOrderState.has_value());
    EXPECT_FALSE(decoded.vehicleAutoRenewalState.has_value());

    auto versionOne = CargoDist::encodeState(CargoDist::State{});
    versionOne[8] = std::byte{ 1 };
    versionOne.resize(versionOne.size() - sizeof(uint32_t));
    writeU32(versionOne, 12, static_cast<uint32_t>(versionOne.size() - 16));
    const auto decodedVersionOne = S5::SaveExtension::decode(versionOne);
    ASSERT_TRUE(decodedVersionOne.cargoDistState.has_value());
    EXPECT_FALSE(decodedVersionOne.sharedOrderState.has_value());
    EXPECT_FALSE(decodedVersionOne.vehicleAutoRenewalState.has_value());
}

TEST(SaveExtension, RejectsInvalidVehicleAutoRenewalState)
{
    const auto renewal = vehicleAutoRenewalState();
    auto invalidEnabled = S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &renewal });
    invalidEnabled[28] = std::byte{ 2 };
    EXPECT_THROW(S5::SaveExtension::decode(invalidEnabled), std::runtime_error);

    auto invalidThreshold = S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &renewal });
    invalidThreshold[29] = std::byte{ 101 };
    EXPECT_THROW(S5::SaveExtension::decode(invalidThreshold), std::runtime_error);

    auto invalidLength = S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &renewal });
    writeU32(invalidLength, 24, readU32(invalidLength, 24) - 1);
    EXPECT_THROW(S5::SaveExtension::decode(invalidLength), std::runtime_error);
}

TEST(SaveExtension, RejectsDuplicateVehicleAutoRenewalSection)
{
    const auto renewal = vehicleAutoRenewalState();
    auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, nullptr, &renewal });
    const std::vector duplicate(encoded.begin() + 16, encoded.end());
    encoded.insert(encoded.end(), duplicate.begin(), duplicate.end());
    writeU32(encoded, 12, static_cast<uint32_t>(encoded.size() - 16));

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RejectsMalformedLengths)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto invalidOuter = S5::SaveExtension::encode({ nullptr, &shared });
    writeU32(invalidOuter, 12, readU32(invalidOuter, 12) + 1);
    EXPECT_THROW(S5::SaveExtension::decode(invalidOuter), std::runtime_error);

    auto invalidSection = S5::SaveExtension::encode({ nullptr, &shared });
    writeU32(invalidSection, 24, std::numeric_limits<uint32_t>::max());
    EXPECT_THROW(S5::SaveExtension::decode(invalidSection), std::runtime_error);
}

TEST(SaveExtension, RejectsDuplicateKnownSection)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto encoded = S5::SaveExtension::encode({ nullptr, &shared });
    const std::vector duplicate(encoded.begin() + 16, encoded.end());
    encoded.insert(encoded.end(), duplicate.begin(), duplicate.end());
    writeU32(encoded, 12, static_cast<uint32_t>(encoded.size() - 16));

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, SkipsUnknownOptionalAndRejectsUnknownRequiredSection)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto encoded = S5::SaveExtension::encode({ nullptr, &shared });
    encoded[16] = std::byte{ 'U' };
    encoded[17] = std::byte{ 'N' };
    encoded[18] = std::byte{ 'K' };
    encoded[19] = std::byte{ 'N' };
    writeU16(encoded, 22, 0);

    const auto decoded = S5::SaveExtension::decode(encoded);
    EXPECT_FALSE(decoded.cargoDistState.has_value());
    EXPECT_FALSE(decoded.sharedOrderState.has_value());

    writeU16(encoded, 22, 1);
    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, SkipsUnknownOptionalSectionFlags)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto encoded = S5::SaveExtension::encode({ nullptr, &shared });
    encoded[16] = std::byte{ 'U' };
    encoded[17] = std::byte{ 'N' };
    encoded[18] = std::byte{ 'K' };
    encoded[19] = std::byte{ 'N' };
    writeU16(encoded, 22, 1U << 4);

    EXPECT_NO_THROW(S5::SaveExtension::decode(encoded));
}

TEST(SaveExtension, RejectsNonCanonicalSharedOrderMembers)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto encoded = S5::SaveExtension::encode({ nullptr, &shared });
    writeU16(encoded, 36, 2);
    writeU16(encoded, 38, 1);

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RejectsInvalidSharedOrderGroupStructure)
{
    const auto shared = sharedOrderState({ { 1, 2 } });
    auto singleton = S5::SaveExtension::encode({ nullptr, &shared });
    writeU32(singleton, 32, 1);
    EXPECT_THROW(S5::SaveExtension::decode(singleton), std::runtime_error);

    auto invalidEntity = S5::SaveExtension::encode({ nullptr, &shared });
    writeU16(invalidEntity, 38, 20000);
    EXPECT_THROW(S5::SaveExtension::decode(invalidEntity), std::runtime_error);
}
