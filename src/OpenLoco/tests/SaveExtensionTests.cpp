// SPDX-License-Identifier: MIT
#include <OpenLoco/S5/SaveExtension.h>

#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/GameRules.h>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <string_view>
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

    Vehicles::RailTraffic::State railTrafficState()
    {
        Vehicles::RailTraffic::State state;
        state.history.push_back({ { 320, 352, 32, 4, 0 }, 12 * Vehicles::RailTraffic::kOneTick, 123, 7 });
        state.active.push_back({ entity(9), entity(3), { 288, 352, 32, 0, 0 }, 100 * Vehicles::RailTraffic::kOneTick, true });
        return state;
    }

    Vehicles::VehicleReplacement::State vehicleReplacementState()
    {
        Vehicles::VehicleReplacement::State::PendingPlacement placement;
        placement.args.pos = World::Pos3(512, 384, 32);
        placement.args.trackAndDirection = 3;
        placement.args.trackProgress = 5;
        placement.args.head = entity(42);
        placement.start = true;

        Vehicles::VehicleReplacement::State state;
        state.requests.push_back({ entity(5), entity(11) });
        state.requests.push_back({ entity(7), entity(11) });
        state.pendingPlacements.push_back(placement);
        return state;
    }

    std::vector<S5::SaveExtension::StationTileOverflow> stationTileOverflow(uint16_t station, uint16_t stationTileSize)
    {
        std::vector<S5::SaveExtension::StationTileOverflow> result;
        auto& entry = result.emplace_back();
        entry.station = static_cast<StationId>(station);
        entry.stationTileSize = stationTileSize;
        for (uint16_t i = 0; i < stationTileSize; ++i)
        {
            entry.stationTiles.emplace_back(64, 64 + i, 0);
        }
        return result;
    }

    ObjectHeader vehicleHeader(uint32_t checksum)
    {
        ObjectHeader header{};
        header.flags = enumValue(ObjectType::vehicle);
        std::ranges::copy(std::string_view("VEHICLE "), header.name);
        header.checksum = checksum;
        return header;
    }

    S5::SaveExtension::VehicleObjectState vehicleObjectState()
    {
        S5::SaveExtension::VehicleObjectState state;
        state.objects = {
            { 999, vehicleHeader(2) },
            { 224, vehicleHeader(1) },
        };
        state.companyUnlocks[0].set(0, true);
        state.companyUnlocks.back().set(S5::SaveExtension::kExtendedVehicleObjectCount - 1, true);
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

TEST(SaveExtension, RoundTripsPathReservationContinuations)
{
    Vehicles::RoutingManager::State reservations;
    reservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    reservations.continuations[7] = { 0, 0 };

    const auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, &reservations });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.pathReservationState.has_value());
    EXPECT_EQ(*decoded.pathReservationState, reservations);
    EXPECT_FALSE(decoded.discardPathReservationsOnLoad);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "PRES");
    EXPECT_EQ(readU16(encoded, 20), 3);
    EXPECT_EQ(readU32(encoded, 24), Limits::kMaxVehicles * sizeof(uint64_t) + 2 + 4 + 4);
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
    writeU16(encoded, 20, 4);

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RejectsTruncatedPathReservationContinuation)
{
    Vehicles::RoutingManager::State reservations;
    reservations.pathReservedRoutings[7] = uint64_t{ 1 } << 3;
    reservations.continuations[7] = { 0 };
    auto encoded = S5::SaveExtension::encode({ nullptr, nullptr, &reservations });
    constexpr auto kEntryCountOffset = 28 + Limits::kMaxVehicles * sizeof(uint64_t) + sizeof(uint16_t) * 2;
    writeU16(encoded, kEntryCountOffset, 2);

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

TEST(SaveExtension, RoundTripsRailTraffic)
{
    const auto traffic = railTrafficState();
    const auto encoded = S5::SaveExtension::encode({ .railTrafficState = &traffic });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.railTrafficState.has_value());
    EXPECT_EQ(*decoded.railTrafficState, traffic);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "RTFC");
    EXPECT_EQ(readU16(encoded, 20), 1);
    EXPECT_EQ(readU16(encoded, 22), 1);
}

TEST(SaveExtension, RoundTripsRequiredGameRules)
{
    const GameRules::State rules{ .vehiclesNeverExpire = true, .extendedVehicleObjects = true };
    const auto encoded = S5::SaveExtension::encode({ .gameRulesState = &rules });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.gameRulesState.has_value());
    EXPECT_EQ(*decoded.gameRulesState, rules);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "RULE");
    EXPECT_EQ(readU16(encoded, 20), 1);
    EXPECT_EQ(readU16(encoded, 22), 1);
    EXPECT_EQ(readU32(encoded, 24), 1);
    EXPECT_EQ(encoded[28], std::byte{ 3 });
}

TEST(SaveExtension, EncodesGameRuleBitsIndependently)
{
    const GameRules::State neverExpire{ .vehiclesNeverExpire = true };
    const auto neverExpireData = S5::SaveExtension::encode({ .gameRulesState = &neverExpire });
    EXPECT_EQ(neverExpireData[28], std::byte{ 1 });

    const GameRules::State extendedObjects{ .extendedVehicleObjects = true };
    const auto extendedObjectsData = S5::SaveExtension::encode({ .gameRulesState = &extendedObjects });
    EXPECT_EQ(extendedObjectsData[28], std::byte{ 2 });
}

TEST(SaveExtension, RejectsInvalidGameRules)
{
    const GameRules::State rules{};

    auto unsupportedVersion = S5::SaveExtension::encode({ .gameRulesState = &rules });
    writeU16(unsupportedVersion, 20, 2);
    EXPECT_THROW(S5::SaveExtension::decode(unsupportedVersion), std::runtime_error);

    auto unknownBits = S5::SaveExtension::encode({ .gameRulesState = &rules });
    unknownBits[28] = std::byte{ 4 };
    EXPECT_THROW(S5::SaveExtension::decode(unknownBits), std::runtime_error);

    auto optionalSection = S5::SaveExtension::encode({ .gameRulesState = &rules });
    writeU16(optionalSection, 22, 0);
    EXPECT_THROW(S5::SaveExtension::decode(optionalSection), std::runtime_error);

    auto duplicate = S5::SaveExtension::encode({ .gameRulesState = &rules });
    const std::vector duplicateSection(duplicate.begin() + 16, duplicate.end());
    duplicate.insert(duplicate.end(), duplicateSection.begin(), duplicateSection.end());
    writeU32(duplicate, 12, static_cast<uint32_t>(duplicate.size() - 16));
    EXPECT_THROW(S5::SaveExtension::decode(duplicate), std::runtime_error);
}

TEST(SaveExtension, RoundTripsCanonicalExtendedVehicleObjects)
{
    const auto state = vehicleObjectState();
    const auto encoded = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.vehicleObjectState.has_value());
    auto expected = state;
    std::ranges::sort(expected.objects, {}, &S5::SaveExtension::VehicleObjectSlot::slot);
    EXPECT_EQ(*decoded.vehicleObjectState, expected);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    EXPECT_FALSE(decoded.gameRulesState.has_value());
    EXPECT_EQ(decoded.gameRulesState.value_or(GameRules::kDefaultState), GameRules::kDefaultState);

    expectTag(encoded, 16, "VOBJ");
    EXPECT_EQ(readU16(encoded, 20), 1);
    EXPECT_EQ(readU16(encoded, 22), 1);
    EXPECT_EQ(readU16(encoded, 28), 2);
    EXPECT_EQ(readU16(encoded, 30), S5::Limits::kMaxCompanies);
    EXPECT_EQ(readU16(encoded, 32), 13);
    EXPECT_EQ(readU16(encoded, 34), 224);
    EXPECT_EQ(readU16(encoded, 52), 999);
}

TEST(SaveExtension, RejectsInvalidExtendedVehicleObjectEntries)
{
    auto invalidSlot = vehicleObjectState();
    invalidSlot.objects[0].slot = 1000;
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleObjectState = &invalidSlot }), std::runtime_error);

    auto invalidType = vehicleObjectState();
    invalidType.objects[0].header.flags = enumValue(ObjectType::road);
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleObjectState = &invalidType }), std::runtime_error);

    auto duplicate = vehicleObjectState();
    duplicate.objects[0].slot = duplicate.objects[1].slot;
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleObjectState = &duplicate }), std::runtime_error);

    const S5::SaveExtension::VehicleObjectState empty;
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleObjectState = &empty }), std::runtime_error);
}

TEST(SaveExtension, RejectsMalformedExtendedVehicleObjectEntries)
{
    const auto state = vehicleObjectState();

    auto nonCanonical = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(nonCanonical, 52, 224);
    EXPECT_THROW(S5::SaveExtension::decode(nonCanonical), std::runtime_error);

    auto outOfBounds = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(outOfBounds, 34, 223);
    EXPECT_THROW(S5::SaveExtension::decode(outOfBounds), std::runtime_error);

    auto invalidType = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU32(invalidType, 36, enumValue(ObjectType::road));
    EXPECT_THROW(S5::SaveExtension::decode(invalidType), std::runtime_error);

    auto unsupportedVersion = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(unsupportedVersion, 20, 2);
    EXPECT_THROW(S5::SaveExtension::decode(unsupportedVersion), std::runtime_error);

    auto optionalSection = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(optionalSection, 22, 0);
    EXPECT_THROW(S5::SaveExtension::decode(optionalSection), std::runtime_error);
}

TEST(SaveExtension, RejectsInvalidExtendedVehicleUnlockDimensions)
{
    const auto state = vehicleObjectState();

    auto companyCount = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(companyCount, 30, S5::Limits::kMaxCompanies - 1);
    EXPECT_THROW(S5::SaveExtension::decode(companyCount), std::runtime_error);

    auto wordCount = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    writeU16(wordCount, 32, 12);
    EXPECT_THROW(S5::SaveExtension::decode(wordCount), std::runtime_error);
}

TEST(SaveExtension, RejectsUnlocksForEmptyVehicleSlotsAndPadding)
{
    auto emptySlot = vehicleObjectState();
    emptySlot.companyUnlocks[0].set(1, true);
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleObjectState = &emptySlot }), std::runtime_error);

    const auto state = vehicleObjectState();
    constexpr size_t kUnlockDataOffset = 34 + 2 * (sizeof(uint16_t) + sizeof(ObjectHeader));
    auto decodedEmptySlot = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    decodedEmptySlot[kUnlockDataOffset] |= std::byte{ 2 };
    EXPECT_THROW(S5::SaveExtension::decode(decodedEmptySlot), std::runtime_error);

    auto padding = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    padding[kUnlockDataOffset + 12 * sizeof(uint64_t) + 1] = std::byte{ 1 };
    EXPECT_THROW(S5::SaveExtension::decode(padding), std::runtime_error);
}

TEST(SaveExtension, RejectsTruncatedExtendedVehicleObjects)
{
    const auto state = vehicleObjectState();
    auto encoded = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    encoded.pop_back();

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RejectsDuplicateExtendedVehicleObjectSection)
{
    const auto state = vehicleObjectState();
    auto encoded = S5::SaveExtension::encode({ .vehicleObjectState = &state });
    const std::vector duplicate(encoded.begin() + 16, encoded.end());
    encoded.insert(encoded.end(), duplicate.begin(), duplicate.end());
    writeU32(encoded, 12, static_cast<uint32_t>(encoded.size() - 16));

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RailTrafficEncodingIsDeterministic)
{
    auto first = railTrafficState();
    first.history.push_back({ { 288, 352, 32, 0, 0 }, 8 * Vehicles::RailTraffic::kOneTick, 124, 3 });
    first.active.push_back({ entity(5), entity(2), { 256, 352, 32, 0, 0 }, 101 * Vehicles::RailTraffic::kOneTick, false });
    auto second = first;
    std::ranges::reverse(second.history);
    std::ranges::reverse(second.active);

    EXPECT_EQ(
        S5::SaveExtension::encode({ .railTrafficState = &first }),
        S5::SaveExtension::encode({ .railTrafficState = &second }));
}

TEST(SaveExtension, RoundTripsVehicleReplacement)
{
    const auto replacement = vehicleReplacementState();

    const auto encoded = S5::SaveExtension::encode({ .vehicleReplacementState = &replacement });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.vehicleReplacementState.has_value());
    EXPECT_EQ(*decoded.vehicleReplacementState, replacement);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "VRPL");
    EXPECT_EQ(readU16(encoded, 20), 3);
    EXPECT_EQ(readU16(encoded, 22), 1);
}

TEST(SaveExtension, DecodesVersionOneVehicleReplacement)
{
    Vehicles::VehicleReplacement::State replacement;
    replacement.requests.push_back({ entity(5), entity(11) });

    auto encoded = S5::SaveExtension::encode({ .vehicleReplacementState = &replacement });
    // Version 1 omits the trailing pending placement count field.
    writeU16(encoded, 20, 1);
    writeU32(encoded, 12, readU32(encoded, 12) - sizeof(uint16_t));
    writeU32(encoded, 24, readU32(encoded, 24) - sizeof(uint16_t));
    encoded.resize(encoded.size() - sizeof(uint16_t));

    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.vehicleReplacementState.has_value());
    EXPECT_EQ(decoded.vehicleReplacementState->requests, replacement.requests);
    EXPECT_TRUE(decoded.vehicleReplacementState->pendingPlacements.empty());
}

TEST(SaveExtension, DecodesVersionTwoVehicleReplacement)
{
    const auto replacement = vehicleReplacementState();
    auto encoded = S5::SaveExtension::encode({ .vehicleReplacementState = &replacement });
    // Version 2 omits the trailing per-pending start byte.
    writeU16(encoded, 20, 2);
    writeU32(encoded, 12, readU32(encoded, 12) - 1);
    writeU32(encoded, 24, readU32(encoded, 24) - 1);
    encoded.resize(encoded.size() - 1);

    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.vehicleReplacementState.has_value());
    ASSERT_EQ(decoded.vehicleReplacementState->pendingPlacements.size(), 1);
    EXPECT_EQ(decoded.vehicleReplacementState->pendingPlacements[0].args, replacement.pendingPlacements[0].args);
    EXPECT_FALSE(decoded.vehicleReplacementState->pendingPlacements[0].start);
}

TEST(SaveExtension, RejectsInvalidVehicleReplacementState)
{
    auto duplicateHead = vehicleReplacementState();
    duplicateHead.pendingPlacements.push_back(vehicleReplacementState().pendingPlacements[0]);
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleReplacementState = &duplicateHead }), std::runtime_error);

    Vehicles::VehicleReplacement::State::PendingPlacement nullPlacement;
    nullPlacement.args.head = EntityId::null;
    auto invalid = vehicleReplacementState();
    invalid.pendingPlacements.push_back(nullPlacement);
    EXPECT_THROW(S5::SaveExtension::encode({ .vehicleReplacementState = &invalid }), std::runtime_error);

    const auto replacement = vehicleReplacementState();
    auto unsupportedVersion = S5::SaveExtension::encode({ .vehicleReplacementState = &replacement });
    writeU16(unsupportedVersion, 20, 4);
    EXPECT_THROW(S5::SaveExtension::decode(unsupportedVersion), std::runtime_error);

    auto truncated = S5::SaveExtension::encode({ .vehicleReplacementState = &replacement });
    writeU16(truncated, 38, 99);
    EXPECT_THROW(S5::SaveExtension::decode(truncated), std::runtime_error);
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
    EXPECT_FALSE(decoded.gameRulesState.has_value());

    auto versionOne = CargoDist::encodeState(CargoDist::State{});
    versionOne[8] = std::byte{ 1 };
    versionOne.resize(versionOne.size() - sizeof(uint32_t) * 8);
    writeU32(versionOne, 12, static_cast<uint32_t>(versionOne.size() - 16));
    const auto decodedVersionOne = S5::SaveExtension::decode(versionOne);
    ASSERT_TRUE(decodedVersionOne.cargoDistState.has_value());
    EXPECT_FALSE(decodedVersionOne.sharedOrderState.has_value());
    EXPECT_FALSE(decodedVersionOne.vehicleAutoRenewalState.has_value());
    EXPECT_FALSE(decodedVersionOne.gameRulesState.has_value());
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

TEST(SaveExtension, RoundTripsStationTileOverflow)
{
    const auto overflow = stationTileOverflow(3, 120);
    const auto encoded = S5::SaveExtension::encode({ .stationTileOverflowState = &overflow });
    const auto decoded = S5::SaveExtension::decode(encoded);

    ASSERT_TRUE(decoded.stationTileOverflowState.has_value());
    ASSERT_EQ(decoded.stationTileOverflowState->size(), 1);
    EXPECT_EQ((*decoded.stationTileOverflowState)[0].station, static_cast<StationId>(3));
    EXPECT_EQ((*decoded.stationTileOverflowState)[0].stationTileSize, 120);
    EXPECT_EQ((*decoded.stationTileOverflowState)[0].stationTiles, overflow[0].stationTiles);
    EXPECT_EQ(S5::SaveExtension::encode(decoded), encoded);
    expectTag(encoded, 16, "STNS");
    EXPECT_EQ(readU16(encoded, 20), 1);
    EXPECT_EQ(readU16(encoded, 22), 1);
}

TEST(SaveExtension, StationTileOverflowEncodingIsDeterministic)
{
    auto first = stationTileOverflow(3, 90);
    first.push_back(stationTileOverflow(1, 120)[0]);
    auto second = first;
    std::swap(second[0], second[1]);

    EXPECT_EQ(
        S5::SaveExtension::encode({ .stationTileOverflowState = &first }),
        S5::SaveExtension::encode({ .stationTileOverflowState = &second }));
}

TEST(SaveExtension, RejectsInvalidStationTileOverflowSize)
{
    auto tooSmall = stationTileOverflow(3, 90);
    tooSmall[0].stationTileSize = 80;
    EXPECT_THROW(S5::SaveExtension::encode({ .stationTileOverflowState = &tooSmall }), std::runtime_error);

    auto tooLarge = stationTileOverflow(3, 90);
    tooLarge[0].stationTileSize = 257;
    EXPECT_THROW(S5::SaveExtension::encode({ .stationTileOverflowState = &tooLarge }), std::runtime_error);
}

TEST(SaveExtension, RejectsStationTileOverflowWithMismatchedCount)
{
    auto missingTile = stationTileOverflow(3, 90);
    missingTile[0].stationTiles.pop_back();
    EXPECT_THROW(S5::SaveExtension::encode({ .stationTileOverflowState = &missingTile }), std::runtime_error);
}

TEST(SaveExtension, RejectsStationTileOverflowWithInvalidTile)
{
    auto invalidX = stationTileOverflow(3, 90);
    invalidX[0].stationTiles[10] = { 20000, 64, 0 };
    EXPECT_THROW(S5::SaveExtension::encode({ .stationTileOverflowState = &invalidX }), std::runtime_error);

    auto invalidZ = stationTileOverflow(3, 90);
    invalidZ[0].stationTiles[10] = { 64, 64, -1 };
    EXPECT_THROW(S5::SaveExtension::encode({ .stationTileOverflowState = &invalidZ }), std::runtime_error);
}

TEST(SaveExtension, RejectsNonCanonicalStationTileOverflow)
{
    auto overflow = stationTileOverflow(1, 90);
    overflow.push_back(stationTileOverflow(3, 90)[0]);
    auto encoded = S5::SaveExtension::encode({ .stationTileOverflowState = &overflow });
    const auto secondEntryOffset = 28 + 4 + 2 + 2 + 90 * 6;
    writeU16(encoded, secondEntryOffset, 0);

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}

TEST(SaveExtension, RejectsTruncatedStationTileOverflow)
{
    const auto overflow = stationTileOverflow(3, 90);
    auto encoded = S5::SaveExtension::encode({ .stationTileOverflowState = &overflow });
    writeU32(encoded, 24, readU32(encoded, 24) - 1);

    EXPECT_THROW(S5::SaveExtension::decode(encoded), std::runtime_error);
}
