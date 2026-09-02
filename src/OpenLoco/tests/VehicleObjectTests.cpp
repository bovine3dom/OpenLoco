#include "Graphics/Colour.h"
#include "Objects/ObjectManager.h"
#include "Objects/VehicleObject.h"
#include "S5/Limits.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <string_view>
#include <vector>

using namespace OpenLoco;

namespace
{
    ObjectHeader makeHeader(const std::string_view name, const SourceGame source = SourceGame::vanilla, const ObjectType type = ObjectType::vehicle)
    {
        ObjectHeader header{};
        header.flags = static_cast<uint32_t>(type) | (static_cast<uint32_t>(source) << 6);
        std::copy(name.begin(), name.end(), header.name);
        return header;
    }

    size_t objectTypeOffset(const ObjectType target)
    {
        size_t offset = 0;
        for (uint8_t type = 0; type < enumValue(target); ++type)
        {
            offset += ObjectManager::getMaxObjects(static_cast<ObjectType>(type));
        }
        return offset;
    }

}

TEST(VehicleObjectTest, AppliesCapacityOverrides)
{
    struct TestCase
    {
        std::string_view name;
        uint8_t originalCapacity;
        uint8_t correctedCapacity;
    };

    constexpr TestCase kTestCases[] = {
        { "142     ", 90, 121 },
        { "2EPB    ", 100, 186 },
        { "AILSA1  ", 60, 79 },
        { "CLASSIC ", 45, 78 },
        { "COMET   ", 90, 44 },
        { "CONCOR  ", 250, 100 },
        { "ESTAR2  ", 67, 44 },
        { "HCRAFT1 ", 200, 254 },
        { "JFOIL1  ", 125, 250 },
        { "LEOP1   ", 45, 75 },
        { "RBE24   ", 50, 100 },
        { "RTMASTER", 50, 69 },
        { "TDH5301 ", 40, 75 },
        { "TGV2    ", 70, 48 },
        { "TRAM1   ", 35, 102 },
        { "TRAM2   ", 20, 33 },
        { "TRAM3   ", 55, 74 },
        { "TRAM4   ", 65, 120 },
        { "TRAMCOMB", 85, 176 },
        { "VULCAN  ", 14, 28 },
    };

    for (const auto& testCase : kTestCases)
    {
        SCOPED_TRACE(testCase.name);
        const auto header = makeHeader(testCase.name);
        const auto capacity = getEffectiveVehicleCapacity(header, testCase.originalCapacity);
        EXPECT_EQ(capacity, testCase.correctedCapacity);
        EXPECT_EQ(getEffectiveVehicleCapacity(header, capacity), testCase.correctedCapacity);
    }
}

TEST(VehicleObjectTest, DoesNotOverrideOtherObjects)
{
    EXPECT_EQ(getEffectiveVehicleCapacity(makeHeader("TRAMCOMB", SourceGame::custom), 85), 85);
    EXPECT_EQ(getEffectiveVehicleCapacity(makeHeader("TRAMCOMB", SourceGame::openLoco), 85), 85);
    EXPECT_EQ(getEffectiveVehicleCapacity(makeHeader("WMCBUS  "), 12), 12);
    EXPECT_EQ(getEffectiveVehicleCapacity(makeHeader("TRAMCOMB", SourceGame::vanilla, ObjectType::road), 85), 85);
}

TEST(VehicleObjectTest, IdentifiesOfficialSrn4Hovercraft)
{
    EXPECT_TRUE(isSrn4HovercraftObject(makeHeader("HCRAFT1 ")));
    EXPECT_FALSE(isSrn4HovercraftObject(makeHeader("HCRAFT1 ", SourceGame::custom)));
    EXPECT_FALSE(isSrn4HovercraftObject(makeHeader("HCRAFT1 ", SourceGame::openLoco)));
    EXPECT_FALSE(isSrn4HovercraftObject(makeHeader("HCRAFT1 ", SourceGame::vanilla, ObjectType::road)));
    EXPECT_FALSE(isSrn4HovercraftObject(makeHeader("JFOIL1  ")));
}

TEST(VehicleObjectTest, DoesNotOverrideCompactedSecondaryCargo)
{
    EXPECT_EQ(getEffectiveVehicleCapacity(makeHeader("2EPB    "), 5), 5);
}

TEST(VehicleObjectTest, IdentifiesTgvLaPosteObjects)
{
    EXPECT_TRUE(isOfficialTgvPassengerCarriage(kOfficialTgvPassengerCarriageHeader));
    EXPECT_FALSE(isOfficialTgvPassengerCarriage(makeHeader("TGV2    ")));
    EXPECT_FALSE(isOfficialTgvPassengerCarriage(makeHeader("TGV2    ", SourceGame::custom)));
    EXPECT_TRUE(isOfficialMailCargo(kOfficialMailCargoHeader));
    EXPECT_FALSE(isOfficialMailCargo(makeHeader("MAIL    ", SourceGame::vanilla, ObjectType::cargo)));
    EXPECT_TRUE(isTgvLaPosteObject(kTgvLaPosteObjectHeader));

    auto wrongSource = kTgvLaPosteObjectHeader;
    wrongSource.flags = static_cast<uint32_t>(ObjectType::vehicle) | (static_cast<uint32_t>(SourceGame::custom) << 6);
    EXPECT_FALSE(isTgvLaPosteObject(wrongSource));
    auto wrongChecksum = kTgvLaPosteObjectHeader;
    wrongChecksum.checksum++;
    EXPECT_FALSE(isTgvLaPosteObject(wrongChecksum));
}

TEST(VehicleObjectTest, AppliesTgvLaPosteOverrides)
{
    VehicleObject vehicle{};
    vehicle.flags = VehicleObjectFlags::refittable;

    applyTgvLaPosteVehicleOverrides(vehicle, 1U << 5);

    EXPECT_EQ(vehicle.name, StringIds::tgv_la_poste_mail_carriage);
    EXPECT_EQ(vehicle.maxCargo[0], kTgvLaPosteMailCapacity);
    EXPECT_EQ(vehicle.maxCargo[1], 0);
    EXPECT_EQ(vehicle.compatibleCargoCategories[0], 1U << 5);
    EXPECT_EQ(vehicle.compatibleCargoCategories[1], 0);
    EXPECT_EQ(vehicle.numSimultaneousCargoTypes, 1);
    EXPECT_FALSE(vehicle.hasFlags(VehicleObjectFlags::refittable));
    EXPECT_TRUE(vehicle.hasFlags(VehicleObjectFlags::quietInvention));
}

TEST(VehicleObjectTest, UsesFixedTgvLaPosteColours)
{
    const ColourScheme requested{ Colour::red, Colour::white };
    const auto fixed = getEffectiveVehicleColourScheme(kTgvLaPosteObjectHeader, requested);

    EXPECT_EQ(fixed.primary, Colour::yellow);
    EXPECT_EQ(fixed.secondary, Colour::darkBlue);
    const auto ordinary = getEffectiveVehicleColourScheme(kOfficialTgvPassengerCarriageHeader, requested);
    EXPECT_EQ(ordinary.primary, requested.primary);
    EXPECT_EQ(ordinary.secondary, requested.secondary);
}

TEST(VehicleObjectTest, InjectsTgvLaPosteIntoFirstExtendedVehicleSlot)
{
    std::vector<ObjectHeader> objects(ObjectManager::kMaxObjects, kEmptyObjectHeader);
    const auto vehicleOffset = objectTypeOffset(ObjectType::vehicle);
    const auto cargoOffset = objectTypeOffset(ObjectType::cargo);
    objects[vehicleOffset + 7] = kOfficialTgvPassengerCarriageHeader;
    objects[cargoOffset] = kOfficialMailCargoHeader;

    const auto slot = ObjectManager::injectTgvLaPosteObject(objects);

    ASSERT_TRUE(slot.has_value());
    EXPECT_EQ(*slot, S5::Limits::kMaxVehicleObjects);
    EXPECT_TRUE(isTgvLaPosteObject(objects[vehicleOffset + *slot]));
    EXPECT_EQ(ObjectManager::injectTgvLaPosteObject(objects), slot);
}

TEST(VehicleObjectTest, InjectsTgvLaPosteWithoutConsumingLegacySlots)
{
    std::vector<ObjectHeader> objects(ObjectManager::kMaxObjects, kEmptyObjectHeader);
    const auto vehicleOffset = objectTypeOffset(ObjectType::vehicle);
    const auto cargoOffset = objectTypeOffset(ObjectType::cargo);
    std::fill_n(objects.begin() + vehicleOffset, S5::Limits::kMaxVehicleObjects, kOfficialTgvPassengerCarriageHeader);
    objects[cargoOffset] = kOfficialMailCargoHeader;

    const auto slot = ObjectManager::injectTgvLaPosteObject(objects);

    ASSERT_TRUE(slot.has_value());
    EXPECT_EQ(*slot, S5::Limits::kMaxVehicleObjects);
    EXPECT_TRUE(isTgvLaPosteObject(objects[vehicleOffset + *slot]));
}

TEST(VehicleObjectTest, DoesNotInjectTgvLaPosteWithoutAFreeVehicleSlot)
{
    std::vector<ObjectHeader> objects(ObjectManager::kMaxObjects, kEmptyObjectHeader);
    const auto vehicleOffset = objectTypeOffset(ObjectType::vehicle);
    const auto cargoOffset = objectTypeOffset(ObjectType::cargo);
    std::fill_n(objects.begin() + vehicleOffset, ObjectManager::getMaxObjects(ObjectType::vehicle), makeHeader("OTHER   "));
    objects[vehicleOffset] = kOfficialTgvPassengerCarriageHeader;
    objects[cargoOffset] = kOfficialMailCargoHeader;

    EXPECT_FALSE(ObjectManager::injectTgvLaPosteObject(objects).has_value());
}

TEST(VehicleObjectTest, DoesNotInjectTgvLaPosteWithoutMail)
{
    std::vector<ObjectHeader> objects(ObjectManager::kMaxObjects, kEmptyObjectHeader);
    objects[objectTypeOffset(ObjectType::vehicle)] = kOfficialTgvPassengerCarriageHeader;

    EXPECT_FALSE(ObjectManager::injectTgvLaPosteObject(objects).has_value());
}

TEST(VehicleObjectTest, ConvertsRefittedCapacity)
{
    const auto concorde = makeHeader("CONCOR  ");
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 100, 4, 10), 40);
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 40, 4, 10), 40);
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 99, 4, 10), 99);
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 100, 4, 0), 100);

    const auto hovercraft = makeHeader("HCRAFT1 ");
    EXPECT_EQ(getEffectiveVehicleCapacity(hovercraft, 100, 1, 2), 127);

    const auto jetfoil = makeHeader("JFOIL1  ");
    EXPECT_EQ(getEffectiveVehicleCapacity(jetfoil, 250, 2, 1), 255);

    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 244, 2, 1), 200);
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 255, 2, 1), 200);
    EXPECT_EQ(getEffectiveVehicleCapacity(concorde, 243, 2, 1), 243);
}
