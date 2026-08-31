#include "Objects/VehicleObject.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <string_view>

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
