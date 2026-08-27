#include "Localisation/FormatArguments.hpp"
#include "Speed.hpp"
#include <OpenLoco/Core/Exception.hpp>
#include <gtest/gtest.h>
#include <limits>

using namespace OpenLoco;

TEST(FormatArgumentsTests, AllowsExactCapacity)
{
    FormatArgumentsBuffer buffer;
    FormatArguments args{ buffer };

    args.skip(buffer.capacity());

    EXPECT_EQ(args.getLength(), buffer.capacity());
}

TEST(FormatArgumentsTests, RejectsOverflowWithoutAdvancing)
{
    FormatArgumentsBuffer buffer;
    FormatArguments args{ buffer };
    args.skip(buffer.capacity());

    EXPECT_THROW(args.skip(2), Exception::OutOfRange);
    EXPECT_THROW(args.skip(std::numeric_limits<size_t>::max()), Exception::OutOfRange);
    EXPECT_EQ(args.getLength(), buffer.capacity());
}

TEST(FormatArgumentsTests, PreservesMapTooltipTransportMode)
{
    auto args = FormatArguments::mapToolTip(uint16_t{ 1 });
    args.setTransportMode(3);

    EXPECT_EQ(FormatArguments::mapToolTip().getTransportMode(), 3);
    EXPECT_EQ(FormatArguments::mapToolTip(uint16_t{ 1 }).getTransportMode(), 0xFF);
}

TEST(SpeedTests, ConvertsSpeedToTilesPerDay)
{
    EXPECT_EQ(speedToTilesPerDay(21, 21), 300);
    EXPECT_EQ(speedToTilesPerDay(31, 31), 300);
    EXPECT_EQ(speedToTilesPerDay(36, 36), 300);
    EXPECT_EQ(speedToTilesPerDay(100, 21), 1430);
    EXPECT_EQ(speedToTilesPerDay(100, 31), 969);
    EXPECT_EQ(speedToTilesPerDay(100, 36), 834);
    EXPECT_EQ(speedToTilesPerDay(0, 21), 0);
}

TEST(SpeedTests, ConvertsSpeedToDaysPerTile)
{
    EXPECT_EQ(speedToDaysPerTile(21, 21), 33);
    EXPECT_EQ(speedToDaysPerTile(31, 31), 33);
    EXPECT_EQ(speedToDaysPerTile(36, 36), 33);
    EXPECT_EQ(speedToDaysPerTile(100, 21), 7);
    EXPECT_EQ(speedToDaysPerTile(100, 31), 10);
    EXPECT_EQ(speedToDaysPerTile(100, 36), 12);
    EXPECT_EQ(speedToDaysPerTile(32767, 21), 0);
    EXPECT_EQ(speedToDaysPerTile(0, 21), std::nullopt);
}
