#include "Localisation/FormatArguments.hpp"
#include "Localisation/Formatting.h"
#include "Localisation/StringManager.h"
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

TEST(FormatArgumentsTests, StopsReadingAtBufferBoundary)
{
    FormatArgumentsBuffer buffer;
    FormatArgumentsView view{ buffer };
    for (size_t i = 0; i < buffer.capacity() / sizeof(uint16_t); ++i)
    {
        view.skip<uint16_t>();
    }

    EXPECT_EQ(view.pop<uint16_t>(), 0);
    view.skip<uint32_t>();
    EXPECT_EQ(view.pop<uint32_t>(), 0);
}

TEST(FormatArgumentsTests, TruncatesBeforeIncompleteControlCode)
{
    const char input[] = { static_cast<char>(ControlCodes::inlineSpriteStr), 1, 2, 3, 4, 'X', '\0' };

    EXPECT_EQ(StringManager::locoStrlenS(input, 4), 0);
    EXPECT_EQ(StringManager::locoStrlenS(input, 5), 5);
    EXPECT_EQ(StringManager::locoStrlenS(input, 6), 6);
}

TEST(FormatArgumentsTests, RejectsInvalidGeneratedTown)
{
    FormatArgumentsBuffer buffer;
    FormatArguments args{ buffer };
    args.push(TownId::null);
    char output[128]{};

    StringManager::formatString(output, StringManager::kTownNamesStart, args);

    EXPECT_STREQ(output, "(invalid town id: 65535)");
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
