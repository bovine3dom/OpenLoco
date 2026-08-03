#include "Localisation/FormatArguments.hpp"
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
