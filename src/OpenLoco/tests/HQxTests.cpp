#include <OpenLoco/Graphics/PixelScaling.h>

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Gfx;

namespace
{
    std::array<PaletteEntry, 256> makePalette()
    {
        std::array<PaletteEntry, 256> palette{};
        palette[1] = { 0xFF, 0xFF, 0xFF, 0 };
        return palette;
    }

    void expectDiagonalEdge(const std::array<PaletteEntry, 256>& palette, uint8_t factor, std::string_view expected)
    {
        constexpr std::array<PaletteIndex_t, 9> src{ 0, 0, 1, 0, 1, 1, 1, 1, 1 };
        const auto width = static_cast<size_t>(factor) * 3;
        std::vector<uint32_t> dst(width * width);

        ASSERT_EQ(expected.size(), dst.size());
        ASSERT_TRUE(scaleHqx(src, 3, 3, 3, dst, width, palette, factor));
        for (size_t i = 0; i < dst.size(); ++i)
        {
            // Oracle characters encode grayscale channels as 0x00, 0x1F, ..., 0xFF.
            const auto nibble = expected[i] <= '9' ? expected[i] - '0' : expected[i] - 'A' + 10;
            const auto channel = nibble == 0 ? 0U : static_cast<uint32_t>(nibble * 16 + 15);
            EXPECT_EQ(dst[i], 0xFF000000U | channel * 0x00010101U) << "pixel " << i;
        }
    }
}

TEST(HQxTests, UniformOneByOneScalesAtEveryFactor)
{
    auto palette = makePalette();
    palette[17] = { 0x33, 0x22, 0x11, 0x7F };
    constexpr std::array<PaletteIndex_t, 1> src = { 17 };

    for (const uint8_t factor : std::array<uint8_t, 3>{ 2, 3, 4 })
    {
        std::vector<uint32_t> dst(factor * factor);
        ASSERT_TRUE(scaleHqx(src, 1, 1, 1, dst, factor, palette, factor));
        for (const auto pixel : dst)
        {
            EXPECT_EQ(pixel, 0xFF112233U);
        }
    }
}

TEST(HQxTests, RejectsInvalidFactorsDimensionsAndBuffers)
{
    const auto palette = makePalette();
    constexpr std::array<PaletteIndex_t, 1> src = { 0 };
    std::array<uint32_t, 4> dst{};

    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, dst, 2, palette, 1));
    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, dst, 2, palette, 5));
    EXPECT_FALSE(scaleHqx(src, 0, 1, 1, dst, 2, palette, 2));
    EXPECT_FALSE(scaleHqx(src, 1, -1, 1, dst, 2, palette, 2));
    EXPECT_FALSE(scaleHqx(src, std::numeric_limits<int32_t>::max(), 1, 1, dst, 2, palette, 4));
    EXPECT_FALSE(scaleHqx(std::span<const PaletteIndex_t>{}, 1, 1, 1, dst, 2, palette, 2));
    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, std::span<uint32_t>{}, 2, palette, 2));
    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, std::span<uint32_t>{ dst }.first(3), 2, palette, 2));
    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, dst, 1, palette, 2));
    EXPECT_FALSE(scaleHqx(src, 1, 1, 1, dst, 2, std::span<const PaletteEntry>{ palette }.first(255), 2));
}

TEST(HQxTests, HonoursSourceAndDestinationPitch)
{
    auto palette = makePalette();
    palette[1] = { 0x56, 0x34, 0x12, 0 };
    constexpr std::array<PaletteIndex_t, 6> src = {
        1,
        1,
        0xFF,
        1,
        1,
        0xFF,
    };
    constexpr size_t kDstPitch = 6;
    constexpr uint32_t kPadding = 0xDEADBEEFU;
    std::array<uint32_t, kDstPitch * 4> dst{};
    dst.fill(kPadding);

    ASSERT_TRUE(scaleHqx(src, 2, 2, 3, dst, kDstPitch, palette, 2));
    for (size_t y = 0; y < 4; ++y)
    {
        for (size_t x = 0; x < 4; ++x)
        {
            EXPECT_EQ(dst[y * kDstPitch + x], 0xFF123456U);
        }
        EXPECT_EQ(dst[y * kDstPitch + 4], kPadding);
        EXPECT_EQ(dst[y * kDstPitch + 5], kPadding);
    }
}

TEST(HQxTests, InterpolatesDiagonalEdgeLikeFFmpegAtEveryFactor)
{
    const auto palette = makePalette();

    expectDiagonalEdge(palette, 2, "0003FF"
                                   "000BFF"
                                   "007FFF"
                                   "3BFFFF"
                                   "FFFFFF"
                                   "FFFFFF");
    expectDiagonalEdge(palette, 3, "000003FFF"
                                   "00000BFFF"
                                   "00003FFFF"
                                   "0001DFFFF"
                                   "003DFFFFF"
                                   "3BFFFFFFF"
                                   "FFFFFFFFF"
                                   "FFFFFFFFF"
                                   "FFFFFFFFF");
    expectDiagonalEdge(palette, 4, "00000003FFFF"
                                   "0000000BFFFF"
                                   "0000003FFFFF"
                                   "000000BFFFFF"
                                   "000007FFFFFF"
                                   "00007FFFFFFF"
                                   "003BFFFFFFFF"
                                   "3BFFFFFFFFFF"
                                   "FFFFFFFFFFFF"
                                   "FFFFFFFFFFFF"
                                   "FFFFFFFFFFFF"
                                   "FFFFFFFFFFFF");
}
