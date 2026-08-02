#include <OpenLoco/Graphics/PixelScaling.h>

#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <span>

using OpenLoco::PaletteIndex_t;
using namespace OpenLoco::Gfx;

TEST(MMPXTest, ScalesOnePixelWithClampedBorders)
{
    const std::array<PaletteIndex_t, 1> src{ 42 };
    std::array<PaletteIndex_t, 4> dst{};
    const std::array<PaletteEntry, 256> palette{};

    ASSERT_TRUE(scaleMmpx2x(src, 1, 1, 1, dst, 2, palette));

    const std::array<PaletteIndex_t, 4> expected{ 42, 42, 42, 42 };
    EXPECT_EQ(dst, expected);
}

TEST(MMPXTest, ScalesConstantImageWithPaddedPitches)
{
    constexpr size_t kSrcPitch = 4;
    constexpr size_t kDstPitch = 6;
    std::array<PaletteIndex_t, 8> src;
    src.fill(17);
    src[2] = 91;
    src[3] = 92;
    src[6] = 93;
    src[7] = 94;
    std::array<PaletteIndex_t, 24> dst;
    dst.fill(200);
    const std::array<PaletteEntry, 256> palette{};

    ASSERT_TRUE(scaleMmpx2x(src, 2, 2, kSrcPitch, dst, kDstPitch, palette));

    for (size_t y = 0; y < 4; ++y)
    {
        for (size_t x = 0; x < 4; ++x)
        {
            EXPECT_EQ(dst[y * kDstPitch + x], 17);
        }
        for (size_t x = 4; x < kDstPitch; ++x)
        {
            EXPECT_EQ(dst[y * kDstPitch + x], 200);
        }
    }
}

TEST(MMPXTest, RejectsInvalidDimensionsBuffersPaletteAndOverflow)
{
    const std::array<PaletteIndex_t, 4> src{ 1, 1, 1, 1 };
    std::array<PaletteIndex_t, 16> dst;
    dst.fill(200);
    const auto untouched = dst;
    const std::array<PaletteEntry, 256> palette{};

    EXPECT_FALSE(scaleMmpx2x(src, 0, 2, 2, dst, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 2, -1, 2, dst, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, std::numeric_limits<int32_t>::max(), 1, 1, dst, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 2, 2, 1, dst, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 2, 2, 2, dst, 3, palette));
    EXPECT_FALSE(scaleMmpx2x(std::span<const PaletteIndex_t>{ src.data(), 3 }, 2, 2, 2, dst, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 2, 2, 2, std::span<PaletteIndex_t>{ dst.data(), 15 }, 4, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 2, 2, 2, dst, 4, std::span<const PaletteEntry>{ palette.data(), 255 }));
    EXPECT_FALSE(scaleMmpx2x(src, 1, 2, std::numeric_limits<size_t>::max(), dst, 2, palette));
    EXPECT_FALSE(scaleMmpx2x(src, 1, 1, 1, dst, std::numeric_limits<size_t>::max(), palette));
    EXPECT_EQ(dst, untouched);
}

TEST(MMPXTest, UsesExactIndicesAndPaletteRgbLuminanceForTopEdgeRule)
{
    constexpr size_t kWidth = 5;
    std::array<PaletteIndex_t, kWidth * kWidth> src;
    src.fill(7);
    // Arrange the reference neighborhood so only the top luminance intersection rule matches.
    src[1 * kWidth + 1] = 3;
    src[1 * kWidth + 2] = 2;
    src[1 * kWidth + 3] = 4;
    src[2 * kWidth + 1] = 5;
    src[2 * kWidth + 2] = 1;
    src[2 * kWidth + 3] = 6;
    src[3 * kWidth + 1] = src[3 * kWidth + 2] = src[3 * kWidth + 3] = src[4 * kWidth + 2] = 1;
    std::array<PaletteIndex_t, 100> dst{};
    std::array<PaletteEntry, 256> palette{};
    palette[1] = { 10, 10, 10, 255 };
    palette[2] = { 1, 1, 1, 0 };
    palette[3] = palette[1];
    palette[4] = palette[1];
    palette[5] = palette[1];
    palette[6] = palette[1];

    ASSERT_TRUE(scaleMmpx2x(src, kWidth, kWidth, kWidth, dst, kWidth * 2, palette));
    EXPECT_EQ(dst[4 * kWidth * 2 + 4], 2);
    EXPECT_EQ(dst[4 * kWidth * 2 + 5], 2);
    EXPECT_EQ(dst[5 * kWidth * 2 + 4], 1);
    EXPECT_EQ(dst[5 * kWidth * 2 + 5], 1);

    palette[2] = { 20, 20, 20, 0 };
    ASSERT_TRUE(scaleMmpx2x(src, kWidth, kWidth, kWidth, dst, kWidth * 2, palette));
    EXPECT_EQ(dst[4 * kWidth * 2 + 4], 1);
    EXPECT_EQ(dst[4 * kWidth * 2 + 5], 1);
}
