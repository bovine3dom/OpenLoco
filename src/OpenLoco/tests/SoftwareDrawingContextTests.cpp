#include "Graphics/DrawSprite.h"
#include "Graphics/Gfx.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/SoftwareDrawingContext.h"
#include "Paint/Paint.h"
#include "Ui/ViewportInteraction.h"
#include <array>
#include <gtest/gtest.h>

using namespace OpenLoco;

namespace
{
    struct G1ElementGuard
    {
        Gfx::G1Element* element;
        Gfx::G1Element previous;

        G1ElementGuard(
            const uint32_t image,
            uint8_t* pixels,
            const int16_t width,
            const int16_t height,
            const Gfx::G1ElementFlags flags = Gfx::G1ElementFlags::none)
            : element(Gfx::getG1Element(image))
            , previous(*element)
        {
            element->offset = pixels;
            element->width = width;
            element->height = height;
            element->xOffset = 0;
            element->yOffset = 0;
            element->flags = flags;
            element->zoomOffset = 0;
        }

        ~G1ElementGuard()
        {
            *element = previous;
        }
    };

    template<size_t TSize>
    void expectFilledRect(
        const std::array<uint8_t, TSize>& pixels,
        const int32_t width,
        const int32_t height,
        const Ui::Rect& rect,
        const uint8_t value)
    {
        for (int32_t y = 0; y < height; ++y)
        {
            for (int32_t x = 0; x < width; ++x)
            {
                const auto expected = x >= rect.left() && x < rect.right() && y >= rect.top() && y < rect.bottom() ? value : 0;
                EXPECT_EQ(pixels[y * width + x], expected) << "at " << x << ", " << y;
            }
        }
    }

    bool isSpriteHit(const ZoomLevel zoom, const Ui::Point& position, const Ui::Point& rasterOffset)
    {
        const Gfx::RenderTarget target{ nullptr, position.x, position.y, 1, 1, 0 };
        Paint::SessionOptions options{};
        Paint::PaintSession session(target, zoom, options);
        session.setCurrentItem(reinterpret_cast<void*>(1));
        session.setItemType(Ui::ViewportInteraction::InteractionItem::entity);
        session.setEntityPosition({ 0, 0 }, rasterOffset);
        if (session.addToPlotListAsParent(ImageId(0), { 0, 0, 0 }, { 1, 1, 1 }) == nullptr)
        {
            return false;
        }
        session.arrangeStructs();
        return session.getNormalInteractionInfo(Ui::ViewportInteraction::InteractionItemFlags::none).type
            == Ui::ViewportInteraction::InteractionItem::entity;
    }
}

TEST(SoftwareDrawingContextTest, AppliesRasterOffsetAfterMagnification)
{
    constexpr uint32_t kImage = 0;
    uint8_t sourcePixel = 7;
    const G1ElementGuard imageGuard(kImage, &sourcePixel, 1, 1);

    std::array<uint8_t, 12 * 12> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 12, 12, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);
    drawingCtx.drawImage(ZoomLevel::quadrupled, { 1, 1 }, ImageId(kImage), { 1, 2 });

    expectFilledRect(pixels, 12, 12, Ui::Rect::fromLTRB(5, 6, 9, 10), sourcePixel);
}

TEST(SoftwareDrawingContextTest, MagnifiesTransparentBitmapBlocksAtEveryScale)
{
    constexpr uint32_t kImage = 0;
    std::array<uint8_t, 4> sourcePixels{ 1, 0, 2, 3 };
    const G1ElementGuard imageGuard(kImage, sourcePixels.data(), 2, 2, Gfx::G1ElementFlags::hasTransparency);

    constexpr auto kStride = 32;
    constexpr auto kBackground = 17;
    for (const auto zoom : std::array{ ZoomLevel{ ZoomLevel::doubled }, ZoomLevel{ ZoomLevel::quadrupled }, ZoomLevel{ ZoomLevel::eightfold }, ZoomLevel{ ZoomLevel::sixteenfold } })
    {
        const auto scale = zoom.applyInversedTo(1);
        std::array<uint8_t, kStride * kStride> pixels;
        pixels.fill(kBackground);
        const Gfx::RenderTarget target{ pixels.data(), 0, 0, 2 * scale, 2 * scale, kStride - 2 * scale };
        Gfx::SoftwareDrawingContext drawingCtx;
        drawingCtx.pushRenderTarget(target);
        drawingCtx.drawImage(zoom, { 0, 0 }, ImageId(kImage));

        for (int32_t y = 0; y < target.height; ++y)
        {
            for (int32_t x = 0; x < target.width; ++x)
            {
                const auto source = sourcePixels[(y / scale) * 2 + x / scale];
                EXPECT_EQ(pixels[y * kStride + x], source == 0 ? kBackground : source) << "at zoom " << static_cast<int32_t>(static_cast<int8_t>(zoom)) << ", " << x << ", " << y;
            }
        }
    }
}

TEST(SoftwareDrawingContextTest, AppliesRasterOffsetToRleSprites)
{
    constexpr uint32_t kImage = 0;
    std::array<uint8_t, 5> sourcePixels{ 2, 0, 0x81, 0, 9 };
    const G1ElementGuard imageGuard(kImage, sourcePixels.data(), 1, 1, Gfx::G1ElementFlags::isRLECompressed);

    std::array<uint8_t, 10 * 10> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 10, 10, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);
    drawingCtx.drawImage(ZoomLevel::quadrupled, { 1, 1 }, ImageId(kImage), { -1, -2 });

    expectFilledRect(pixels, 10, 10, Ui::Rect::fromLTRB(3, 2, 7, 6), 9);
}

TEST(SoftwareDrawingContextTest, MagnifiesClippedSparseRleSpritesAtEveryScale)
{
    constexpr uint32_t kImage = 0;
    std::array<uint8_t, 20> sourcePixels{
        6, 0, 13, 0, 15, 0,
        0x02, 0, 1, 2, 0x81, 3, 3,
        0x80, 0,
        0x83, 1, 4, 0, 5,
    };
    const G1ElementGuard imageGuard(kImage, sourcePixels.data(), 4, 3, Gfx::G1ElementFlags::isRLECompressed);

    constexpr auto kStride = 64;
    constexpr auto kBackground = 17;
    constexpr std::array<uint8_t, 12> expectedSource{
        1, 2, kBackground, 3,
        kBackground, kBackground, kBackground, kBackground,
        kBackground, 4, 0, 5,
    };
    for (const auto zoom : std::array{ ZoomLevel{ ZoomLevel::doubled }, ZoomLevel{ ZoomLevel::quadrupled }, ZoomLevel{ ZoomLevel::eightfold }, ZoomLevel{ ZoomLevel::sixteenfold } })
    {
        const auto scale = zoom.applyInversedTo(1);
        const auto width = 4 * scale - 2;
        const auto height = 3 * scale - 3;
        std::array<uint8_t, kStride * 48> pixels;
        pixels.fill(kBackground);
        const Gfx::RenderTarget target{ pixels.data(), 2, 1, width, height, kStride - width };
        Gfx::SoftwareDrawingContext drawingCtx;
        drawingCtx.pushRenderTarget(target);
        drawingCtx.drawImage(zoom, { 0, 0 }, ImageId(kImage), { 1, -1 });

        for (int32_t y = 0; y < height; ++y)
        {
            for (int32_t x = 0; x < width; ++x)
            {
                const auto sourceX = (target.x + x - 1) / scale;
                const auto sourceY = (target.y + y + 1) / scale;
                EXPECT_EQ(pixels[y * kStride + x], expectedSource[sourceY * 4 + sourceX]) << "at zoom " << static_cast<int32_t>(static_cast<int8_t>(zoom)) << ", " << x << ", " << y;
            }
        }
    }
}

TEST(SoftwareDrawingContextTest, MagnifiedDestinationBlendUsesEachBackgroundPixel)
{
    uint8_t bitmapSource = 1;
    Gfx::G1Element bitmap{};
    bitmap.offset = &bitmapSource;
    bitmap.width = 1;
    bitmap.height = 1;
    bitmap.flags = Gfx::G1ElementFlags::hasTransparency;
    std::array<uint8_t, 5> rleSource{ 2, 0, 0x81, 0, 1 };
    Gfx::G1Element rle = bitmap;
    rle.offset = rleSource.data();
    rle.flags = Gfx::G1ElementFlags::isRLECompressed;
    Gfx::PaletteMap::Buffer<Gfx::PaletteMap::kDefaultSize> palette;
    for (size_t i = 0; i < palette.size(); ++i)
    {
        palette[i] = static_cast<uint8_t>(255 - i);
    }

    constexpr auto zoom = ZoomLevel{ ZoomLevel::quadrupled };
    constexpr auto scale = 4;
    std::array<uint8_t, scale * scale> bitmapPixels;
    for (size_t i = 0; i < bitmapPixels.size(); ++i)
    {
        bitmapPixels[i] = static_cast<uint8_t>(i + 1);
    }
    auto rlePixels = bitmapPixels;
    const Gfx::RenderTarget bitmapTarget{ bitmapPixels.data(), 0, 0, scale, scale, 0 };
    const Gfx::RenderTarget rleTarget{ rlePixels.data(), 0, 0, scale, scale, 0 };
    const Gfx::DrawSpriteArgs bitmapArgs{ palette, bitmap, {}, {}, { scale, scale }, nullptr };
    const Gfx::DrawSpriteArgs rleArgs{ palette, rle, {}, {}, { scale, scale }, nullptr };
    constexpr auto blend = Gfx::DrawBlendOp::transparent | Gfx::DrawBlendOp::dst;

    Gfx::drawSpriteToBufferMagnify<false>(bitmapTarget, zoom, bitmapArgs, blend);
    Gfx::drawSpriteToBufferMagnify<true>(rleTarget, zoom, rleArgs, blend);

    for (size_t i = 0; i < bitmapPixels.size(); ++i)
    {
        EXPECT_EQ(bitmapPixels[i], palette[i + 1]);
        EXPECT_EQ(rlePixels[i], palette[i + 1]);
    }
}

TEST(SoftwareDrawingContextTest, MagnifiedBitmapHonoursNoiseMask)
{
    std::array<uint8_t, 2> source{ 9, 8 };
    std::array<uint8_t, 2> noise{ 0, 0xFF };
    Gfx::G1Element bitmap{};
    bitmap.offset = source.data();
    bitmap.width = 2;
    bitmap.height = 1;
    Gfx::G1Element noiseImage = bitmap;
    noiseImage.offset = noise.data();
    constexpr auto zoom = ZoomLevel{ ZoomLevel::quadrupled };
    constexpr auto scale = 4;
    std::array<uint8_t, scale * scale * 2> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, scale * 2, scale, 0 };
    const Gfx::DrawSpriteArgs args{ {}, bitmap, {}, {}, { scale * 2, scale }, &noiseImage };

    Gfx::drawSpriteToBufferMagnify<false>(target, zoom, args, Gfx::DrawBlendOp::transparent | Gfx::DrawBlendOp::noiseMask);

    expectFilledRect(pixels, scale * 2, scale, Ui::Rect::fromLTRB(scale, 0, scale * 2, scale), 8);
}

TEST(SoftwareDrawingContextTest, AppliesRasterOffsetToMaskedSprites)
{
    constexpr uint32_t kImage = 0;
    constexpr uint32_t kMask = 1;
    uint8_t sourcePixel = 7;
    uint8_t maskPixel = 3;
    const G1ElementGuard imageGuard(kImage, &sourcePixel, 1, 1);
    const G1ElementGuard maskGuard(kMask, &maskPixel, 1, 1);

    std::array<uint8_t, 10 * 10> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 10, 10, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);
    drawingCtx.drawImageMasked(ZoomLevel::quadrupled, { 1, 1 }, ImageId(kImage), ImageId(kMask), { 1, 1 });

    expectFilledRect(pixels, 10, 10, Ui::Rect::fromLTRB(5, 5, 9, 9), 3);
}

TEST(SoftwareDrawingContextTest, DrawsMagnifiedSpritesAsAnOpaqueSolidColour)
{
    constexpr uint32_t kImage = 0;
    std::array<uint8_t, 5> sourcePixels{ 2, 0, 0x81, 0, 7 };
    const G1ElementGuard imageGuard(kImage, sourcePixels.data(), 1, 1, Gfx::G1ElementFlags::isRLECompressed);

    std::array<uint8_t, 12 * 12> pixels{};
    const Gfx::RenderTarget target{ pixels.data(), 0, 0, 12, 12, 0 };
    Gfx::SoftwareDrawingContext drawingCtx;
    drawingCtx.pushRenderTarget(target);
    drawingCtx.drawImageSolid(ZoomLevel::quadrupled, { 1, 1 }, ImageId(kImage), 42, { 1, 2 });

    expectFilledRect(pixels, 12, 12, Ui::Rect::fromLTRB(5, 6, 9, 10), 42);
}

TEST(PaintInteractionTest, AppliesRasterOffsetAfterMagnification)
{
    uint8_t sourcePixel = 7;
    const G1ElementGuard imageGuard(0, &sourcePixel, 1, 1, Gfx::G1ElementFlags::hasTransparency);

    for (int32_t y = 2; y < 6; ++y)
    {
        for (int32_t x = 1; x < 5; ++x)
        {
            EXPECT_TRUE(isSpriteHit(ZoomLevel::quadrupled, { x, y }, { 1, 2 })) << "at " << x << ", " << y;
        }
    }
    EXPECT_FALSE(isSpriteHit(ZoomLevel::quadrupled, { 0, 2 }, { 1, 2 }));
    EXPECT_FALSE(isSpriteHit(ZoomLevel::quadrupled, { 5, 2 }, { 1, 2 }));
    EXPECT_FALSE(isSpriteHit(ZoomLevel::quadrupled, { 1, 1 }, { 1, 2 }));
    EXPECT_FALSE(isSpriteHit(ZoomLevel::quadrupled, { 1, 6 }, { 1, 2 }));
    EXPECT_TRUE(isSpriteHit(ZoomLevel::quadrupled, { -1, -2 }, { -1, -2 }));
    EXPECT_FALSE(isSpriteHit(ZoomLevel::quadrupled, { 3, -2 }, { -1, -2 }));

    EXPECT_TRUE(isSpriteHit(ZoomLevel::full, { 0, 0 }, { 1, 2 }));
    EXPECT_FALSE(isSpriteHit(ZoomLevel::full, { 1, 2 }, { 1, 2 }));
}
