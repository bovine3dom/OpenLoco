#include "Graphics/Gfx.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/SoftwareDrawingContext.h"
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
