#pragma once

#include "DrawSprite.h"
#include "DrawSpriteHelper.hpp"
#include "Graphics/Gfx.h"
#include "Graphics/RenderTarget.h"

namespace OpenLoco::Gfx
{
    template<DrawBlendOp TBlendOp>
    inline void drawBMPSpriteDoubled(const RenderTarget& rt, const DrawSpriteArgs& args)
    {
        constexpr auto zoom = ZoomLevel{ ZoomLevel::doubled };
        const auto& g1 = args.sourceImage;
        const auto& paletteMap = args.palMap;
        const size_t srcLineWidth = g1.width;
        const size_t dstLineWidth = static_cast<size_t>(rt.width) + rt.pitch;
        auto* dst = rt.bits + dstLineWidth * args.dstPos.y + args.dstPos.x;

        for (int32_t y = 0; y < args.size.height; ++y)
        {
            auto* nextDst = dst + dstLineWidth;
            const auto sourceY = zoom.applyTo(args.srcPos.y + y);
            const auto* srcLine = g1.offset + srcLineWidth * sourceY;
            if constexpr ((TBlendOp & DrawBlendOp::noiseMask) != DrawBlendOp::none)
            {
                const auto* noiseLine = args.noiseImage->offset + srcLineWidth * sourceY;
                for (int32_t x = 0; x < args.size.width; ++x, ++dst)
                {
                    const auto sourceX = zoom.applyTo(args.srcPos.x + x);
                    blitPixel<TBlendOp>(srcLine[sourceX], *dst, paletteMap, noiseLine[sourceX]);
                }
            }
            else
            {
                for (int32_t x = 0; x < args.size.width; ++x, ++dst)
                {
                    const auto sourceX = zoom.applyTo(args.srcPos.x + x);
                    blitPixel<TBlendOp>(srcLine[sourceX], *dst, paletteMap, 0xFF);
                }
            }
            dst = nextDst;
        }
    }

    template<DrawBlendOp TBlendOp>
    inline void drawBMPSpriteMagnify(const RenderTarget& rt, ZoomLevel zoom, const DrawSpriteArgs& args)
    {
        if (zoom == ZoomLevel::doubled)
        {
            drawBMPSpriteDoubled<TBlendOp>(rt, args);
            return;
        }

        const auto& g1 = args.sourceImage;
        const auto& paletteMap = args.palMap;
        const auto scale = zoom.applyInversedTo(1);
        const auto clipLeft = args.srcPos.x;
        const auto clipTop = args.srcPos.y;
        const auto clipRight = clipLeft + args.size.width;
        const auto clipBottom = clipTop + args.size.height;
        const auto sourceLeft = zoom.applyTo(clipLeft);
        const auto sourceTop = zoom.applyTo(clipTop);
        const auto sourceRight = zoom.applyTo(clipRight - 1) + 1;
        const auto sourceBottom = zoom.applyTo(clipBottom - 1) + 1;
        const size_t srcLineWidth = g1.width;
        const size_t dstLineWidth = static_cast<size_t>(rt.width) + rt.pitch;
        auto* const dst0 = rt.bits + dstLineWidth * args.dstPos.y + args.dstPos.x;

        for (auto sourceY = sourceTop; sourceY < sourceBottom; ++sourceY)
        {
            const auto dstTop = std::max(sourceY * scale, clipTop) - clipTop;
            const auto dstBottom = std::min((sourceY + 1) * scale, clipBottom) - clipTop;
            const auto* srcLine = g1.offset + srcLineWidth * sourceY;
            const uint8_t* noiseLine = nullptr;
            if constexpr ((TBlendOp & DrawBlendOp::noiseMask) != DrawBlendOp::none)
            {
                noiseLine = args.noiseImage->offset + srcLineWidth * sourceY;
            }

            for (auto sourceX = sourceLeft; sourceX < sourceRight; ++sourceX)
            {
                const auto dstLeft = std::max(sourceX * scale, clipLeft) - clipLeft;
                const auto dstRight = std::min((sourceX + 1) * scale, clipRight) - clipLeft;
                const auto sourcePixel = srcLine[sourceX];
                auto noisePixel = uint8_t{ 0xFF };
                if constexpr ((TBlendOp & DrawBlendOp::noiseMask) != DrawBlendOp::none)
                {
                    noisePixel = noiseLine[sourceX];
                }

                blitPixelBlock<TBlendOp>(sourcePixel, noisePixel, paletteMap, dst0, dstLineWidth, dstLeft, dstTop, dstRight, dstBottom);
            }
        }
    }

    template<DrawBlendOp TBlendOp, uint8_t TZoomLevel>
    inline void drawBMPSprite(const RenderTarget& rt, const DrawSpriteArgs& args)
    {
        const auto& g1 = args.sourceImage;
        const auto* src = g1.offset + ((static_cast<size_t>(g1.width) * args.srcPos.y) + args.srcPos.x);
        const auto& paletteMap = args.palMap;
        const int32_t width = args.size.width;
        int32_t height = args.size.height;
        const size_t srcLineWidth = g1.width << TZoomLevel;
        const size_t dstLineWidth = static_cast<size_t>(rt.width) + rt.pitch;
        auto* dst = rt.bits;
        // Move the pointer to the start point of the destination
        dst += dstLineWidth * args.dstPos.y + args.dstPos.x;

        constexpr auto zoom = 1 << TZoomLevel;
        if constexpr ((TBlendOp & DrawBlendOp::noiseMask) != DrawBlendOp::none)
        {
            const auto* noiseMask = args.noiseImage->offset + ((static_cast<size_t>(g1.width) * args.srcPos.y) + args.srcPos.x);
            for (; height > 0; height -= zoom)
            {
                auto* nextSrc = src + srcLineWidth;
                auto* nextDst = dst + dstLineWidth;
                auto* nextNoiseMask = noiseMask + srcLineWidth;
                for (int32_t widthRemaining = width; widthRemaining > 0; widthRemaining -= zoom, src += zoom, noiseMask += zoom, dst++)
                {
                    blitPixel<TBlendOp>(*src, *dst, paletteMap, *noiseMask);
                }
                src = nextSrc;
                dst = nextDst;
                noiseMask = nextNoiseMask;
            }
        }
        else
        {
            for (; height > 0; height -= zoom)
            {
                auto* nextSrc = src + srcLineWidth;
                auto* nextDst = dst + dstLineWidth;
                for (int32_t widthRemaining = width; widthRemaining > 0; widthRemaining -= zoom, src += zoom, dst++)
                {
                    blitPixel<TBlendOp>(*src, *dst, paletteMap, 0xFF);
                }
                src = nextSrc;
                dst = nextDst;
            }
        }
    }
}
