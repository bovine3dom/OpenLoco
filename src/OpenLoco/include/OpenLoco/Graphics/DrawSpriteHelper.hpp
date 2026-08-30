#pragma once
#include "DrawSprite.h"
#include "Graphics/Gfx.h"
#include "Graphics/PaletteMap.h"
#include <algorithm>

namespace OpenLoco::Gfx
{
    inline uint8_t blend(const PaletteMap::View paletteMap, uint8_t src, uint8_t dst)
    {
        // src = 0 would be transparent so there is no blend palette for that, hence src - 1
        assert(src != 0);

        // src is treated as a row in the palette map, validate its in range.
        const auto row = src - 1u;
        assert(row < (paletteMap.size() / PaletteMap::kDefaultSize));

        const auto idx = (row * PaletteMap::kDefaultSize) + dst;
        assert(idx < paletteMap.size());

        return paletteMap[idx];
    }

    template<DrawBlendOp TBlendOp>
    bool blitPixel(uint8_t src, uint8_t& dst, [[maybe_unused]] const PaletteMap::View paletteMap, const uint8_t noiseMask)
    {
        if constexpr ((TBlendOp & DrawBlendOp::noiseMask) != DrawBlendOp::none)
        {
            // noiseMask is either 0 or 0xFF
            src &= noiseMask;
        }
        if constexpr ((TBlendOp & DrawBlendOp::transparent) != DrawBlendOp::none)
        {
            // Ignore transparent pixels
            if (src == PaletteIndex::transparent)
            {
                return false;
            }
        }

        if constexpr (((TBlendOp & DrawBlendOp::src) != DrawBlendOp::none) && ((TBlendOp & DrawBlendOp::dst) != DrawBlendOp::none))
        {
            auto pixel = blend(paletteMap, src, dst);
            if constexpr ((TBlendOp & DrawBlendOp::transparent) != DrawBlendOp::none)
            {
                if (pixel == PaletteIndex::transparent)
                {
                    return false;
                }
            }
            dst = pixel;
            return true;
        }
        else if constexpr ((TBlendOp & DrawBlendOp::src) != DrawBlendOp::none)
        {
            auto pixel = paletteMap[src];
            if constexpr ((TBlendOp & DrawBlendOp::transparent) != DrawBlendOp::none)
            {
                if (pixel == PaletteIndex::transparent)
                {
                    return false;
                }
            }
            dst = pixel;
            return true;
        }
        else if constexpr ((TBlendOp & DrawBlendOp::dst) != DrawBlendOp::none)
        {
            auto pixel = paletteMap[dst];
            if constexpr ((TBlendOp & DrawBlendOp::transparent) != DrawBlendOp::none)
            {
                if (pixel == PaletteIndex::transparent)
                {
                    return false;
                }
            }
            dst = pixel;
            return true;
        }
        else
        {
            dst = src;
            return true;
        }
    }

    template<DrawBlendOp TBlendOp>
    void blitPixelBlock(
        const uint8_t src,
        const uint8_t noiseMask,
        const PaletteMap::View paletteMap,
        uint8_t* const dst0,
        const size_t dstLineWidth,
        const int32_t left,
        const int32_t top,
        const int32_t right,
        const int32_t bottom)
    {
        const auto width = right - left;
        if constexpr ((TBlendOp & DrawBlendOp::dst) == DrawBlendOp::none)
        {
            uint8_t outputPixel = 0;
            if (!blitPixel<TBlendOp>(src, outputPixel, paletteMap, noiseMask))
            {
                return;
            }
            auto* firstDst = dst0 + top * dstLineWidth + left;
            std::fill_n(firstDst, width, outputPixel);
            for (auto y = top + 1; y < bottom; ++y)
            {
                std::copy_n(firstDst, width, dst0 + y * dstLineWidth + left);
            }
        }
        else
        {
            for (auto y = top; y < bottom; ++y)
            {
                auto* dst = dst0 + y * dstLineWidth + left;
                for (auto x = 0; x < width; ++x)
                {
                    blitPixel<TBlendOp>(src, *dst++, paletteMap, noiseMask);
                }
            }
        }
    }
}
